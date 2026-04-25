#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        
        # Публикатор для управления движением с использованием TwistStamped
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        # Подписчик на одометрию
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Текущая позиция и ориентация
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Начальная позиция (для отслеживания относительного движения)
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
        
        # Целевые значения
        self.target_distance = 0.0
        self.target_angle = 0.0
        
        # Флаги состояния
        self.moving = False
        self.rotating = False
        
        # Таймер для основного цикла управления
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.action_list = [
            {'type': 'forward', 'value': 0.4},    # Вперед на 2.8 метра
            {'type': 'rotate', 'value': 90.0},   # Поворот на -90 градусов

            {'type': 'forward', 'value': 1.2},    # Вперед на 2.8 метр
            {'type': 'rotate', 'value': 180.0},   # Поворот на -90 градусов

            {'type': 'forward', 'value': 1.2},    # Вперед на 2.0 метра
            {'type': 'rotate', 'value': -90.0},   # Поворот на -90 градусов

            {'type': 'forward', 'value': 0.4},    # Вперед на 2.0 метра
            #{'type': 'rotate', 'value': -90.0},   # Поворот на -90 градусов

            #{'type': 'forward', 'value': 1.2},    # Вперед на 2.0 метра
            #{'type': 'rotate', 'value': -90.0},   # Поворот на -90 градусов

            #{'type': 'forward', 'value': 2.0},    # Вперед на 2.0 метр
            #{'type': 'rotate', 'value': -90.0},   # Поворот на -90 градусов

            #{'type': 'forward', 'value': 1.0},    # Вперед на 0.8 метра
        ]

        
        self.current_action_index = 0
        self.action_started = False
        
        self.get_logger().info('TurtleBot3 Controller инициализирован (использует TwistStamped)')

    def quaternion_to_yaw(self, quaternion):
        """Конвертирует кватернион в угол yaw (в радианах)"""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Вычисляем угол yaw из кватерниона
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def odom_callback(self, msg):
        """Обработка сообщений одометрии"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Получаем ориентацию из кватерниона
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(orientation)

    def normalize_angle(self, angle):
        """Нормализует угол в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def start_movement(self, distance):
        """Начинает движение вперед на заданное расстояние"""
        self.moving = True
        self.rotating = False
        
        # Сохраняем начальную позицию
        self.initial_x = self.current_x
        self.initial_y = self.current_y
        self.target_distance = distance
        
        self.get_logger().info(f'Начинаем движение на {distance:.2f} метров')

    def start_rotation(self, angle_degrees):
        """Начинает поворот на заданный угол"""
        self.moving = False
        self.rotating = True
        
        # Сохраняем начальную ориентацию
        self.initial_yaw = self.current_yaw
        
        # Конвертируем градусы в радианы и нормализуем
        angle_rad = math.radians(angle_degrees)
        self.target_angle = self.normalize_angle(angle_rad)
        
        self.get_logger().info(f'Начинаем поворот на {angle_degrees} градусов')

    def control_loop(self):
        """Основной цикл управления"""
        if self.current_action_index >= len(self.action_list):
            # Все действия выполнены
            self.stop_robot()
            return
        
        if not self.action_started:
            # Начинаем новое действие
            current_action = self.action_list[self.current_action_index]
            
            if current_action['type'] == 'forward':
                self.start_movement(current_action['value'])
            elif current_action['type'] == 'rotate':
                self.start_rotation(current_action['value'])
            
            self.action_started = True
            return
        
        # Проверяем завершение текущего действия
        if self.moving:
            if self.is_movement_complete():
                self.complete_current_action()
        
        elif self.rotating:
            if self.is_rotation_complete():
                self.complete_current_action()
        
        # Публикуем команды управления
        self.publish_control_command()

    def is_movement_complete(self):
        """Проверяет завершение движения вперед"""
        # Вычисляем пройденное расстояние
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        distance_traveled = math.sqrt(dx*dx + dy*dy)
        
        # Движение завершено, если пройденное расстояние достигло цели
        return distance_traveled >= self.target_distance

    def is_rotation_complete(self):
        """Проверяет завершение поворота"""
        # Вычисляем пройденный угол
        angle_traveled = self.normalize_angle(self.current_yaw - self.initial_yaw)
        
        # Для положительных углов
        if self.target_angle >= 0:
            return angle_traveled >= self.target_angle
        # Для отрицательных углов
        else:
            return angle_traveled <= self.target_angle

    def complete_current_action(self):
        """Завершает текущее действие и переходит к следующему"""
        self.stop_robot()
        time.sleep(0.5)  # Пауза между действиями
        
        self.current_action_index += 1
        self.action_started = False
        
        if self.current_action_index < len(self.action_list):
            action = self.action_list[self.current_action_index]
            self.get_logger().info(f'Завершено действие {self.current_action_index}. Следующее: {action["type"]} {action["value"]}')
        else:
            self.get_logger().info('Все действия выполнены!')

    def publish_control_command(self):
        """Публикует команды управления с использованием TwistStamped"""
        cmd_vel_stamped = TwistStamped()
        
        # Устанавливаем временную метку
        cmd_vel_stamped.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_stamped.header.frame_id = 'base_link'
        
        if self.moving:
            # Линейная скорость для движения вперед
            cmd_vel_stamped.twist.linear.x = 0.15  # м/с
            
            # Вычисляем оставшееся расстояние для регулировки скорости
            dx = self.current_x - self.initial_x
            dy = self.current_y - self.initial_y
            distance_traveled = math.sqrt(dx*dx + dy*dy)
            remaining_distance = self.target_distance - distance_traveled
            
            # Замедляемся при приближении к цели
            if remaining_distance < 0.2:
                cmd_vel_stamped.twist.linear.x = 0.05
            
        elif self.rotating:
            # Угловая скорость для поворота
            angle_traveled = self.normalize_angle(self.current_yaw - self.initial_yaw)
            remaining_angle = self.normalize_angle(self.target_angle - angle_traveled)
            
            # Пропорциональный контроль
            angular_speed = 0.8 * remaining_angle
            
            # Ограничиваем максимальную угловую скорость
            if angular_speed > 0.5:
                angular_speed = 0.5
            elif angular_speed < -0.5:
                angular_speed = -0.5
            elif abs(angular_speed) < 0.1:
                # Минимальная скорость для преодоления трения
                angular_speed = 0.1 if angular_speed > 0 else -0.1
            
            cmd_vel_stamped.twist.angular.z = angular_speed
        
        self.cmd_vel_pub.publish(cmd_vel_stamped)

    def stop_robot(self):
        """Останавливает робота"""
        cmd_vel_stamped = TwistStamped()
        cmd_vel_stamped.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_stamped.header.frame_id = 'base_link'
        cmd_vel_stamped.twist.linear.x = 0.0
        cmd_vel_stamped.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_stamped)
        
        self.moving = False
        self.rotating = False

    def add_action(self, action_type, value):
        """Добавляет действие в список"""
        self.action_list.append({'type': action_type, 'value': value})

    def clear_actions(self):
        """Очищает список действий"""
        self.action_list.clear()
        self.current_action_index = 0
        self.action_started = False

def main(args=None):
    rclpy.init(args=args)
    
    controller = TurtleBot3Controller()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
