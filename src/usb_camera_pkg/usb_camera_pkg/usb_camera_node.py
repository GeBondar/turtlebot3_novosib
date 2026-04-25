#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Параметры
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_compressed', True)
        self.declare_parameter('use_mjpg', True)  # Новый параметр для MJPG
        self.declare_parameter('backend', 'auto')  # auto, v4l2, msmt, etc
        
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.enable_compressed = self.get_parameter('enable_compressed').value
        self.use_mjpg = self.get_parameter('use_mjpg').value
        self.backend = self.get_parameter('backend').value
        
        # Публикаторы
        self.image_pub = self.create_publisher(Image, 'image_raw', 30)
        self.compressed_pub = None
        if self.enable_compressed:
            self.compressed_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        
        # Инициализация камеры
        self.bridge = CvBridge()
        self.cap = None
        
        self.get_logger().info(f'USB Camera node starting with camera_id: {self.camera_id}')
        self.get_logger().info(f'MJPG mode: {self.use_mjpg}')
        
        # Автопоиск камеры если указанная не доступна
        if not self.init_camera(self.camera_id):
            self.get_logger().warning(f'Camera {self.camera_id} not available, searching for available camera...')
            self.find_available_camera()
        
        # Таймер для захвата и публикации кадров
        if self.cap and self.cap.isOpened():
            self.timer_period = 1.0 / self.fps
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.get_logger().info('Camera node started successfully')
        else:
            self.get_logger().error('Failed to initialize any camera')
    
    def find_available_camera(self):
        """Поиск доступной камеры"""
        for camera_id in range(5):  # Проверяем первые 5 камер
            if self.init_camera(camera_id):
                self.camera_id = camera_id
                self.get_logger().info(f'Using camera ID: {camera_id}')
                return True
        return False
    
    def init_camera(self, camera_id):
        """Инициализация камеры по ID с поддержкой MJPG"""
        try:
            if self.cap:
                self.cap.release()
                
            # Выбираем бэкенд в зависимости от параметра
            if self.backend == 'v4l2':
                self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
            elif self.backend == 'msmf':
                self.cap = cv2.VideoCapture(camera_id, cv2.CAP_MSMF)
            else:
                self.cap = cv2.VideoCapture(camera_id)
                
            if not self.cap.isOpened():
                self.get_logger().error(f'Cannot open camera {camera_id}')
                return False
            
            # Пробуем установить MJPG формат если включен
            if self.use_mjpg:
                # Пробуем установить MJPG (обычно FOURCC='MJPG')
                mjpg_success = self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                if mjpg_success:
                    self.get_logger().info('MJPG format set successfully')
                else:
                    self.get_logger().warning('MJPG format not supported, using default')
            
            # Устанавливаем разрешение и FPS
            width_success = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            height_success = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            fps_success = self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Устанавливаем дополнительные параметры для стабильности
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            
            # Получаем актуальные установленные значения
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            
            # Декодируем FOURCC код в читаемый формат
            fourcc_str = ''.join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
            
            self.get_logger().info(f'Camera {camera_id} settings:')
            self.get_logger().info(f'  Resolution: {actual_width}x{actual_height} (requested: {self.frame_width}x{self.frame_height})')
            self.get_logger().info(f'  FPS: {actual_fps:.1f} (requested: {self.fps})')
            self.get_logger().info(f'  Format: {fourcc_str}')
            
            # Тестовый захват кадра
            for i in range(5):  # Несколько попыток
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.get_logger().info(f'Test frame captured: {frame.shape[1]}x{frame.shape[0]}')
                    return True
                self.get_logger().warning(f'Test frame capture failed, attempt {i+1}/5')
                
            self.cap.release()
            return False
            
        except Exception as e:
            self.get_logger().error(f'Error initializing camera {camera_id}: {str(e)}')
            return False
    
    def timer_callback(self):
        """Захват и публикация кадра"""
        if not self.cap or not self.cap.isOpened():
            return
            
        try:
            ret, frame = self.cap.read()
            
            if ret and frame is not None:
                current_time = self.get_clock().now().to_msg()
                
                # Публикация обычного изображения
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    img_msg.header.stamp = current_time
                    img_msg.header.frame_id = "camera_frame"
                    self.image_pub.publish(img_msg)
                except Exception as e:
                    self.get_logger().error(f'Error converting image: {str(e)}')
                
                # Публикация сжатого изображения
                if self.compressed_pub:
                    try:
                        compressed_msg = CompressedImage()
                        compressed_msg.header.stamp = current_time
                        compressed_msg.header.frame_id = "camera_frame"
                        compressed_msg.format = "jpeg"
                        
                        # Кодируем в JPEG
                        ret, jpeg_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if ret:
                            compressed_msg.data = jpeg_data.tobytes()
                            self.compressed_pub.publish(compressed_msg)
                    except Exception as e:
                        self.get_logger().error(f'Error creating compressed image: {str(e)}')
                
                # Логируем каждые 100 кадров чтобы не засорять консоль
                if hasattr(self, 'frame_count'):
                    self.frame_count += 1
                else:
                    self.frame_count = 0
                    
                if self.frame_count % 100 == 0:
                    self.get_logger().info(f'Published {self.frame_count} frames')
                    
            else:
                self.get_logger().warning('Failed to capture frame from camera', throttle_duration_sec=5.0)
                
        except Exception as e:
            self.get_logger().error(f'Camera error: {str(e)}')
    
    def destroy_node(self):
        """Корректное завершение работы"""
        if self.cap:
            self.cap.release()
        self.get_logger().info('Camera node shutdown')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = USBCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
