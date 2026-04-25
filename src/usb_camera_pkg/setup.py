from setuptools import setup, find_packages

package_name = 'usb_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='USB Camera package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_camera_node = usb_camera_pkg.usb_camera_node:main',
            'camera_subscriber = usb_camera_pkg.camera_subscriber:main',
        ],
    },
)
