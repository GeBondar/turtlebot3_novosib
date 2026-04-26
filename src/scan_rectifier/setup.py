from setuptools import setup

package_name = 'scan_rectifier'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='LaserScan angular rectifier for calibrated RPLIDAR C1 scans.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'scan_rectifier_node = scan_rectifier.scan_rectifier_node:main',
        ],
    },
)
