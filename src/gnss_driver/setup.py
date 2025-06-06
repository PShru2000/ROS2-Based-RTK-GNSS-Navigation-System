from glob import glob
from setuptools import setup

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'gps_msgs', 'pyserial', 'utm'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='GPS driver for ROS2 that reads GPS data from a serial port and publishes it as a custom ROS2 message.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = gps_driver.driver:main',
        ],
    },
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),   
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/gps_driver', ['package.xml']),
    ],
)

