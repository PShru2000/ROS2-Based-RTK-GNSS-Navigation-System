import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import glob

def detect_serial_port():
    """Automatically detects the first available serial port."""
    possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/pts/*')
    # Return the first available port or '/dev/ttyUSB0' if none are found
    return possible_ports[0] if possible_ports else '/dev/ttyUSB0'

def generate_launch_description():
    # Declare a launch argument to specify the GPS serial port
    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value=detect_serial_port(),  # Automatically detect or use default
        description='The serial port to use for the GPS device'
    )

    # Define the GPS driver node that will be launched
    gps_driver_node = Node(
        package='gps_driver',  # ROS package name
        executable='driver',   # The executable within the package
        name='gps_driver_node',  # The name of the node
        output='screen',  # Output log to the screen
        parameters=[{'port': LaunchConfiguration('port')}]  # Set the serial port parameter
    )

    # Return the launch description which includes the serial port argument and the node to launch
    return LaunchDescription([
        serial_port_arg,
        gps_driver_node
    ])

