from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beach_robot_esp32_bridge',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200,
                'timeout': 0.05,
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
