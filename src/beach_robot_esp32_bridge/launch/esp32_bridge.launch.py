from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/esp32_beach",
            description="Serial port for ESP32 bridge"
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Baud rate"
        ),
        DeclareLaunchArgument(
            "timeout",
            default_value="0.05",
            description="Serial read timeout (seconds)"
        ),

        Node(
            package="beach_robot_esp32_bridge",
            executable="esp32_bridge",
            name="esp32_bridge",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "timeout": LaunchConfiguration("timeout"),
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])