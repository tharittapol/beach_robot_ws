from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyESP32",
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
        DeclareLaunchArgument(
            "enc_vel_max_abs_mps",
            default_value="3.0",
            description="Drop encoder velocity samples with any wheel above this absolute speed. <=0 disables."
        ),
        DeclareLaunchArgument(
            "enc_vel_max_step_mps",
            default_value="1.0",
            description="Drop encoder velocity samples with any wheel jump above this speed. <=0 disables."
        ),
        DeclareLaunchArgument(
            "wheel_cmd_send_rate_hz",
            default_value="20.0",
            description="Rate-limit wheel_cmd serial writes to keep ESP32 telemetry/debug responsive."
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
                "enc_vel_max_abs_mps": LaunchConfiguration("enc_vel_max_abs_mps"),
                "enc_vel_max_step_mps": LaunchConfiguration("enc_vel_max_step_mps"),
                "wheel_cmd_send_rate_hz": LaunchConfiguration("wheel_cmd_send_rate_hz"),
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
