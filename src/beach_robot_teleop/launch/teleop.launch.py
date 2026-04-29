from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_device = LaunchConfiguration('joy_device')
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_device',
            default_value='/dev/input/js_joy',
            description='Joystick device path. Use /dev/input/js0 if no stable symlink is configured.',
        ),
        DeclareLaunchArgument(
            'max_linear',
            default_value='0.17',
            description='Maximum manual linear speed in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular',
            default_value='0.30',
            description='Maximum manual angular speed in rad/s.',
        ),

        # joy node (must install joy package first: sudo apt install ros-<distro>-joy)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device': joy_device,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # teleop node
        Node(
            package='beach_robot_teleop',
            executable='teleop_4wd',
            name='teleop_4wd',
            output='screen',
            parameters=[{
                'max_linear': max_linear,
                'max_angular': max_angular,

                # your mapping (F710 left stick)
                'axis_linear': 1,     # left stick Y
                'axis_angular': 0,    # left stick X
                'scale_linear': 1.0,
                'scale_angular': 1.0,

                'invert_linear': False,
                'invert_angular': False,     # set True if turning direction feels reversed
                'deadzone_linear': 0.08,
                'deadzone_angular': 0.08,
                'expo_linear': 1.0,
                'expo_angular': 1.0,
                'publish_rate_hz': 20.0,
                'joy_timeout_sec': 0.5,
                'linear_slew_rate': 0.35,
                'angular_slew_rate': 0.60,
                'linear_decel_rate': 3.0,
                'angular_decel_rate': 4.0,

                'cmd_vel_out': '/cmd_vel',
            }]
        )
    ])
