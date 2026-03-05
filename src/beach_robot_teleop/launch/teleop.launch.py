from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # joy node (must install joy package first: sudo apt install ros-<distro>-joy)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device': 'js_joy',
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
                'max_linear': 1.0,
                'max_angular': 0.5,

                # your mapping (F710 left stick)
                'axis_linear': 1,     # left stick Y
                'axis_angular': 0,    # left stick X
                'scale_linear': 1.0,
                'scale_angular': 1.0,

                'invert_linear': False,       #
                'invert_angular': False,     # set True if turning direction feels reversed
                'deadzone_linear': 0.08,
                'deadzone_angular': 0.08,

                'cmd_vel_out': '/cmd_vel',
            }]
        )
    ])
