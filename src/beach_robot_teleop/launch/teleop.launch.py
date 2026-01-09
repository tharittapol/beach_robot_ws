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
                'device': '/dev/input/js0',
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
                'max_linear': 0.5,
                'max_angular': 1.0,
                'axis_linear': 1,
                'axis_angular': 0,
                'scale_linear': -1.0,
                'scale_angular': 1.0,
                'cmd_vel_out': '/cmd_vel',
            }]
        )
    ])
