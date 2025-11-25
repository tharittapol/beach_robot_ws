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
                'track_width': 0.6,
                'axis_linear': 1,
                'axis_angular': 0,
                'scale_linear': 1.0,
                'scale_angular': 1.0,

                # front tracks (If you feel that it drags harder than the rear wheel, reduce it, for example, by 0.9)
                'front_left_scale': 1.0,
                'front_right_scale': 1.0,

                # rear wheels (If smaller/different, may need to increase scale)
                'rear_left_scale': 1.0,
                'rear_right_scale': 1.0,
            }]
        )
    ])
