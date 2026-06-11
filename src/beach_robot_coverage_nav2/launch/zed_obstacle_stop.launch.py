import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    zed_filter_pkg = get_package_share_directory('zed_nav2_cloud_filter')

    launch_zed = LaunchConfiguration('launch_zed')
    use_static_tf = LaunchConfiguration('use_static_tf')
    zed_params_override = LaunchConfiguration('zed_params_override')
    cloud_topic = LaunchConfiguration('cloud_topic')
    stop_distance = LaunchConfiguration('stop_distance')
    cone_half_width = LaunchConfiguration('cone_half_width')
    clear_time_sec = LaunchConfiguration('clear_time_sec')
    fail_safe_on_cloud_timeout = LaunchConfiguration('fail_safe_on_cloud_timeout')

    zed_filter_launch = os.path.join(
        zed_filter_pkg, 'launch', 'zedm_nav2_filtered.launch.py')
    default_depth_params = os.path.join(
        zed_filter_pkg, 'config', 'zedm_orin_nano_depth.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_zed',
            default_value='true',
            description='Launch the ZED camera and filtered point-cloud stack. Set false if already running.',
        ),
        DeclareLaunchArgument(
            'use_static_tf',
            default_value='true',
            description='Publish base_link to ZED static TF when running standalone.',
        ),
        DeclareLaunchArgument('zed_params_override', default_value=default_depth_params),
        DeclareLaunchArgument('cloud_topic', default_value='/zed/filtered_cloud'),
        DeclareLaunchArgument('stop_distance', default_value='2.0'),
        DeclareLaunchArgument('cone_half_width', default_value='0.8'),
        DeclareLaunchArgument('clear_time_sec', default_value='3.0'),
        DeclareLaunchArgument('fail_safe_on_cloud_timeout', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_filter_launch),
            launch_arguments={
                'use_static_tf': use_static_tf,
                'target_frame': 'base_link',
                'zed_params_override': zed_params_override,
            }.items(),
            condition=IfCondition(launch_zed),
        ),

        Node(
            package='beach_robot_coverage_nav2',
            executable='zed_obstacle_stop',
            name='zed_obstacle_stop',
            output='screen',
            parameters=[{
                'cloud_topic': cloud_topic,
                'stop_distance': stop_distance,
                'cone_half_width': cone_half_width,
                'clear_time_sec': clear_time_sec,
                'fail_safe_on_cloud_timeout': fail_safe_on_cloud_timeout,
            }],
        ),
    ])
