from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')
    keepout_mask_yaml = LaunchConfiguration('keepout_mask_yaml')
    start_coverage = LaunchConfiguration('start_coverage')
    start_delay_sec = LaunchConfiguration('start_delay_sec')

    pkg = get_package_share_directory('beach_robot_coverage_nav2')
    default_nav2_params = os.path.join(pkg, 'config', 'nav2_params_keepout.yaml')
    default_keepout_mask_yaml = os.path.join(pkg, 'config', 'keepout_mask.yaml')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params),
        DeclareLaunchArgument('keepout_mask_yaml', default_value=default_keepout_mask_yaml),
        DeclareLaunchArgument('start_coverage', default_value='true'),
        DeclareLaunchArgument('start_delay_sec', default_value='15.0'),
    ]

    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic_name': '/keepout_mask',
            'yaml_filename': keepout_mask_yaml,
        }],
    )

    keepout_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    nav2_pkg = get_package_share_directory('nav2_bringup')
    navigation_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': keepout_mask_yaml,
            'autostart': 'true',
        }.items()
    )

    coverage = Node(
        package='beach_robot_coverage_nav2',
        executable='coverage_follow_waypoints',
        name='coverage_follow_waypoints',
        output='screen',
        parameters=[{
            # --- area (rectangle) ---
            'area.origin_x': 0.0,
            'area.origin_y': 0.0,
            'area.width': 30.0,     # ด้านยาว (แนววิ่งขนานชายหาด)
            'area.height': 10.0,    # ด้านกว้าง (แนว shift lane)
            'area.yaw': 0.0,        # ถ้าแกน X ของ map ขนานชายหาดอยู่แล้ว

            # --- coverage ---
            'tool_width': 0.60,
            'overlap': 0.15,
            'boundary_margin': 0.30,
            'waypoint_step': 2.0,

            # --- turn ---
            'turn_style': 'arc',
            'turn_radius': 1.0,

            # --- run ---
            'autostart': start_coverage,
            'start_delay_sec': start_delay_sec,
        }],
    )

    return LaunchDescription(declare + [
        keepout_mask_server,
        keepout_info_server,
        nav2,
        coverage,
    ])
