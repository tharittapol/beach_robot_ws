from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')
    keepout_mask_yaml = LaunchConfiguration('keepout_mask_yaml')
    use_robot_stack = LaunchConfiguration('use_robot_stack')
    use_zed = LaunchConfiguration('use_zed')
    use_gnss = LaunchConfiguration('use_gnss')
    esp32_port = LaunchConfiguration('esp32_port')
    wheel_cmd_send_rate_hz = LaunchConfiguration('wheel_cmd_send_rate_hz')
    publish_raw_json = LaunchConfiguration('publish_raw_json')
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    mixer_params_file = LaunchConfiguration('mixer_params_file')
    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf')
    start_coverage = LaunchConfiguration('start_coverage')
    start_delay_sec = LaunchConfiguration('start_delay_sec')
    coverage_pattern = LaunchConfiguration('coverage_pattern')
    area_origin_x = LaunchConfiguration('area_origin_x')
    area_origin_y = LaunchConfiguration('area_origin_y')
    area_width = LaunchConfiguration('area_width')
    area_height = LaunchConfiguration('area_height')
    area_yaw = LaunchConfiguration('area_yaw')
    tool_width = LaunchConfiguration('tool_width')
    overlap = LaunchConfiguration('overlap')
    lane_spacing = LaunchConfiguration('lane_spacing')
    auto_widen_lanes_for_turn = LaunchConfiguration('auto_widen_lanes_for_turn')
    boundary_margin = LaunchConfiguration('boundary_margin')
    waypoint_step = LaunchConfiguration('waypoint_step')
    turn_style = LaunchConfiguration('turn_style')
    turn_radius = LaunchConfiguration('turn_radius')

    pkg = get_package_share_directory('beach_robot_coverage_nav2')
    localization_pkg = get_package_share_directory('beach_robot_localization')
    mixer_pkg = get_package_share_directory('beach_wheel_mixer')
    default_nav2_params = os.path.join(pkg, 'config', 'nav2_params_keepout.yaml')
    default_keepout_mask_yaml = os.path.join(pkg, 'config', 'keepout_mask.yaml')
    localization_launch = os.path.join(localization_pkg, 'launch', 'localization_full_test.launch.py')
    default_mixer_params = os.path.join(mixer_pkg, 'config', 'mixer.yaml')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params),
        DeclareLaunchArgument('keepout_mask_yaml', default_value=default_keepout_mask_yaml),
        DeclareLaunchArgument(
            'use_robot_stack',
            default_value='true',
            description='Launch ESP32, mixer, sensors, and EKF so Nav2 has odom->base_link TF.',
        ),
        DeclareLaunchArgument('use_zed', default_value='true'),
        DeclareLaunchArgument('use_gnss', default_value='false'),
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyESP32'),
        DeclareLaunchArgument('wheel_cmd_send_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('publish_raw_json', default_value='false'),
        DeclareLaunchArgument('linear_scale', default_value='1.45'),
        DeclareLaunchArgument('angular_scale', default_value='1.0'),
        DeclareLaunchArgument('mixer_params_file', default_value=default_mixer_params),
        DeclareLaunchArgument(
            'publish_map_to_odom_tf',
            default_value='true',
            description='Publish identity map->odom for local coverage maps. Disable if a global localization source publishes map->odom.',
        ),
        DeclareLaunchArgument('start_coverage', default_value='true'),
        DeclareLaunchArgument('start_delay_sec', default_value='15.0'),
        DeclareLaunchArgument('coverage_pattern', default_value='boustrophedon'),
        DeclareLaunchArgument('area_origin_x', default_value='0.0'),
        DeclareLaunchArgument('area_origin_y', default_value='0.0'),
        DeclareLaunchArgument('area_width', default_value='30.0'),
        DeclareLaunchArgument('area_height', default_value='10.0'),
        DeclareLaunchArgument('area_yaw', default_value='0.0'),
        DeclareLaunchArgument('tool_width', default_value='0.60'),
        DeclareLaunchArgument('overlap', default_value='0.0'),
        DeclareLaunchArgument('lane_spacing', default_value='0.60'),
        DeclareLaunchArgument('auto_widen_lanes_for_turn', default_value='false'),
        DeclareLaunchArgument('boundary_margin', default_value='0.30'),
        DeclareLaunchArgument('waypoint_step', default_value='1.0'),
        DeclareLaunchArgument('turn_style', default_value='arc'),
        DeclareLaunchArgument('turn_radius', default_value='0.30'),
    ]

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_teleop': 'false',
            'use_zed': use_zed,
            'use_gnss': use_gnss,
            'esp32_port': esp32_port,
            'wheel_cmd_send_rate_hz': wheel_cmd_send_rate_hz,
            'publish_raw_json': publish_raw_json,
            'linear_scale': linear_scale,
            'angular_scale': angular_scale,
            'mixer_params_file': mixer_params_file,
        }.items(),
        condition=IfCondition(use_robot_stack),
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(publish_map_to_odom_tf),
    )

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
            'area.origin_x': area_origin_x,
            'area.origin_y': area_origin_y,
            'area.width': area_width,     # ด้านยาว (แนววิ่งขนานชายหาด)
            'area.height': area_height,   # ด้านกว้าง (แนว shift lane)
            'area.yaw': area_yaw,         # ถ้าแกน X ของ map ขนานชายหาดอยู่แล้ว

            # --- coverage ---
            'pattern': coverage_pattern,
            'tool_width': tool_width,
            'overlap': overlap,
            'lane_spacing': lane_spacing,
            'auto_widen_lanes_for_turn': auto_widen_lanes_for_turn,
            'boundary_margin': boundary_margin,
            'waypoint_step': waypoint_step,

            # --- turn ---
            'turn_style': turn_style,
            'turn_radius': turn_radius,

            # --- run ---
            'autostart': start_coverage,
            'start_delay_sec': start_delay_sec,
        }],
    )

    return LaunchDescription(declare + [
        robot_stack,
        map_to_odom_tf,
        keepout_mask_server,
        keepout_info_server,
        nav2,
        coverage,
    ])
