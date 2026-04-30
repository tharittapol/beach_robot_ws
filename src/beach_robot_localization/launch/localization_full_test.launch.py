import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(pkg_name, relative_path, launch_args=None, condition=None):
    launch_file = os.path.join(
        get_package_share_directory(pkg_name),
        relative_path,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=(launch_args or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    mixer_pkg = get_package_share_directory('beach_wheel_mixer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_teleop = LaunchConfiguration('use_teleop')
    use_mixer = LaunchConfiguration('use_mixer')
    use_zed = LaunchConfiguration('use_zed')

    esp32_port = LaunchConfiguration('esp32_port')
    esp32_baudrate = LaunchConfiguration('esp32_baudrate')
    esp32_timeout = LaunchConfiguration('esp32_timeout')
    enc_vel_max_abs_mps = LaunchConfiguration('enc_vel_max_abs_mps')
    enc_vel_max_step_mps = LaunchConfiguration('enc_vel_max_step_mps')
    wheel_cmd_send_rate_hz = LaunchConfiguration('wheel_cmd_send_rate_hz')

    joy_device = LaunchConfiguration('joy_device')
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')

    mixer_params_file = LaunchConfiguration('mixer_params_file')
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    wheel_scale_fl = LaunchConfiguration('wheel_scale_fl')
    wheel_scale_fr = LaunchConfiguration('wheel_scale_fr')
    wheel_scale_rl = LaunchConfiguration('wheel_scale_rl')
    wheel_scale_rr = LaunchConfiguration('wheel_scale_rr')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_esp32', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_mixer', default_value='true'),
        DeclareLaunchArgument('use_zed', default_value='true'),

        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyESP32'),
        DeclareLaunchArgument('esp32_baudrate', default_value='230400'),
        DeclareLaunchArgument('esp32_timeout', default_value='0.05'),
        DeclareLaunchArgument('enc_vel_max_abs_mps', default_value='3.0'),
        DeclareLaunchArgument('enc_vel_max_step_mps', default_value='1.0'),
        DeclareLaunchArgument(
            'wheel_cmd_send_rate_hz',
            default_value='30.0',
            description='Rate-limit wheel_cmd serial writes from the ESP32 bridge.',
        ),

        DeclareLaunchArgument('joy_device', default_value='/dev/input/js_joy'),
        DeclareLaunchArgument('max_linear', default_value='0.17'),
        DeclareLaunchArgument('max_angular', default_value='0.30'),

        DeclareLaunchArgument(
            'mixer_params_file',
            default_value=os.path.join(mixer_pkg, 'config', 'mixer.yaml'),
        ),
        DeclareLaunchArgument(
            'linear_scale',
            default_value='1.0',
            description='Wheel odometry linear scale. Start neutral for no-spin arc tests.',
        ),
        DeclareLaunchArgument(
            'angular_scale',
            default_value='1.0',
            description='Wheel odometry yaw scale. Start neutral for no-spin arc tests.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_fl',
            default_value='1.0',
            description='Per-wheel encoder scale applied before wheel odometry integration.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_fr',
            default_value='1.0',
            description='Per-wheel encoder scale applied before wheel odometry integration.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_rl',
            default_value='0.64',
            description='Rear-left encoder scale from chain-fixed straight response tests.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_rr',
            default_value='0.64',
            description='Rear-right encoder scale from chain-fixed straight response tests.',
        ),

        _include(
            'beach_robot_esp32_bridge',
            'launch/esp32_bridge.launch.py',
            launch_args={
                'port': esp32_port,
                'baudrate': esp32_baudrate,
                'timeout': esp32_timeout,
                'enc_vel_max_abs_mps': enc_vel_max_abs_mps,
                'enc_vel_max_step_mps': enc_vel_max_step_mps,
                'wheel_cmd_send_rate_hz': wheel_cmd_send_rate_hz,
            },
            condition=IfCondition(use_esp32),
        ),

        _include(
            'beach_robot_teleop',
            'launch/teleop.launch.py',
            launch_args={
                'joy_device': joy_device,
                'max_linear': max_linear,
                'max_angular': max_angular,
            },
            condition=IfCondition(use_teleop),
        ),

        _include(
            'beach_wheel_mixer',
            'launch/wheel_mps_mixer.launch.py',
            launch_args={
                'params_file': mixer_params_file,
                'use_sim_time': use_sim_time,
                'input_topic': '/cmd_vel',
                'output_topic': '/wheel_cmd',
            },
            condition=IfCondition(use_mixer),
        ),

        _include(
            'zed_nav2_cloud_filter',
            'launch/zedm_nav2_filtered.launch.py',
            launch_args={
                # localization_imu_compare publishes the static sensor TFs.
                'use_static_tf': 'false',
                'target_frame': 'base_link',
            },
            condition=IfCondition(use_zed),
        ),

        _include(
            'beach_robot_localization',
            'launch/localization_imu_compare.launch.py',
            launch_args={
                'use_sim_time': use_sim_time,
                'use_static_tf': 'true',
                'enc_vel_topic': '/enc_vel',
                'wheel_odom_topic': '/wheel/odom',
                'bno_imu_topic': '/imu/data',
                'zed_imu_topic': '/zed/zed_node/imu/data',
                'linear_scale': linear_scale,
                'angular_scale': angular_scale,
                'wheel_scale_fl': wheel_scale_fl,
                'wheel_scale_fr': wheel_scale_fr,
                'wheel_scale_rl': wheel_scale_rl,
                'wheel_scale_rr': wheel_scale_rr,
            },
        ),
    ])
