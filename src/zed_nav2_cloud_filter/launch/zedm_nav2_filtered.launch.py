from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ZED wrapper launch file path
    zed_launch = (
        get_package_share_directory("zed_wrapper")
        + "/launch/zed_camera.launch.py"
    )

    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_model": "zedm",
            "camera_name": "zed",

            # IMPORTANT: avoid TF duplication with your static TF broadcaster
            "publish_tf": "false",

            # Disable positional tracking (no ZED odom/map)
            "pos_tracking.pos_tracking_enabled": "false",
            "pos_tracking.publish_tf": "false",

            # Optional (often used when disabling tracking)
            "depth.depth_stabilization": "0",
        }.items(),
    )

    cloud_filter_node = Node(
        package="zed_nav2_cloud_filter",
        executable="zed_cloud_filter_node",
        name="zed_cloud_filter_node",
        output="screen",
        parameters=[{
            # ZED wrapper typical registered cloud topic
            "input_cloud_topic": "/zed/zed_node/point_cloud/cloud_registered",

            # Filtered output cloud for Nav2 costmap
            "output_cloud_topic": "/zed/filtered_cloud",

            # Transform cloud into this frame before filtering
            "target_frame": "base_link",

            # Range gate (XY distance in target_frame)
            "min_range": 0.25,
            "max_range": 6.0,

            # Height gate (Z in target_frame) to remove floor/sky
            "min_z": 0.05,
            "max_z": 1.50,

            # Cheap downsample: keep 1 of every N points
            "decimate_n": 8,

            # Limit publish rate (0 = no throttle)
            "throttle_hz": 10.0,
        }],
    )

    return LaunchDescription([
        zed_node,
        cloud_filter_node,
    ])
