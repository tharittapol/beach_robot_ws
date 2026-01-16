from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Set these roughly first, then measure and tune ----
    # base_link -> imu_link
    imu_xyz = (0.00, 0.00, 0.20)   # meters
    imu_rpy = (0.0, 0.0, 0.0)      # radians

    # base_link -> gps_link (antenna position)
    gps_xyz = (0.30, 0.00, 0.50)   # meters
    gps_rpy = (0.0, 0.0, 0.0)

    # base_link -> gnss_heading_link
    # If UM982 heading is “robot yaw”, usually place at the main antenna.
    head_xyz = gps_xyz
    head_rpy = (0.0, 0.0, 0.0)

    # base_link -> zed_camera_link  (ZED Mini mounting)
    zed_xyz = (0.12, 0.00, 0.25)   # meters
    zed_rpy = (0.0, 0.0, 0.0)      # radians

    def static_tf(name, xyz, rpy, parent, child):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            output='screen',
            arguments=[
                str(xyz[0]), str(xyz[1]), str(xyz[2]),
                str(rpy[0]), str(rpy[1]), str(rpy[2]),
                parent, child
            ]
        )

    return LaunchDescription([
        static_tf('tf_base_to_imu', imu_xyz, imu_rpy, 'base_link', 'imu_link'),
        static_tf('tf_base_to_gps', gps_xyz, gps_rpy, 'base_link', 'gps_link'),
        static_tf('tf_base_to_gnss_heading', head_xyz, head_rpy, 'base_link', 'gnss_heading_link'),
        static_tf('tf_base_to_zed', zed_xyz, zed_rpy, 'base_link', 'zed_camera_link'),
    ])
