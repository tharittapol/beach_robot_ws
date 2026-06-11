#!/usr/bin/env python3
"""Front-box obstacle detection from the ZED filtered point cloud.

`/zed/filtered_cloud` is already range- and height-filtered (0.25-6.0 m, z 0.05-1.5 m)
and decimated to ~10 Hz by `zed_nav2_cloud_filter`, and is published in the `base_link`
frame (x forward, y left, z up). This module looks only at a forward box in front of the
robot and reports whether something is close enough to stop for.

Two pieces:
  * ``evaluate_front_box(cloud, ...)`` — pure function doing the numpy math.
  * ``FrontBoxMonitor`` — wires a subscription on a given node and calls back on every
    cloud with ``(present, nearest_x)``; the coverage node uses this to drive its
    pause/resume state machine.

Run standalone for bring-up testing::

    ros2 run beach_robot_coverage_nav2 obstacle_detector
"""
from typing import Callable, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def extract_front_box_points(
    cloud: PointCloud2,
    *,
    min_forward_distance: float,
    stop_distance: float,
    box_width: float,
    min_z: float,
    max_z: float,
) -> np.ndarray:
    """Return XYZ points inside the straight rectangular front box.

    A point counts as an obstacle if it is ahead of the robot
    (``min_forward_distance <= x <= stop_distance``), inside the lateral half-width
    (``|y| <= box_width / 2``) and within the height band (``min_z <= z <= max_z``).
    The Y limit is constant at every X distance, so this is a straight rectangular box,
    not an angle-based cone.
    ``min_z`` sits above the cloud filter's 0.05 m floor to keep ground points from
    triggering a stop when the robot pitches on soft sand.
    """
    try:
        arr = point_cloud2.read_points_numpy(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True)
    except AttributeError:  # older sensor_msgs_py without read_points_numpy
        # read_points may yield tuples or a structured array; row-index works for both.
        arr = np.array(
            [(p[0], p[1], p[2]) for p in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)],
            dtype=np.float32,
        )

    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    arr = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
    xs, ys, zs = arr[:, 0], arr[:, 1], arr[:, 2]
    box_half_width = max(0.0, float(box_width)) / 2.0

    mask = (
        (xs >= min_forward_distance)
        & (xs <= stop_distance)
        & (np.abs(ys) <= box_half_width)
        & (zs >= min_z)
        & (zs <= max_z)
    )
    return arr[mask]


def evaluate_front_box(
    cloud: PointCloud2,
    *,
    min_forward_distance: float,
    stop_distance: float,
    box_width: float,
    min_z: float,
    max_z: float,
    min_points: int,
) -> Tuple[bool, Optional[float]]:
    """Return ``(obstacle_present, nearest_x)`` for the forward box."""
    points = extract_front_box_points(
        cloud,
        min_forward_distance=min_forward_distance,
        stop_distance=stop_distance,
        box_width=box_width,
        min_z=min_z,
        max_z=max_z,
    )
    count = len(points)
    if count >= min_points:
        return True, float(np.min(points[:, 0]))
    return False, None


class FrontBoxMonitor:
    """Subscribes to the filtered cloud and reports front-box obstacles to a callback."""

    def __init__(
        self,
        node: Node,
        on_update: Callable[[bool, Optional[float]], None],
        *,
        cloud_topic: str = '/zed/filtered_cloud',
        min_forward_distance: float = 0.25,
        stop_distance: float = 2.0,
        box_width: float = 1.6,
        min_z: float = 0.12,
        max_z: float = 1.5,
        min_points: int = 5,
        debug_cloud_topic: str = '/safety/obstacle_points',
        publish_debug_cloud: bool = True,
    ):
        self._node = node
        self._on_update = on_update
        self.min_forward_distance = max(0.0, float(min_forward_distance))
        self.stop_distance = max(self.min_forward_distance, float(stop_distance))
        self.box_width = max(0.0, float(box_width))
        self.min_z = float(min_z)
        self.max_z = max(self.min_z, float(max_z))
        self.min_points = max(1, int(min_points))

        self.present: bool = False
        self.nearest_x: Optional[float] = None
        self.point_count: int = 0
        self._debug_pub = (
            node.create_publisher(PointCloud2, debug_cloud_topic, qos_profile_sensor_data)
            if publish_debug_cloud else None
        )

        self._sub = node.create_subscription(
            PointCloud2, cloud_topic, self._cloud_cb, qos_profile_sensor_data)

    def _cloud_cb(self, msg: PointCloud2):
        try:
            points = extract_front_box_points(
                msg,
                min_forward_distance=self.min_forward_distance,
                stop_distance=self.stop_distance,
                box_width=self.box_width,
                min_z=self.min_z,
                max_z=self.max_z,
            )
        except Exception as exc:  # never let a malformed cloud kill the node
            self._node.get_logger().warn(f'Obstacle scan failed: {exc}')
            return

        self.point_count = len(points)
        present = self.point_count >= self.min_points
        nearest_x = float(np.min(points[:, 0])) if present else None
        if self._debug_pub is not None:
            debug_cloud = point_cloud2.create_cloud_xyz32(msg.header, points.tolist())
            self._debug_pub.publish(debug_cloud)

        self.present = present
        self.nearest_x = nearest_x
        self._on_update(present, nearest_x)


def main(args=None):
    """Standalone diagnostic: log front-box obstacle state from the live cloud."""
    rclpy.init(args=args)
    node = Node('obstacle_detector')
    node.declare_parameter('cloud_topic', '/zed/filtered_cloud')
    node.declare_parameter('min_forward_distance', 0.25)
    node.declare_parameter('stop_distance', 2.0)
    node.declare_parameter('box_width', 1.6)
    node.declare_parameter('min_z', 0.12)
    node.declare_parameter('max_z', 1.5)
    node.declare_parameter('min_points', 5)

    def _log(present: bool, nearest_x: Optional[float]):
        if present:
            node.get_logger().info(f'OBSTACLE  nearest_x={nearest_x:.2f} m')
        else:
            node.get_logger().info('clear')

    FrontBoxMonitor(
        node, _log,
        cloud_topic=str(node.get_parameter('cloud_topic').value),
        min_forward_distance=float(node.get_parameter('min_forward_distance').value),
        stop_distance=float(node.get_parameter('stop_distance').value),
        box_width=float(node.get_parameter('box_width').value),
        min_z=float(node.get_parameter('min_z').value),
        max_z=float(node.get_parameter('max_z').value),
        min_points=int(node.get_parameter('min_points').value),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
