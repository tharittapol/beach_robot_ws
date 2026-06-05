#!/usr/bin/env python3
"""Front-cone obstacle detection from the ZED filtered point cloud.

`/zed/filtered_cloud` is already range- and height-filtered (0.25-6.0 m, z 0.05-1.5 m)
and decimated to ~10 Hz by `zed_nav2_cloud_filter`, and is published in the `base_link`
frame (x forward, y left, z up). This module looks only at a forward cone in front of the
robot and reports whether something is close enough to stop for.

Two pieces:
  * ``evaluate_front_cone(cloud, ...)`` — pure function doing the numpy math.
  * ``FrontConeMonitor`` — wires a subscription on a given node and calls back on every
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


def evaluate_front_cone(
    cloud: PointCloud2,
    *,
    stop_distance: float,
    cone_half_width: float,
    min_z: float,
    max_z: float,
    min_points: int,
) -> Tuple[bool, Optional[float]]:
    """Return ``(obstacle_present, nearest_x)`` for the forward cone.

    A point counts as an obstacle if it is ahead of the robot
    (``0 < x <= stop_distance``), inside the lateral half-width
    (``|y| <= cone_half_width``) and within the height band (``min_z <= z <= max_z``).
    ``min_z`` sits above the cloud filter's 0.05 m floor to keep ground points from
    triggering a stop when the robot pitches on soft sand. Obstacle is declared only when
    at least ``min_points`` such points are present (noise rejection).
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
        return False, None
    arr = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
    xs, ys, zs = arr[:, 0], arr[:, 1], arr[:, 2]

    mask = (
        (xs > 0.0)
        & (xs <= stop_distance)
        & (np.abs(ys) <= cone_half_width)
        & (zs >= min_z)
        & (zs <= max_z)
    )
    count = int(np.count_nonzero(mask))
    if count >= min_points:
        return True, float(np.min(xs[mask]))
    return False, None


class FrontConeMonitor:
    """Subscribes to the filtered cloud and reports front-cone obstacles to a callback."""

    def __init__(
        self,
        node: Node,
        on_update: Callable[[bool, Optional[float]], None],
        *,
        cloud_topic: str = '/zed/filtered_cloud',
        stop_distance: float = 2.0,
        cone_half_width: float = 0.8,
        min_z: float = 0.12,
        max_z: float = 1.5,
        min_points: int = 5,
    ):
        self._node = node
        self._on_update = on_update
        self.stop_distance = float(stop_distance)
        self.cone_half_width = float(cone_half_width)
        self.min_z = float(min_z)
        self.max_z = float(max_z)
        self.min_points = int(min_points)

        self.present: bool = False
        self.nearest_x: Optional[float] = None

        self._sub = node.create_subscription(
            PointCloud2, cloud_topic, self._cloud_cb, qos_profile_sensor_data)

    def _cloud_cb(self, msg: PointCloud2):
        try:
            present, nearest_x = evaluate_front_cone(
                msg,
                stop_distance=self.stop_distance,
                cone_half_width=self.cone_half_width,
                min_z=self.min_z,
                max_z=self.max_z,
                min_points=self.min_points,
            )
        except Exception as exc:  # never let a malformed cloud kill the node
            self._node.get_logger().warn(f'Obstacle scan failed: {exc}')
            return
        self.present = present
        self.nearest_x = nearest_x
        self._on_update(present, nearest_x)


def main(args=None):
    """Standalone diagnostic: log front-cone obstacle state from the live cloud."""
    rclpy.init(args=args)
    node = Node('obstacle_detector')
    node.declare_parameter('cloud_topic', '/zed/filtered_cloud')
    node.declare_parameter('stop_distance', 2.0)
    node.declare_parameter('cone_half_width', 0.8)
    node.declare_parameter('min_z', 0.12)
    node.declare_parameter('max_z', 1.5)
    node.declare_parameter('min_points', 5)

    def _log(present: bool, nearest_x: Optional[float]):
        if present:
            node.get_logger().info(f'OBSTACLE  nearest_x={nearest_x:.2f} m')
        else:
            node.get_logger().info('clear')

    FrontConeMonitor(
        node, _log,
        cloud_topic=str(node.get_parameter('cloud_topic').value),
        stop_distance=float(node.get_parameter('stop_distance').value),
        cone_half_width=float(node.get_parameter('cone_half_width').value),
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
