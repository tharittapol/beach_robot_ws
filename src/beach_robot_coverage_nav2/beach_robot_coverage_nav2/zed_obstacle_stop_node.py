#!/usr/bin/env python3
"""Standalone ZED front-obstacle stop with a clear-time-delayed resume."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from .obstacle_detector import FrontConeMonitor


class ZedObstacleStopNode(Node):
    """Latch a hardware safety stop until the ZED front cone stays clear."""

    def __init__(self):
        super().__init__('zed_obstacle_stop')

        self.declare_parameter('cloud_topic', '/zed/filtered_cloud')
        self.declare_parameter('min_forward_distance', 0.25)
        self.declare_parameter('stop_distance', 2.0)
        self.declare_parameter('cone_half_width', 0.8)
        self.declare_parameter('min_z', 0.12)
        self.declare_parameter('max_z', 1.5)
        self.declare_parameter('min_points', 5)
        self.declare_parameter('clear_time_sec', 3.0)
        self.declare_parameter('cloud_timeout_sec', 1.0)
        self.declare_parameter('fail_safe_on_cloud_timeout', True)
        self.declare_parameter('state_publish_rate_hz', 10.0)
        self.declare_parameter('beep_period_sec', 1.0)
        self.declare_parameter('beep_duration_sec', 0.4)
        self.declare_parameter('safety_estop_topic', '/safety/e_stop')
        self.declare_parameter('obstacle_state_topic', '/safety/obstacle_stop')
        self.declare_parameter('obstacle_distance_topic', '/safety/obstacle_distance')
        self.declare_parameter('buzzer_topic', '/buzzer_duration')

        cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.clear_time_sec = max(0.0, float(self.get_parameter('clear_time_sec').value))
        self.cloud_timeout_sec = max(
            0.0, float(self.get_parameter('cloud_timeout_sec').value))
        self.fail_safe_on_cloud_timeout = self._as_bool(
            self.get_parameter('fail_safe_on_cloud_timeout').value)
        state_rate = max(1.0, float(self.get_parameter('state_publish_rate_hz').value))
        self.beep_period_sec = max(
            0.2, float(self.get_parameter('beep_period_sec').value))
        self.beep_duration_sec = max(
            0.0, float(self.get_parameter('beep_duration_sec').value))

        self._stop_active = False
        self._raw_obstacle = False
        self._nearest_x: Optional[float] = None
        self._last_cloud_time = None
        self._clear_start = None
        self._timeout_warned = False

        self._estop_pub = self.create_publisher(
            Bool, str(self.get_parameter('safety_estop_topic').value), 10)
        self._state_pub = self.create_publisher(
            Bool, str(self.get_parameter('obstacle_state_topic').value), 10)
        self._distance_pub = self.create_publisher(
            Float32, str(self.get_parameter('obstacle_distance_topic').value), 10)
        self._buzzer_pub = self.create_publisher(
            Float32, str(self.get_parameter('buzzer_topic').value), 10)

        self._monitor = FrontConeMonitor(
            self,
            self._on_scan,
            cloud_topic=cloud_topic,
            min_forward_distance=float(
                self.get_parameter('min_forward_distance').value),
            stop_distance=float(self.get_parameter('stop_distance').value),
            cone_half_width=float(self.get_parameter('cone_half_width').value),
            min_z=float(self.get_parameter('min_z').value),
            max_z=float(self.get_parameter('max_z').value),
            min_points=int(self.get_parameter('min_points').value),
        )
        self.create_timer(1.0 / state_rate, self._state_timer_cb)
        self.create_timer(self.beep_period_sec, self._beep_timer_cb)

        self.get_logger().info(
            f'ZED obstacle stop: cloud={cloud_topic} '
            f'x={self._monitor.min_forward_distance:.2f}..'
            f'{self._monitor.stop_distance:.2f}m '
            f'y=+/-{self._monitor.cone_half_width:.2f}m '
            f'z={self._monitor.min_z:.2f}..{self._monitor.max_z:.2f}m '
            f'min_points={self._monitor.min_points} '
            f'clear_for={self.clear_time_sec:.1f}s '
            f'fail_safe_timeout={self.fail_safe_on_cloud_timeout}')

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_scan(self, present: bool, nearest_x: Optional[float]):
        self._last_cloud_time = self._now_sec()
        self._timeout_warned = False
        self._raw_obstacle = present
        self._nearest_x = nearest_x
        if present:
            self._clear_start = None
            self._set_stop(True, nearest_x, 'obstacle')
        elif self._stop_active and self._clear_start is None:
            self._clear_start = self._last_cloud_time

    def _state_timer_cb(self):
        now = self._now_sec()
        cloud_timed_out = (
            self._last_cloud_time is None
            or (
                self.cloud_timeout_sec > 0.0
                and (now - self._last_cloud_time) > self.cloud_timeout_sec
            )
        )

        if cloud_timed_out:
            self._clear_start = None
            if self.fail_safe_on_cloud_timeout:
                self._set_stop(True, None, 'cloud timeout')
            elif not self._timeout_warned:
                self.get_logger().warn('No fresh ZED filtered cloud; obstacle stop is not fail-safe.')
                self._timeout_warned = True
        elif not self._raw_obstacle and self._stop_active:
            if self._clear_start is None:
                self._clear_start = now
            elif (now - self._clear_start) >= self.clear_time_sec:
                self._set_stop(False, None, 'path clear')

        self._publish_state()

    def _set_stop(self, active: bool, nearest_x: Optional[float], reason: str):
        if active == self._stop_active:
            return
        self._stop_active = active
        if active:
            dist = f'{nearest_x:.2f}m' if nearest_x is not None else 'unknown distance'
            self.get_logger().warn(f'SAFETY STOP: {reason}, {dist}')
        else:
            self.get_logger().info(
                f'RESUME: path clear continuously for {self.clear_time_sec:.1f}s')

    def _publish_state(self):
        estop_msg = Bool()
        estop_msg.data = self._stop_active
        self._estop_pub.publish(estop_msg)

        state_msg = Bool()
        state_msg.data = self._stop_active
        self._state_pub.publish(state_msg)

        distance_msg = Float32()
        distance_msg.data = (
            float(self._nearest_x) if self._nearest_x is not None else math.nan)
        self._distance_pub.publish(distance_msg)

    def _beep_timer_cb(self):
        if not self._stop_active or self.beep_duration_sec <= 0.0:
            return
        msg = Float32()
        msg.data = self.beep_duration_sec
        self._buzzer_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedObstacleStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Do not publish a release on shutdown. If the detector dies while stopped, the
        # bridge must keep the last safety stop until a healthy detector reports clear.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
