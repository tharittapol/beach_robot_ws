#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints, NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass
class RectArea:
    origin_x: float = 0.0
    origin_y: float = 0.0
    width: float = 20.0
    height: float = 15.0
    yaw: float = 0.0


class CoverageFollowWaypoints(Node):
    def __init__(self):
        super().__init__('coverage_follow_waypoints')

        self.declare_parameter('area.origin_x', 0.0)
        self.declare_parameter('area.origin_y', 0.0)
        self.declare_parameter('area.width', 20.0)
        self.declare_parameter('area.height', 15.0)
        self.declare_parameter('area.yaw', 0.0)

        self.declare_parameter('pattern', 'boustrophedon')  # boustrophedon|spiral
        self.declare_parameter('tool_width', 0.60)
        self.declare_parameter('overlap', 0.0)
        self.declare_parameter('lane_spacing', 0.0)  # <=0: tool_width*(1-overlap)
        self.declare_parameter('auto_widen_lanes_for_turn', False)
        self.declare_parameter('boundary_margin', 0.30)
        self.declare_parameter('waypoint_step', 1.0)
        self.declare_parameter('turn_style', 'arc')  # arc|corner
        self.declare_parameter('turn_radius', 0.30)

        self.declare_parameter('action_name', 'follow_waypoints')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('preview_path_topic', '/coverage/path')
        self.declare_parameter('autostart', True)
        self.declare_parameter('start_delay_sec', 5.0)

        self.area = RectArea(
            origin_x=float(self.get_parameter('area.origin_x').value),
            origin_y=float(self.get_parameter('area.origin_y').value),
            width=float(self.get_parameter('area.width').value),
            height=float(self.get_parameter('area.height').value),
            yaw=float(self.get_parameter('area.yaw').value),
        )

        self.pattern = str(self.get_parameter('pattern').value).lower()
        self.tool_width = max(0.01, float(self.get_parameter('tool_width').value))
        self.overlap = min(0.90, max(0.0, float(self.get_parameter('overlap').value)))
        self.lane_spacing_param = float(self.get_parameter('lane_spacing').value)
        self.auto_widen_lanes = self._as_bool(self.get_parameter('auto_widen_lanes_for_turn').value)
        self.margin = max(0.0, float(self.get_parameter('boundary_margin').value))
        self.waypoint_step = max(0.10, float(self.get_parameter('waypoint_step').value))
        self.turn_style = str(self.get_parameter('turn_style').value).lower()
        self.turn_radius = max(0.0, float(self.get_parameter('turn_radius').value))
        self.lane_spacing = self._effective_lane_spacing()

        self.action_name = str(self.get_parameter('action_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.preview_path_topic = str(self.get_parameter('preview_path_topic').value)

        # Pre-compute geometry (used by both preview and nav)
        self._x0 = self.margin
        self._x1 = max(self.margin, self.area.width - self.margin)
        _y0 = self.margin
        _y1 = max(self.margin, self.area.height - self.margin)
        self._lane_ys: List[float] = self._lane_values(_y0, _y1, self.lane_spacing)

        # Action clients
        self._follow_ac = ActionClient(self, FollowWaypoints, self.action_name)
        self._nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._through_ac = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # Robot pose from /odometry/local (map≡odom for local coverage)
        self._odom: Optional[Odometry] = None
        self.create_subscription(Odometry, '/odometry/local', self._odom_cb, 10)

        # Preview publishers
        preview_qos = QoSProfile(depth=1)
        preview_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.preview_path_topic, preview_qos)
        self.path_pub_viz = self.create_publisher(Path, self.preview_path_topic + '_viz', 10)
        self._preview_poses: Optional[List[PoseStamped]] = None

        # Navigation state
        self._lane_idx = 0
        self._forward = True
        self._current_y: float = self._lane_ys[0] if self._lane_ys else 0.0
        self._started = False
        self._turn_y_target: float = 0.0

        if self._as_bool(self.get_parameter('autostart').value):
            delay = float(self.get_parameter('start_delay_sec').value)
            self.create_timer(delay, self._start_once)
        else:
            self._started = True
            self.publish_preview()

        # Republish viz path every 3 s for late-joining RViz2
        self.create_timer(3.0, self._republish_viz)

        self.get_logger().info(
            f'Coverage planner: pattern={self.pattern} '
            f'area={self.area.width:.2f}×{self.area.height:.2f}m '
            f'lanes={len(self._lane_ys)} spacing={self.lane_spacing:.2f}m '
            f'turn_radius={self.turn_radius:.2f}m'
        )

    # ---- helpers ----

    def _as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def _effective_lane_spacing(self) -> float:
        spacing = self.lane_spacing_param
        if spacing <= 0.0:
            spacing = self.tool_width * (1.0 - self.overlap)
        spacing = max(0.05, spacing)
        min_arc = 2.0 * self.turn_radius
        if self.turn_style == 'arc' and self.turn_radius > 0.0:
            if self.auto_widen_lanes and spacing < min_arc:
                self.get_logger().warn(
                    f'Widening lane_spacing {spacing:.2f}→{min_arc:.2f}m (2×turn_radius).')
                spacing = min_arc
            elif spacing < min_arc:
                self.get_logger().warn(
                    f'lane_spacing={spacing:.2f}m < 2×turn_radius={min_arc:.2f}m. '
                    'Turns may become skid turns. Add auto_widen_lanes_for_turn:=true if needed.')
        return spacing

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    # ---- geometry ----

    def _rot(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        c = math.cos(yaw); s = math.sin(yaw)
        return (c*x - s*y, s*x + c*y)

    def _to_map(self, lx: float, ly: float) -> Tuple[float, float]:
        rx, ry = self._rot(lx, ly, self.area.yaw)
        return (self.area.origin_x + rx, self.area.origin_y + ry)

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        q = quaternion_from_yaw(yaw)
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return ps

    def _sample_line(self, p0, p1, step):
        x0, y0 = p0; x1, y1 = p1
        dx, dy = x1-x0, y1-y0
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return [(x0, y0)]
        n = max(1, int(L / max(step, 0.1)))
        pts = [(x0 + dx*(k/n), y0 + dy*(k/n)) for k in range(n)]
        pts.append((x1, y1))
        return pts

    def _lane_values(self, start, stop, spacing):
        vals = []
        y = start
        while y <= stop + 1e-6:
            vals.append(y)
            y += spacing
        if vals and (stop - vals[-1]) > (0.5 * spacing):
            vals.append(stop)
        if not vals:
            vals.append(start)
        return vals

    def _append_line_points(self, out, p0, p1, yaw):
        for idx, (lx, ly) in enumerate(self._sample_line(p0, p1, self.waypoint_step)):
            if out and idx == 0 and self._same_xy(out[-1], (lx, ly)):
                continue
            out.append((lx, ly, yaw))

    def _same_xy(self, point, xy, eps=1e-6):
        return abs(point[0] - xy[0]) < eps and abs(point[1] - xy[1]) < eps

    def _turn_arc(self, x_end, y_cur, y_next, forward):
        """Arc curve from lane end to next-lane start — alternates direction (S-shape)."""
        dy = y_next - y_cur
        r = abs(dy) / 2.0 if abs(dy) > 1e-6 else 0.0
        if r <= 0.05:
            yaw = 0.0 if forward else math.pi
            return [(x_end, y_next, yaw)]

        # forward: arc bulges right (+x beyond x_end); backward: bulges left (-x)
        sign = 1.0 if forward else -1.0
        cy = (y_cur + y_next) / 2.0
        steps = max(8, int(math.ceil(math.pi * r / max(self.waypoint_step, 0.1))))
        theta0 = -math.pi / 2.0 if dy >= 0.0 else math.pi / 2.0
        theta1 = math.pi / 2.0 if dy >= 0.0 else -math.pi / 2.0
        pts = []
        for i in range(1, steps + 1):
            th = theta0 + (theta1 - theta0) * (i / steps)
            lx = x_end + sign * r * math.cos(th)
            ly = cy + r * math.sin(th)
            yaw = th + math.pi / 2.0 if forward else math.pi / 2.0 - th
            pts.append((lx, ly, yaw))
        return pts

    # ---- preview path (S-shape with arc curves) ----

    def generate_coverage_poses(self) -> List[PoseStamped]:
        if self.pattern in ('boustrophedon', 'lawnmower', 'zigzag'):
            return self._boustrophedon_preview(self._lane_ys)
        if self.pattern in ('spiral', 'inward_spiral', 'outer_to_inner'):
            return self.generate_spiral_poses()
        self.get_logger().warn(f'Unknown pattern={self.pattern!r}; using boustrophedon.')
        return self._boustrophedon_preview(self._lane_ys)

    def _boustrophedon_preview(self, lane_ys: List[float]) -> List[PoseStamped]:
        """S-shape path with arc curves — for visualization only."""
        x0, x1 = self._x0, self._x1
        poses: List[PoseStamped] = []
        forward = True
        for i, ly in enumerate(lane_ys):
            if forward:
                p0, p1 = (x0, ly), (x1, ly)
                yaw_lane = 0.0
            else:
                p0, p1 = (x1, ly), (x0, ly)
                yaw_lane = math.pi

            for lx, ly2 in self._sample_line(p0, p1, self.waypoint_step):
                mx, my = self._to_map(lx, ly2)
                poses.append(self._pose(mx, my, self.area.yaw + yaw_lane))

            # Arc curve into next lane (makes S-shape in RViz2)
            if i < len(lane_ys) - 1:
                x_end = p1[0]
                y_next = lane_ys[i + 1]
                for lx, lya, lyaw in self._turn_arc(x_end, ly, y_next, forward):
                    mx, my = self._to_map(lx, lya)
                    poses.append(self._pose(mx, my, self.area.yaw + lyaw))

            forward = not forward
        return poses

    def publish_preview(self, poses=None):
        if poses is None:
            poses = self.generate_coverage_poses()
        self._preview_poses = poses
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        self.path_pub.publish(path)
        self.path_pub_viz.publish(path)
        self.get_logger().info(f'Preview path published ({len(poses)} poses, S-shape with arcs)')

    def _republish_viz(self):
        if self._preview_poses is not None:
            path = Path()
            path.header.frame_id = self.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = self._preview_poses
            self.path_pub_viz.publish(path)

    # ---- navigation: boustrophedon state machine ----
    #
    # Flow per lane:
    #   _run_lane       →  FollowWaypoints (full lane including endpoint p1 = arc start)
    #   _on_lane_done   →  _run_turn (NavigateThroughPoses with arc waypoints)
    #   _on_turn_done   →  _after_turn (read actual pose, adjust next lane y)
    #   _run_lane       →  ...
    #
    # Arc turn: NavigateThroughPoses sends all arc waypoints from _turn_arc() at once.
    # Nav2 plans a continuous path through every arc point → robot traces the arc shape.
    # Arc radius = (y_target - y_cur) / 2, so it adapts when lane y drifts.
    # After _after_turn: next lane starts from ACTUAL robot y, not planned y.

    def _start_once(self):
        if self._started:
            return
        self._started = True

        self.publish_preview()

        if self.pattern not in ('boustrophedon', 'lawnmower', 'zigzag'):
            self._start_static()
            return

        self.get_logger().info('Waiting for Nav2 action servers...')
        if not self._follow_ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('FollowWaypoints action server not available.')
            return
        if not self._through_ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('NavigateThroughPoses action server not available.')
            return

        self._lane_idx = 0
        self._forward = True
        self._current_y = self._lane_ys[0]
        self.get_logger().info(f'Starting boustrophedon: {len(self._lane_ys)} lanes')
        self._run_lane()

    def _start_static(self):
        """Fallback for spiral / unknown patterns: send all waypoints at once."""
        self.get_logger().info('Waiting for Nav2 FollowWaypoints...')
        if not self._follow_ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('FollowWaypoints action server not available.')
            return
        poses = self.generate_coverage_poses()
        self.get_logger().info(f'Sending {len(poses)} {self.pattern} waypoints...')
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._follow_ac.send_goal_async(goal)

    def _run_lane(self):
        y = self._current_y

        if self._forward:
            p0, p1 = (self._x0, y), (self._x1, y)
            yaw_lane = 0.0
        else:
            p0, p1 = (self._x1, y), (self._x0, y)
            yaw_lane = math.pi

        pts = self._sample_line(p0, p1, self.waypoint_step)

        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + yaw_lane)
            for lx, ly in pts
        ]
        arrow = '→' if self._forward else '←'
        self.get_logger().info(
            f'Lane {self._lane_idx + 1}/{len(self._lane_ys)} '
            f'y={y:.3f} {arrow}  {len(poses)} waypoints')

        goal = FollowWaypoints.Goal()
        goal.poses = poses
        future = self._follow_ac.send_goal_async(goal)
        future.add_done_callback(self._on_lane_accepted)

    def _on_lane_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Lane goal rejected by FollowWaypoints server.')
            return
        handle.get_result_async().add_done_callback(self._on_lane_done)

    def _on_lane_done(self, future):
        self._lane_idx += 1
        if self._lane_idx >= len(self._lane_ys):
            self.get_logger().info('Coverage complete!')
            return
        self._run_turn()

    def _run_turn(self):
        """Navigate arc waypoints via NavigateThroughPoses — traces the actual arc shape.

        All arc points are sent at once so Nav2 plans a continuous path through
        every point without stopping.  Arc radius = (y_target - y_cur) / 2, so
        it automatically adapts when lane y drifts after drift correction.
        """
        y_cur = self._current_y
        y_target = self._lane_ys[self._lane_idx]
        x_end = self._x1 if self._forward else self._x0

        self._turn_y_target = y_target

        arc_pts = self._turn_arc(x_end, y_cur, y_target, self._forward)
        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + lyaw)
            for lx, ly, lyaw in arc_pts
        ]

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ''

        self.get_logger().info(
            f'Turn arc → lane {self._lane_idx + 1}: '
            f'{len(poses)} arc poses  y={y_cur:.3f}→{y_target:.3f}  '
            f'r={(abs(y_target - y_cur) / 2.0):.3f}m')

        future = self._through_ac.send_goal_async(goal)
        future.add_done_callback(self._on_turn_accepted)

    def _on_turn_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Turn arc goal rejected — adjusting next lane from actual pose.')
            self._after_turn()
            return
        handle.get_result_async().add_done_callback(self._on_turn_done)

    def _on_turn_done(self, future):
        self._after_turn()

    def _after_turn(self):
        """Read actual robot y and use it as the start of the next lane."""
        planned_y = self._turn_y_target
        if self._odom is not None:
            actual_y = self._odom.pose.pose.position.y
            self._current_y = actual_y
            delta = actual_y - planned_y
            self.get_logger().info(
                f'Turn done: planned_y={planned_y:.3f}  actual_y={actual_y:.3f}  '
                f'Δ={delta:+.3f}m  → next lane at actual y')
        else:
            self._current_y = planned_y
            self.get_logger().warn('No odometry — using planned y for next lane.')

        self._forward = not self._forward
        self._run_lane()

    # ---- spiral (static, not dynamically adjusted) ----

    def generate_spiral_poses(self) -> List[PoseStamped]:
        spacing = self.lane_spacing
        x0 = self._x0
        x1 = self._x1
        y0 = self.margin
        y1 = max(self.margin, self.area.height - self.margin)
        local_points = []
        while (x1 - x0) > 1e-6 and (y1 - y0) > 1e-6:
            self._append_line_points(local_points, (x0, y0), (x1, y0), 0.0)
            self._append_line_points(local_points, (x1, y0), (x1, y1), math.pi / 2.0)
            self._append_line_points(local_points, (x1, y1), (x0, y1), math.pi)
            next_y0 = y0 + spacing
            if next_y0 >= y1:
                break
            self._append_line_points(local_points, (x0, y1), (x0, next_y0), -math.pi / 2.0)
            x0 += spacing; y0 += spacing; x1 -= spacing; y1 -= spacing
            if x0 > x1 or y0 > y1:
                break
            self._append_line_points(local_points, (x0 - spacing, y0), (x0, y0), 0.0)
        poses = []
        for lx, ly, lyaw in local_points:
            mx, my = self._to_map(lx, ly)
            poses.append(self._pose(mx, my, self.area.yaw + lyaw))
        return poses


def main():
    rclpy.init()
    node = CoverageFollowWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
