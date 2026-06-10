#!/usr/bin/env python3
import math
from dataclasses import dataclass
from functools import partial
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import FollowWaypoints, NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from rclpy.qos import DurabilityPolicy, QoSProfile

from .obstacle_detector import FrontConeMonitor


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
        self.declare_parameter('boundary_margin', 0.0)   # 0 → first lane at area origin = robot spawn pose
        self.declare_parameter('waypoint_step', 1.0)
        self.declare_parameter('turn_style', 'arc')  # arc|corner
        self.declare_parameter('turn_radius', 0.30)

        self.declare_parameter('action_name', 'follow_waypoints')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('preview_path_topic', '/coverage/path')
        self.declare_parameter('autostart', True)
        self.declare_parameter('start_delay_sec', 5.0)

        # --- multipass coverage (interleaved passes for 100% coverage) ---
        # num_passes>1: lay fine lanes at lane_spacing/num_passes (= tool_width) and run
        # them as N interleaved passes; in-pass lanes stay lane_spacing apart (arc turns),
        # the offsets fill the gaps. Between passes the robot loops outside the work area.
        self.declare_parameter('num_passes', 1)
        self.declare_parameter('deadhead_style', 'outside')   # outside|direct
        self.declare_parameter('deadhead_clearance', 0.9)

        # --- auto-mode obstacle stop (ZED front cone) ---
        self.declare_parameter('obstacle_stop.enabled', True)
        self.declare_parameter('obstacle_stop.cloud_topic', '/zed/filtered_cloud')
        self.declare_parameter('obstacle_stop.stop_distance', 2.0)
        self.declare_parameter('obstacle_stop.cone_half_width', 0.8)
        self.declare_parameter('obstacle_stop.min_z', 0.12)
        self.declare_parameter('obstacle_stop.max_z', 1.5)
        self.declare_parameter('obstacle_stop.min_points', 5)
        self.declare_parameter('obstacle_stop.clear_time_sec', 3.0)
        self.declare_parameter('obstacle_stop.beep_period_sec', 1.0)
        self.declare_parameter('obstacle_stop.beep_duration_sec', 0.4)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('buzzer_topic', 'buzzer_duration')

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
        self.num_passes = max(1, int(self.get_parameter('num_passes').value))
        self.deadhead_style = str(self.get_parameter('deadhead_style').value).lower()
        self.deadhead_clearance = max(
            float(self.get_parameter('deadhead_clearance').value), self.turn_radius)
        self.lane_spacing = self._effective_lane_spacing()

        self.action_name = str(self.get_parameter('action_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.preview_path_topic = str(self.get_parameter('preview_path_topic').value)

        self.obstacle_enabled = self._as_bool(self.get_parameter('obstacle_stop.enabled').value)
        self.obstacle_cloud_topic = str(self.get_parameter('obstacle_stop.cloud_topic').value)
        self.obstacle_stop_distance = float(self.get_parameter('obstacle_stop.stop_distance').value)
        self.obstacle_cone_half_width = float(self.get_parameter('obstacle_stop.cone_half_width').value)
        self.obstacle_min_z = float(self.get_parameter('obstacle_stop.min_z').value)
        self.obstacle_max_z = float(self.get_parameter('obstacle_stop.max_z').value)
        self.obstacle_min_points = int(self.get_parameter('obstacle_stop.min_points').value)
        self.obstacle_clear_time_sec = float(self.get_parameter('obstacle_stop.clear_time_sec').value)
        self.beep_period_sec = max(0.2, float(self.get_parameter('obstacle_stop.beep_period_sec').value))
        self.beep_duration_sec = float(self.get_parameter('obstacle_stop.beep_duration_sec').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.buzzer_topic = str(self.get_parameter('buzzer_topic').value)

        # Pre-compute geometry (used by both preview and nav)
        self._x0 = self.margin
        self._x1 = max(self.margin, self.area.width - self.margin)
        self._y0 = self.margin
        self._y1 = max(self.margin, self.area.height - self.margin)
        # _lane_ys is the lane y's in execution order; _lane_pass[i] is which pass lane i
        # belongs to (a change in _lane_pass between consecutive lanes ⇒ a between-pass
        # deadhead instead of an in-pass arc turn).
        if self.num_passes > 1:
            fine = self._lane_values(self._y0, self._y1, self.lane_spacing / self.num_passes)
            self._lane_ys = []
            self._lane_pass = []
            for p in range(self.num_passes):
                for y in fine[p::self.num_passes]:
                    self._lane_ys.append(y)
                    self._lane_pass.append(p)
        else:
            self._lane_ys = self._lane_values(self._y0, self._y1, self.lane_spacing)
            self._lane_pass = [0] * len(self._lane_ys)

        # Guard: for gap-free, non-overlapping interleaved passes the fine spacing
        # (lane_spacing / num_passes) must equal the tool width.
        if self.num_passes > 1:
            fine_sp = self.lane_spacing / self.num_passes
            if abs(fine_sp - self.tool_width) > 0.1 * self.tool_width:
                self.get_logger().warn(
                    f'num_passes={self.num_passes}: fine spacing '
                    f'lane_spacing/num_passes={fine_sp:.2f}m ≠ tool_width={self.tool_width:.2f}m '
                    f'→ passes will overlap or leave gaps. For 100% no-overlap set '
                    f'lane_spacing = num_passes×tool_width = '
                    f'{self.num_passes * self.tool_width:.2f}m.')

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
        # Turn yaw monitoring (cancel arc goal when yaw reaches next-lane heading)
        self._in_turn: bool = False
        self._turn_triggered: bool = False
        self._turn_target_yaw: float = 0.0
        self._turn_start_time = None
        self._turn_goal_handle = None
        self._lane_goal_handle = None
        # Between-pass deadhead bookkeeping
        self._deadhead_next_y: float = 0.0
        self._deadhead_new_forward: bool = True

        # Obstacle-stop state.
        # _nav_epoch tags every goal we send; a result whose epoch != current is stale
        # (its goal was cancelled for an obstacle) and must not advance the state machine.
        self._phase = 'idle'            # 'idle' | 'lane' | 'turn'
        self._nav_epoch = 0
        self._paused = False
        self._obstacle_present = False
        self._clear_start = None

        # Obstacle-stop IO (cmd_vel override + buzzer + ZED front-cone monitor)
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._buzzer_pub = self.create_publisher(Float32, self.buzzer_topic, 10)
        if self.obstacle_enabled:
            self._monitor = FrontConeMonitor(
                self, self._on_obstacle_update,
                cloud_topic=self.obstacle_cloud_topic,
                stop_distance=self.obstacle_stop_distance,
                cone_half_width=self.obstacle_cone_half_width,
                min_z=self.obstacle_min_z,
                max_z=self.obstacle_max_z,
                min_points=self.obstacle_min_points,
            )
            # Always-on timers; they only act while _paused.
            self.create_timer(1.0 / 20.0, self._override_cmd_cb)
            self.create_timer(self.beep_period_sec, self._beep_cb)
            self.get_logger().info(
                f'Obstacle-stop enabled: stop<{self.obstacle_stop_distance:.1f}m '
                f'cone±{self.obstacle_cone_half_width:.1f}m resume after '
                f'{self.obstacle_clear_time_sec:.0f}s clear')

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
            f'passes={self.num_passes} lanes={len(self._lane_ys)} '
            f'in-pass spacing={self.lane_spacing:.2f}m '
            f'turn_radius={self.turn_radius:.2f}m '
            f'deadhead={self.deadhead_style}'
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

    # Yaw error within this threshold (rad) → arc done, start next lane immediately
    _TURN_YAW_THRESHOLD = 0.25   # ≈ 14° — triggers at ~95% of arc completion

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if self._in_turn and not self._turn_triggered and not self._paused:
            self._check_turn_yaw(msg)

    def _check_turn_yaw(self, msg: Odometry):
        if self._turn_start_time is None or self._paused:
            return
        elapsed = (self.get_clock().now() - self._turn_start_time).nanoseconds / 1e9
        if elapsed < 1.0:   # ignore first 1 s — let the arc begin
            return

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        err = abs(math.atan2(
            math.sin(yaw - self._turn_target_yaw),
            math.cos(yaw - self._turn_target_yaw),
        ))

        if err < self._TURN_YAW_THRESHOLD:
            self._turn_triggered = True
            self._in_turn = False
            self.get_logger().info(
                f'Arc done (yaw): yaw={math.degrees(yaw):.1f}°  '
                f'target={math.degrees(self._turn_target_yaw):.1f}°  '
                f'err={math.degrees(err):.1f}°  → starting next lane')
            if self._turn_goal_handle is not None:
                self._turn_goal_handle.cancel_goal_async()
                self._turn_goal_handle = None
            self._after_turn()

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

    def _round_corners(self, corners, r, step):
        """Replace each vertex of a polyline with a tangent arc (radius ≤ r) so the
        deadhead is a smooth curve, never a hard 90° corner. Radius is clamped to what the
        adjacent legs allow (end legs use their full length, shared legs half), so a tight
        inter-pass gap degrades to a smaller arc instead of a sharp corner. → [(x,y,yaw),...]."""
        pts = []
        n = len(corners)
        if n == 0:
            return pts
        if n <= 2:
            a, b = corners[0], corners[-1]
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return [(x, y, yaw) for (x, y) in self._sample_line(a, b, step)]

        start = corners[0]            # start of the current straight run (moves to each arc exit)
        for i in range(1, n - 1):
            v, nxt = corners[i], corners[i + 1]
            ix, iy = v[0] - start[0], v[1] - start[1]
            lin = math.hypot(ix, iy)
            ox, oy = nxt[0] - v[0], nxt[1] - v[1]
            lout = math.hypot(ox, oy)
            if lin < 1e-6 or lout < 1e-6:
                continue
            ix, iy = ix / lin, iy / lin
            ox, oy = ox / lout, oy / lout
            phi = math.atan2(ix * oy - iy * ox, ix * ox + iy * oy)   # signed heading change
            yaw_in = math.atan2(iy, ix)
            if abs(phi) < 1e-3:
                continue                                            # collinear → no corner
            tan_h = math.tan(abs(phi) / 2.0)
            out_avail = lout if (i + 1 == n - 1) else 0.5 * lout
            rr = min(r, lin / tan_h, out_avail / tan_h) if tan_h > 1e-6 else 0.0
            t = rr * tan_h
            if rr < 0.05 or t < 1e-3:                               # too tight to fillet
                for (x, y) in self._sample_line(start, v, step):
                    pts.append((x, y, yaw_in))
                start = v
                continue
            t_in = (v[0] - ix * t, v[1] - iy * t)
            t_out = (v[0] + ox * t, v[1] + oy * t)
            for (x, y) in self._sample_line(start, t_in, step):
                pts.append((x, y, yaw_in))
            sgn = 1.0 if phi > 0 else -1.0                          # +1 = left turn
            cx, cy = t_in[0] - iy * sgn * rr, t_in[1] + ix * sgn * rr
            a0 = math.atan2(t_in[1] - cy, t_in[0] - cx)
            asteps = max(3, int(math.ceil(rr * abs(phi) / max(step, 0.1))))
            for k in range(1, asteps + 1):
                ang = a0 + phi * (k / asteps)
                pts.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang),
                            ang + sgn * math.pi / 2.0))
            start = t_out
        last = corners[-1]
        yaw_f = math.atan2(last[1] - start[1], last[0] - start[0])
        for (x, y) in self._sample_line(start, last, step):
            pts.append((x, y, yaw_f))
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
        """Full route preview: lanes + in-pass arc turns + between-pass deadhead loops.

        Mirrors the navigation dispatch (same nearest-side rule), so /coverage/path[_viz]
        shows the true planned route. With num_passes==1 every transition is an arc, i.e.
        the original single-pass S-shape."""
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

            if i >= len(lane_ys) - 1:
                break

            end_x = p1[0]
            next_y = lane_ys[i + 1]
            same_pass = self._lane_pass[i + 1] == self._lane_pass[i]
            if same_pass:
                # in-pass arc turn (S-shape)
                for lx, lya, lyaw in self._turn_arc(end_x, ly, next_y, forward):
                    mx, my = self._to_map(lx, lya)
                    poses.append(self._pose(mx, my, self.area.yaw + lyaw))
                forward = not forward
            else:
                # between-pass deadhead: local teardrop loop on the SAME end → start nearest side
                new_forward = abs(end_x - self._x0) < abs(end_x - self._x1)
                start_x = self._x0 if new_forward else self._x1
                for lx, lya, lyaw in self._deadhead_path((end_x, ly), (start_x, next_y)):
                    mx, my = self._to_map(lx, lya)
                    poses.append(self._pose(mx, my, self.area.yaw + lyaw))
                forward = new_forward
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
    #   _run_lane       →  NavigateThroughPoses (continuous, full lane incl. endpoint p1 = arc start)
    #   _on_lane_done   →  _run_turn (NavigateThroughPoses with arc waypoints)
    #   _run_turn       →  starts _in_turn=True, monitors yaw via _check_turn_yaw
    #
    # Arc completes via whichever fires first:
    #   (a) _check_turn_yaw: yaw ≈ next-lane heading → cancel goal, call _after_turn
    #   (b) _on_turn_done: NavigateThroughPoses reaches arc endpoint normally
    #
    # _after_turn: reads actual robot y from /odometry/local → _current_y → _run_lane
    # Arc radius = (y_target - y_cur) / 2 (adapts when lane y drifts).

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

    def _run_lane(self, start_x=None):
        """Drive the current lane. ``start_x`` overrides the lane start (used on resume
        after an obstacle stop, to continue from the robot's current x rather than the
        lane's far end)."""
        self._phase = 'lane'
        y = self._current_y

        if self._forward:
            x_start = self._x0 if start_x is None else start_x
            p0, p1 = (x_start, y), (self._x1, y)
            yaw_lane = 0.0
        else:
            x_start = self._x1 if start_x is None else start_x
            p0, p1 = (x_start, y), (self._x0, y)
            yaw_lane = math.pi

        pts = self._sample_line(p0, p1, self.waypoint_step)

        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + yaw_lane)
            for lx, ly in pts
        ]
        arrow = '→' if self._forward else '←'
        resumed = '' if start_x is None else ' (resumed)'
        self.get_logger().info(
            f'Lane {self._lane_idx + 1}/{len(self._lane_ys)} '
            f'y={y:.3f} {arrow}  {len(poses)} waypoints{resumed}')

        self._nav_epoch += 1
        epoch = self._nav_epoch
        # NavigateThroughPoses follows the whole lane as one continuous path (does NOT stop /
        # decelerate at each waypoint the way FollowWaypoints does), while still tracking the
        # exact lane line — same mechanism as the arc turns / deadheads.
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        future = self._through_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_lane_accepted, epoch=epoch))

    def _on_lane_accepted(self, future, epoch):
        handle = future.result()
        if epoch != self._nav_epoch:
            # Paused/superseded before this goal was accepted: cancel the orphan so the
            # controller stops pursuing it. A dropped ClientGoalHandle is NOT auto-cancelled
            # — without this the robot would keep driving against the stop override.
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('Lane goal rejected by NavigateThroughPoses server.')
            return
        self._lane_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_lane_done, epoch=epoch))

    def _on_lane_done(self, future, epoch):
        if epoch != self._nav_epoch or self._paused:
            return  # stale result from a cancelled goal — do not advance
        self._lane_goal_handle = None
        self._lane_idx += 1
        if self._lane_idx >= len(self._lane_ys):
            self.get_logger().info('Coverage complete!')
            self._phase = 'idle'
            return
        # Same pass → in-pass arc S-turn; new pass → loop outside to the next pass.
        if self._lane_pass[self._lane_idx] == self._lane_pass[self._lane_idx - 1]:
            self._run_turn()
        else:
            self._run_deadhead()

    def _run_turn(self):
        """Navigate arc waypoints via NavigateThroughPoses.

        Arc completes via ONE of two paths (whichever fires first):
          (a) _check_turn_yaw: yaw reaches next-lane heading → cancel goal, start lane
          (b) _on_turn_done:  NavigateThroughPoses reaches arc endpoint normally
        """
        self._phase = 'turn'
        y_cur = self._current_y
        y_target = self._lane_ys[self._lane_idx]
        x_end = self._x1 if self._forward else self._x0

        self._turn_y_target = y_target
        self._turn_target_yaw = math.pi if self._forward else 0.0
        self._in_turn = True
        self._turn_triggered = False
        self._turn_start_time = self.get_clock().now()
        self._turn_goal_handle = None

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
            f'r={(abs(y_target - y_cur) / 2.0):.3f}m  '
            f'target_yaw={"π" if self._forward else "0"}')

        self._nav_epoch += 1
        epoch = self._nav_epoch
        future = self._through_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_turn_accepted, epoch=epoch))

    def _on_turn_accepted(self, future, epoch):
        handle = future.result()
        if epoch != self._nav_epoch:
            if handle.accepted:
                handle.cancel_goal_async()  # superseded before accept — cancel the orphan
            return
        if not handle.accepted:
            self.get_logger().warn('Turn arc goal rejected — adjusting next lane from actual pose.')
            self._in_turn = False
            self._after_turn()
            return
        self._turn_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_turn_done, epoch=epoch))

    def _on_turn_done(self, future, epoch):
        if epoch != self._nav_epoch or self._paused:
            return
        if not self._turn_triggered:
            # Normal goal completion — yaw check didn't fire first
            self._in_turn = False
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

    # ---- between-pass deadhead (multipass coverage) ----
    #
    # Between two passes the lanes are only tool_width apart, too tight for an in-pass arc,
    # so the robot repositions by looping OUTSIDE the work rectangle to the next pass's
    # first lane. Path = perimeter waypoints; RegulatedPurePursuit rounds the corners as
    # long as the clearance ≥ turn_radius. Uses NavigateThroughPoses (same as arc turns).

    def _sample_arc(self, c, p0, p1, rho, r, step, out):
        """Append points along the arc on circle (centre c, radius r) from p0 to p1, turning
        rho=+1 CCW / -1 CW. Each point is (x, y, yaw=tangent heading)."""
        a0 = math.atan2(p0[1] - c[1], p0[0] - c[0])
        a1 = math.atan2(p1[1] - c[1], p1[0] - c[0])
        sweep = (a1 - a0) % (2.0 * math.pi) if rho > 0 else -((a0 - a1) % (2.0 * math.pi))
        if abs(sweep) < 1e-9:
            return
        n = max(2, int(math.ceil(r * abs(sweep) / max(step, 0.1))))
        tang = math.pi / 2.0 if rho > 0 else -math.pi / 2.0
        for k in range(1, n + 1):
            ang = a0 + sweep * (k / n)
            out.append((c[0] + r * math.cos(ang), c[1] + r * math.sin(ang), ang + tang))

    def _teardrop(self, xe, ya, yb, s, r, step):
        """Forward-only minimum-radius loop (Dubins LRL/RLR) that repositions a lane end at
        (xe, ya) heading outward (s=+1 → +x, s=-1 → -x) to (xe, yb) heading inward, keeping
        every arc ≥ r. Used for the between-pass deadhead: it shifts the fine (< 2r) pass
        offset locally — bulges outward ≈ (√3+1)·r — instead of looping around the whole area.
        Falls back to a plain semicircle when |Δy| ≥ 2r (gap already wide enough)."""
        d_local = (yb - ya) if s > 0 else (ya - yb)

        def to_global(lx, ly, lyaw):
            return (xe + lx, ya + ly, lyaw) if s > 0 else (xe - lx, ya - ly, lyaw + math.pi)

        if abs(d_local) < 1e-6:
            return [(xe, yb, math.pi if s > 0 else 0.0)]
        loc = [(0.0, 0.0, 0.0)]
        if abs(d_local) >= 2.0 * r:                       # wide gap → semicircle (radius ≥ r)
            self._sample_arc((0.0, d_local / 2.0), (0.0, 0.0), (0.0, d_local),
                             1.0 if d_local > 0 else -1.0, abs(d_local) / 2.0, step, loc)
        else:                                             # tight gap → LRL/RLR teardrop
            sg = 1.0 if d_local > 0 else -1.0
            c1 = (0.0, sg * r)
            c2 = (0.0, d_local - sg * r)
            half = abs(c1[1] - c2[1]) / 2.0
            xm = math.sqrt(max(0.0, (2.0 * r) ** 2 - half * half))
            cm = (xm, d_local / 2.0)
            t1 = ((c1[0] + cm[0]) / 2.0, (c1[1] + cm[1]) / 2.0)
            t2 = ((cm[0] + c2[0]) / 2.0, (cm[1] + c2[1]) / 2.0)
            self._sample_arc(c1, (0.0, 0.0), t1, sg, r, step, loc)
            self._sample_arc(cm, t1, t2, -sg, r, step, loc)
            self._sample_arc(c2, t2, (0.0, d_local), sg, r, step, loc)
        return [to_global(*p) for p in loc]

    def _deadhead_path(self, a, b):
        """Lane-end a=(xa,ya) → next-lane-start b=(xb,yb). 'direct' = straight line; 'outside' =
        a local minimum-radius teardrop loop (every arc ≥ turn_radius). a and b share the end
        x-rail (same-side deadhead)."""
        xa, ya = a
        xb, yb = b
        if self.deadhead_style == 'direct':
            yaw = math.atan2(yb - ya, xb - xa)
            return [(lx, ly, yaw) for lx, ly in self._sample_line(a, b, self.waypoint_step)]
        xe = xa
        s = 1.0 if abs(xe - self._x1) < abs(xe - self._x0) else -1.0   # +1 outward = +x
        return self._teardrop(xe, ya, yb, s, self.turn_radius, self.waypoint_step)

    def _run_deadhead(self):
        self._phase = 'deadhead'
        end_x = self._x1 if self._forward else self._x0
        a = (end_x, self._current_y)
        next_y = self._lane_ys[self._lane_idx]
        # Same-side teardrop loop: start the next pass on the side nearest the current end,
        # so the local Dubins loop stays on this end x-rail (shifts the fine offset, ≥R arcs).
        new_forward = abs(end_x - self._x0) < abs(end_x - self._x1)
        start_x = self._x0 if new_forward else self._x1
        self._deadhead_next_y = next_y
        self._deadhead_new_forward = new_forward

        pts = self._deadhead_path(a, (start_x, next_y))
        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + lyaw)
            for lx, ly, lyaw in pts
        ]

        self.get_logger().info(
            f'Deadhead → pass {self._lane_pass[self._lane_idx] + 1} '
            f'lane {self._lane_idx + 1}/{len(self._lane_ys)}: {len(poses)} poses  '
            f'({a[0]:.2f},{a[1]:.2f})→({start_x:.2f},{next_y:.2f})  style={self.deadhead_style}')

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ''

        self._nav_epoch += 1
        epoch = self._nav_epoch
        future = self._through_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_deadhead_accepted, epoch=epoch))

    def _on_deadhead_accepted(self, future, epoch):
        handle = future.result()
        if epoch != self._nav_epoch:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().warn('Deadhead goal rejected — starting next pass from current pose.')
            self._after_deadhead()
            return
        self._turn_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_deadhead_done, epoch=epoch))

    def _on_deadhead_done(self, future, epoch):
        if epoch != self._nav_epoch or self._paused:
            return
        self._turn_goal_handle = None
        self._after_deadhead()

    def _after_deadhead(self):
        # Start the new pass at the exact planned lane y (keeps the interleave gap-free).
        self._current_y = self._deadhead_next_y
        self._forward = self._deadhead_new_forward
        self._run_lane()

    # ---- auto-mode obstacle stop ----
    #
    # The ZED front-cone monitor calls _on_obstacle_update on every cloud (~10 Hz).
    # We only pause during a LANE (not a turn): turns are short, and mid-arc the robot
    # is bulged out to the side so there is no clean "remaining arc" to resume — if an
    # obstacle persists through the turn, the pause engages the instant the next lane
    # starts. On pause we cancel the active Nav2 goal (so the progress checker can't
    # abort and skip a lane), zero /cmd_vel for a crisp stop, and beep. On 3 s of clear
    # we re-send the lane from the robot's current x.

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_obstacle_update(self, present: bool, nearest_x):
        self._obstacle_present = present
        if self._paused:
            if present:
                self._clear_start = None          # restart the clear timer
            else:
                now = self._now_sec()
                if self._clear_start is None:
                    self._clear_start = now
                elif (now - self._clear_start) >= self.obstacle_clear_time_sec:
                    self._resume_after_clear()
        else:
            if present and self._started and self._phase == 'lane':
                self._pause_for_obstacle(nearest_x)

    def _pause_for_obstacle(self, nearest_x):
        self._paused = True
        self._clear_start = None
        # Invalidate any in-flight goal's result callback, then cancel it.
        self._nav_epoch += 1
        if self._lane_goal_handle is not None:
            self._lane_goal_handle.cancel_goal_async()
            self._lane_goal_handle = None
        if self._turn_goal_handle is not None:
            self._turn_goal_handle.cancel_goal_async()
            self._turn_goal_handle = None
        self._in_turn = False
        dist = f'{nearest_x:.2f} m' if nearest_x is not None else 'unknown'
        self.get_logger().warn(f'Obstacle at {dist} → STOP + buzzer; pausing coverage.')

    def _resume_after_clear(self):
        self._paused = False
        self._clear_start = None
        self.get_logger().info(
            f'Path clear ≥ {self.obstacle_clear_time_sec:.0f}s → resuming lane.')
        start_x = None
        if self._odom is not None:
            start_x = self._odom.pose.pose.position.x
        self._run_lane(start_x=start_x)

    def _override_cmd_cb(self):
        # While paused, hold the robot stopped (also feeds the mixer's 0.3 s watchdog).
        if self._paused:
            self._cmd_pub.publish(Twist())

    def _beep_cb(self):
        if self._paused:
            msg = Float32()
            msg.data = float(self.beep_duration_sec)
            self._buzzer_pub.publish(msg)

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
