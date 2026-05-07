#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler
from rclpy.qos import DurabilityPolicy, QoSProfile


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
        self.declare_parameter('lane_spacing', 0.0)  # <=0: tool_width * (1-overlap)
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

        self._ac = ActionClient(self, FollowWaypoints, self.action_name)
        preview_qos = QoSProfile(depth=1)
        preview_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.preview_path_topic, preview_qos)

        self._started = False
        if self._as_bool(self.get_parameter('autostart').value):
            delay = float(self.get_parameter('start_delay_sec').value)
            self.create_timer(delay, self._start_once)
        else:
            self._started = True
            self.publish_preview()

        self.get_logger().info(
            f'Coverage planner: pattern={self.pattern} area={self.area.width:.2f}x{self.area.height:.2f}m, '
            f'tool={self.tool_width:.2f}m overlap={self.overlap:.2f} -> '
            f'lane_spacing={self.lane_spacing:.2f}m turn_radius={self.turn_radius:.2f}m'
        )

    def _as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def _effective_lane_spacing(self) -> float:
        spacing = self.lane_spacing_param
        if spacing <= 0.0:
            spacing = self.tool_width * (1.0 - self.overlap)
        spacing = max(0.05, spacing)

        min_arc_spacing = 2.0 * self.turn_radius
        if self.turn_style == 'arc' and self.turn_radius > 0.0:
            if self.auto_widen_lanes and spacing < min_arc_spacing:
                self.get_logger().warn(
                    f'Widening lane spacing from {spacing:.2f}m to {min_arc_spacing:.2f}m '
                    f'to fit turn_radius={self.turn_radius:.2f}m U-turns.'
                )
                spacing = min_arc_spacing
            elif spacing < min_arc_spacing:
                self.get_logger().warn(
                    f'lane_spacing={spacing:.2f}m is tighter than 2*turn_radius='
                    f'{min_arc_spacing:.2f}m. Turns may become skid/point turns. '
                    'Set auto_widen_lanes_for_turn:=true or increase lane_spacing if needed.'
                )
        return spacing

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self.get_logger().info('Waiting for Nav2 FollowWaypoints...')
        if not self._ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('FollowWaypoints action server not available.')
            return

        poses = self.generate_coverage_poses()
        self.publish_preview(poses)
        self.get_logger().info(f'Sending {len(poses)} {self.pattern} waypoints...')
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._ac.send_goal_async(goal)

    # ---- geometry helpers ----
    def _rot(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        c = math.cos(yaw); s = math.sin(yaw)
        return (c*x - s*y, s*x + c*y)

    def _to_map(self, lx: float, ly: float) -> Tuple[float, float]:
        rx, ry = self._rot(lx, ly, self.area.yaw)
        return (self.area.origin_x + rx, self.area.origin_y + ry)

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        q = quaternion_from_euler(0.0, 0.0, yaw)
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return ps

    # ---- coverage ----
    def generate_coverage_poses(self) -> List[PoseStamped]:
        if self.pattern in ('boustrophedon', 'lawnmower', 'zigzag'):
            return self.generate_boustrophedon_poses()
        if self.pattern in ('spiral', 'inward_spiral', 'outer_to_inner'):
            return self.generate_spiral_poses()
        self.get_logger().warn(f'Unknown pattern={self.pattern!r}; using boustrophedon.')
        return self.generate_boustrophedon_poses()

    def generate_boustrophedon_poses(self) -> List[PoseStamped]:
        spacing = self.lane_spacing

        x0 = self.margin
        x1 = max(self.margin, self.area.width - self.margin)
        y0 = self.margin
        y1 = max(self.margin, self.area.height - self.margin)

        lane_ys = self._lane_values(y0, y1, spacing)

        poses: List[PoseStamped] = []
        forward = True
        for i, ly in enumerate(lane_ys):
            if forward:
                p0, p1 = (x0, ly), (x1, ly)
                yaw_lane = 0.0
            else:
                p0, p1 = (x1, ly), (x0, ly)
                yaw_lane = math.pi

            # sample lane
            for lx, ly2 in self._sample_line(p0, p1, self.waypoint_step):
                mx, my = self._to_map(lx, ly2)
                poses.append(self._pose(mx, my, self.area.yaw + yaw_lane))

            # turn to next lane
            if i != len(lane_ys) - 1:
                next_y = lane_ys[i + 1]
                x_end = p1[0]
                if self.turn_style == 'arc':
                    turn_pts = self._turn_arc(x_end, ly, next_y, forward)
                else:
                    turn_pts = [(x_end, next_y, yaw_lane)]
                for lx, ly3, lyaw in turn_pts:
                    mx, my = self._to_map(lx, ly3)
                    poses.append(self._pose(mx, my, self.area.yaw + lyaw))

            forward = not forward

        return poses

    def generate_spiral_poses(self) -> List[PoseStamped]:
        spacing = self.lane_spacing
        x0 = self.margin
        x1 = max(self.margin, self.area.width - self.margin)
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

            x0 += spacing
            y0 += spacing
            x1 -= spacing
            y1 -= spacing
            if x0 > x1 or y0 > y1:
                break

            self._append_line_points(local_points, (x0 - spacing, y0), (x0, y0), 0.0)

        poses = []
        for lx, ly, lyaw in local_points:
            mx, my = self._to_map(lx, ly)
            poses.append(self._pose(mx, my, self.area.yaw + lyaw))
        return poses

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
        dy = y_next - y_cur
        r = abs(dy) / 2.0 if abs(dy) > 1e-6 else 0.0
        if r <= 0.05:
            yaw = 0.0 if forward else math.pi
            return [(x_end, y_next, yaw)]

        cx = x_end
        cy = (y_cur + y_next) / 2.0
        steps = max(8, int(math.ceil(math.pi * r / max(self.waypoint_step, 0.1))))
        if dy >= 0.0:
            theta0 = -math.pi / 2.0
            theta1 = math.pi / 2.0
        else:
            theta0 = math.pi / 2.0
            theta1 = -math.pi / 2.0
        pts = []
        for i in range(1, steps + 1):
            th = theta0 + (theta1 - theta0) * (i / steps)
            lx = cx + r * math.cos(th)
            ly = cy + r * math.sin(th)
            yaw = th + math.pi/2.0
            if not forward:
                yaw += math.pi
            pts.append((lx, ly, yaw))
        return pts

    def publish_preview(self, poses=None):
        if poses is None:
            poses = self.generate_coverage_poses()
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        self.path_pub.publish(path)
        self.get_logger().info(f'Published preview path on {self.preview_path_topic}: {len(poses)} poses')


def main():
    rclpy.init()
    node = CoverageFollowWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
