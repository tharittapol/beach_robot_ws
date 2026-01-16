#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from tf_transformations import quaternion_from_euler


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

        self.declare_parameter('tool_width', 0.60)
        self.declare_parameter('overlap', 0.15)
        self.declare_parameter('boundary_margin', 0.30)
        self.declare_parameter('waypoint_step', 2.0)
        self.declare_parameter('turn_style', 'arc')  # arc|corner
        self.declare_parameter('turn_radius', 1.0)

        self.declare_parameter('action_name', 'follow_waypoints')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autostart', True)
        self.declare_parameter('start_delay_sec', 5.0)

        self.area = RectArea(
            origin_x=float(self.get_parameter('area.origin_x').value),
            origin_y=float(self.get_parameter('area.origin_y').value),
            width=float(self.get_parameter('area.width').value),
            height=float(self.get_parameter('area.height').value),
            yaw=float(self.get_parameter('area.yaw').value),
        )

        self.tool_width = float(self.get_parameter('tool_width').value)
        self.overlap = float(self.get_parameter('overlap').value)
        self.margin = float(self.get_parameter('boundary_margin').value)
        self.waypoint_step = float(self.get_parameter('waypoint_step').value)
        self.turn_style = str(self.get_parameter('turn_style').value).lower()
        self.turn_radius = float(self.get_parameter('turn_radius').value)

        self.action_name = str(self.get_parameter('action_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self._ac = ActionClient(self, FollowWaypoints, self.action_name)

        self._started = False
        if bool(self.get_parameter('autostart').value):
            delay = float(self.get_parameter('start_delay_sec').value)
            self.create_timer(delay, self._start_once)
        else:
            self._started = True

        spacing = self.tool_width * (1.0 - self.overlap)
        self.get_logger().info(
            f'Coverage planner: area={self.area.width}x{self.area.height}m, '
            f'tool={self.tool_width}m overlap={self.overlap} -> spacing={spacing:.3f}m'
        )

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self.get_logger().info('Waiting for Nav2 FollowWaypoints...')
        if not self._ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('FollowWaypoints action server not available.')
            return

        poses = self.generate_boustrophedon_poses()
        self.get_logger().info(f'Sending {len(poses)} waypoints...')
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
    def generate_boustrophedon_poses(self) -> List[PoseStamped]:
        spacing = self.tool_width * (1.0 - self.overlap)

        x0 = self.margin
        x1 = max(self.margin, self.area.width - self.margin)
        y0 = self.margin
        y1 = max(self.margin, self.area.height - self.margin)

        lane_ys = []
        y = y0
        while y <= y1 + 1e-6:
            lane_ys.append(y)
            y += spacing

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

    def _turn_arc(self, x_end, y_cur, y_next, forward):
        dy = y_next - y_cur
        r = min(max(0.3, self.turn_radius), abs(dy)/2.0) if abs(dy) > 1e-6 else 0.0
        if r <= 0.3:
            yaw = 0.0 if forward else math.pi
            return [(x_end, y_next, yaw)]

        cx = x_end
        cy = (y_cur + y_next) / 2.0
        thetas = [(-math.pi/2.0) + (i/10.0)*math.pi for i in range(11)]
        pts = []
        for th in thetas:
            lx = cx + r * math.cos(th)
            ly = cy + r * math.sin(th)
            yaw = th + math.pi/2.0
            if not forward:
                yaw += math.pi
            pts.append((lx, ly, yaw))
        return pts


def main():
    rclpy.init()
    node = CoverageFollowWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
