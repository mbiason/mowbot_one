#!/usr/bin/env python3

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
import tf2_ros
from std_msgs.msg import Float32


def yaw_from_quat(q: Quaternion) -> float:
    """Quaternion -> yaw (rad), roll=pitch ignored."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    """Yaw (rad) -> Quaternion with roll=pitch=0."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def apply_se2(x, y, yaw, px, py, pyaw):
    """Apply T=(x,y,yaw) to pose p=(px,py,pyaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    X = x + c * px - s * py
    Y = y + s * px + c * py
    YAW = (pyaw + yaw + math.pi) % (2 * math.pi) - math.pi
    return X, Y, YAW


class AlignedWaypointFollower(Node):
    def __init__(self):
        super().__init__('aligned_waypoint_follower')

        # ---- Parameters ----
        self.declare_parameter('waypoint_file', os.path.expanduser('~/waypoints.yaml'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('skip_if_within_m', 1.0)    # skip first wp if already within this radius

        self.yaml_path = self.get_parameter('waypoint_file').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        self.skip_if_within = float(self.get_parameter('skip_if_within_m').value)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Action client ----
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # ---- Yaw correction from reflector node ----
        self.yaw_correction = 0.0
        self.got_yaw_correction = True # change to False to enable
        self.sub_corr = self.create_subscription(Float32, '/reflector/yaw_correction', self._on_corr, 10)

        # ---- State machine ----
        self.stage = 0
        self.recorded = []
        self.poses = []
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info(f"Waypoints file: {self.yaml_path}")
        self.get_logger().info(f"Frames: map='{self.map_frame}', base='{self.base_frame}'")

    # ---------- Utils ----------
    def _quat_from_yaml(self, dct) -> Quaternion:
        q = Quaternion()
        q.x = float(dct['x']); q.y = float(dct['y'])
        q.z = float(dct['z']); q.w = float(dct['w'])
        return q

    def _distance_xy(self, a: PoseStamped, b: PoseStamped) -> float:
        return math.hypot(a.pose.position.x - b.pose.position.x,
                          a.pose.position.y - b.pose.position.y)

    # ---------- Reflector correction ----------
    def _on_corr(self, msg: Float32):
        if not self.got_yaw_correction:
            self.yaw_correction = msg.data
            self.got_yaw_correction = True
            self.get_logger().info(f"Received yaw correction: {math.degrees(self.yaw_correction):.2f}°")
            # unsubscribe after first message
            self.destroy_subscription(self.sub_corr)

    # ---------- Main loop ----------
    def tick(self):
        if self.stage == 0:
            if not self.client.wait_for_server(timeout_sec=0.1):
                self.get_logger().debug('Waiting for follow_waypoints action server...')
                return
            if not self.got_yaw_correction:
                self.get_logger().debug('Waiting for yaw correction from /reflector/yaw_correction...')
                return
            self.stage = 1

        elif self.stage == 1:
            try:
                with open(self.yaml_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                self.recorded = data.get('waypoints', [])
                if not self.recorded:
                    self.get_logger().error('No waypoints in YAML')
                    self._shutdown()
                    return
                self.get_logger().info(f"Loaded {len(self.recorded)} recorded waypoint(s).")
                self.stage = 2
            except Exception as e:
                self.get_logger().error(f'Failed to load YAML: {e}')
                self._shutdown()

        elif self.stage == 2:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time(), timeout=self.tf_timeout
                )
            except Exception as e:
                self.get_logger().warn(f'Waiting for {self.map_frame}->{self.base_frame} TF: {e}')
                return

            Pn_x = trans.transform.translation.x
            Pn_y = trans.transform.translation.y
            Pn_yaw = yaw_from_quat(trans.transform.rotation)
            self.get_logger().info(f"Current pose: ({Pn_x:.2f}, {Pn_y:.2f}, {math.degrees(Pn_yaw):.1f}°)")

            wp0 = self.recorded[0]['pose']
            Pr_x = float(wp0['position']['x'])
            Pr_y = float(wp0['position']['y'])
            Pr_q = self._quat_from_yaml(wp0['orientation'])
            Pr_yaw = yaw_from_quat(Pr_q)

            c, s = math.cos(Pr_yaw), math.sin(Pr_yaw)
            inv_x = -(c * Pr_x + s * Pr_y)
            inv_y = -(-s * Pr_x + c * Pr_y)
            inv_yaw = -Pr_yaw
            T_x, T_y, T_yaw = apply_se2(Pn_x, Pn_y, Pn_yaw, inv_x, inv_y, inv_yaw)

            T_yaw += self.yaw_correction

            self.get_logger().info(f"Alignment: Δx={T_x:.2f}, Δy={T_y:.2f}, Δyaw={math.degrees(T_yaw):.1f}°")

            now = self.get_clock().now().to_msg()
            self.poses = []
            for wp in self.recorded:
                p = wp['pose']['position']
                q = self._quat_from_yaml(wp['pose']['orientation'])
                pyaw = yaw_from_quat(q)

                X, Y, YAW = apply_se2(T_x, T_y, T_yaw, float(p['x']), float(p['y']), pyaw)

                ps = PoseStamped()
                ps.header.frame_id = self.map_frame
                ps.header.stamp = now
                ps.pose.position.x = X
                ps.pose.position.y = Y
                ps.pose.position.z = 0.0
                ps.pose.orientation = quat_from_yaw(YAW)
                self.poses.append(ps)

            start_now = PoseStamped()
            start_now.header.frame_id = self.map_frame
            start_now.pose.position.x = Pn_x
            start_now.pose.position.y = Pn_y

            if self.poses and self._distance_xy(self.poses[0], start_now) <= self.skip_if_within:
                self.get_logger().info(f"Skipping first waypoint (within {self.skip_if_within:.2f} m).")
                self.poses = self.poses[1:]

            if not self.poses:
                self.get_logger().warn("No waypoints to send after skipping; exiting.")
                self._shutdown()
                return

            goal = FollowWaypoints.Goal()
            goal.poses = self.poses
            self.get_logger().info(f"Sending {len(self.poses)} aligned waypoint(s)...")
            fut = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
            fut.add_done_callback(self._on_goal_response)
            self.stage = 3

    # ---------- Action callbacks ----------
    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal rejected.')
            self._shutdown()
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        try:
            self.get_logger().info_throttle(2000, f"Feedback: current={fb.current_waypoint}, remaining={fb.remaining}")
        except Exception:
            pass

    def _on_result(self, future):
        result = future.result()
        self.get_logger().info(f'FollowWaypoints finished, result code={result.result}')
        self._shutdown()

    # ---------- Clean shutdown ----------
    def _shutdown(self):
        self.destroy_node()
        rclpy.try_shutdown()


def main():
    rclpy.init()
    node = AlignedWaypointFollower()
    rclpy.spin(node)
    # after spin exits, just ensure cleanup
    if rclpy.ok():
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
