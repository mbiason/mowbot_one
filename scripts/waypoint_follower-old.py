#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import yaml
import os

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Path to your recorded waypoints
        self.waypoint_file = os.path.expanduser('~/waypoints.yaml')

        # Action client for FollowWaypoints
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.get_logger().info("Waiting for FollowWaypoints action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to FollowWaypoints server")

        # Load waypoints and send goal
        waypoints = self.load_waypoints()
        if not waypoints:
            self.get_logger().error("No waypoints loaded. Exiting.")
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info(f"Sending {len(waypoints)} waypoints...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def load_waypoints(self):
        """Load YAML waypoints into PoseStamped list"""
        try:
            with open(self.waypoint_file, 'r') as f:
                data = yaml.safe_load(f)

            poses = []
            for wp in data['waypoints']:
                pose = PoseStamped()
                pose.header.frame_id = "map"   # important: must match Nav2 global frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = wp['pose']['position']['x']
                pose.pose.position.y = wp['pose']['position']['y']
                pose.pose.position.z = wp['pose']['position']['z']
                pose.pose.orientation.x = wp['pose']['orientation']['x']
                pose.pose.orientation.y = wp['pose']['orientation']['y']
                pose.pose.orientation.z = wp['pose']['orientation']['z']
                pose.pose.orientation.w = wp['pose']['orientation']['w']
                poses.append(pose)

            return poses
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"FollowWaypoints finished with result: {result}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
