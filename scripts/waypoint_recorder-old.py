#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import yaml
import math
import os
import time
from geometry_msgs.msg import Pose

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.min_distance = 0.5  # meters between saved waypoints
        self.last_pose = None

        self.output_file = os.path.expanduser('~/waypoints.yaml')

        self.create_timer(0.2, self.record_pose)

    def record_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', now, rclpy.duration.Duration(seconds=0.5)
            )

            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation

            if self.last_pose is None or self.distance(pose, self.last_pose) > self.min_distance:
                self.waypoints.append({'pose': {
                    'position': {
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z,
                    },
                    'orientation': {
                        'x': pose.orientation.x,
                        'y': pose.orientation.y,
                        'z': pose.orientation.z,
                        'w': pose.orientation.w,
                    }
                }})
                self.last_pose = pose
                self.get_logger().info(f"Recorded waypoint #{len(self.waypoints)}")

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def distance(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return math.sqrt(dx*dx + dy*dy)

    def save_waypoints(self):
        with open(self.output_file, 'w') as f:
            yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False)
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {self.output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down, saving waypoints...")
        node.save_waypoints()
    finally:
        node.save_waypoints()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
