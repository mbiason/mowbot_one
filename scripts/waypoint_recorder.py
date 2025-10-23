#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import yaml
import os
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # TF listener for map->base_footprint
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.output_file = os.path.expanduser('~/waypoints.yaml')

        # Subscribe to joystick
        self.button_index = 0  # <-- change this to whichever button you want (0=A, 1=B, etc.)
        self.last_button_state = 0  # for edge detection
        self.sub_joy = self.create_subscription(Joy, 'joy_orig', self.joy_callback, 10)

        self.get_logger().info("Ready: press joystick button to record waypoints.")

    def joy_callback(self, msg: Joy):
        if self.button_index < len(msg.buttons):
            current = msg.buttons[self.button_index]
            if current == 1 and self.last_button_state == 0:
                # Rising edge: button just pressed
                self.record_pose()
            self.last_button_state = current

    def record_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', now, timeout=rclpy.duration.Duration(seconds=0.5)
            )

            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation

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

            self.get_logger().info(f"Recorded waypoint #{len(self.waypoints)}")

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def save_waypoints(self):
        # Load existing YAML if present
        existing = {}
        if os.path.exists(self.output_file):
            try:
                with open(self.output_file, 'r') as f:
                    existing = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().warn(f"Failed to load existing YAML: {e}")
                existing = {}

        # Replace only the waypoints
        existing['waypoints'] = self.waypoints

        # Save back
        with open(self.output_file, 'w') as f:
            yaml.safe_dump(existing, f, default_flow_style=False)

        self.get_logger().info(
            f"Saved {len(self.waypoints)} waypoints to {self.output_file} "
            f"(preserved other keys: {list(existing.keys())})"
        )


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
        rclpy.try_shutdown()  # safe shutdown


if __name__ == '__main__':
    main()
