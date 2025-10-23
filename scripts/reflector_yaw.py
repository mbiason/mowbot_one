#!/usr/bin/env python3

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

def angle_wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def circular_mean(angles, weights=None):
    if not angles:
        return None
    s, c = 0.0, 0.0
    if weights is None:
        weights = [1.0] * len(angles)
    for a, w in zip(angles, weights):
        s += math.sin(a) * w
        c += math.cos(a) * w
    return math.atan2(s, c)

class ReflectorYaw(Node):
    def __init__(self):
        super().__init__('reflector_yaw')

        # Modes: calibration (record bearings) or playback (use bearings to correct yaw)
        self.declare_parameter('mode', 'calibration')
        self.declare_parameter('waypoint_file', os.path.expanduser('~/waypoints.yaml'))

        # Initial "search guesses" (not saved)
        self.declare_parameter('house_guess_deg', 180.0)   # house behind
        self.declare_parameter('shed_guess_deg', 90.0)     # shed left
        self.declare_parameter('window_deg', 10.0)

        self.mode = self.get_parameter('mode').value
        self.yaml_path = self.get_parameter('waypoint_file').get_parameter_value().string_value

        self.house_guess = math.radians(float(self.get_parameter('house_guess_deg').value))
        self.shed_guess = math.radians(float(self.get_parameter('shed_guess_deg').value))
        self.window = math.radians(float(self.get_parameter('window_deg').value))

        # Recorded values (loaded later in playback)
        self.house_recorded = self.house_guess
        self.shed_recorded = self.shed_guess

        if self.mode == 'playback':
            try:
                with open(self.yaml_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                self.house_recorded = math.radians(float(data.get('house_recorded_deg', 180.0)))
                self.shed_recorded = math.radians(float(data.get('shed_recorded_deg', 90.0)))
                self.get_logger().info(
                    f"Loaded recorded bearings: house={math.degrees(self.house_recorded):.1f}°, "
                    f"shed={math.degrees(self.shed_recorded):.1f}°"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to load waypoint YAML {self.yaml_path}: {e}")

        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.pub_corr = self.create_publisher(Float32, '/reflector/yaw_correction', 10)

        self.get_logger().info(f"ReflectorYaw node started in {self.mode} mode")

    def find_reflector(self, scan: LaserScan, target_angle: float):
        best_range = float('inf')
        best_angle = None
        angle = scan.angle_min
        for r in scan.ranges:
            if r < scan.range_min or r > scan.range_max:
                angle += scan.angle_increment
                continue
            a = angle_wrap(angle)
            da = abs(angle_wrap(a - target_angle))
            if da <= self.window and r < best_range:
                best_range = r
                best_angle = a
            angle += scan.angle_increment
        return best_angle

    def on_scan(self, scan: LaserScan):
        house_now = self.find_reflector(scan, self.house_guess)
        shed_now = self.find_reflector(scan, self.shed_guess)

        # --- Calibration mode ---
        if self.mode == 'calibration':
            if house_now is None or shed_now is None:
                self.get_logger().warn("Could not see both reflectors, retrying...")
                return

            house_deg = math.degrees(house_now)
            shed_deg = math.degrees(shed_now)
            self.get_logger().info(f"Calibration: house={house_deg:.1f}°, shed={shed_deg:.1f}°")

            # Load existing YAML or start new
            data = {}
            if os.path.exists(self.yaml_path):
                try:
                    with open(self.yaml_path, 'r') as f:
                        data = yaml.safe_load(f) or {}
                except Exception:
                    pass

            # Save bearings into YAML
            data['house_recorded_deg'] = house_deg
            data['shed_recorded_deg'] = shed_deg
            try:
                with open(self.yaml_path, 'w') as f:
                    yaml.safe_dump(data, f)
                self.get_logger().info(f"Saved calibration to {self.yaml_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save YAML: {e}")

            # Done
            self.destroy_node()
            rclpy.shutdown()
            return

        # --- Playback mode ---
        cors, weights = [], []

        if house_now is not None:
            yc = angle_wrap(house_now - self.house_recorded)
            cors.append(yc); weights.append(1.0)

        if shed_now is not None:
            yc = angle_wrap(shed_now - self.shed_recorded)
            cors.append(yc); weights.append(1.0)

        if not cors:
            return  # no correction available

        yaw_corr = circular_mean(cors, weights)

        # Check consistency if both seen
        if len(cors) == 2:
            diffs = [angle_wrap(c - yaw_corr) for c in cors]
            var = sum(d*d for d in diffs) / 2
            if var > (0.05**2):  # ~2.8°
                self.get_logger().warn("Landmark disagreement, skipping correction")
                self.destroy_node()
                rclpy.shutdown()
                return

        msg = Float32()
        msg.data = yaw_corr
        self.pub_corr.publish(msg)

        self.get_logger().info(f"Published yaw correction: {math.degrees(yaw_corr):.2f}°")
        self.get_logger().info("Playback done, shutting down.")
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ReflectorYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
