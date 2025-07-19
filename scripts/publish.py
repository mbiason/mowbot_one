#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
from calc_path import goal_poses


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self._clock = ROSClock()

        self.publisher_ = self.create_publisher(Path, 'local_plan', 10)
        self.publisher2_ = self.create_publisher(PoseArray, 'poses', 10)
        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        path = Path()
        path.header.stamp = self._clock.now().to_msg()
        path.header.frame_id = 'map'

        pose_array = PoseArray()
        pose_array.header.stamp = self._clock.now().to_msg()
        pose_array.header.frame_id = 'map'

        poses = goal_poses()
        path.poses = poses
        pose_array.poses = [i.pose for i in poses]

        self.publisher_.publish(path)
        self.publisher2_.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()