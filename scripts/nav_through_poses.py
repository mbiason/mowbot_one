#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # set goal poses
    goal_poses = []
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -3.40776
    goal_pose.pose.position.y = -7.3018
    goal_pose.pose.orientation.z = -0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -3.79028
    goal_pose.pose.position.y = -17.1199
    goal_pose.pose.orientation.z = -0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -4.1548
    goal_pose.pose.position.y = -23.3288
    goal_pose.pose.orientation.z = -0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -5.01135
    goal_pose.pose.position.y = -34.4459
    goal_pose.pose.orientation.z = -0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)

    ########

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -5.76135
    goal_pose.pose.position.y = -34.4459
    goal_pose.pose.orientation.z = 0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -4.9048
    goal_pose.pose.position.y = -24.000
    goal_pose.pose.orientation.z = 0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -4.54028
    goal_pose.pose.position.y = -19.1199
    goal_pose.pose.orientation.z = 0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -4.15776
    goal_pose.pose.position.y = -7.3018
    goal_pose.pose.orientation.z = 0.707
    goal_pose.pose.orientation.w = 0.707
    goal_poses.append(goal_pose)

    # sanity check a valid path exists
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()