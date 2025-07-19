#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock


def goal_poses():
    return [_convert_pose(pose) for pose in _calc_path()]


def _calc_path():
    list = [
        {
            "x": -3.4,
            "y": -7.3,
            "z": -0.707,
            "w": 0.707
        },
        {
            "x": -4.6,
            "y": -34.5,
            "z": -0.707,
            "w": 0.707
        }
    ]

    for i in range(2, 18):
        if i % 2 == 0:
            last = list[i-1]
            turn = {
                "x": last["x"] - 0.75,
                "y": last["y"],
                "z": 1.0,
                "w": 0.0
            }
            list.append(turn)
        else:
            back = {
                "x": list[i-3]["x"] - 0.75,
                "y": list[i-3]["y"],
                "z": -list[i-2]["z"],
                "w": list[i-2]["w"]
            }
            list.append(back)

    list.append({
                "x": list[1]["x"] + 0.75,
                "y": list[1]["y"],
                "z": 0.0,
                "w": 1.0
            })
    list.append({
                "x": list[0]["x"] + 0.75,
                "y": list[0]["y"],
                "z": 0.707,
                "w": 0.707
            })
    
    for i in range(20, 36):
        if i % 2 == 0:
            last = list[i-1]
            turn = {
                "x": last["x"] + 0.75,
                "y": last["y"],
                "z": 0.0,
                "w": 1.0
            }
            list.append(turn)
        else:
            back = {
                "x": list[i-3]["x"] + 0.75,
                "y": list[i-3]["y"],
                "z": -list[i-2]["z"],
                "w": list[i-2]["w"]
            }
            list.append(back)

    # around shed
    list[35]["y"] = list[35]["y"] - 5
    list[32]["y"] = list[32]["y"] - 5
    list[31]["y"] = list[31]["y"] - 5

    return list

def _convert_pose(pose):
    print(pose)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = ROSClock().now().to_msg()
    goal_pose.pose.position.x = pose["x"]
    goal_pose.pose.position.y = pose["y"]
    goal_pose.pose.orientation.z = pose["z"]
    goal_pose.pose.orientation.w = pose["w"]
    return goal_pose
