# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('rtabmap_launch'), 'launch'),
                '/rtabmap.launch.py']),
                launch_arguments={
                    'frame_id': 'base_footprint',
                    'stereo': 'true',
                    'subscribe_rgbd': 'true',
                    'subscribe_scan': 'true',
                    'rgbd_sync': 'true',
                    'approx_rgbd_sync': 'false',
                    'approx_sync': 'true',
                    'wait_imu_to_init': 'true',
                    'sync_queue_size': '200',
                    'use_action_for_goal': 'true',
                    'rtabmap_viz': 'false',
                    'rviz': 'false',
                    'map_topic': '/map',

                    'left_image_topic': '/camera/infra1/image_rect_raw',
                    'right_image_topic': '/camera/infra2/image_rect_raw',
                    'left_camera_info_topic': '/camera/infra1/camera_info',
                    'right_camera_info_topic': '/camera/infra2/camera_info',
                    'rgb_topic_relay': '/camera/color/image_raw',
                    'depth_topic_relay': '/camera/depth/image_rect_raw',
                    'camera_info_topic': '/camera/color/camera_info',

                    'Rtabmap/DetectionRate': '10',
                    'Grid/MaxObstacleHeight': '1',
                    'Grid/FromDepth': 'false',
                    'RGBD/NeighborLinkRefining': 'true',
                    'RGDB/ProximityBySpace': 'true',
                    'Icp/VoxelSize': '0.2',
                    'Icp/MaxCorrespondenceDistance': '0.4',
                    'Reg/Strategy': '1',
                }.items(),
        ),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[parameters],
        #     remappings=remappings),

        # Launch Rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch'),
                '/rviz_launch.py']),
                launch_arguments={}.items(),
        ),
    ])