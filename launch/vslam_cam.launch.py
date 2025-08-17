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

    parameters={
        'frame_id': 'base_footprint',
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'wait_imu_to_init': True,
        'queue_size': 200,
        'use_action_for_goal': True,
        'Grid/MaxObstacleHeight': '1',
        'Rtabmap/DetectionRate': '30',
    }

    remappings=[
        ('imu', '/imu/data'),
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info')
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),

        #Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={
                    'camera_namespace': '',
                    'enable_gyro': 'true',
                    'enable_accel': 'true',
                    'unite_imu_method': '2',
                    'enable_infra1': 'true',
                    'enable_infra2': 'true',
                    'enable_sync': 'true',
                    'json_file_path': '/home/mbiason/ros2_ws/src/mowbot_one/config/camera_high_accuracy.json',
                    }.items(),
        ),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory': 'False',
               'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

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

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ])