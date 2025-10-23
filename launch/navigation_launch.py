import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    my_robot_pkg_path = get_package_share_directory('mowbot_one')
    ekf_config_path = os.path.join(my_robot_pkg_path, 'config', 'ekf.yaml')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'params_file', default_value='/home/mbiason/ros2_ws/src/mowbot_one/config/nav2_params_rpp.yaml',
            description='Nav2 params file'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch'),
                '/navigation_launch.py']),
                launch_arguments={
                    'params_file': params_file
                }.items(),
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        Node(
            package='mowbot_utils',
            executable='feed_forward_mixer_node',
            output='screen',
        ),    
    ])