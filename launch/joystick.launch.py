from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('mowbot_one'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/joy','/joy_orig')],
         )
    
    joy_remap_node = Node(
            package='mowbot_utils',
            executable='joy_remap_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/joy_in','/joy_orig'), ('/joy_out','/joy')],
    )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_drive_base_controller/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_drive_base_controller/cmd_vel')]
    #      )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        joy_remap_node,
        teleop_node,
        # twist_stamper       
    ])