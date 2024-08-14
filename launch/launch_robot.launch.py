import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    package_name='mowbot_one'

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    twist_mux_config = PathJoinSubstitution([
        FindPackageShare(package_name), "config", "twist_mux.yaml"
    ])

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out','/diff_drive_base_controller/cmd_vel_unstamped')]
    )

    xacro_file_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'description', 'robot.urdf.xacro'
    ])

    robot_description = {
        "robot_description": Command(['xacro ', xacro_file_path, ' use_ros2_control:=', 'true', ' sim_mode:=', 'false']),
        "use_sim_time": False
    }

    robot_controllers_config = PathJoinSubstitution([
        FindPackageShare(package_name), "config", "robot_controllers.yaml"
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_config],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        remappings=[],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # Launch them all!
    return LaunchDescription([
        joystick,
        twist_mux_node,
        ros2_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])
