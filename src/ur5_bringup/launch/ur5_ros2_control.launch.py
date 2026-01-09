#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_ros2_control.launch.py
# Summary: Launches UR5 ros2_control with robot_state_publisher and controllers.
"""Launch UR5 ros2_control with robot_state_publisher and controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable


def generate_launch_description():
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    ur_type = LaunchConfiguration("ur_type")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_path = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        "urdf",
        xacro_file
    ])

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            xacro_path, " ",
            "ur_type:=", ur_type, " ",
            "name:=", robot_name
        ]),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            PathJoinSubstitution([FindPackageShare("ur5_bringup"), "config", controllers_yaml]),
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("description_pkg", default_value="ur5_description"),
        DeclareLaunchArgument("xacro_file", default_value="ur5.urdf.xacro"),
        DeclareLaunchArgument("controllers_yaml", default_value="ur5_mock_controllers.yaml"),
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("robot_name", default_value="ur5"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        rsp,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner,
    ])
