#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py
# Summary: Launches robot_state_publisher for UR5 using joint_states from bridge.
"""Launch robot_state_publisher for UR5 (use_sim_time)."""

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
    ur_type = LaunchConfiguration("ur_type")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_path = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        "urdf",
        xacro_file,
    ])

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            xacro_path, " ",
            "ur_type:=", ur_type, " ",
            "name:=", robot_name,
        ]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_description_content}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("description_pkg", default_value="ur5_description"),
        DeclareLaunchArgument("xacro_file", default_value="ur5.urdf.xacro"),
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("robot_name", default_value="ur5"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        rsp,
    ])
