# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/view_ur5.launch.py
# Summary: Launches robot_state_publisher and joint_state_publisher for viewing UR5.
"""Launch robot_state_publisher and joint_state_publisher for UR5 viewing."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        FindPackageShare("ur5_description"),
        "urdf",
        "ur5.urdf.xacro",
    ])

    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": robot_description_content,
            }],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),
    ])
