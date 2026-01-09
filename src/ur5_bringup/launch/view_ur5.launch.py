# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/view_ur5.launch.py
# Summary: Launches robot_state_publisher and joint_state_publisher for viewing UR5.
"""Launch robot_state_publisher and joint_state_publisher for UR5 viewing (ROS 2 Jazzy).

Fix:
- Generate robot_description once via xacro.
- Pass the same robot_description *parameter* to BOTH nodes.
- Force joint_state_publisher to NOT wait for /robot_description topic.
"""

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

    ur_type = "ur5"
    robot_description_cmd = Command([
        FindExecutable(name="xacro"), " ",
        xacro_path, " ",
        "ur_type:=", ur_type,
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_cmd, value_type=str)
    }

    common = {"use_sim_time": True}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, common],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                common,
                {"use_robot_description_topic": False},
            ],
        ),
    ])