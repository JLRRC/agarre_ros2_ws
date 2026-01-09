from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("ur5_moveit_config")
    ur5_description_share = get_package_share_directory("ur5_description")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([ur5_description_share, "urdf", "ur5.urdf.xacro"]),
            ]
        ),
        value_type=str,
    )

    robot_description_semantic = ParameterValue(
        Command(
            [
                "/bin/cat",
                " ",
                PathJoinSubstitution([pkg_share, "config", "ur5.srdf"]),
            ]
        ),
        value_type=str,
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5_rg2", package_name="ur5_moveit_config")
        .robot_description(
            file_path=os.path.join(ur5_description_share, "urdf", "ur5.urdf.xacro"),
            mappings={"ur_type": "ur5", "name": "ur5_rg2"},
        )
        .robot_description_semantic(file_path=os.path.join(pkg_share, "config", "ur5.srdf"))
        .robot_description_kinematics(file_path=os.path.join(pkg_share, "config", "kinematics.yaml"))
        .joint_limits(file_path=os.path.join(pkg_share, "config", "joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path=os.path.join(pkg_share, "config", "moveit_controllers.yaml"))
        .to_moveit_configs()
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "robot_description_semantic": robot_description_semantic},
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])
