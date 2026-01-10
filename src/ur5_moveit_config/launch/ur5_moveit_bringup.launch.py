import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ws_dir = os.environ.get("WS_DIR", "")
    ws_src = Path(ws_dir) / "src" if ws_dir else None

    def _pkg_share(pkg_name: str) -> str:
        try:
            return get_package_share_directory(pkg_name)
        except PackageNotFoundError:
            if ws_src:
                candidate = ws_src / pkg_name
                if candidate.is_dir():
                    return str(candidate)
            raise

    ur5_bringup_share = _pkg_share("ur5_bringup")
    ur5_description_share = _pkg_share("ur5_description")
    moveit_share = _pkg_share("ur5_moveit_config")

    srdf_path = Path(moveit_share) / "config" / "ur5.srdf"
    if not srdf_path.is_file() and ws_src:
        fallback = ws_src / "ur5_moveit_config" / "config" / "ur5.srdf"
        if fallback.is_file():
            srdf_path = fallback

    start_ros2_control = LaunchConfiguration("start_ros2_control")
    launch_rviz = LaunchConfiguration("launch_rviz")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur5_bringup_share, "launch", "ur5_ros2_control.launch.py"])
        ),
        condition=IfCondition(start_ros2_control),
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5_rg2", package_name="ur5_moveit_config")
        .robot_description(
            file_path=str(Path(ur5_description_share) / "urdf" / "ur5.urdf.xacro"),
            mappings={"ur_type": "ur5", "name": "ur5_rg2"},
        )
        .robot_description_semantic(file_path=str(srdf_path))
        .robot_description_kinematics(
            file_path=str(Path(moveit_share) / "config" / "kinematics.yaml")
        )
        .joint_limits(
            file_path=str(Path(moveit_share) / "config" / "joint_limits.yaml")
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(
            file_path=str(Path(moveit_share) / "config" / "moveit_controllers.yaml")
        )
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
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_ros2_control", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            bringup_launch,
            move_group_node,
            rviz_node,
        ]
    )
