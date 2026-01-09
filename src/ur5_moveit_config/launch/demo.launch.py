from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("ur5_moveit_config")
    ur5_description_share = get_package_share_directory("ur5_description")

    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                PathJoinSubstitution([ur5_description_share, "urdf", "ur5.urdf.xacro"]),
            ]
        ),
        value_type=str,
    )

    robot_description_semantic = ParameterValue(
        Command(
            [
                "/bin/cat",
                PathJoinSubstitution([pkg_share, "ur5.srdf"]),
            ]
        ),
        value_type=str,
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "robot_description_semantic": robot_description_semantic},
            PathJoinSubstitution([pkg_share, "config", "ompl_planning.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "kinematics.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "planning_scene_monitor_parameters.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "joint_limits.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "controllers.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "moveit_controllers.yaml"]),
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
