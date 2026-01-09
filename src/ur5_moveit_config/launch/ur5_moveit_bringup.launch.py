from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur5_bringup_share = get_package_share_directory("ur5_bringup")
    ur5_description_share = get_package_share_directory("ur5_description")
    moveit_share = get_package_share_directory("ur5_moveit_config")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur5_bringup_share, "launch", "ur5_ros2_control.launch.py"])
        ),
    )

    robot_description_cmd = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([ur5_description_share, "urdf", "ur5.urdf.xacro"]),
            " ",
            "ur_type:=ur5",
            " ",
            "name:=ur5_rg2",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_cmd, value_type=str)
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    "/bin/cat",
                    PathJoinSubstitution([moveit_share, "ur5.srdf"]),
                ]
            ),
            value_type=str,
        )
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            PathJoinSubstitution([moveit_share, "config", "ompl_planning.yaml"]),
            PathJoinSubstitution([moveit_share, "config", "kinematics.yaml"]),
            PathJoinSubstitution([moveit_share, "config", "planning_scene_monitor_parameters.yaml"]),
            PathJoinSubstitution([moveit_share, "config", "joint_limits.yaml"]),
            PathJoinSubstitution([moveit_share, "config", "controllers.yaml"]),
            PathJoinSubstitution([moveit_share, "config", "moveit_controllers.yaml"]),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[robot_description, robot_description_semantic],
    )

    return LaunchDescription([bringup_launch, move_group_node, rviz_node])
