#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Summary: Official unified launch for Gazebo + bridge + ros2_control + panel.
"""Official unified launch for the UR5 stack (Gazebo, bridge, ros2_control, panel)."""

from __future__ import annotations

import os
import re
import time
import shutil
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _prepare_runtime(context, *_args) -> List[object]:
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    world_file = LaunchConfiguration("world_file").perform(context)
    log_dir = os.path.join(ws_dir, "log")
    os.makedirs(log_dir, exist_ok=True)

    world_name = "ur5_mesa_objetos"
    try:
        with open(world_file, "r", encoding="utf-8") as f:
            data = f.read()
        match = re.search(r"<world name=\"([^\"]+)\">", data)
        if match:
            world_name = match.group(1)
    except Exception:
        pass

    gz_partition = os.environ.get("GZ_PARTITION", "")
    if not gz_partition:
        gz_partition = f"ur5pro_{int(time.time())}"
    try:
        with open(os.path.join(log_dir, "gz_partition.txt"), "w", encoding="utf-8") as f:
            f.write(gz_partition)
    except Exception:
        pass

    base_yaml = os.path.join(ws_dir, "scripts", "bridge_cameras.yaml")
    runtime_yaml = os.path.join(log_dir, "bridge_runtime.yaml")
    try:
        with open(base_yaml, "r", encoding="utf-8") as f:
            yaml_text = f.read()
        if world_name:
            yaml_text = yaml_text.replace(
                "/world/ur5_mesa_objetos/",
                f"/world/{world_name}/",
            )
        with open(runtime_yaml, "w", encoding="utf-8") as f:
            f.write(yaml_text)
    except Exception:
        runtime_yaml = base_yaml

    runtime_models_root = os.path.join(log_dir, "gz_models")
    runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")
    src_ur5_model = os.path.join(ws_dir, "models", "ur5_rg2")
    try:
        if os.path.isdir(src_ur5_model):
            shutil.copytree(src_ur5_model, runtime_ur5_model, dirs_exist_ok=True)
    except Exception:
        pass

    resource_path = f"{runtime_models_root}:{ws_dir}/models:{ws_dir}/worlds:{ws_dir}/install"
    existing_resource = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource:
        resource_path = f"{resource_path}:{existing_resource}"
    plugin_path = "/opt/ros/jazzy/lib"
    existing_plugin = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    if existing_plugin:
        plugin_path = f"{plugin_path}:{existing_plugin}"
    fastdds_profile = os.path.join(ws_dir, "scripts", "fastdds_no_shm.xml")
    launch_flags = [
        LaunchConfiguration("launch_gazebo").perform(context),
        LaunchConfiguration("launch_rsp").perform(context),
        LaunchConfiguration("launch_bridge").perform(context),
        LaunchConfiguration("launch_ros2_control").perform(context),
        LaunchConfiguration("launch_moveit").perform(context),
    ]
    managed = any(str(flag).lower() in ("1", "true", "yes") for flag in launch_flags)
    managed_str = "1" if managed else "0"
    controllers_yaml = os.path.join(
        get_package_share_directory("ur5_description"),
        "config",
        "ur5_controllers.yaml",
    )
    try:
        model_sdf = os.path.join(runtime_ur5_model, "model.sdf")
        if os.path.isfile(model_sdf):
            with open(model_sdf, "r", encoding="utf-8") as f:
                sdf_text = f.read()
            plugin_re = re.compile(
                r'(<plugin filename="gz_ros2_control-system"[^>]*>)(.*?)(</plugin>)',
                re.DOTALL,
            )
            match = plugin_re.search(sdf_text)
            if match:
                header, body, footer = match.groups()
                if "<parameters>" in body:
                    body = re.sub(
                        r"<parameters>.*?</parameters>",
                        f"<parameters>{controllers_yaml}</parameters>",
                        body,
                        flags=re.DOTALL,
                    )
                else:
                    body = body + f"\n            <parameters>{controllers_yaml}</parameters>\n"
                sdf_text = sdf_text[: match.start()] + header + body + footer + sdf_text[match.end() :]
                with open(model_sdf, "w", encoding="utf-8") as f:
                    f.write(sdf_text)
    except Exception:
        pass

    runtime_world = world_file
    try:
        if os.path.isfile(world_file):
            with open(world_file, "r", encoding="utf-8") as f:
                world_text = f.read()
            world_text = world_text.replace(
                "<uri>model://ur5_rg2</uri>",
                f"<uri>file://{runtime_ur5_model}</uri>",
            )
            runtime_world = os.path.join(log_dir, "world_runtime.sdf")
            with open(runtime_world, "w", encoding="utf-8") as f:
                f.write(world_text)
    except Exception:
        runtime_world = world_file
    return [
        SetEnvironmentVariable("WS_DIR", ws_dir),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("UR5_CONTROLLERS_YAML", controllers_yaml),
        SetEnvironmentVariable("UR5_CONTROLLERS_FILE", controllers_yaml),
        SetEnvironmentVariable("PANEL_AUTO_BRIDGE", LaunchConfiguration("panel_auto_bridge")),
        SetEnvironmentVariable("PANEL_AUTO_BRIDGE_DELAY_MS", LaunchConfiguration("panel_auto_bridge_delay_ms")),
        SetEnvironmentVariable("PANEL_MANAGED", managed_str),
        SetEnvironmentVariable("PANEL_MOVEIT_REQUIRED", LaunchConfiguration("launch_moveit")),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")),
        SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
        SetLaunchConfiguration("runtime_yaml", runtime_yaml),
        SetLaunchConfiguration("controllers_file", controllers_yaml),
        SetLaunchConfiguration("world_file", runtime_world),
        SetLaunchConfiguration("world_name", world_name),
        SetLaunchConfiguration("panel_managed", managed_str),
    ]


def _maybe_moveit(context, *_args) -> List[object]:
    launch_moveit = LaunchConfiguration("launch_moveit").perform(context)
    if str(launch_moveit).lower() not in ("1", "true", "yes"):
        return []
    moveit_start_ros2_control = LaunchConfiguration("moveit_start_ros2_control")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ur5_moveit_config"), "launch", "ur5_moveit_bringup.launch.py"]
                )
            ),
            launch_arguments={
                "start_ros2_control": moveit_start_ros2_control,
                "launch_rviz": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    world_default = os.path.join(ws_dir, "worlds", "ur5_mesa_objetos.sdf")

    headless = LaunchConfiguration("headless")
    launch_panel = LaunchConfiguration("launch_panel")
    launch_bridge = LaunchConfiguration("launch_bridge")
    launch_gazebo = LaunchConfiguration("launch_gazebo")
    launch_rsp = LaunchConfiguration("launch_rsp")
    launch_ros2_control = LaunchConfiguration("launch_ros2_control")
    launch_world_tf = LaunchConfiguration("launch_world_tf")
    launch_release_service = LaunchConfiguration("launch_release_service")
    launch_system_state = LaunchConfiguration("launch_system_state")
    launch_moveit = LaunchConfiguration("launch_moveit")
    moveit_start_ros2_control = LaunchConfiguration("moveit_start_ros2_control")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur5_bringup"), "launch", "ur5_rsp.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(launch_rsp),
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur5_bringup"), "launch", "ur5_ros2_control.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "controllers_yaml": "ur5_mock_controllers.yaml",
        }.items(),
        condition=IfCondition(launch_ros2_control),
    )

    controller_bootstrap = Node(
        package="ur5_tools",
        executable="controller_bootstrap",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"controller_manager": "/controller_manager"},
            {"wait_for_clock": True},
        ],
        condition=IfCondition(launch_ros2_control),
    )

    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "--headless-rendering", world_file],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_group = GroupAction(
        actions=[gz_headless, gz_gui],
        condition=IfCondition(launch_gazebo),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("runtime_yaml")}],
        condition=IfCondition(launch_bridge),
    )

    world_tf = Node(
        package="ur5_tools",
        executable="world_tf_publisher",
        output="screen",
        parameters=[
            {"world_name": LaunchConfiguration("world_name")},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_world_tf),
    )

    system_state = Node(
        package="ur5_tools",
        executable="system_state_manager",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_name": LaunchConfiguration("world_name")},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"ee_frame": "tool0"},
            {"camera_topic": "/camera_overhead/image"},
            {"moveit_required": ParameterValue(launch_moveit, value_type=bool)},
        ],
        condition=IfCondition(launch_system_state),
    )

    release_service = Node(
        package="ur5_tools",
        executable="release_objects_service",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_release_service),
    )

    panel = ExecuteProcess(
        cmd=["ros2", "run", "ur5_qt_panel", "panel_v2"],
        output="screen",
        condition=IfCondition(launch_panel),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=world_default),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_panel", default_value="true"),
            DeclareLaunchArgument("launch_bridge", default_value="true"),
            DeclareLaunchArgument("launch_gazebo", default_value="true"),
            DeclareLaunchArgument("launch_rsp", default_value="true"),
            DeclareLaunchArgument("launch_ros2_control", default_value="true"),
            DeclareLaunchArgument("launch_world_tf", default_value="true"),
            DeclareLaunchArgument("launch_release_service", default_value="true"),
            DeclareLaunchArgument("launch_system_state", default_value="true"),
            DeclareLaunchArgument("launch_moveit", default_value="false"),
            DeclareLaunchArgument("moveit_start_ros2_control", default_value="false"),
            DeclareLaunchArgument("panel_auto_bridge", default_value="0"),
            DeclareLaunchArgument("panel_auto_bridge_delay_ms", default_value="1200"),
            DeclareLaunchArgument("panel_managed", default_value="1"),
            DeclareLaunchArgument("rmw_implementation", default_value="rmw_fastrtps_cpp"),
            OpaqueFunction(function=_prepare_runtime),
            OpaqueFunction(function=_maybe_moveit),
            rsp_launch,
            ros2_control_launch,
            controller_bootstrap,
            gz_group,
            bridge,
            world_tf,
            system_state,
            release_service,
            panel,
        ]
    )
