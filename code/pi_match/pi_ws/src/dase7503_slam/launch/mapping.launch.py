import atexit
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dase7503_slam.configuration import MOUNT_FIELDS, load_interfaces, mapping_parameters


def start_mapping(context):
    config = load_interfaces(LaunchConfiguration("interfaces").perform(context))
    toolbox = Path(get_package_share_directory("slam_toolbox"))
    parameters = mapping_parameters(
        toolbox / "config/mapper_params_online_async.yaml",
        LaunchConfiguration("mapping_params").perform(context), config,
    )
    descriptor, filename = tempfile.mkstemp(prefix="dase7503_slam_", suffix=".yaml")
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        yaml.safe_dump(parameters, stream)
    atexit.register(Path(filename).unlink, missing_ok=True)
    actions = []
    if config["publish_lidar_tf"]:
        arguments = []
        for key in MOUNT_FIELDS:
            arguments.extend(["--" + key, str(config["lidar_mount"][key])])
        arguments.extend(["--frame-id", config["base_frame"],
                          "--child-frame-id", config["lidar_frame"]])
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher",
                            arguments=arguments, output="screen"))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(toolbox / "launch/online_async_launch.py")),
        launch_arguments={"slam_params_file": filename, "use_sim_time": "false"}.items(),
    ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("dase7503_slam"))
    return LaunchDescription([
        DeclareLaunchArgument("interfaces", default_value=str(share / "config/interfaces.yaml")),
        DeclareLaunchArgument("mapping_params", default_value=str(share / "config/mapping.yaml")),
        OpaqueFunction(function=start_mapping),
    ])
