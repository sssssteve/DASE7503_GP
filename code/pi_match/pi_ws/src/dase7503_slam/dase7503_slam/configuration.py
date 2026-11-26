import math
from pathlib import Path

import yaml


REQUIRED_INTERFACES = (
    "scan_topic", "odom_topic", "odom_frame", "base_frame", "lidar_frame", "map_frame"
)
MOUNT_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


def load_interfaces(path):
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("interfaces.yaml must contain a YAML mapping")
    missing = [key for key in REQUIRED_INTERFACES
               if not isinstance(config.get(key), str) or not config[key].strip()]
    if missing:
        raise ValueError("Unconfirmed interfaces: " + ", ".join(missing))
    for key in REQUIRED_INTERFACES:
        if config[key] != config[key].strip():
            raise ValueError(f"{key} must not contain surrounding whitespace")
    frames = [config[key] for key in ("map_frame", "odom_frame", "base_frame", "lidar_frame")]
    if len(set(frames)) != len(frames) or any(frame.startswith("/") for frame in frames):
        raise ValueError("TF frames must be distinct names without a leading slash")
    if type(config.get("publish_lidar_tf")) is not bool:
        raise ValueError("publish_lidar_tf must be true or false")
    if config["publish_lidar_tf"]:
        mount = config.get("lidar_mount") or {}
        for key in MOUNT_FIELDS:
            value = mount.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Measured lidar_mount.{key} is required")
    return config


def mapping_parameters(default_path, overrides_path, config):
    document = yaml.safe_load(Path(default_path).read_text(encoding="utf-8"))
    parameters = document["slam_toolbox"]["ros__parameters"]
    overrides = yaml.safe_load(Path(overrides_path).read_text(encoding="utf-8"))
    parameters.update(overrides)
    for key in ("map_frame", "odom_frame", "base_frame", "scan_topic"):
        parameters[key] = config[key]
    parameters.update(mode="mapping", use_sim_time=False)
    return document
