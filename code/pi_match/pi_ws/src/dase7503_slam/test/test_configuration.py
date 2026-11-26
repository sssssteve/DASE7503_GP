import pytest
import yaml

from dase7503_slam.configuration import load_interfaces, mapping_parameters


def write_config(tmp_path, **changes):
    config = {
        "scan_topic": "/test_scan", "odom_topic": "/test_odom", "map_frame": "map",
        "odom_frame": "odom", "base_frame": "base", "lidar_frame": "laser",
        "publish_lidar_tf": False,
    }
    config.update(changes)
    path = tmp_path / "interfaces.yaml"
    path.write_text(yaml.safe_dump(config))
    return path


def test_blank_interfaces_are_rejected(tmp_path):
    path = write_config(tmp_path, scan_topic=None, odom_topic=None, lidar_frame=None)
    with pytest.raises(ValueError, match="Unconfirmed interfaces"):
        load_interfaces(path)


def test_external_tf_needs_no_guessed_mount(tmp_path):
    assert load_interfaces(write_config(tmp_path))["publish_lidar_tf"] is False


@pytest.mark.parametrize("bad_mount", [{}, {"x": 0, "y": 0, "z": 0,
                                         "roll": 0, "pitch": 0, "yaw": float("nan")}])
def test_unmeasured_mount_is_rejected(tmp_path, bad_mount):
    with pytest.raises(ValueError, match="Measured lidar_mount"):
        load_interfaces(write_config(tmp_path, publish_lidar_tf=True, lidar_mount=bad_mount))


def test_measured_mount_can_publish_tf(tmp_path):
    mount = dict(x=0.1, y=0, z=0.12, roll=0, pitch=0, yaw=0)
    assert load_interfaces(write_config(tmp_path, publish_lidar_tf=True,
                                        lidar_mount=mount))["lidar_mount"] == mount


def test_frames_cannot_form_self_transform(tmp_path):
    with pytest.raises(ValueError, match="distinct"):
        load_interfaces(write_config(tmp_path, lidar_frame="base"))


def test_interfaces_override_toolbox_defaults_and_keep_solver(tmp_path):
    default = tmp_path / "defaults.yaml"
    default.write_text(yaml.safe_dump({"slam_toolbox": {"ros__parameters": {
        "solver_plugin": "solver_plugins::CeresSolver", "base_frame": "wrong",
        "use_sim_time": True,
    }}}))
    overrides = tmp_path / "mapping.yaml"
    overrides.write_text("resolution: 0.02\nbase_frame: also_wrong\n")
    config = load_interfaces(write_config(tmp_path))
    params = mapping_parameters(default, overrides, config)["slam_toolbox"]["ros__parameters"]
    assert params["base_frame"] == "base"
    assert params["scan_topic"] == "/test_scan"
    assert params["use_sim_time"] is False
    assert params["mode"] == "mapping"
    assert params["resolution"] == 0.02
    assert params["solver_plugin"] == "solver_plugins::CeresSolver"
