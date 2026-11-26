#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if ros2 node list --no-daemon --spin-time 4 | grep -qx /slam_toolbox; then
    echo '先退出在线 SLAM，再启动官方图定位' >&2
    exit 1
fi
prefix="$PWD/overlay/opt/ros/jazzy"
if [[ -x "$prefix/lib/nav2_amcl/amcl" ]]; then
    export AMENT_PREFIX_PATH="$prefix:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
else
    prefix=/opt/ros/jazzy
fi
read -r x y z roll pitch yaw < <(python3 - <<'PY'
from pathlib import Path
import yaml
cfg=yaml.safe_load(Path('../../pi_ws/src/dase7503_slam/config/interfaces.yaml').read_text())
m=cfg['lidar_mount']
print(*(float(m[k]) for k in ('x','y','z','roll','pitch','yaw')))
PY
)
pids=()
trap 'for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done; wait || true' EXIT
trap 'exit 0' TERM INT
/opt/ros/jazzy/lib/nav2_map_server/map_server --ros-args -p yaml_filename:="$PWD/../../pi_ws/maps/official_reference/field.yaml" &
pids+=("$!")
/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher --x "$x" --y "$y" --z "$z" --roll "$roll" --pitch "$pitch" --yaw "$yaw" --frame-id base_link --child-frame-id laser_frame &
pids+=("$!")
"$prefix/lib/nav2_amcl/amcl" --ros-args -p robot_model_type:=nav2_amcl::OmniMotionModel -p base_frame_id:=base_link -p scan_topic:=/scan_filtered -p set_initial_pose:=false -p update_min_d:=0.03 -p update_min_a:=0.07 &
pids+=("$!")
manager=/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager
if [[ -x "$manager" ]]; then
    "$manager" --ros-args -r __node:=official_localization_manager -p autostart:=true -p "node_names:=[map_server, amcl]" &
    pids+=("$!")
else
    python3 activate.py
fi
python3 localize.py &
pids+=("$!")
wait -n "${pids[@]}"
