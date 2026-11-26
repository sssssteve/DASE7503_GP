#!/usr/bin/env bash
set -eo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${DASE_CHASSIS_SETUP:-$HOME/microros_ws/install/setup.bash}"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DASE_GUI=1
export PYTHONPATH="$root/simulation/logistics:${PYTHONPATH:-}"
running="$(ros2 node list --no-daemon --spin-time 4)"
if grep -Eq '^/(slam_toolbox|amcl|map_server|mapping_web_view|chassis_odom_and_control_gate)$' <<<"$running"; then
    echo '先退出旧建图、官方定位、网页和调试里程计节点；ESP32 Agent 可保留' >&2
    exit 1
fi
if [[ $# -gt 0 ]]; then
    export DASE_MATCH_CONFIG="$(realpath -- "$1")"
    official="$root/pi_ws/maps/official_reference/field.yaml"
    python3 - "$official" <<'PY'
import os
from pathlib import Path
import sys
import yaml

config=Path(os.environ['DASE_MATCH_CONFIG'])
values=yaml.safe_load(config.read_text())
map_file=values.get('map')
if not isinstance(map_file,str) or (config.parent/map_file).resolve()!=Path(sys.argv[1]).resolve():
    sys.exit('网页比赛配置须指向包内官方地图 pi_ws/maps/official_reference/field.yaml')
PY
    bash "$root/pi_hardware/pi.sh" config "$DASE_MATCH_CONFIG"
    ros2 pkg prefix nav2_bringup >/dev/null
    ros2 pkg prefix nav2_collision_monitor >/dev/null
fi
pids=()
trap 'for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done; wait || true' EXIT
trap 'exit 0' TERM INT
bash "$here/start.sh" &
pids+=("$!")
if [[ $# -gt 0 ]]; then
    DASE_GUI_EXTERNAL_LOCALIZATION=1 bash "$root/pi_hardware/pi.sh" nav "$DASE_MATCH_CONFIG" &
else
    python3 "$root/pi_hardware/control.py" &
fi
pids+=("$!")
python3 "$root/mapping_view/mapping_web.py" &
pids+=("$!")
wait -n "${pids[@]}"
