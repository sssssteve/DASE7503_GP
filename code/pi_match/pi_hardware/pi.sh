#!/usr/bin/env bash
set -eo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
root="$(dirname "$here")"
export DASE_MATCH_CONFIG="${2:-$here/match.yaml}"
source /opt/ros/jazzy/setup.bash
source "${DASE_CHASSIS_SETUP:-$HOME/microros_ws/install/setup.bash}"
if [[ ! -f "$root/pi_ws/install/setup.bash" ]]; then
  echo "请先按 pi_ws/README_mapping.md 在 $root/pi_ws 完成 colcon 构建。" >&2
  exit 1
fi
source "$root/pi_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
camera="${DASE_CAMERA_DIR:-$root/../QR_scan_picture}"
if [[ -z "${DASE_CAMERA_DIR:-}" && ! -r "$camera/DASE7503_QR_Vision_single/scan_node.py" ]]; then
  camera="$HOME/QR_scan_picture"
fi
export PYTHONPATH="$here:$camera/DASE7503_QR_Vision_single:$root/simulation/logistics:${PYTHONPATH:-}"
check_config() {
  /usr/bin/python3 - <<'PYCONFIG'
import os,sys,yaml
from config import load
try:
    load(os.environ['DASE_MATCH_CONFIG'])
except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
    sys.exit(str(error))
print('配置、地图与进架尺寸校验通过')
PYCONFIG
}

# Run only device drivers for which no ROS publisher is available.
has_publisher() {
  local info
  info="$(LC_ALL=C ros2 topic info "$1" --no-daemon --spin-time 2 2>/dev/null)" || return 1
  grep -Eq '^Publisher count: [1-9][0-9]*$' <<<"$info"
}

start_devices() {
  local port binary
  if ! has_publisher /scan && ! pgrep -x oradar_scan >/dev/null; then
    setsid /usr/bin/python3 "$here/lidar_ports.py" "$(ros2 pkg prefix oradar_lidar)/lib/oradar_lidar/oradar_scan" 9>&- &
    owned+=("$!")
  fi
  if ! has_publisher /camera/image_raw && ! pgrep -x camera_node >/dev/null; then
    local driver="$camera"
    if [[ ! -f "$driver/camera_ws/install/local_setup.bash" && -f "$HOME/QR_scan_picture/camera_ws/install/local_setup.bash" ]]; then
      driver="$HOME/QR_scan_picture"
    fi
    setsid bash "$driver/vision.sh" camera 9>&- &
    owned+=("$!")
  fi
  if ! has_publisher /chassis/status; then
    if ! pgrep -x micro_ros_agent >/dev/null; then
      local bridges
      while :; do
        bridges=(/dev/serial/by-id/*CP2102*)
        port="${ESP_PORT:-/dev/ttyUSB0}"
        if [[ -z "${ESP_PORT:-}" && -e "${bridges[0]}" ]]; then
          if (( ${#bridges[@]} != 1 )); then echo '多个 CP2102，请指定 ESP_PORT' >&2; return 1; fi
          port="${bridges[0]}"
        fi
        [[ -c "$port" ]] && break
        echo "等待 ESP32 串口 $port"
        sleep 2
      done
      [[ -w "$port" && -r "$port" ]] || { echo "ESP32 串口不可用 $port" >&2; return 1; }
      binary="$(ros2 pkg prefix micro_ros_agent)/lib/micro_ros_agent/micro_ros_agent"
      setsid "$binary" serial --dev "$port" -b 115200 9>&- &
      owned+=("$!")
    fi
    echo '等待 ESP32。若没有底盘反馈，请保持车静止并按板上 EN/RESET。'
  fi
  echo '等候 ESP32 底盘反馈…'
  ros2 topic echo /chassis/status chassis_interfaces/msg/ChassisStatus --once --field fault >/dev/null
  echo '等候雷达扫描…'
  ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --field header --qos-reliability best_effort >/dev/null
  timeout 20 ros2 topic echo /camera/image_raw sensor_msgs/msg/Image --once --field header --qos-reliability best_effort >/dev/null || { echo '相机未出图，请检查 vision.sh camera 的输出' >&2; return 1; }
}

run_auto() (
  check_config
  exec 9>"${XDG_RUNTIME_DIR:-/tmp}/dase7503-match-${UID}-${ROS_DOMAIN_ID}.lock"
  if ! flock -n 9; then
    echo '已有一键比赛任务在运行' >&2
    return 1
  fi
  running="$(ros2 node list --no-daemon --spin-time 4)"
  if grep -Eq '^/(map_server|amcl|bt_navigator|slam_toolbox|real_rack_match|dase7503_auto_delivery|qr_scan|chassis_odom_and_control_gate)$' <<<"$running"; then
    echo '请先退出已启动的建图、导航或比赛进程' >&2
    return 1
  fi

  owned=()
  cleanup() {
    local pid
    for pid in "${owned[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    wait || true
  }
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap 'exit 129' HUP
  export DASE_EMPTY_NAV_TEST=0
  local mission_action=run
  local camera_args=()
  if [[ "${1:-}" == headless || "${1:-}" == nav-test ]]; then
    unset DASE_GUI_MATCH_ID DASE_GUI_EXTERNAL_LOCALIZATION
    export DASE_MATCH_TERMINAL=1
    if [[ "$1" == nav-test ]]; then
      mission_action=nav-test
      export DASE_EMPTY_NAV_TEST=1
      if [[ -n "${DASE_NAV_MONITOR_DIR:-}" ]]; then
        setsid journalctl -k -f -n 0 --no-pager --case-sensitive=no \
          --grep='undervoltage|voltage normalised|over-current|USB disconnect|attached to tty|cdc_acm.*ttyACM' 9>&- &
        owned+=("$!")
        setsid /usr/bin/python3 -u "$here/nav_monitor.py" 9>&- &
        owned+=("$!")
      fi
      for pkg in nav2_bringup nav2_amcl nav2_collision_monitor; do
        ros2 pkg prefix "$pkg" >/dev/null || { echo "缺少 $pkg，请先安装 Nav2。" >&2; return 1; }
      done
      echo '空车钻架测试：请确认托板已在低位。车辆会实际钻入每个货架并倒退退出，全程不发送升降指令。'
    else
      /usr/bin/python3 -c 'import sys,yaml,os; c=yaml.safe_load(open(os.environ["DASE_MATCH_CONFIG"])); sys.exit("人工确认模式需要交互终端，请使用 ssh -t") if c.get("lift_policy")=="operator" and not sys.stdin.isatty() else None'
    fi
    start_devices
    camera_args=(--camera-prechecked)
    if [[ -n "${DASE_NAV_MONITOR_DIR:-}" ]]; then
      log_file="$DASE_NAV_MONITOR_DIR/nav2.log"
    else
      log_file="$(mktemp "${TMPDIR:-/tmp}/dase7503-nav.XXXXXXXX")"
    fi
    echo "导航启动日志：$log_file"
    setsid ros2 launch "$here/nav.launch.py" 9>&- >"$log_file" 2>&1 &
  else
    setsid ros2 launch "$here/nav.launch.py" 9>&- &
  fi
  owned+=("$!")
  /usr/bin/python3 "$here/match.py" "$mission_action" "$DASE_MATCH_CONFIG" "${camera_args[@]}" 9>&-
)

case "${1:-check}" in
  lidar)
    if pgrep -x oradar_scan >/dev/null; then echo '已有雷达驱动'; exit 0; fi
    exec /usr/bin/python3 "$here/lidar_ports.py" "$(ros2 pkg prefix oradar_lidar)/lib/oradar_lidar/oradar_scan";;
  config) check_config;;
  auto|headless|nav-test) run_auto "$1";;
  nav) exec ros2 launch "$here/nav.launch.py";;
  check|run) exec /usr/bin/python3 "$here/match.py" "$1" "$DASE_MATCH_CONFIG";;
  *) echo '用法：bash pi.sh lidar|config|nav|check|run|auto [match.yaml]'; exit 2;;
esac
