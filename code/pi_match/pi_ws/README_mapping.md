# 树莓派实车建图

### 在 Pi 上编译

检出本仓库后进入 `code/pi_match/pi_ws`。如果目录放在别处，按实际路径进入。编译会生成 `pi_ws/install/setup.bash`。以下在 **Pi 的 SSH 终端**执行：

```bash
cd ~/DASE7503_GP/code/pi_match/pi_ws &&
source /opt/ros/jazzy/setup.bash &&
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy &&
colcon build --symlink-install --executor sequential --packages-select oradar_lidar dase7503_slam &&
test -f install/setup.bash
```

若 Pi 还没安装 `colcon` 或初始化 `rosdep`，先按本目录 [README.md](README.md) 准备。底盘 Agent 与消息包继续使用已编译的 `~/microros_ws`。

下面每个命令块都可以单独复制到树莓派的 Bash 终端。建图前关闭比赛导航和旧遥控，允许时先把可搬货架移开。

### 终端 1 · 底盘 Agent

沿用底盘 README 的 `/dev/ttyUSB0`。看到 Agent 显示 `running...` 后复位 ESP32，保持终端运行。

```bash
source /opt/ros/jazzy/setup.bash &&
source ~/microros_ws/install/local_setup.bash && (
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 run micro_ros_agent micro_ros_agent serial --dev "${ESP_PORT:-/dev/ttyUSB0}" -b 115200
)
```

### 终端 2 · MS200p 雷达

使用单雷达启动入口，自动跟踪 `ttyACM0` 和 `ttyACM1`，两种编号共用同一个 `/scan`。

```bash
bash ~/DASE7503_GP/code/pi_match/pi_hardware/pi.sh lidar
```

如果 Pi 上端口不同，拔插确认后，在对应终端先设 `ESP_PORT` 或 `LIDAR_PORT` 为实际路径，再复制上面的命令。ESP32 和雷达使用不同的设备。权限报错时，按底盘 README 检查 `dialout`。

### 终端 3 · 轮式里程计

适配器把 ESP32 的 `/chassis/status` 转成 `/odom` 和 `odom → base_link`。保留速度话题重映射，让键盘控制底盘。已有节点发布同一组里程计和 TF 时，跳过这一终端。

```bash
source /opt/ros/jazzy/setup.bash &&
source ~/microros_ws/install/local_setup.bash &&
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
status="$(timeout 10s ros2 topic echo /chassis/status chassis_interfaces/msg/ChassisStatus --once)" &&
printf '%s\n' "$status" | python3 -c '
import sys, yaml
s = next(yaml.safe_load_all(sys.stdin), {})
if not s or s.get("fault") != 0 or s.get("emergency_stopped") is not False:
    sys.exit("底盘状态异常或急停已触发，先排查 /chassis/status")
print("底盘状态正常，启动里程计")
' &&
python3 ~/DASE7503_GP/code/pi_match/simulation/logistics/adapter.py \
  --ros-args -r /cmd_vel:=/commissioning/unused_velocity
```

### 终端 4 · 建图

包内配置已填雷达相对车体几何中心前移 **0.10 米**（车长 28 cm，雷达中心距车尾 24 cm，横向居中），其余偏移和安装角为 0。建图会发布 `base_link → laser_frame`；若车上已有节点发布这段 TF，在 `interfaces.yaml` 将 `publish_lidar_tf` 设为 `false`。

```bash
source /opt/ros/jazzy/setup.bash &&
source ~/DASE7503_GP/code/pi_match/pi_ws/install/setup.bash &&
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
ros2 launch dase7503_slam mapping.launch.py \
  interfaces:="$HOME/DASE7503_Pi5/pi_ws/src/dase7503_slam/config/interfaces.yaml"
```

### 终端 5 · 检查并打开键盘遥控

用交互式终端执行。这条命令直接运行联调包内小组的 `keyboard_teleop.py`。建图时只开一个遥控程序。

```bash
source /opt/ros/jazzy/setup.bash &&
source ~/microros_ws/install/local_setup.bash &&
source ~/DASE7503_GP/code/pi_match/pi_ws/install/setup.bash &&
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
ros2 run dase7503_slam check_inputs \
  --config "$HOME/DASE7503_Pi5/pi_ws/src/dase7503_slam/config/interfaces.yaml" --timeout 10 &&
python3 "$HOME/DASE7503_Pi5/vehicle/code/microros_ws/exmaple/keyboard_teleop.py"
```

### 终端 6 · 建图窗口

**用 SSH 在自己电脑上看（推荐）。** 在 Mac 新开一个终端，输入平时登录树莓派的用户名和地址，例如 `pi@树莓派地址`：

```bash
printf '树莓派 SSH 地址（用户名@主机）：'
read -r PI_SSH
ssh -o ExitOnForwardFailure=yes -L 127.0.0.1:8765:127.0.0.1:8765 "$PI_SSH"
```

登录后，在这一个 SSH 终端执行：

```bash
source /opt/ros/jazzy/setup.bash &&
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
python3 "$HOME/DASE7503_Pi5/pi_ws/mapping_web.py"
```

保留 SSH 窗口，在 **Mac 浏览器**打开 <http://127.0.0.1:8765/>。网页只显示建图数据。白色是空闲区，黑色是障碍，灰色是未知区，红点是实时雷达，青色箭头是车头。等待地图和激光出现再开车。若 Mac 的 8765 端口已占用，把 SSH 命令中第一个 8765 改为 8766，然后打开 <http://127.0.0.1:8766/>。

**树莓派接了显示器时**，也可以从图形桌面用原来的 RViz 窗口。

```bash
source /opt/ros/jazzy/setup.bash &&
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
ros2 run rviz2 rviz2 -d "$HOME/DASE7503_Pi5/pi_ws/mapping.rviz"
```

先前进、横移一小段，确认方向正确且墙线基本不动，再低速巡场。**W/S** 前后、**Q/E** 横移、**A/D** 转向、**空格**停车。回到起始区后，停车检查两面墙是否贴合。

### 保存地图

按 Ctrl+C 退出遥控，**保持终端 4 运行**。在终端 5 复制：

```bash
source /opt/ros/jazzy/setup.bash &&
source ~/DASE7503_GP/code/pi_match/pi_ws/install/setup.bash &&
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
ros2 run dase7503_slam save_map \
  --output "$HOME/DASE7503_Pi5/pi_ws/maps/gamefield_$(date +%Y%m%d_%H%M%S)"
```

保存成功后，YAML 和同名地图图像一起保留。比赛时把 YAML 路径和地图上的起始区边界 `start_region` 填入 `pi_hardware/match.yaml`。
