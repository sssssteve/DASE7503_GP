# DASE7503 雷达建图基础包

面向树莓派上的 ROS 2 Jazzy。负责雷达/里程计/TF 输入检查、启动
`slam_toolbox` 建图和保存地图。未知接口以 `null` 留空，启动时拒绝未填写配置。

这是树莓派上的 ROS 软件包，复制或 `git clone` 后用 `colcon` 构建运行。
无需把这个包烧录到 ESP32；ESP32 电机、编码器和 micro-ROS 固件由底盘同学提供。

## 当前边界

- 雷达驱动、串口和波特率待确认，由外部驱动发布真实 `LaserScan`。
- 底盘负责提供真实 `Odometry` 和动态 `odom → base_frame`。
- 不产生假雷达/里程计、不发布固定的里程计变换，也不发送电机指令。
- 本版先做建图；AMCL 比赛定位、Nav2 路径规划、二维码和升降不在此包内。
- 配置测试或 ROS 构建通过不代表实体建图或定位精度已经验证。

## 树莓派安装与构建

以下命令仅适用于已经安装 ROS 2 Jazzy 的 Ubuntu 环境；学校镜像和其他系统先确认。
在树莓派的工作目录克隆私密仓库，使用你已有的 GitHub 身份认证：

```bash
git clone https://github.com/sssssteve/dase7503-slam.git
cd dase7503-slam
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-yaml
# 如果 rosdep 尚未初始化，执行一次 sudo rosdep init；已初始化则跳过。
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

之后每个 ROS 终端都要加载 `/opt/ros/jazzy/setup.bash` 和本仓库的
`install/setup.bash`。不要将访问令牌写进仓库或命令行。

## 1. 填写真实接口

编辑 `src/dase7503_slam/config/interfaces.yaml`。

| 字段 | 谁确认 | 当前值 |
|---|---|---|
| `scan_topic`、`lidar_frame` | 雷达驱动，查看 `/scan` 或实际话题的 header | `null` |
| `odom_topic`、`odom_frame`、`base_frame` | 底盘同学，检查里程计消息与 TF | `null` |
| `map_frame` | 与导航同学统一，本包使用 `map` | `map` |
| `lidar_mount` 六个数值 | 机械实测，位置为米、角度为弧度 | `null` |
| 雷达驱动包、launch、串口、波特率 | 实物版本与厂商驱动 | 待确认，不在本包猜测 |

`publish_lidar_tf: false` 表示车体到雷达的 TF 已由外部 URDF/节点提供。
只有你负责发布该 TF 时才改为 `true`，并补齐全部实测安装参数。
不要与外部节点重复发布；不要发布静态 `odom → base_frame` 来绕过里程计。

## 2. 启动雷达、底盘，然后检查输入

先按确认过的厂商配置启动雷达驱动、ESP32/micro-ROS 里程计和机器人 TF。
本包不代替这些驱动。先查看真实接口：

```bash
ros2 topic list -t
ros2 topic info /scan -v         # 替换为你们的实际 scan_topic
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once     # 替换为你们的实际 odom_topic
```

如果安装 TF 由本包发布，先启动下一节的 mapping，再在第二个终端做检查。
否则可以先检查再启动建图。

```bash
ros2 run dase7503_slam check_inputs \
  --config "$PWD/src/dase7503_slam/config/interfaces.yaml" --timeout 10
```

检查读取真实扫描和里程计，核对消息坐标系、有效距离、非零时间戳，
并检查扫描时刻的两段 TF。消息最多允许落后本机 ROS 时钟 2 秒；
出现未来时间戳或过期数据时先检查时钟同步。返回码 0 是接口检查通过，
1 是数据/TF 未就绪，2 是配置或参数无效。

一次通过只表示这一时刻的输入链路可用。还需检查持续扫描频率、
前进/横移/转向时里程计方向与距离，以及载货时遮挡情况。

## 3. 启动建图

```bash
ros2 launch dase7503_slam mapping.launch.py \
  interfaces:="$PWD/src/dase7503_slam/config/interfaces.yaml"
```

启动读取当前安装版本的 `slam_toolbox` 完整默认参数，并覆盖已确认的
坐标系、扫描话题及 `config/mapping.yaml`。真实硬件强制使用系统时间。
临时参数文件在 launch 进程退出时清理。

另一个已加载环境的终端运行 `rviz2`：Fixed Frame 设为 `map`，添加
Map(`/map`)、LaserScan(实际扫描话题)和 TF。若扫描不可见，检查 RViz QoS。
用底盘同学提供的手动控制低速绕场地行驶并回到起点；不要抬车换位置。

起始地图分辨率为 0.02 米，关键帧移动阈值 0.05 米、转角阈值 0.1 弧度。
这些是小场地的起始参数，不是 2 cm 定位精度保证。先检查扫描、里程计、
安装 TF 和时间，再根据地图重影/更新速度调整参数。

## 4. 保存地图及原始数据

保持建图运行：

```bash
ros2 run dase7503_slam save_map --output "$PWD/maps/gamefield"
ros2 bag record -o "$PWD/bags/mapping_run_01" /scan /odom /tf /tf_static
```

录制命令中的话题也要换成实际名称；用 Ctrl+C 结束录制。
地图保存包装器调用 Nav2 map saver，失败返回非零码；已存在同名地图时
拒绝覆盖，选择新名字。地图图像与 YAML 需一起保留。
地图和 bag 默认不提交 Git，以免把大体积实测数据混入基础代码。

## 5. 建图验收与交接

1. 静止、前进、横移、转向时扫描和 TF 持续正常。
2. 场地边界与实际尺寸一致，绕行回到起点后无明显双层墙或错位。
3. 保存地图后能由定位同学重新加载；停止建图再启动 AMCL，避免两个
   节点同时发布 `map → odom`。
4. 记录实测位置误差、扫描频率、重启定位和载货遮挡测试。
5. 明确可搬货架的地图处理策略，避免搬走后仍作为静态障碍物残留。

交接：驱动版本/串口配置、本包配置、实测安装 TF、地图图像及 YAML、
建图截图和 bag。没有实体测试之前，只能标记“基础代码已完成”。

## 测试与参考

```bash
colcon test --packages-select dase7503_slam --python-testing pytest --event-handlers console_direct+
colcon test-result --verbose
```

GitHub Actions 在 ROS Jazzy 容器中构建、运行配置测试和检查 launch 参数；
它不连接实体雷达，也不验证真实建图。

- [MS200 厂商资料](https://www.yahboom.net/study/MS200)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 建图与定位](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)
