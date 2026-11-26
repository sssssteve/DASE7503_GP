# Pi 比赛导航与自动搬运

这份源码来自 11 月 25 日实车联调归档。入口在本目录的 `start_match.sh`。ESP32 固件、底盘消息和扫码驱动沿用团队仓库。

- [ESP32 固件](https://github.com/vectorBH6/DASE7503_group_project/blob/76ad4dbf15db6e0707a990ba43e6b7b2ee7cac27/code/esp32_chassis/README.md)
- [micro-ROS 底盘与 Agent](https://github.com/vectorBH6/DASE7503_group_project/blob/76ad4dbf15db6e0707a990ba43e6b7b2ee7cac27/code/microros_ws/README.md)
- [相机与二维码](https://github.com/vectorBH6/DASE7503_group_project/blob/76ad4dbf15db6e0707a990ba43e6b7b2ee7cac27/code/QR_scan_picture/README.md)

## 在 Pi 上准备

Pi 使用 ROS 2 Jazzy、Nav2 和已经构建的团队 micro-ROS 工作区。初始化步骤见上面的模块文档。默认可复用 Pi 上的 `~/QR_scan_picture`。若相机部署在其他目录，设置 `DASE_CAMERA_DIR` 指向包含 `vision.sh` 和扫码代码的目录。

在本目录构建雷达与建图工作区。

```bash
source /opt/ros/jazzy/setup.bash
cd pi_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --executor sequential --packages-select oradar_lidar dase7503_slam
cd ..
```

复制 `pi_hardware/match.yaml` 为同目录的 `pi_hardware/match.local.yaml`，填入本组二维码，并复核车身、舵机、立柱和场地坐标。默认模板的 `XXXX` 是占位值。若 micro-ROS 工作区放在其他目录，设置 `DASE_CHASSIS_SETUP` 指向它的 `install/setup.bash`。

```bash
bash start_match.sh --check pi_hardware/match.local.yaml
bash start_match.sh --nav-test --monitor pi_hardware/match.local.yaml
```

空车测试会实际驱动车辆钻架和退出。完整比赛在完成空车和升降联调后，运行 `bash start_match.sh pi_hardware/match.local.yaml`。详细流程和参数见 [比赛总控说明](pi_hardware/README.md)。

## 在 Mac 上看监控

在同一仓库目录运行以下命令。使用 `--pi-root` 指向 Pi 上实际检出的比赛目录。

```bash
python3 mapping_view/local_gui.py --radar --pi-root '~/DASE7503_GP/code/pi_match'
```

`--radar` 使用 SSH 只读订阅 Pi 的 ROS 状态，网页服务运行在 Mac 本机。只看底图时用 `python3 mapping_view/local_gui.py --offline`。
