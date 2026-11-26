# 本地场地控制台

在 Mac 的终端进入联调包目录，运行 `python3 mapping_view/local_gui.py`。浏览器会打开 **http://127.0.0.1:8767/**，连接使用现有的 `pi@pi` SSH 密钥。连接失败时仍显示官方底图。只看图可以加 `--offline`。

## 树莓派

使用包内更新的 `pi_hardware/official_localization/`、`pi_hardware/start_region.py`、`pi_hardware/nav.launch.py` 和 `mapping_view/mapping_web.{py,html}`。官方底图位于 `pi_ws/maps/official_reference/`。Pi 上需要 ROS 2 Jazzy、Nav2、团队 micro-ROS 工作区和底盘固件。若 AMCL 已装到系统，脚本使用系统版本。若此前使用 Pi 上现有的 AMCL overlay，保留它。

保持雷达驱动运行并发布 `/scan`。先退出旧建图、旧官方定位、旧网页、调试里程计和键盘遥控。ESP32 Agent 可以保留。启动前把车放在蓝色 START 区，**车头和相机正对 START 码**。

```bash
cd ~/DASE7503_GP/code/pi_match
bash pi_hardware/official_localization/gui.sh
```

上面的入口支持遥控和墙面复核。要使用点选导航和比赛导航，在 Pi 上用已填写并校验通过的实车配置启动同一个入口：

```bash
cd ~/DASE7503_GP/code/pi_match
bash pi_hardware/official_localization/gui.sh pi_hardware/match.yaml
```

实车配置的 `map` 指向包内 `pi_ws/maps/official_reference/field.yaml`，车体半径、货架顺序、两根二维码立柱和落位用实测数据。运行相机驱动后再开始比赛。网页负责启动现有比赛总控，和手动遥控、点选导航共用一个底盘入口。网页比赛期间请勿另开 `pi.sh run`。

## 操作

1. 点击「启动底盘」。页面区分找不到串口、权限不足、Agent 退出、ESP32 无反馈和底盘故障或急停。默认找唯一的 CP2102 串口。若你们换了转换器或有多个同类设备，启动后端前用 `export ESP_PORT=/dev/serial/by-id/实际设备` 指定。Agent 已运行却无反馈时，保持车静止，按 ESP32 板上的 EN/RESET，等待页面显示「ESP32 反馈正常」。 Agent 启动日志位于 Pi 的 `pi_hardware/official_localization/gui-agent.log`。
2. 点「遥控」，按住前后左右按钮低速点动，松开停止。点「导航」后先在地图点选，再按「发送地图目标」。可随时点「停止」。
3. 发车时手动面向 START 码，地图上的位置和朝向由实时扫描确定。移动中 AMCL 用滤波扫描跟踪，移动超过 4 cm 或转向超过 4°，停稳后用三圈扫描复核两面垂直墙。小幅、证据一致的修正才提交给 AMCL。手动复核也可点击「复核墙面」。
4. P1–P4 周边的紫圈是实时扫描到的候选腿。比赛总控确认四腿后，会额外标出参与当前取架定位的四个实测腿中心。比赛使用 `leg_spacing_m` 的实测间距。红点是滤波扫描，原始 `/scan` 保留供避障使用。

5. 已定位在蓝色 START 区、车头朝约定方向时，点「开始比赛」。页面显示相机正在等哪个码、货架顺序、剩余时间和已落架位置。采用人工确认配置时，目视看到货架已经离地或落稳，再按对应的确认按钮。比赛阶段由原总控处理。
6. 比赛中保持网页在前台。网页断开、底盘反馈中断或定位失效会触发停车。启动过计时后，需现场清场并重启整套 Pi 网页比赛后端，才可开下一轮。课程流程日志写在 Pi 的 `pi_hardware/official_localization/gui-match.log`。

在 Mac 运行 `python3 mapping_view/local_gui.py --radar` 可只读监看 Pi 的 /scan。雷达驱动须在 Pi 上运行。网页在一张官方地图上叠加同一时刻的车头和雷达回波，默认过滤车身，可勾选查看原始回波。若调试时已知车在蓝区一侧靠角落，在 Pi 的网页启动命令前加 `DASE_GUI_START_SIDE=1`，允许沿起点侧墙定位。正式比赛仍须从蓝区发车。

本地控制及 `--radar` 监看页面订阅 `/camera/image_raw`，以 5 Hz 输出最多 480 像素宽的 JPEG 预览，图像与雷达通过独立的 SSH 连接传输，扫码继续使用原始分辨率。页面同时报告雷达、底盘、里程计和起始定位交接状态。相机启动后即可预览。

相机下方的雷达近场直接按车体坐标画点，车头朝上，圈距 10 cm。黄框标出右缘固定小簇的过滤范围，勾选“查看原始回波”可对照。主图优先采用扫描时刻的地图 TF。没有 TF 时，本机从起点侧搜索官方矩形围墙，连续三次匹配一致后，在主图叠加同帧实测红点和车头，并标注“围墙估计预览”。该位姿只影响显示。只有孤立单束命中的点会被去孤点规则剔除。

黄框是实车配置的候选架位，紫色连线只画由多帧观测支持的四腿矩形。墙线取有足够点支撑的实测线段，不跨越明显的遮挡缺口。

车身外扩 1 cm 的过滤带以黄色圆角轮廓显示，原始视图可对照被过滤的点。

主图预览用墙段回波细化位姿，并要求两组垂直墙线支撑。暂时失去匹配时，虚线箭头标出上次确认位置和经过时间；恢复匹配后再叠加当前扫描。
