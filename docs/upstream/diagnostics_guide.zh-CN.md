# scripts 目录运行说明

本文档记录当前在现场直接运行的调试脚本。项目长期维护的 ROS 2 包仍应放在 `src/` 对应功能包中；这里主要保留硬件 bringup、单机验证和临时可视化脚本。

## 船体动力学测量

`measure_rover_dynamics.py` 使用 `/smartboat/localization/yaw` 和
`/smartboat/localization/ground_speed` 测量实际偏航角速度、转弯半径和反推停车距离。
默认由操作员使用遥控器驾驶，脚本只读取数据，不控制电机。每次只做一个动作，便于在小水域中重新摆正船位。

先启动 MAVLink yaw/地速节点，然后测试左右 90 度转向：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/measure_rover_dynamics.py turn --direction left --test-speed 0.7
python3 scripts/measure_rover_dynamics.py turn --direction right --test-speed 0.7
```

测量遥控器全反推停车距离：

```bash
python3 scripts/measure_rover_dynamics.py brake --test-speed 0.7
python3 scripts/measure_rover_dynamics.py brake --test-speed 1.0
python3 scripts/measure_rover_dynamics.py brake --test-speed 1.5
```

脚本也提供主动控制模式，但不会自行解锁或切换 PX4 模式。主动模式要求：遥控器急停可用、
`cmd_vel_to_mavlink` 已显式允许硬件输出、当前 PX4 模式在 bridge 白名单中，并且控制 topic
没有其他发布者。满足现场安全条件后才可显式启用：

```bash
python3 scripts/measure_rover_dynamics.py turn \
  --direction left \
  --test-speed 0.7 \
  --active \
  --confirm I_HAVE_RC_ESTOP
```

主动测试期间 yaw 或地速超时会立即中止并持续发布一段时间的零指令。正式采用主动测试前，
应先完成被动遥控器测试，确认正负速度、左右偏航方向和 PX4 bridge 映射正确。
当前脚本默认正 `angular.z` 经 MAVLink bridge 后使船右转；如果低速验证结果相反，主动测试时加
`--positive-command-turns left`。这项验证也用于决定正式 ROS/PX4 桥接是否需要转换 yaw 符号。

## 毫米波雷达 / PEAK PCAN-USB FD / SocketCAN

当前可用链路是：

```text
毫米波雷达 CANH/CANL -> PEAK PCAN-USB FD -> Jetson/Ubuntu -> Docker 容器 -> Radar.py
```

已验证雷达 CAN 波特率为 `500000`，`Radar.py` 当前解析标准 CAN ID `0x60B` 的 `Obj_1_General` 消息，并默认打开 112 度扇形视场图。程序会自动寻找 `canX` 接口；如果接口已启动就直接连接，如果接口处于 down 状态，会尝试按 500K bitrate 拉起。旧 iTEK USBCAN 代码仍保留在 `Radar.py` 中，但当前推荐优先使用 SocketCAN。

### 接线和总线

- 正常通信至少需要 `CANH`、`CANL`。
- 信号地不一定必须接；如果现场接地后出现异常，可先只保留 `CANH/CANL` 排查。
- 断电测单个终端常见约 `120 ohm`；完整 CAN 总线两端各一个终端时，`CANH-CANL` 等效常见约 `60 ohm`。
- 无帧但接口是 `ERROR-ACTIVE` 时，优先检查 CANH/CANL 是否接反、雷达供电、终端电阻和波特率。

### 安装 PEAK 驱动

Jetson 当前内核如果没有内置 `peak_usb`，需要安装 PEAK 官方 Linux 驱动的 SocketCAN/netdev 模式：

```bash
cd /home/ubuntu2204/smartboat/boat_ws
sudo bash scripts/install_peak_pcan_driver.sh
```

脚本会下载 PEAK 官方驱动到 `/tmp`，针对当前内核编译并安装 `pcan` 模块，然后加载模块。内核升级后通常需要重新运行一次。正常情况下不需要重装系统。

### 插拔后启动 CAN 口

USB 重新插拔后，SocketCAN 网口会重新出现但通常是 `DOWN/STOPPED`。`Radar.py` 启动时会尝试自动拉起；也可以手动设置波特率并拉起：

```bash
cd /home/ubuntu2204/smartboat/boat_ws
sudo bash scripts/bringup_peak_can.sh 500000
```

如果当前目录已经是容器内的 `/workspace/boat_ws/scripts`，则执行：

```bash
sudo bash bringup_peak_can.sh 500000
```

主一键launch和宿主机启动脚本都会先调用这个脚本，再启动无GUI的
`sr73f_radar_node`；`Radar.py`仅用于人工查看扇形窗口，不参与一键运行。

脚本会自动寻找 PEAK 对应的 `canX` 接口并打印接口名。当前现场常见是 `can2`；`can0/can1` 多半是 Jetson 自带 mttcan，不一定是 USB 盒子。

### 宿主机检查

```bash
ip -details link show can2
candump can2
```

`ip` 输出中应看到接口 `UP`，CAN state 通常应为 `ERROR-ACTIVE`，bitrate 为 `500000`。如果没有 `candump`，需要安装 `can-utils`。

### 容器内启动雷达

进入容器后：

```bash
cd /workspace/boat_ws
python3 /workspace/boat_ws/scripts/Radar.py
```

默认会自动选择 PEAK 对应的 `canX`，打开 OpenCV 扇形视图，按 `Ctrl-C` 退出。如果接口自动选择不符合预期，可以手动指定：

```bash
python3 /workspace/boat_ws/scripts/Radar.py --socketcan can2
```

常用参数：

| 参数 | 用途 |
| --- | --- |
| `--socketcan can2` | 手动指定 SocketCAN 接口；默认自动选择 |
| `--headless` | 不打开 OpenCV 窗口 |
| `--save <path>` | 保存当前扇形图 |
| `--once` | 读取和渲染一次后退出 |
| `--debug` | 打印解析后的目标字段 |

保存一张扇形图：

```bash
python3 /workspace/boat_ws/scripts/Radar.py \
  --headless \
  --once \
  --save /tmp/radar_sector.png
```

打印目标字段：

```bash
python3 /workspace/boat_ws/scripts/Radar.py --headless --once --debug
```

可视化颜色使用类似 RealSense 深度图的距离着色：近处偏暖色，远处偏冷色，并带距离色条和距离刻度。该雷达没有高度信息，所以显示是 2D 扇形目标图，不是严格 3D 密集点云。

### GUI 注意事项

默认图形窗口需要 X11 显示环境。推荐用桌面启动脚本进入容器，它会传递 `DISPLAY`：

```bash
/home/ubuntu2204/Desktop/启动智能船ROS2.sh
```

如果手动进入容器，确保传入 `DISPLAY`，例如：

```bash
docker exec -it \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -w /workspace/boat_ws \
  smartboat_ros2_g39 bash
```

不要用 root 用户跑 GUI；root 往往没有当前桌面的 X11 授权。没有 GUI 时用 `--headless --once --save /tmp/radar_sector.png` 先保存图片验证。

### 常见故障

| 现象 | 处理 |
| --- | --- |
| `OSError: [Errno 100] Network is down` | USB 盒子重新插拔后接口没拉起，运行 `sudo bash scripts/bringup_peak_can.sh 500000` |
| `No PEAK SocketCAN interface found` | 检查 PCAN-USB 是否插好，`lsusb` 是否能看到 PEAK，`lsmod | grep pcan` 是否已加载驱动 |
| 接口 `UP` 但 `receive_num: 0` | 检查雷达供电、CANH/CANL、终端电阻、波特率 `500000`、雷达是否真的发 ID `0x60B` |
| OpenCV `Can't initialize GTK backend` | 没有 DISPLAY 或 X11 授权；用普通容器用户运行，或改用 `--headless --once --save /tmp/radar_sector.png` |
| 重启/升级内核后没有 CAN 口 | 先 `sudo modprobe pcan`，如果模块缺失则重新运行 `install_peak_pcan_driver.sh` |
