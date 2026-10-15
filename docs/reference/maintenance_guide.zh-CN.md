# Smartboat ROS 2 项目维护说明

本仓库根目录是智能船 ROS 2 项目目录：

```bash
/home/ubuntu2204/smartboat
```

ROS 2 工作区源码目录：

```bash
/home/ubuntu2204/smartboat/boat_ws/src
```

容器内通常对应：

```bash
/workspace/boat_ws/src
```

本文档面向两类读者：

- 人类开发者：能快速知道怎么启动、每个包负责什么、哪些地方已经可用、哪些地方还只是待开发骨架。
- Codex / 后续自动化开发：保持包边界、命名、topic、message、参数和启动方式一致，避免把新代码随手堆到错误位置。

## 在开始之前

Jetson 宿主机已配置 NoMachine + XFCE 作为无头下水测试时的远程桌面兜底方案，NoMachine 服务监听默认端口 `4000`，并已设置为开机自启。客户端在同一局域网内连接 Jetson IP，使用宿主机用户 `ubuntu2204` 登录即可。本文档不记录登录密码。

物理 DP 显示器优先使用 Ubuntu/GNOME 桌面，由 `gdm3` 启动；NoMachine 在无头时创建独立的 XFCE 虚拟桌面。不要把 NoMachine 虚拟桌面强行切成 GNOME，GNOME 虚拟会话在无头 Jetson 上更容易受 GPU/显示会话影响。需要 Ubuntu 风格桌面时，在有 DP 物理桌面运行的情况下，让 NoMachine 通过同一个 `4000` 端口连接 physical desktop / physical display 会话即可；下水无头远控仍保留 XFCE 虚拟桌面兜底。

当前不配置第二个 NoMachine 端口 `4001`。NoMachine 这套安装是单 `nxserver` 服务和单配置目录，强行拆第二个服务实例会增加断开无头远控的风险；物理 GNOME 和无头 XFCE 都由 `4000` 入口提供，靠客户端选择会话区分。

已添加登录自启动脚本：

```text
/home/ubuntu2204/smartboat/scripts/smartboat_display_setup.sh
```

该脚本会在检测到 `DP-0` 物理显示器时，把输出固定为 `1920x1080@60Hz`，避免部分显示器在高刷新率下黑屏或远程桌面显示正常但物理屏无信号；同时允许 NoMachine 服务用户 `nx` 读取当前物理 Xorg 会话，方便远程复制 DP 物理桌面输出。

如需在重装宿主机后重新配置 NoMachine + XFCE，可运行：

```bash
cd /home/ubuntu2204/smartboat
sudo bash scripts/setup_nomachine_xfce_host.sh
```

脚本会安装 XFCE/NoMachine，配置 NoMachine 使用 XFCE 虚拟桌面，启用 NoMachine 开机自启，并保持物理桌面使用 `gdm3`/Ubuntu。Jetson 原有 `/etc/X11/xorg.conf` 已包含 `Driver "nvidia"` 和 `AllowEmptyInitialConfiguration`，不要额外强制写死 headless screen，避免影响物理 DP 输出。

## GitHub 打包约定

`~/smartboat` 是 Git 工作区根目录。仓库应只提交源码、ROS 包、launch/config、测试用例、Docker 配方和启动脚本。

仓库包含 `.gitattributes`，统一把文本文件规范为 LF 换行，避免 Windows checkout 后把 `.sh`、Python、Dockerfile、ROS launch/config 改成 CRLF。模型、wheel、压缩包、图片等二进制文件被标记为 binary，不做换行转换。

不提交以下内容：

- `boat_ws/build/`、`boat_ws/install/`、`boat_ws/log/`：colcon 构建、安装和日志产物，可重新生成。
- `__pycache__/`、`*.pyc`、`.pytest_cache/`：Python/pytest 缓存。
- `boat_ws/trt_engine_cache/`：TensorRT engine/timing cache，可重新生成。
- `boat_ws/src/librealsense/`：仅用于本机编译 `pyrealsense2` 的第三方源码树，体积大，不作为项目源码提交。
- `boat_ws/models/*.onnx`、`boat_ws/wheels/*.whl`：运行资产/离线依赖，体积大，普通 GitHub 仓库不适合直接提交；迁移时随 `smartboat-runtime-assets.tar.gz` 单独携带，或改用 Git LFS/Release 资产。

代码交付包按日期命名，日期格式使用本机当前日期 `YYYYMMDD`：

```text
smartboat-github-<YYYYMMDD>-ready.bundle
smartboat-github-<YYYYMMDD>-ready.tar.gz
```

这两个 GitHub 代码包只包含 Git 已跟踪的源码、配置、launch、文档、Docker 文件和脚本；不包含模型、wheel、构建产物、缓存、压缩包或其他运行资产。打包前应确保 `.gitattributes` 仍保持文本文件 `eol=lf`，避免 LF/CRLF 来回转换。

运行资产包单独命名为：

```text
smartboat-<YYYYMMDD>-ruritime-assets.tar.gz
```

该运行资产包仅用于携带 `boat_ws/models/*.onnx` 和 `boat_ws/wheels/*.whl` 等模型/wheel 大文件；不要放入普通 Git 历史。这次只生成 GitHub 代码包时，不生成运行资产包。

在新机器上恢复运行资产后，路径仍保持：

```text
boat_ws/models/ewasr_resnet18.onnx
boat_ws/wheels/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl
```

当前长期保留的 RealSense 融合避障测试脚本在 `boat_ws/scripts`：

```text
boat_ws/scripts/realsense_ewasr_fusion_avoidance_demo.py
boat_ws/scripts/realsense_ewasr_fusion_avoidance_viz_demo.py
```

这两个脚本都自包含预处理、ONNX Runtime session 和融合判断逻辑，不 import 项目内其他 demo。默认模型解析顺序为 `SMARTBOAT_EWASR_MODEL`、容器路径 `/workspace/boat_ws/models/ewasr_resnet18.onnx`、宿主机路径 `/home/ubuntu2204/smartboat/boat_ws/models/ewasr_resnet18.onnx`。

## 1. 当前可运行能力

目前 `src` 和 `scripts` 中已经有实际代码的运行单元主要有：

| 能力 | 包/文件 | 状态 | 说明 |
| --- | --- | --- | --- |
| RealSense + EWaSR 正式感知 | `smartboat_perception/ewasr_fusion_node.py` | 默认一键启动 | D435i采集、EWaSR推理和深度融合，发布正式融合消息；不负责推流 |
| RealSense + EWaSR dashboard | `smartboat_perception/dashboard_stream_node.py` | 默认一键启动 | 独立订阅感知与雷达消息，使用启动时预渲染的中文贴图显示避障状态、中文四宫格标题和雷达目标距离标签，按既有身份和SRT规则推流；故障不影响安全感知 |
| 毫米波雷达 SocketCAN 读取和扇形可视化 | `scripts/Radar.py`、`scripts/bringup_peak_can.sh`、`scripts/install_peak_pcan_driver.sh` | 已验证，可单独运行 | 通过 PEAK PCAN-USB FD 读取 SocketCAN，默认自动选择 `canX`、500K 拉起接口，解析雷达 CAN ID `0x60B` 的 `Obj_1_General`，显示 112 度扇形视场 |
| 水质数据 WebSocket 上传 | `smartboat_drivers/smartboat_drivers/water_quality_node.py` | 默认一键启动 | 通过 PL2303GT/RS-232 读取 YSI EXO，保持既有 WebSocket 协议上传真实水质数据 |
| MAVLink 动态端口转发 | `smartboat_mission/smartboat_mission/mavlink_autoconnect.py` | 已有代码，可运行，依赖宿主机挂载 | 读取船名，请求服务器分配端口，然后执行 `mavlink-routerd` |

整船边缘节点的一键启动入口：

```bash
ros2 launch smartboat_mission edge_nodes.launch.py
```

正式SR73F节点自行选择、配置并重连SocketCAN接口；接口为DOWN时，节点会
独立执行`sudo -n ip link ...`设置500K波特率并拉起接口。它不调用
`Radar.py`或`bringup_peak_can.sh`。`boat_ws/scripts`中的bring-up和可视化
程序只用于人工测试，不参与一键启动。

## 2. 推荐启动流程

### 2.1 进入容器或工作区

宿主机项目根目录：

```bash
cd /home/ubuntu2204/smartboat
```

Docker 启动脚本通常在：

```bash
/home/ubuntu2204/smartboat/scripts/start_smartboat_ros2.sh
/home/ubuntu2204/Desktop/启动智能船ROS2.sh
```

不带参数运行项目脚本会直接启动`edge_nodes.launch.py`及默认开启的生产避障链；
需要只进入容器调试时使用：

```bash
/home/ubuntu2204/smartboat/scripts/start_smartboat_ros2.sh --shell
```

脚本检测到已有整船、MAVLink或避障执行进程时会拒绝重复启动，避免出现两个
执行器输出源。

compose 服务名和容器名在现有环境中通常是：

```text
service: boat_ros2
container: smartboat_ros2_g39
```

进入容器后工作区一般是：

```bash
cd /workspace/boat_ws
```

镜像中需要包含 `iproute2`、`can-utils`与PyPI版`pymavlink==2.4.49`（Jammy arm64软件源没有`python3-pymavlink`这个APT包）。SR73F驱动启动时会使用
`ip link` 自动选择并配置 SocketCAN 接口（默认 500 kbit/s）。如果刚更新过
Dockerfile，需要先重建镜像；旧容器缺少`ip`时雷达接口无法自动拉起，缺少
`pymavlink`时MAVLink必需进程会退出并触发整套launch安全关闭。

图传四象限仪表盘的中文标签使用Pillow和Noto CJK字体，镜像对应安装
`python3-pil`与`fonts-noto-cjk`；缺少字体时中文会显示为方框。

### 2.2 构建

重要：重建 ROS 包时请先进入 Docker 容器，在容器内的 `/workspace/boat_ws` 执行 `colcon build`。不要在宿主机 `/home/ubuntu2204/smartboat/boat_ws` 直接重建 ROS 包，否则容易因为 CMake 缓存路径、ROS 环境和依赖版本不一致导致构建失败。

首次运行或修改代码后执行：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip librealsense2
source install/setup.bash
```

`--symlink-install` 会让 Python 源码修改更容易生效，但新增 console script、launch、config、package 元数据后仍建议重新 build。

工作区内的`boat_ws/src/librealsense/`是本机保留的 RealSense SDK 源码/运行资产，
不属于日常需要由colcon重建的项目包。常规增量或完整工作区构建必须使用
`--packages-skip librealsense2`：否则headless容器会进入其可选GLFW图形组件，常因
缺少RandR开发头文件而中断，并把尚未构建的Smartboat包显示为“取消”。这不表示
被取消的包自身编译失败。只有明确需要重编RealSense SDK时才单独处理该源码树及
其图形依赖。

### 2.3 一键启动边缘节点

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smartboat_mission edge_nodes.launch.py
```

默认控制台日志级别为 `warn`，也就是只输出 warning/error/critical，避免正常运行时刷大量调试信息。需要看详细调试信息时使用：

```bash
ros2 launch smartboat_mission edge_nodes.launch.py debug:=true
```

说明：`ros2 launch` 自带的 `--debug` 更偏向 ROS 2 launch 系统自身调试；本项目业务节点统一使用 `debug:=true` 开启详细日志。

默认会启动：

- `ewasr_fusion`：RealSense D435i + EWaSR 正式安全感知。
- `dashboard_stream`：与安全感知隔离的dashboard/SRT推流。
- `water_quality`：水质上传节点。
- `mavlink_autoconnect.py`：MAVLink 动态端口转发辅助进程。

临时不启动 MAVLink：

```bash
ros2 launch smartboat_mission edge_nodes.launch.py enable_mavlink:=false
```

### 2.4 单独启动节点

单独启动正式D435i/EWaSR感知：

```bash
ros2 run smartboat_perception ewasr_fusion_node
```

单独启动dashboard/SRT推流：

```bash
ros2 run smartboat_perception dashboard_stream_node
```

水质上传：

```bash
ros2 run smartboat_drivers water_quality_node
```

查看该节点详细日志：

```bash
ros2 run smartboat_drivers water_quality_node --ros-args --log-level debug
```

MAVLink helper 当前没有注册成 console script，单独运行：

```bash
python3 /workspace/boat_ws/src/smartboat_mission/smartboat_mission/mavlink_autoconnect.py
```

查看 MAVLink helper 详细日志：

```bash
python3 /workspace/boat_ws/src/smartboat_mission/smartboat_mission/mavlink_autoconnect.py --log-level debug
```

### 2.5 单独启动毫米波雷达

当前现场已验证的雷达链路是：

```text
毫米波雷达 CANH/CANL -> PEAK PCAN-USB FD -> Jetson/Ubuntu SocketCAN -> Docker 容器 -> scripts/Radar.py
```

雷达使用 CAN 波特率 `500000`，当前 `Radar.py` 默认使用 PEAK PCAN-USB FD 的 SocketCAN 模式。程序会自动寻找 `canX` 接口；如果接口已启动就直接连接，如果接口处于 down 状态，会通过 `sudo -n ip link ...` 按 500K bitrate 拉起。维护容器已给运行用户配置免密 sudo，因此不需要用 sudo 启动整个 Python 程序；若宿主机或自建容器没有这项授权，程序会明确报错并退出，可按下文命令手动拉起接口。

正式节点监听标准 CAN ID `0x60A`（扫描状态/数据心跳）和 `0x60B`（厂家手册中的 `Obj_1_General`）。`0x60B`字段解析包括：

| 字段 | 说明 |
| --- | --- |
| `Objects_ID` | 目标 ID |
| `Objects_DistLong` | 目标纵向距离，单位 m |
| `Objects_DistLat` | 目标横向距离，单位 m |
| `Objects_VrelLong` | 目标纵向相对速度，单位 m/s |
| `Objects_VrelLat` | 目标横向相对速度，单位 m/s |
| `Objects_DynProp` | 目标动态属性 |
| `Objects_RCS` | 目标 RCS |

调试脚本默认显示为112度扇形视场。正式ROS节点 `sr73f_radar_node` 只接收原始雷达距离3-10m、正前方左右各56度的目标，并使用雷达到船头的0.45m安装偏移发布船头到障碍物的距离。该雷达没有高度信息，因此只能提供2D水平目标，桥洞净高仍由D435i判断。

ROS节点发布：

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/smartboat/perception/radar/closest_obstacle_distance` | `std_msgs/msg/Float32` | 最近目标到船头距离；收到新鲜扫描但本轮无目标为`-1.0`；雷达零帧时停止发布以触发安全超时 |
| `/smartboat/perception/radar/objects` | `std_msgs/msg/String` | 目标列表JSON，包含位置、相对速度、RCS、TTC |
| `/smartboat/perception/radar/status` | `std_msgs/msg/String` | CAN连接、数据新鲜度、接口、状态/目标帧数和目标数 |

单独启动：

```bash
ros2 launch smartboat_drivers sr73f_radar.launch.py
```

如果当前 Jetson 内核没有内置 PEAK USB-CAN 驱动，先在宿主机安装官方 PEAK 驱动：

```bash
cd /home/ubuntu2204/smartboat/boat_ws
sudo bash scripts/install_peak_pcan_driver.sh
```

USB 盒子重新插拔后，CAN 网口通常会回到 `DOWN/STOPPED`。`Radar.py` 启动时会尝试自动拉起接口；也可以手动执行：

```bash
cd /home/ubuntu2204/smartboat/boat_ws
sudo bash scripts/bringup_peak_can.sh 500000
```

脚本会自动寻找 PEAK 对应的 `canX` 接口并打印接口名。当前现场常见为 `can2`；`can0/can1` 多半是 Jetson 自带 mttcan。

容器内启动雷达扇形窗口：

```bash
cd /workspace/boat_ws
python3 /workspace/boat_ws/scripts/Radar.py
```

常用调试方式：

```bash
python3 /workspace/boat_ws/scripts/Radar.py --debug
python3 /workspace/boat_ws/scripts/Radar.py --headless --once
python3 /workspace/boat_ws/scripts/Radar.py --headless --once --save /tmp/radar_sector.png
python3 /workspace/boat_ws/scripts/Radar.py --socketcan can2
```

当前保留的参数很少：

| 参数 | 说明 |
| --- | --- |
| `--socketcan can2` | 手动指定 SocketCAN 接口；默认自动选择 |
| `--headless` | 不打开 OpenCV 窗口 |
| `--save <path>` | 保存当前扇形图 |
| `--once` | 读取和渲染一次后退出 |
| `--debug` | 打印解析后的目标字段 |

图形窗口需要 X11 显示权限。推荐通过桌面脚本进入容器，它会传递 `DISPLAY`；手动 `docker exec` 时不要用 root 用户跑 GUI，否则容易出现 OpenCV GTK/X11 授权错误。没有 GUI 时用 `--headless --once --save /tmp/radar_sector.png` 验证。

更完整的现场命令和故障排查见：

```text
boat_ws/scripts/README.md
```

## 3. 目录和包状态

规则：只有文件夹、没有可安装包元数据或业务代码的目录，视为**待开发**。有 `package.xml/setup.py` 但没有业务节点的包，视为**骨架/保留**。

| 路径 | 状态 | 职责 | 备注 |
| --- | --- | --- | --- |
| `smartboat_perception` | 已有代码 | D435i、图像、AI感知、dashboard推流 | 已有正式融合节点和独立推流节点 |
| `smartboat_drivers` | 已有代码 | 传感器、硬件驱动、外设客户端 | 已有 `water_quality_node.py` 和 `sr73f_radar_node.py` |
| `smartboat_mission` | 已有代码 | 整船级 launch、任务编排、边缘节点组合启动 | 已有 `edge_nodes.launch.py`、`edge_nodes.yaml`、`mavlink_autoconnect.py` |
| `smartboat_bringup` | 骨架/保留 | 早期临时bringup和旧代码归档 | 旧camera-only PID在`legacy/avoidance_pid`，不参与安装运行 |
| `smartboat_msgs` | 已有代码 | 自定义 `msg` / `srv` / `action` | 已定义融合检测、雷达目标和雷达状态消息 |
| `smartboat_px4_bridge` | 已有代码 | PX4、MAVLink、飞控桥接 | 已有recommendation链路使用的桥接节点，硬件输出由参数门控 |
| `smartboat_navigation` | 已有代码 | 导航、路径规划、航点跟踪 | 安全评估监督器和生产避障执行状态机均默认一键启动 |
| `smartboat_localization` | 待开发 | 定位、坐标变换、状态估计 | 当前仅骨架目录 |
| `smartboat_control` | 已有代码 | 控制算法、执行器控制、底层运动控制 | 保留yaw读取节点；旧avoidance PID已归档 |
| `smartboat_tools` | 待开发 | 调试脚本、离线工具、运维辅助 | 当前仅骨架目录 |
| `scripts/realsense_ewasr_fusion_avoidance_demo.py` | 测试脚本 | RealSense D435i + EWaSR 融合避障命令行 demo | 自包含版本，使用 `boat_ws/models/ewasr_resnet18.onnx` |
| `scripts/realsense_ewasr_fusion_avoidance_viz_demo.py` | 测试脚本 | RealSense D435i + EWaSR 图形化融合避障 demo | 自包含版本，使用 `boat_ws/models/ewasr_resnet18.onnx` |
| `boat_ws/models/ewasr_resnet18.onnx` | 运行资产 | 语义分割/感知模型 | 不进普通 Git；通过运行资产包或 Git LFS/Release 携带 |
| `boat_ws/wheels/*.whl` | 运行资产 | Jetson/ARM64 离线 Python wheel | Docker build 需要，随运行资产包携带 |

## 4. 已有包详细说明

### 4.1 `smartboat_perception`

职责：

- 摄像头接入。
- 图像流处理。
- AI 感知推理。
- 感知相关模型和参数。

主要正式文件：

```text
smartboat_perception/
  package.xml
  setup.py
  setup.cfg
  resource/smartboat_perception
  config/
  smartboat_perception/
    __init__.py
    ewasr_core.py
    fusion_model.py
    realsense_source.py
    ewasr_fusion_node.py
    dashboard_pipeline.py
    dashboard_stream_node.py
```

#### `ewasr_fusion_node.py`

该节点是正式安全感知入口，负责：

- D435i depth/color采集和断线重连；
- EWaSR ONNX Runtime/TensorRT推理；
- 对齐深度与语义mask融合；
- 发布`smartboat_msgs/FusionDetection`；
- 每两帧推理只发布一次同时间戳的color/depth/mask显示输入，即第1帧用于
  dashboard、第2帧跳过、第3帧再用于dashboard；安全检测结果仍逐帧发布。

正式融合topic：

```text
/smartboat/perception/fusion/detection
```

#### `dashboard_stream_node.py`

该节点不接触相机硬件，也不发布安全决策。它按推理时间戳精确配对
color/depth/mask和检测结果，完整显示帧到达后才触发后台合成，不使用固定
15Hz定时器。合成与FFmpeg写入运行在latest-only线程中；如果编码或网络短暂
变慢，只保留最新待处理帧，不阻塞ROS回调和安全推理。FFmpeg使用采集时钟和
VFR时间戳，允许dashboard帧率随推理与编码负载自然波动。停止该节点不会停止
D435i/EWaSR感知。

连接远程服务器的规则保持：

```text
srt://<server>:<port>?streamid=publish:<station>/<boat>&pkt_size=1316&mode=caller
```

`identity_dir`仍为`/host_boot`，服务器、端口、身份默认值和现有凭据不变。

### 4.2 `smartboat_drivers`

职责：

- 传感器驱动。
- 硬件外设读写。
- 与传感器相关的边缘客户端。

当前文件：

```text
smartboat_drivers/
  package.xml
  setup.py
  setup.cfg
  resource/smartboat_drivers
  config/
  launch/
  smartboat_drivers/
    __init__.py
    water_quality_node.py
```

#### `water_quality_node.py`

ROS 节点名：

```text
water_quality
```

console script：

```text
water_quality_node = smartboat_drivers.water_quality_node:main
```

作用：

1. 从 `identity_dir` 读取：
   - `name.txt`。
   - `boat_name.txt`。
2. 连接 WebSocket：

   ```text
   ws://<server_host>:<server_port>/uav/<station>/<boat>
   ```

3. 周期性读取 `SensorProvider`。
4. 发送 JSON：

   ```json
   {
     "timestamp": 1710000000.0,
     "station": "test",
     "boat": "test01",
     "data": [
       {"name": "温度（°C）", "value": 22.5},
       {"name": "pH（无量纲）", "value": 7.2}
     ]
   }
   ```

5. 发送失败后按 `reconnect_delay_sec` 重连。
6. 单次水质发送失败视为正常网络波动，仅在 debug 日志中记录；连续失败超过 2 次后开始输出 warning。

当前数据源：

```text
ExoSensorProvider -> TelemetryClient
```

串口协议和采集线程在 `exo_serial.py` 中，WebSocket 连接、身份读取、时间戳和 JSON 外层结构仍由 `TelemetryClient` 负责。两者通过 `SensorProvider` 接口注入；服务器断线不会停止串口采集，串口采集失败也不会主动断开已建立的 WebSocket。

当前 EXO 配置上传的真实字段：

| 字段名 | 说明 |
| --- | --- |
| `叶绿素（μg/L）` | EXO 参数 193，荧光法叶绿素估算值 |
| `电导率（μS/cm）` | EXO 参数 5，未补偿到 25°C 的原始电导率 |
| `溶解氧（mg/L）` | 溶解氧浓度 |
| `铵态氮（mg/L-N）` | EXO 参数 48，NH4 离子选择电极读数 |
| `总藻-PC（cells/mL）` | EXO 参数 215，TAL-PC 相关模型估算，不是直接细胞计数 |
| `浊度（NTU）` | EXO 参数 37，保留传感器原始读数，不在上传层强制归零 |
| `温度（°C）` | 水温 |
| `pH（无量纲）` | 酸碱度 |

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `server_host` | `111.229.242.175` | WebSocket 服务器 |
| `server_port` | `11452` | WebSocket 端口 |
| `identity_dir` | `/host_boot` | 身份文件目录 |
| `station_default` | `test` | `name.txt` 缺失或为空时使用 |
| `boat_default` | `test01` | `boat_name.txt` 缺失或为空时使用 |
| `publish_period_sec` | `1.0` | 上传周期 |
| `reconnect_delay_sec` | `5.0` | 断线后重连等待时间 |
| `serial_port` | `auto` | 自动枚举串口，不依赖会变化的 `/dev/ttyUSB*` 编号 |
| `serial_number` | `CRADb2A6710` | 与 `067b:23a3` 一起唯一选中 EXO 的 PL2303GT 适配器 |
| `serial_baud_rate` | `9600` | EXO RS-232 波特率，8-N-1、无流控 |
| `sensor_sample_rate_hz` | `0.5` | `setperiod 2000` 对应的采样率 |
| `expected_parameter_codes` | `[5, 1, 18, 212, 48, 193, 215, 37]` | 必须与 EXO `para` 返回的顺序严格一致 |
| `sensor_startup_timeout_sec` | `10.0` | EXO 上电后命令模式等待时间 |
| `sensor_reconnect_delay_sec` | `5.0` | 串口掉线后独立重连等待 |
| `sensor_sample_max_age_sec` | `5.0` | 允许上传的最新样本最大年龄 |
| `sensor_sample_wait_timeout_sec` | `20.0` | 等待可用真实样本的超时 |

宿主机必须先加载与当前 Jetson 内核匹配的 `pl2303` 模块并生成 `/dev/ttyUSB*`。串口测试和 ROS 2 运行均在容器内进行；compose 已通过 `/dev:/dev` 和 `dialout` 组提供设备访问。

ROS topic：

| 方向 | Topic | Message | 当前状态 |
| --- | --- | --- | --- |
| publish | 无 | 无 | 当前不发布 ROS 水质 topic，只上传 WebSocket |
| subscribe | 无 | 无 | 当前不订阅 ROS topic |

外部输入/输出：

| 类型 | 地址/路径 | 说明 |
| --- | --- | --- |
| 输入文件 | `/host_boot/name.txt` | 站点/设备名 |
| 输入文件 | `/host_boot/boat_name.txt` | 船名 |
| 输出网络 | `ws://111.229.242.175:11452/uav/<station>/<boat>` | 水质上传 WebSocket |

维护建议：

- 真实传感器接入时，优先保留 `SensorProvider` 抽象：采集逻辑放 provider，上传逻辑继续由 `TelemetryClient` 负责。
- 如果要给 ROS 内部其他节点使用水质数据，应在 `smartboat_msgs/msg` 中定义 `WaterQuality` 或类似 message，再由此节点同步 publish。
- `publish_period_sec` 不要写死在代码里，继续通过 YAML 或 launch 参数覆盖。

### 4.3 `smartboat_mission`

职责：

- 整船级启动。
- 多节点编排。
- 边缘侧运行参数集中管理。
- 不直接承载具体传感器/控制/导航算法。

当前文件：

```text
smartboat_mission/
  package.xml
  setup.py
  setup.cfg
  resource/smartboat_mission
  config/
    edge_nodes.yaml
  launch/
    edge_nodes.launch.py
  smartboat_mission/
    __init__.py
    mavlink_autoconnect.py
```

#### `launch/edge_nodes.launch.py`

一键启动文件：

```bash
ros2 launch smartboat_mission edge_nodes.launch.py
```

启动内容：

| 类型 | 名称 | 来源 | 说明 |
| --- | --- | --- | --- |
| `Node` | `water_quality` | `smartboat_drivers/water_quality_node` | 水质上传 |
| `Node` | `ewasr_fusion` | `smartboat_perception/ewasr_fusion_node` | D435i + EWaSR正式安全感知，Jetson默认使用TensorRT FP16 |
| `Node` | `dashboard_stream` | `smartboat_perception/dashboard_stream_node` | 独立dashboard/SRT推流 |
| `ExecuteProcess` | `mavlink_autoconnect` | `smartboat_mission/mavlink_autoconnect` | 从包安装目录启动的非ROS辅助进程；异常退出后每5秒重启，不关闭其他节点 |
| `Node` | `sr73f_radar` | `smartboat_drivers/sr73f_radar_node` | 自行配置SocketCAN并读取3-10m前向雷达目标 |
| `Node` | `avoidance_supervisor` | `smartboat_navigation/avoidance_supervisor_node` | 雷达+D435i动态安全监督 |
| `Node` | `avoidance_executor` | `smartboat_navigation/avoidance_executor_node` | Mission/Manual模式避障执行状态机 |
| `Node` | `cmd_vel_to_mavlink` | `smartboat_px4_bridge/cmd_vel_to_mavlink_node` | 唯一的激活门控PX4执行桥接和输入源恢复 |

launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `debug` | `false` | 是否把正式边缘节点日志级别切到`debug`；默认统一为`warn` |
| `enable_mavlink` | `true` | 是否启动 MAVLink helper |
| `enable_radar` | `true` | 是否启动SR73F SocketCAN雷达节点 |
| `enable_avoidance_supervisor` | `true` | 是否启动新的雷达+D435i避障监督节点 |
| `enable_avoidance_executor` | `true` | 是否启动Mission/Manual生产避障执行状态机 |
| `enable_px4_cmd_bridge` | `true` | 是否启动唯一的激活门控PX4执行桥接；生产配置默认允许避障输出 |
| `enable_ewasr_perception` | `true` | 是否启动D435i/EWaSR正式安全感知 |
| `enable_dashboard_stream` | `true` | 是否独立启动dashboard SRT推流；关闭不会停止安全感知 |
| `dashboard_stream_server` | `111.229.242.175` | dashboard SRT服务器IP |
| `dashboard_stream_port` | `10000` | dashboard SRT服务器端口 |
| `dashboard_stream_suffix` | 空 | 默认保持`publish:<station>/<boat>` |
| `dashboard_stream_fps` | `15` | FFmpeg名义帧率和GOP基准；实际输出由每隔一帧到达的完整推理显示包触发，允许波动，不强制补帧到15FPS |
| `dashboard_stream_width` | `1280` | dashboard画面宽度 |
| `dashboard_stream_height` | `960` | dashboard画面高度 |
| `dashboard_stream_bitrate` | `1800k` | dashboard H.264码率 |

注意：`edge_nodes.launch.py` 通过 `FindPackageShare('smartboat_mission')` 加载安装后的 `config/edge_nodes.yaml`。修改 YAML 后建议重新 build 并重新 source。

#### `config/edge_nodes.yaml`

集中配置当前边缘节点参数：

```yaml
water_quality:
  ros__parameters:
    server_host: "111.229.242.175"
    server_port: 11452
    identity_dir: "/host_boot"
    station_default: "test"
    boat_default: "test01"
    publish_period_sec: 1.0
    reconnect_delay_sec: 5.0
    serial_port: "auto"
    serial_number: "CRADb2A6710"
    serial_baud_rate: 9600
    sensor_sample_rate_hz: 0.5
    expected_parameter_codes: [5, 1, 18, 212, 48, 193, 215, 37]
    sensor_startup_timeout_sec: 10.0
    sensor_reconnect_delay_sec: 5.0
    sensor_sample_max_age_sec: 5.0
    sensor_sample_wait_timeout_sec: 20.0
```

后续新增边缘节点时，如果它属于一键启动的一部分，建议把运行参数也放进这个 YAML 或按功能拆成独立 YAML 后由 launch 引入。

#### `mavlink_autoconnect.py`

作用：

1. 从以下路径依次读取船名：

   ```text
   /boot/name.txt
   /host_boot/name.txt
   ```

2. 请求服务器：

   ```text
   http://111.229.242.175:11451/connect?name=<name>
   ```

3. 期望返回 JSON 中包含：

   ```json
   {
     "success": true,
     "data": {
       "port": 12345
     }
   }
   ```

4. 检查配置文件：

   ```text
   /etc/mavlink-router/main.conf
   ```

5. 执行：

   ```bash
   /usr/bin/mavlink-routerd -c /etc/mavlink-router/main.conf -r -p 111.229.242.175:<port>
   ```

   Python wrapper 会把 `mavlink-routerd` 的 stdout/stderr 静音，避免 endpoint 统计信息持续刷屏；端口申请、配置缺失等 wrapper 自身错误仍会按 `--log-level` 输出。启动后 wrapper 会通过本地 `tcp:127.0.0.1:5760` 等待 PX4 `HEARTBEAT`；如果容器能看到 `/dev/ttyACM0` 但收不到 MAVLink 数据，会提示硬件重新插拔 PX4 飞控 USB 并退出。主 launch 把它作为非ROS辅助进程启动，异常退出后每5秒重试，不会关闭雷达、D435i感知、dashboard或避障监督/执行节点；yaw和PX4桥接仍按各自的失联保护逻辑工作。

依赖：

- `python3-requests`
- 容器中可访问 `/usr/bin/mavlink-routerd`
- 容器中可访问 `/etc/mavlink-router/main.conf`
- 运行 `mavlink-routerd` 的环境有权限访问飞控串口；ROS 2 yaw 节点不直接访问 `/dev/ttyACM0`，只连接 `tcp:127.0.0.1:5760`

命令行参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--log-level` | `warning` | 控制 helper 自身控制台日志，可选 `debug/info/warning/error/critical` |
| `--heartbeat-timeout-sec` | `8.0` | 启动后等待本地 `tcp:127.0.0.1:5760` 出现 PX4 `HEARTBEAT` 的秒数，超时会提示重新插拔 PX4 飞控 USB 并退出 |

维护建议：

- 当前 helper 不使用 ROS topic，也不是 `rclpy.Node`。
- 如果要做 ROS 化 MAVLink 桥接，建议在 `smartboat_px4_bridge` 中新增包和节点，再由 `smartboat_mission` launch。
- 宿主机上的 `autoconnect.service` 和 `mavlink-router.service` 应保持 disabled/stopped，避免和 ROS 2 launch 重复拉起、抢占串口或默认 TCP 端口。

### 4.4 `smartboat_bringup`

状态：骨架/保留。

当前有：

```text
package.xml
setup.py
setup.cfg
resource/smartboat_bringup
smartboat_bringup/__init__.py
test/
```

没有 console script，也没有业务节点。早期迁移代码已经移动到更明确的包：

- 摄像头：`smartboat_perception`
- 水质：`smartboat_drivers`
- 编排：`smartboat_mission`

维护约定：

- 不要继续把新业务代码放进 `smartboat_bringup`。
- 如果后续保留它，建议只放非常薄的 bringup launch。
- 如果确认没有引用，可以后续清理删除。

## 5. 待开发包边界

### 5.1 `smartboat_msgs`

职责：统一定义自定义接口。

建议结构：

```text
smartboat_msgs/
  msg/
  srv/
  action/
```

当前没有 `.msg`、`.srv`、`.action` 文件，也没有 `package.xml/CMakeLists.txt`，因此还不是一个可构建 ROS 2 interface package。

后续建议优先补齐的 message：

| 建议 message | 用途 |
| --- | --- |
| `WaterQuality.msg` | ROS 内部传递水质数据 |
| `BoatIdentity.msg` | 发布当前站点/船只身份 |
| `CameraStreamStatus.msg` | 摄像头推流状态 |
| `MavlinkConnectionStatus.msg` | MAVLink 端口和连接状态 |
| `MissionState.msg` | 任务状态 |

新增自定义接口时应使用 `ament_cmake` interface package，而不是 `ament_python`。

### 5.2 `smartboat_px4_bridge`

职责：PX4/MAVLink/飞控桥接。

当前 `cmd_vel_to_mavlink_node.py` 订阅`/smartboat/control/cmd_vel`，通过
MAVLink `SET_POSITION_TARGET_LOCAL_NED`向PX4 1.16差速Rover发送body-NED二维
速度向量。PX4差速/Ackermann Rover会忽略MAVLink yaw/yaw-rate，因此bridge把
ROS `angular.z`转换成受限的`vx/vy`方向偏角供PX4推导转向。节点只接受有效PX4
autopilot HEARTBEAT，可自动选择
目标系统或通过`target_system_id`固定目标；模式、解锁状态、HEARTBEAT新鲜度、
避障激活心跳、命令超时和`output_enabled`共同控制是否允许发送。

实体遥控器始终是最高优先级人工输入。生产节点启动时只会把遗留的
`COM_RC_IN_MODE=1`恢复为`2`，运行期间永远不会选择MAVLink-only输入；代码和
生产配置的`mavlink_control_input_mode`均固定为`2`。因此Jetson、bridge或
MAVLink流中断不会再因为持久参数把RC禁用。AUTO.MISSION避障先请求
`AUTO.LOITER`停车，同时以20Hz预热Offboard setpoint至少1.1秒，再进入
`OFFBOARD`执行减速、绕行或停止。若PX4在4秒内未接受Offboard，bridge会停止
重试，回到MANUAL（手动起点）或保持HOLD（任务起点）并锁存人工优先，避免旧
模式请求在很久后意外生效。危险稳定解除后，只有未发生人工接管、Kill、
定位回落或通信故障时才恢复原任务。

从AUTO/HOLD/OFFBOARD切到MANUAL会立即锁存人工接管并停止Offboard输出；仅
Disarm/Rearm不会解除锁存，只有操作员之后明确执行MANUAL→AUTO.MISSION模式切换
才重新允许任务自动化。正常从MANUAL开始的碰撞辅助仍可使用，但只有
实测油门/转向在中位窗口内连续保持0.3秒才允许介入；介入后任一关键摇杆偏离
实测中位超过80us都会立即请求MANUAL并锁存人工优先。介入窗口为±100us，和
夺权阈值分开，摇杆中位的小偏差不会永久阻止交接。本船实测CH1是转向、CH3是
前后，两者回中约1514us；CH7 Kill回中约1514us、拨到底约1933us。中位条件
从不用于阻止人工夺权。
本接收机健康链路的`RC_CHANNELS.rssi`固定报告255（未知），因此255不再视为
失联；RSSI为0、通道无效或RC消息过期仍按不可用处理。辅助期间任一主摇杆动作
都会退出到MANUAL；若RC流本身失效，任务来源的辅助会退出到HOLD而不会假装已把
控制权交给一个不存在的遥控链路。

停车命令先在Offboard中持续发送零速度制动。只有新鲜GPS水平合速度
`hypot(vx, vy)`严格小于0.2m/s时才视为已停，并退出PX4零速闭环：MANUAL来源
回到MANUAL，AUTO.MISSION来源回到HOLD。速度等于或高于0.2m/s、地速无效或
地速过期时都不会提前宣告停车；这样可避免零速度闭环仍保留非中位电机输出。

GPS/地速异常会直接请求MANUAL并把控制权留给已启用的RC，不再等待摇杆完美归中。
执行器激活心跳丢失时不会自动恢复任务：AUTO或OFFBOARD会请求HOLD。生产bridge
仍是唯一允许发送执行器MAVLink消息的节点；所有关键状态变化异步追加到
`boat_ws/log/smartboat_px4_bridge_events.jsonl`，包含模式、Armed、RC通道、
Kill、回落阶段和Offboard阶段，便于事故复盘。
生产与岸测bridge还共享`/tmp/smartboat_px4_bridge_output.lock`进程锁；已有一个
`output_enabled=true`节点时，第二个输出节点会直接启动失败。进程崩溃或被停止
时锁由内核自动释放，避免重复节点同时请求PX4模式和setpoint。

本船PX4还应保持`MAV_FWDEXTSP=1`、`COM_OF_LOSS_T=1.0`和
`COM_OBL_RC_ACT=5`：分别允许Rover外部setpoint，并在Offboard数据流丢失1秒后
进入HOLD。2026-07-18实机已读回确认这三个值为`1/1.0/5`。

最高优先级的`/smartboat/control/emergency_kill`（`std_msgs/msg/Bool`）和PX4
`TERMINATION`可请求Disarm与MANUAL。物理Kill Switch保持由PX4直接负责：bridge
检测到Kill后立即停止Offboard setpoint并保持`COM_RC_IN_MODE=2`。实船验证发现
PX4的Kill `STATUSTEXT`不保证转发到每个mavlink-router连接，因此生产bridge以
`RC_CHANNELS`的CH7为主检测源：1850以上触发、1700以下解除，文本通知仅作为
备用；本船实测回中约1514、拨到底约1933。若Kill在
`AUTO.MISSION`中触发，bridge会额外锁存任务逃生流程并重复请求`Disarm`和
`MANUAL`。其中Disarm使用MAVLink规定的紧急强制参数，仅限这个已触发物理Kill的
任务逃生分支，避免运动状态让普通Disarm被拒绝；只有波轮已解除、
RC输入模式已恢复、飞控已处于`MANUAL`且已Disarm后才清除软件锁存。
此后由操作员主动重新Arm并手动驶离，原航点任务不会自动恢复。若Kill原本在
`MANUAL`中触发，则保持原有行为：只等待波轮解除和RC输入源恢复，不额外强制
Disarm。各项确认事件无论以何种顺序到达都不会提前清锁。该软件链路不替代物理
Kill本身。

第一次岸测使用独立配置：

```text
boat_ws/src/smartboat_px4_bridge/config/cmd_vel_to_mavlink_shore_test.yaml
boat_ws/src/smartboat_px4_bridge/launch/cmd_vel_to_mavlink_shore_test.launch.py
```

该配置固定`target_system_id: 1`、要求飞控已解锁、命令超时为0.3秒，并把
Offboard合速度和输入转向率分别硬限制为0.1m/s和0.1rad/s，并把方向偏角限制为
0.35rad。岸测同样不得把
`COM_RC_IN_MODE`切到1；实体RC保持可用，通过MANUAL/OFFBOARD模式变化验证
人工夺权。岸测节点名
为 `cmd_vel_to_mavlink_shore_test`，只订阅独立的
`/smartboat/control/shore_test_cmd_vel`，状态发布到独立的
`/smartboat/px4_bridge/shore_test_status`，不会与生产桥接节点同名或误收生产
控制topic。配置文件本身和launch默认都不启用输出。安全检查完成后才可显式
启动：

```bash
ros2 launch smartboat_px4_bridge cmd_vel_to_mavlink_shore_test.launch.py \
  enable_output:=true
```

启动真实输出前必须保持急停可用、船体固定、推进器处于厂家允许的岸测状态，
并确保容器内只有一个 `mavlink-routerd` 读取PX4串口。岸测launch复用已经运行的
Router，不要再次启动 `mavlink_autoconnect`。确认
`/smartboat/px4_bridge/shore_test_status` 中 `target_system=1`、
`flight_mode=MANUAL`、`armed=True`、`mode_allowed=True`。不要把
`/smartboat/navigation/avoidance_cmd_vel` 直接重映射到控制topic，也不要同时
启动生产bridge和岸测bridge。

后续桥接功能继续放在此包：

- MAVLink 消息收发。
- PX4 状态解析。
- 飞控串口/UDP/TCP 桥接。
- 与导航、控制节点之间的 ROS topic/service/action。

不要把复杂飞控桥接逻辑长期留在 `smartboat_mission`。

### 5.3 `smartboat_navigation`

职责：路径规划、航点跟踪、避障策略。

`avoidance_supervisor_node.py`负责动态安全距离和传感器安全评估。它订阅：

- `/smartboat/localization/ground_speed`
- `/smartboat/perception/radar/closest_obstacle_distance`
- `/fusion_obstacle_detected`
- `/fusion_obstacle_distance`
- `/fusion_avoidance_decision`

并发布：

- `/smartboat/navigation/avoidance_speed_limit`
- `/smartboat/navigation/emergency_stop_requested`
- `/smartboat/navigation/avoidance_supervisor_status`

监督器发布 `/smartboat/navigation/avoidance_cmd_vel` 作为评估建议，本身不直接
发送MAVLink。`avoidance_executor_node.py`消费监督器状态、建议、有效地速以及
bridge发布的PX4模式/解锁状态，并输出唯一的
`/smartboat/control/cmd_vel`和`/smartboat/control/avoidance_active`。

执行策略：

- `AUTO.MISSION`保持原模式。雷达预警时低速前进，D435i给出明确左右方向时以
  低功率绕行；`EMERGENCY_STOP`、`SENSOR_FAULT`或无安全转向方向时发中位停车。
  清障稳定1秒后恢复遥控输入源，PX4继续航点任务。
- `MANUAL`只有在3D定位/地速有效时才允许接管；检测到前向危险后先发中位
  减速，并要求水平地速严格低于0.2m/s后保持停车。当前没有后向安全传感器，因此生产
  配置禁用自动倒车，绝不主动发送正向油门。
  单次接管最多5秒，超时强制归还RC并锁定；必须先观测到未解锁，再次解锁后
  才允许下一次避障。超时交接会先发送一次中位，再立即开始恢复RC输入源；
  传感器故障只停车，绝不在感知无效时盲退。
- 执行器或激活心跳超时时，`AUTO.MISSION`按fail-closed发中位停车；`MANUAL`
  则释放MAVLink输入源并归还遥控器，避免软件故障永久屏蔽人工控制。

配置中的 `require_radar: true` 会在雷达topic超时时报告 `SENSOR_FAULT`。SR73F
驱动同时监听`0x60A`扫描状态帧与`0x60B`目标帧：只有近期确实收到雷达帧且
当前3-10m窗口没有目标时才发布`-1.0`有效空扫描；接口打开但雷达断电/零帧时
不会假报路径安全。窗口外的过近/过远目标直接丢弃；窗口内同一目标ID还必须在
0.5秒内由实际`0x60B`帧连续确认3次，单帧毫米波杂波不进入监督器。视觉分割区
若完全没有有效D435i深度则保持忽略，不能仅凭无效深度推断障碍很近；只要存在
有效深度，仍使用近端分位数避免中位数掩盖小型近障碍。

当前安全模型使用第一轮水池实测数据：

| 项目 | 实测/配置值 |
| --- | --- |
| 1.506m/s 反推停车 | 4.800s、2.963m |
| 1.257m/s 左转 | 平均35.72deg/s、半径2.016m |
| 巡航/避障速度 | 1.5m/s、0.7m/s |
| 保守动态停车距离 | 1.5m/s约3.9m；0.7m/s约1.35m |
| 雷达减速距离 | 6.0m |

`118.57deg/s` 的95分位偏航角速度暂按遥测尖峰处理；在左右转重复测量前，控制上限暂定30deg/s。配置文件：

```text
boat_ws/src/smartboat_navigation/config/avoidance_supervisor.yaml
boat_ws/src/smartboat_navigation/config/avoidance_executor.yaml
```

单独启动：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smartboat_navigation avoidance_supervisor.launch.py
ros2 launch smartboat_navigation avoidance_executor.launch.py
```

后续继续放：

- 航点任务 action client/server。
- 全局/局部规划。
- 航迹生成。
- 对 `smartboat_control` 输出期望速度、航向或路径。

### 5.4 `smartboat_localization`

职责：定位和状态估计。

建议后续放：

- GPS/RTK 数据融合。
- IMU 数据融合。
- 坐标系转换。
- `tf2` frame 发布。
- 船体状态估计。

### 5.5 `smartboat_control`

职责：控制算法和执行器控制。

当前保留两个yaw读取节点：

- `yaw_from_mavlink_node.py`：只接受目标PX4 autopilot sysid的消息，读取
  `ATTITUDE.yaw`并发布`/smartboat/localization/yaw`。只有近期
  `GPS_RAW_INT.fix_type>=3`且速度小于配置的物理上限时，才发布
  `/smartboat/localization/ground_speed`和有效标志；无GPS时数值不再冒充有效地速。
- `yaw_from_imu_node.py`：订阅 MAVROS IMU 四元数，计算船体 yaw，发布 `/smartboat/localization/yaw`。

旧的camera-only `avoidance_pid_node.py`、配置和launch已经移到：

```text
boat_ws/src/smartboat_bringup/legacy/avoidance_pid/
```

它们只作历史参考，不再安装，也不再由一键启动调用。新的动态安全距离、雷达减速和D435i转向建议由 `smartboat_navigation/avoidance_supervisor_node.py` 负责。

主要配置文件：

```text
boat_ws/src/smartboat_control/config/yaw_from_mavlink.yaml
boat_ws/src/smartboat_control/config/yaw_from_imu.yaml
```

单独启动：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smartboat_control yaw_from_mavlink.launch.py
ros2 launch smartboat_control yaw_from_imu.launch.py
```

D435i具备双目和IMU，可作为后续VIO后端的输入，但当前正式节点没有位姿估计、
相机—IMU外参标定或`nav_msgs/Odometry`输出。无GPS时目前按速度无效停车；接入并
验证RTAB-Map/VINS/ORB-SLAM3等VIO后，可把有效odometry接入定位层作为临时速度
来源，不需要修改避障执行状态机。

### 5.6 `smartboat_tools`

职责：开发、测试、排障、离线转换工具。

建议迁入：

- `benchmark.py`
- `check_onnx_backends.py`
- `check_onnx_backends_safe.py`
- 数据录制/回放脚本。
- 设备连通性检查脚本。

## 6. Topic、Message、Service、Action 现状和规范

### 6.1 当前现状

当前已有运行节点和测试脚本的 ROS topic 通信：

| 节点 | publish | subscribe | service | action |
| --- | --- | --- | --- | --- |
| `camera_stream` | 无 | 无 | 无 | 无 |
| `water_quality` | 无 | 无 | 无 | 无 |
| `mavlink_autoconnect.py` | 无 | 无 | 无 | 无 |
| `realsense_ewasr_fusion_avoidance_demo.py` | `/fusion_obstacle_detected`、`/fusion_obstacle_distance`、`/fusion_avoidance_decision` | 无 | 无 | 无 |
| `realsense_ewasr_fusion_avoidance_viz_demo.py` | `/fusion_obstacle_detected`、`/fusion_obstacle_distance`、`/fusion_avoidance_decision` | 无 | 无 | 无 |
| `yaw_from_mavlink` | `/smartboat/localization/yaw`、`/smartboat/localization/ground_speed` | MAVLink TCP `tcp:127.0.0.1:5760` | 无 | 无 |
| `yaw_from_imu` | `/smartboat/localization/yaw` | `/mavros/imu/data` | 无 | 无 |
| `sr73f_radar` | `/smartboat/perception/radar/closest_obstacle_distance`、`/smartboat/perception/radar/objects`、`/smartboat/perception/radar/status` | SocketCAN `0x60B` | 无 | 无 |
| `avoidance_supervisor` | `/smartboat/navigation/avoidance_speed_limit`、`/smartboat/navigation/avoidance_cmd_vel`、`/smartboat/navigation/emergency_stop_requested`、`/smartboat/navigation/avoidance_supervisor_status` | 雷达最近距离、视觉融合三topic、地速 | 无 | 无 |

当前没有自定义 message/service/action 文件：

```text
smartboat_msgs/msg/      # 空
smartboat_msgs/srv/      # 空
smartboat_msgs/action/   # 空
```

### 6.2 命名建议

ROS topic 建议按功能域分层：

| 功能 | 建议 topic | 建议 message |
| --- | --- | --- |
| 水质数据 | `/smartboat/water_quality` | `smartboat_msgs/msg/WaterQuality` |
| 摄像头状态 | `/smartboat/camera/status` | `smartboat_msgs/msg/CameraStreamStatus` |
| MAVLink 连接状态 | `/smartboat/mavlink/status` | `smartboat_msgs/msg/MavlinkConnectionStatus` |
| 船只身份 | `/smartboat/identity` | `smartboat_msgs/msg/BoatIdentity` |
| 定位 | `/smartboat/localization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` 或自定义 |
| 速度 | `/smartboat/localization/velocity` | `geometry_msgs/msg/TwistWithCovarianceStamped` 或自定义 |
| 规划路径 | `/smartboat/navigation/path` | `nav_msgs/msg/Path` |
| 控制指令 | `/smartboat/control/cmd_vel` | `geometry_msgs/msg/Twist` |
| 控制状态 | `/smartboat/control/status` | 自定义 |

当前融合避障控制链路：

```text
PX4 ATTITUDE.yaw
PX4 GPS_RAW_INT 3D fix + GLOBAL_POSITION_INT horizontal velocity
  -> mavlink-routerd
  -> tcp:127.0.0.1:5760
  -> yaw_from_mavlink
  -> /smartboat/localization/yaw  # Float32, degrees
  -> /smartboat/localization/ground_speed  # only while GPS fix/velocity is valid
  -> /smartboat/localization/ground_speed_valid
RealSense depth/color + EWaSR ONNX
  -> /fusion_obstacle_detected
  -> /fusion_obstacle_distance
  -> /fusion_avoidance_decision
SR73F CAN 0x60B
  -> sr73f_radar (raw radar range 3-10m)
  -> /smartboat/perception/radar/closest_obstacle_distance
RealSense fusion + SR73F + ground speed
  -> avoidance_supervisor
  -> /smartboat/navigation/avoidance_speed_limit
  -> /smartboat/navigation/avoidance_cmd_vel  # assessment recommendation
  -> /smartboat/navigation/emergency_stop_requested
  -> avoidance_executor
  -> /smartboat/control/avoidance_active
  -> /smartboat/control/cmd_vel
  -> cmd_vel_to_mavlink (single output owner, PX4 Offboard body vx/vy vector)
  -> AUTO.MISSION -> HOLD warmup -> OFFBOARD avoidance state machine
  -> COM_RC_IN_MODE=2 invariant + operator-MANUAL/Kill priority
  -> PX4 motor outputs
```

`yaw_from_imu` 是旧的 MAVROS/IMU 备用路径：

```text
/mavros/imu/data -> yaw_from_imu -> /smartboat/localization/yaw
```

命名约定：

- topic 使用小写和下划线。
- 节点名使用小写和下划线，例如 `water_quality`。
- Python 文件名使用小写和下划线，例如 `water_quality_node.py`。
- console script 建议用 `<功能>_node`，例如 `ewasr_fusion_node`。
- 自定义 message 使用 PascalCase，例如 `WaterQuality.msg`。
- 参数使用小写和下划线，例如 `publish_period_sec`。

### 6.3 Message 设计建议

水质数据建议不要只用字符串 JSON 在 ROS 内部传播。可以设计为：

```text
std_msgs/Header header
string station
string boat
WaterQualityReading[] readings
```

其中 `WaterQualityReading.msg`：

```text
string name
float64 value
string unit
```

如果字段集合稳定，也可以把温度、pH、浊度等定义成强类型字段。若服务器协议字段经常变化，则 `reading[]` 更灵活。

## 7. 外部协议和端口清单

| 模块 | 协议 | 地址/端口 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| RealSense/EWaSR dashboard 输出 | SRT | `111.229.242.175:10000` | 容器到服务器 | stream id 默认为 `publish:<station>/<boat>` |
| 水质上传 | WebSocket | `111.229.242.175:11452` | 容器到服务器 | 路径 `/uav/<station>/<boat>` |
| MAVLink 端口申请 | HTTP | `111.229.242.175:11451` | 容器到服务器 | 路径 `/connect?name=<name>` |
| MAVLink 转发 | MAVLink Router | `111.229.242.175:<动态端口>` | 容器到服务器 | 端口由 HTTP API 返回 |

## 8. 身份文件约定

多个节点都依赖船只身份：

```text
/host_boot/name.txt
/host_boot/boat_name.txt
```

含义：

| 文件 | 含义 | 使用方 |
| --- | --- | --- |
| `name.txt` | 站点/设备归属名，或服务器识别用 name | EWaSR 推理结果流、水质、MAVLink |
| `boat_name.txt` | 船名 | EWaSR 推理结果流、水质 |

兼容路径：

- `dashboard_stream_node.py`、RealSense可视化demo和`water_quality_node.py`默认读`/host_boot`。
- `mavlink_autoconnect.py` 会依次尝试 `/boot/name.txt` 和 `/host_boot/name.txt`。

Docker compose 当前应把宿主机 `/boot` 只读挂载到容器 `/host_boot:ro`。MAVLink 兼容场景还需要让容器能读 `/boot/name.txt`。

## 9. 正式程序与测试/demo隔离规则

- `boat_ws/scripts`只用于人工测试、demo、诊断和安装辅助。
- 正式ROS节点、launch、包模块和配置不得import、执行或source该目录中的文件。
- 正式运行链不得依赖demo的文件名、CLI参数、节点名或内部函数。
- 删除整个`boat_ws/scripts`目录后，已安装的正式节点和主launch仍应能够启动。
- EWaSR和雷达demo保持自包含，不能成为正式程序的运行API。

## 10. RealSense D435i 深度相机

当前 Docker 容器内已经从源码构建并安装：

```text
pyrealsense2==2.58.2
```

对应 wheel 保存在：

```text
/workspace/boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl
/home/ubuntu2204/smartboat/boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl
```

构建来源：

```text
https://github.com/realsenseai/librealsense
tag: v2.58.2
```

构建要点：

- Ubuntu 22.04 / Python 3.10 / aarch64。
- CMake 打开 `BUILD_PYTHON_BINDINGS=ON`。
- 使用 `BUILD_SHARED_LIBS=OFF` 静态打包 SDK 主体，让 wheel 尽量自包含。
- 使用 `FORCE_RSUSB_BACKEND=ON`，适合 Docker 里通过 USB/libusb 访问相机。
- wheel 运行时仍依赖系统 `libusb-1.0.so.0` 和 `libudev.so.1`，当前容器已具备。
- `docker/Dockerfile` 已同步安装该 wheel，后续重建镜像会自动包含 `pyrealsense2`。

当前已连接设备的验证结果：

```text
name: Intel RealSense D435I
firmware: 5.17.0.10
usb: 3.2
sensors: Stereo Module(depth/infrared), RGB Camera(color), Motion Module(accel/gyro)
```

测试脚本：

```bash
cd /workspace/boat_ws/src
python3 realsense_d435i_test.py --list-only
python3 realsense_d435i_test.py --frames 30
python3 realsense_d435i_test.py --frames 30 --imu
```

实时彩色摄像头推理脚本：

```bash
cd /workspace/boat_ws/src
python3 realsense_obstacle_pipeline_inference.py --provider CPUExecutionProvider
```

使用 TensorRT/CUDA 时，容器用户需要在 `video` 组中，因为 Jetson 的 `/dev/nvmap`、`/dev/nvhost-*` 等设备默认归属 `root:video`。当前 `docker-compose.yml` 已包含：

```yaml
group_add:
  - dialout
  - video
```

如果刚修改过组权限，需要退出当前容器 shell 后重新进入，或执行 `newgrp video`。确认：

```bash
id
```

应看到 `video`。然后普通用户直接运行即可：

```bash
cd /workspace/boat_ws/src
python3 realsense_obstacle_pipeline_inference.py --provider auto
```

不要优先用 `sudo` 跑带窗口的 OpenCV 程序。sudo 会让进程变成 root，而 root 往往没有当前桌面的 X11 授权，容易出现 `Authorization required, but no authorization protocol specified` 或 `Can't initialize GTK backend`。

如果只是 SSH/无显示器测试，不需要窗口：

```bash
python3 realsense_obstacle_pipeline_inference.py --frames 5 --no-display --provider auto
```

TensorRT 首次运行会在 `/workspace/boat_ws/trt_engine_cache/realsense_obstacle` 下生成 engine/timing cache。看到类似 `Using an engine plan file across different models of devices is not recommended` 的 warning，说明缓存可能来自不同设备或运行环境；更换设备、模型、TensorRT版本或遇到性能异常时应清缓存重建：

```bash
python3 realsense_obstacle_pipeline_inference.py --provider auto --clear-trt-cache
```

实时推理主版本：

```bash
cd /workspace/boat_ws/src
python3 realsense_obstacle_pipeline_inference.py --frames 30 --no-display --provider auto
```

RealSense D435i + EWaSR 多传感器融合避障 demo：

```text
宿主机文件：
/home/ubuntu2204/smartboat/boat_ws/scripts/realsense_ewasr_fusion_avoidance_demo.py

Docker 内运行路径：
/workspace/boat_ws/scripts/realsense_ewasr_fusion_avoidance_demo.py
```

运行命令：

```bash
cd /workspace/boat_ws/scripts
source /opt/ros/humble/setup.bash
python3 realsense_ewasr_fusion_avoidance_demo.py
```

该 demo 使用 RealSense D435i depth/color 和 EWaSR 分割结果做融合判断：只有 EWaSR 视觉障碍区域和 3m 内有效深度同时确认时，才发布避障结果。

模型路径按顺序解析：

```text
SMARTBOAT_EWASR_MODEL
/workspace/boat_ws/models/ewasr_resnet18.onnx
/home/ubuntu2204/smartboat/boat_ws/models/ewasr_resnet18.onnx
```

核心融合阈值：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `MIN_VALID_DEPTH_M` | `0.15` | 小于该距离的深度点视为无效 |
| `MAX_VALID_DEPTH_M` | `3.0` | 只确认 3m 内有效深度 |
| `AVOID_DISTANCE_THRESHOLD_M` | `2.8` | 小于该距离才建议避让 |
| `MIN_VISION_OBSTACLE_PIXELS` | `200` | EWaSR 障碍像素少于该值时忽略 |
| `MAX_DEPTH_SAMPLES` | `3000` | 障碍区域深度最多采样点数 |
| `DEFAULT_OBSTACLE_CLASS` | `0` | 当前 EWaSR 模型已核对的 obstacle 类别 ID |

当前发布 topic：

| Topic | Message | 说明 |
| --- | --- | --- |
| `/fusion_obstacle_detected` | `std_msgs/msg/Bool` | 是否融合确认障碍 |
| `/fusion_obstacle_distance` | `std_msgs/msg/Float32` | 融合确认障碍距离，未确认时为 `-1.0` |
| `/fusion_avoidance_decision` | `std_msgs/msg/String` | `GO_STRAIGHT` / `TURN_LEFT` / `TURN_RIGHT` |

避障方向规则：

- 障碍在画面左三分之一：发布 `TURN_RIGHT`。
- 障碍在画面右三分之一：发布 `TURN_LEFT`。
- 障碍在中间：比较左右半幅障碍像素数量，向更空的一侧转。

这三个topic现在由 `smartboat_navigation/avoidance_supervisor_node.py` 消费，并与SR73F最近目标距离和PX4地速共同生成新的速度、停车和转向建议。

容器内已验证可启动 TensorRT/CUDA、打开 RealSense pipeline，并持续输出 `CONFIRMED OBSTACLE` / `no confirmed obstacle` 日志。使用 `timeout` 强制结束时可能出现 ROS shutdown traceback，不代表 demo 启动失败。

图形化测试版：

```text
宿主机文件：
/home/ubuntu2204/smartboat/boat_ws/scripts/realsense_ewasr_fusion_avoidance_viz_demo.py

Docker 内运行路径：
/workspace/boat_ws/scripts/realsense_ewasr_fusion_avoidance_viz_demo.py
```

运行命令：

```bash
cd /workspace/boat_ws/scripts
source /opt/ros/humble/setup.bash
python3 realsense_ewasr_fusion_avoidance_viz_demo.py
```

该图形化demo自包含同一套融合判断逻辑、模型路径解析和视觉topic发布，并订阅正式SR73F节点的目标、状态和最近距离topic。画布原生固定为1280×960四象限，每个象限为640×480：左上为中文避障类型与运行信息，右上为RealSense彩色画面叠加EWaSR三类遮罩，左下为3–10米雷达扇区目标，右下为0–3米深度伪彩色图。遮罩索引0障碍为红色、索引1水面为蓝色、索引2天空为紫色；避障类型区分无障碍、雷达障碍、视觉障碍和雷达+视觉障碍。TensorRT推理、深度融合与ROS发布在主线程持续运行；dashboard渲染、存图和FFmpeg推流在独立线程中处理，默认每2帧推理结果重绘1帧。中文在启动阶段预渲染为透明文字贴图，正常帧只由OpenCV/NumPy合成，不再对整张画面执行Pillow转换。相机暂时断开时仍输出状态和雷达象限，但不刷新视觉避障topic，使监督器按超时安全停车。

可视化demo把TensorRT缓存按精度分到 `/workspace/boat_ws/trt_engine_cache/ewasr_fp16` 和 `ewasr_fp32`。一键启动默认使用FP16；首次生成engine可能需要数分钟，之后会直接命中当前Jetson生成的缓存。

ROS 2 节点可以弹出这类 X11 图形界面，关键不在 ROS 2，而在进程是否有可用的桌面显示环境。容器内运行 OpenCV 窗口、RViz2、rqt 等 GUI 时，需要宿主机 X11 socket 和 `DISPLAY` 环境变量可用。当前 `docker exec` shell 如果显示 `DISPLAY=unset`，可以在有桌面授权的容器终端中运行，或先尝试：

```bash
export DISPLAY=:0
```

如果仍然遇到 `Authorization required` 或 `Can't initialize GTK backend`，优先从宿主机桌面用户启动容器终端，不要用 `sudo` 跑 GUI 程序。

图形化 demo 支持：

```bash
python3 realsense_ewasr_fusion_avoidance_viz_demo.py --frames 300
python3 realsense_ewasr_fusion_avoidance_viz_demo.py --frames 30 --no-display --save-frame /workspace/boat_ws/log/dashboard.png
python3 realsense_ewasr_fusion_avoidance_viz_demo.py --fp16
python3 realsense_ewasr_fusion_avoidance_viz_demo.py --obstacle-class 0
```

图形化demo也可以把推理结果dashboard直接推到远程SRT服务器，观察EWaSR mask、depth、避障方向和距离：

```bash
python3 realsense_ewasr_fusion_avoidance_viz_demo.py --stream --no-display
```

demo和正式dashboard节点使用相同的服务器和身份文件约定：

```text
server: 111.229.242.175:10000
identity_dir: /host_boot
streamid: publish:<station>/<boat>
format: SRT caller + MPEG-TS + H.264
```

也可以显式指定 stream path 或完整 URL：

```bash
python3 realsense_ewasr_fusion_avoidance_viz_demo.py \
  --stream --no-display \
  --stream-path test/test01

python3 realsense_ewasr_fusion_avoidance_viz_demo.py \
  --stream --no-display \
  --stream-url 'srt://111.229.242.175:10000?streamid=publish:test/test01&pkt_size=1316&mode=caller'
```

推流相关参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--stream-server` | `111.229.242.175` | SRT 服务器 IP |
| `--stream-port` | `10000` | SRT 服务器端口 |
| `--stream-suffix` | 空 | 默认不改船名，保持 `publish:<station>/<boat>`；需要多路视频时再手动设置 |
| `--stream-fps` | `15` | 推理结果视频推流帧率；ROS topic 发布不受影响 |
| `--stream-width` | `1280` | 推流画面宽度 |
| `--stream-height` | `960` | 推流画面高度；默认与原生1280×960四象限dashboard一致 |
| `--dashboard-stride` | `2` | 每N帧TensorRT推理结果重绘1帧dashboard；默认显示一帧、跳过一帧 |
| `--stream-bitrate` | `1800k` | H.264 目标码率 |

推理结果流的SRT URL保持为`srt://<server>:<port>?streamid=publish:<station>/<boat>&pkt_size=1316&mode=caller`。视频编码使用H.264 baseline、MPEG-TS，并在关键帧重复SPS/PPS和PAT/PMT，尽量兼容原地面站播放器。

可以通过`--stream-suffix ''`或`--stream-path <station>/<boat>`控制demo的stream id；避免多个推流进程同时占用同一个stream id。

命令行 demo 支持：

```bash
python3 realsense_ewasr_fusion_avoidance_demo.py --fp16
python3 realsense_ewasr_fusion_avoidance_demo.py --obstacle-class 0
```

两个 demo 都启用 D435i depth/color `640x480@30`，并把 depth 对齐到 color，因为 EWaSR 分割基于彩色图，融合距离需要落回同一像素坐标系。

融合避障到生产执行链路的联调顺序：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip librealsense2
source install/setup.bash
```

终端1：一键启动感知、监督、执行状态机和唯一PX4 bridge。

```bash
ros2 launch smartboat_mission edge_nodes.launch.py
```

一键启动会默认拉起：

```text
mavlink_autoconnect.py
yaw_from_mavlink_node
sr73f_radar_node
avoidance_supervisor_node
avoidance_executor_node
cmd_vel_to_mavlink_node
water_quality_node
ewasr_fusion_node
dashboard_stream_node
```

旧的 `avoidance_pid_node` 不再启动。单独调试新链路：

```bash
ros2 launch smartboat_control yaw_from_mavlink.launch.py
ros2 launch smartboat_drivers sr73f_radar.launch.py
ros2 launch smartboat_navigation avoidance_supervisor.launch.py
ros2 launch smartboat_navigation avoidance_executor.launch.py
```

终端 3：启动 RealSense + EWaSR 融合 demo。

```bash
cd /workspace/boat_ws/scripts
source /opt/ros/humble/setup.bash
source ../install/setup.bash
python3 realsense_ewasr_fusion_avoidance_demo.py
```

观察 topic：

```bash
ros2 topic echo /fusion_obstacle_detected
ros2 topic echo /fusion_obstacle_distance
ros2 topic echo /fusion_avoidance_decision
ros2 topic echo /smartboat/localization/yaw
ros2 topic echo /smartboat/localization/ground_speed
ros2 topic echo /smartboat/localization/ground_speed_valid
ros2 topic echo /smartboat/perception/radar/closest_obstacle_distance
ros2 topic echo /smartboat/perception/radar/status
ros2 topic echo /smartboat/navigation/avoidance_cmd_vel
ros2 topic echo /smartboat/navigation/avoidance_supervisor_status
ros2 topic echo /smartboat/navigation/emergency_stop_requested
ros2 topic echo /smartboat/control/avoidance_executor_status
ros2 topic echo /smartboat/control/avoidance_active
ros2 topic echo /smartboat/px4_bridge/status
```

一键启动现在默认允许真实避障输出。启动前必须确认只存在一个
`cmd_vel_to_mavlink_node`、急停和遥控恢复可用，并先完成无桨/低功率岸测。CLEAR
状态不会进入Offboard；危险时先HOLD并预热Offboard，执行心跳超时只HOLD且不会
恢复任务。bridge永远保持`COM_RC_IN_MODE=2`，不会再切成MAVLink-only。若只做观测，应显式传入
`enable_avoidance_executor:=false enable_px4_cmd_bridge:=false`。

当前主链路不使用容器内 `/dev/ttyACM0`。PX4 串口由 `mavlink-routerd` 读取，ROS 2 节点只连接 MAVLink TCP：

```text
tcp:127.0.0.1:5760
```

如果地速、雷达或D435i融合topic超时，`avoidance_supervisor` 会发布 `SENSOR_FAULT`、速度上限0和紧急停车建议。

历史对比脚本 `realsense_obstacle_inference.py` 和 `realsense_obstacle_freshest_inference.py` 已移到 `smartboat_bringup`，仅作为参考，不再作为主版本维护。

已实测：

- `--list-only` 可枚举 D435i。
- `--frames 30` 可读取 `640x480@30` depth/color。
- `--frames 30 --imu` 可同时读取 depth/color 和 accel/gyro，测试中观察到 `accel=30, gyro=30`。

如果需要重新安装 wheel：

```bash
sudo python3 -m pip install --force-reinstall \
  /workspace/boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl
```

注意：

- Docker compose 需要把宿主机 `/dev` 挂载进容器，当前 `docker-compose.yml` 已有 `- /dev:/dev`，并启用了 `privileged: true`。
- D435i 应接 USB 3 口；测试输出中 `usb: 3.2` 表示当前链路正常。
- 后续正式 ROS 节点建议放到 `smartboat_perception`，不要长期把业务逻辑写在顶层测试脚本里。

## 11. ONNX 和感知工具

当前顶层有以下工具：

```text
benchmark.py
check_onnx_backends.py
check_onnx_backends_safe.py
realsense_d435i_test.py
realsense_obstacle_pipeline_inference.py
ewasr_resnet18.onnx
```

用途：

| 文件 | 作用 |
| --- | --- |
| `check_onnx_backends.py` | 查询当前 ONNX Runtime 可用 execution providers，并可尝试加载模型 |
| `check_onnx_backends_safe.py` | 在子进程中 import/load ONNX Runtime，避免 Jetson/ARM64 动态库崩溃拖垮主进程 |
| `benchmark.py` | 使用随机图像测试 `ewasr_resnet18.onnx` 的 preprocess/inference/postprocess 耗时 |
| `realsense_d435i_test.py` | 测试 D435i 设备枚举、depth/color 取帧和 IMU 数据 |
| `realsense_obstacle_pipeline_inference.py` | D435i 彩色摄像头 ONNX 推理主版本，采集/推理/显示三段流水线 |
| `ewasr_resnet18.onnx` | 当前用于测试的 ONNX 模型 |

示例：

```bash
cd /workspace/boat_ws/src
python3 check_onnx_backends_safe.py --model ewasr_resnet18.onnx
python3 benchmark.py --model ewasr_resnet18.onnx --provider auto
```

维护建议：

- 模型文件长期应放到 `smartboat_perception/models`。
- 工具脚本长期应放到 `smartboat_tools/scripts`。
- 感知运行节点应放到 `smartboat_perception/smartboat_perception`，并注册 console script。

## 12. 开发规范

### 12.1 包边界

新增代码优先按功能放置：

| 功能 | 放置位置 |
| --- | --- |
| 摄像头、图像、AI 模型 | `smartboat_perception` |
| 传感器、硬件驱动 | `smartboat_drivers` |
| PX4/MAVLink/飞控桥接 | `smartboat_px4_bridge` |
| 定位/坐标/状态估计 | `smartboat_localization` |
| 导航/路径/航点 | `smartboat_navigation` |
| 控制器/执行器 | `smartboat_control` |
| 多节点启动和整船编排 | `smartboat_mission` |
| 自定义接口 | `smartboat_msgs` |
| 调试和离线工具 | `smartboat_tools` |

不要把新业务代码继续塞进 `smartboat_bringup`。

### 12.2 ROS 2 Python 包规范

已有 Python 包使用 `ament_python`，典型结构：

```text
smartboat_xxx/
  package.xml
  setup.py
  setup.cfg
  resource/smartboat_xxx
  config/
  launch/
  smartboat_xxx/
    __init__.py
    xxx_node.py
```

新增可执行节点时，需要：

1. 在 `smartboat_xxx/smartboat_xxx/` 下新增 Python 文件。
2. 在 `setup.py` 的 `entry_points['console_scripts']` 中注册。
3. 在 `package.xml` 中声明运行依赖。
4. 如有 YAML/launch，确保 `setup.py` 的 `data_files` 安装对应目录。
5. 重新构建：

   ```bash
   colcon build --symlink-install --packages-skip librealsense2
   source install/setup.bash
   ```

### 12.3 参数规范

- 可变配置必须用 ROS parameter 或 YAML，不要硬编码在业务逻辑里。
- 时间参数统一用 `_sec` 后缀，微秒参数用 `_us` 后缀。
- 路径参数用 `_path` 或 `_dir` 后缀。
- 网络地址拆成 host/ip 和 port，除非协议天然要求完整 URL。
- 默认值要能在开发环境运行，生产值应通过 YAML/launch 覆盖。

### 12.4 日志规范

- 用节点 logger：`self.get_logger().info/warning/error()`。
- 默认运行应只输出 warning/error/critical；一键启动时通过 `debug:=true` 打开详细日志。
- `info/debug` 只用于身份解析、连接目标、启动命令等调试信息，不能作为默认运行必须看到的信息。
- 不打印明文密码、token、完整带密码的 URL。
- 网络重连、外部进程重启、传感器异常要有明确日志。
- 水质发送失败允许短暂波动：单次失败只记 debug，连续失败超过 2 次才 warning。
- 周期高频日志要节制，避免刷屏影响排障。

### 12.5 外部进程规范

当前`dashboard_stream_node.py`管理`ffmpeg`子进程：

- 退出后自动重启。
- 节点关闭时清理子进程。
- 原始帧按到达时的wall-clock生成VFR时间戳，不用固定15Hz定时器重复旧帧。
- 合成和管道写入在latest-only后台线程执行，积压时丢弃旧显示帧。
- 该进程与D435i/EWaSR安全感知隔离，推流失败不影响融合结果发布。

后续如果需要管理其他外部进程，也应遵循类似方式，避免留下僵尸进程或重复占用设备。

### 12.6 异步和重连规范

当前 `water_quality_node.py` 使用 `asyncio` + `aiohttp`：

- 连接失败后等待 `reconnect_delay_sec`。
- 上传周期由 `publish_period_sec` 控制。
- provider 和 client 分离。

后续传感器接入时，不要把硬件读取、协议上传、ROS publish 全部揉在一个大循环里。建议保持：

```text
SensorProvider -> ROS node/client -> external protocol or ROS topic
```

## 13. 测试和检查

当前包内有基本 lint/test 依赖，但业务测试还不完善。建议常用检查：

注意：以下 ROS 构建和测试命令都应在 Docker 容器内执行，工作目录应为 `/workspace/boat_ws`。

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip librealsense2
colcon test
colcon test-result --verbose
```

单独检查 Python 语法：

```bash
python3 -m py_compile \
  src/smartboat_perception/smartboat_perception/ewasr_core.py \
  src/smartboat_perception/smartboat_perception/fusion_model.py \
  src/smartboat_perception/smartboat_perception/realsense_source.py \
  src/smartboat_perception/smartboat_perception/ewasr_fusion_node.py \
  src/smartboat_perception/smartboat_perception/dashboard_stream_node.py \
  src/smartboat_drivers/smartboat_drivers/water_quality_node.py \
  src/smartboat_drivers/smartboat_drivers/sr73f_protocol.py \
  src/smartboat_drivers/smartboat_drivers/sr73f_radar_node.py \
  src/smartboat_mission/smartboat_mission/mavlink_autoconnect.py \
  src/smartboat_navigation/smartboat_navigation/safety_model.py \
  src/smartboat_navigation/smartboat_navigation/avoidance_supervisor_node.py \
  src/smartboat_control/smartboat_control/yaw_from_imu_node.py \
  scripts/realsense_ewasr_fusion_avoidance_demo.py \
  scripts/realsense_ewasr_fusion_avoidance_viz_demo.py
```

建议后续补充的测试：

- `build_srt_url()`身份文件缺失、空文件、正常文件和完整URL覆盖。
- `DashboardStreamer.build_command()`生成的ffmpeg参数。
- `TelemetryClient` 上传 JSON 格式。
- `mavlink_autoconnect.request_mavlink_port()` 对异常返回的处理。
- `yaw_from_quaternion()` 对典型四元数的 yaw 计算。
- `sr73f_protocol` 对3m/10m边界、视场、船头安装偏移的解析和过滤。
- `safety_model` 对雷达空扫描、传感器超时、动态停车距离和左右转向符号的判断。
- `analyze_fusion()` 对视觉无障碍、深度无效、距离过远、左右避让的判断。

## 14. 常见排障

### 14.1 `ros2 run` 找不到节点

检查：

```bash
source /opt/ros/humble/setup.bash
source /workspace/boat_ws/install/setup.bash
ros2 pkg executables smartboat_perception
ros2 pkg executables smartboat_drivers
```

如果刚新增 console script，重新 build。

### 14.2 RealSense D435i不可达

在容器中检查USB设备和RealSense枚举：

```bash
lsusb
rs-enumerate-devices
```

确认容器具有`/dev`访问权限，并安装了与Jetson/Python 3.10匹配的`pyrealsense2`运行资产。

### 14.3 SRT 推流失败

检查：

- `ffmpeg` 是否存在：`which ffmpeg`
- 服务器 `111.229.242.175:10000` 是否可达。
- `name.txt` 和 `boat_name.txt` 是否正确。
- 如果运行的是 EWaSR dashboard 流，确认 RealSense D435i、模型文件和 `SMARTBOAT_EWASR_MODEL`/默认模型路径可用。

### 14.4 水质上传失败

检查：

- 容器是否能访问 `111.229.242.175:11452`。
- WebSocket 路径是否仍为 `/uav/<station>/<boat>`。
- `/host_boot/name.txt` 和 `/host_boot/boat_name.txt` 是否存在。

### 14.5 MAVLink helper 失败

检查：

- `/boot/name.txt` 或 `/host_boot/name.txt` 是否存在且非空。
- 容器是否能访问 `http://111.229.242.175:11451/connect?name=<name>`。
- `/usr/bin/mavlink-routerd` 是否挂载进容器。
- `/etc/mavlink-router/main.conf` 是否挂载进容器。
- 容器用户是否有串口权限。
- 宿主机是否没有同时运行 `autoconnect.service` 或 `mavlink-router.service`。
- 如果 `/dev/ttyACM0` 存在但 helper 提示没有 PX4 `HEARTBEAT`，通常是飞控 USB CDC 数据通道卡住；请在硬件上拔插 PX4 飞控 USB 后重新启动。仅重启 Jetson 或重绑 Linux 驱动不一定能让飞控端重新开始发送 MAVLink。

### 14.6 RealSense D435i 不可用

检查容器是否能看到 USB 设备：

```bash
docker exec smartboat_ros2_g39 lsusb
```

正常应看到类似：

```text
8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
```

检查 Python 包：

```bash
docker exec smartboat_ros2_g39 python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
```

检查取帧：

```bash
docker exec smartboat_ros2_g39 python3 /workspace/boat_ws/scripts/realsense_ewasr_fusion_avoidance_viz_demo.py --frames 30
```

如果枚举不到设备：

- 确认相机接在 USB 3 口。
- 确认 `docker-compose.yml` 有 `/dev:/dev`、`privileged: true`。
- 重新插拔相机后再进容器测试。
- 避免宿主机其他 RealSense 程序同时占用设备。

## 15. 给后续 Codex 的约束

- 先读本 README 和目标包源码，再改代码。
- 新业务按功能进对应 `smartboat_*` 包，不要默认放进 `smartboat_bringup`。
- 已有外部协议不要随意改路径、字段和端口；如果必须改，要同步更新 README、YAML 和调用方。
- 当前没有 ROS topic/message，并不代表以后也不需要。新增可维护能力时，优先把内部数据 ROS 化，再保留外部上传/推流协议。
- 修改 launch/config 后，记得确认 `setup.py` 会安装对应文件。
- 涉及摄像头、MAVLink、WebSocket、HTTP 的改动，README 中的端口清单和排障步骤也要同步更新。
- 不要提交 `__pycache__`、临时日志、模型缓存和 TensorRT engine cache。
