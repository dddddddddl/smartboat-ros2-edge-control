# 智能船下水测试流程

> 注意：本文后续保留了早期PID实船联调记录，其中
> `/smartboat/control/cmd_vel`和`/smartboat/control/avoidance_status`不属于当前
> recommendation-only主运行链。当前正式监督输出是
> `/smartboat/navigation/avoidance_cmd_vel`和
> `/smartboat/navigation/avoidance_supervisor_status`，只提供建议，不直接控制电机。

本文档用于下水前、岸边和低速下水阶段验证当前 Smartboat ROS 2 避障链路。目标是确认：

- 避障只在 PX4 航点模式 `AUTO.MISSION` 下允许介入。
- 手动模式 `MANUAL` 下避障保持关闭，不抢遥控器。
- 未确认安全前，MAVLink 输出保持关闭。
- RealSense/EWaSR、yaw、PID、PX4 bridge 的状态都可观察、可回退。

## 1. 测试原则

1. 先只观察，不控制。
2. 先岸上，再架空或系泊，再低速短航点。
3. 每一步只打开一个新的风险开关。
4. 遥控器手动模式始终作为最高优先级人工接管手段。
5. `output_enabled: false` 是总保险；只有确认进入实控测试时才改为 `true`。

当前设计中，感知和 PID 可以一直运行，但 `/smartboat/control/cmd_vel` 只有通过 `smartboat_px4_bridge/cmd_vel_to_mavlink_node.py` 后才可能进入 PX4。PX4 bridge 需要同时满足：

```text
output_enabled = true
PX4 当前模式 = AUTO.MISSION
MAVLink HEARTBEAT 新鲜
/smartboat/control/cmd_vel 新鲜，或允许超时中位输出
```

手动模式 `MANUAL`、姿态/位置/Offboard 等其他模式默认不允许避障输出。

## 2. 人员和现场准备

### 2.1 人员分工

- 遥控手：全程拿遥控器，负责切手动、解锁/上锁、急停。
- 电脑手：负责 Jetson/ROS 2 启动、topic 观察、日志记录。
- 安全观察员：观察船体、推进器、周边人员和水面障碍。

### 2.2 安全状态

下水前推荐状态：

```text
遥控器：手动模式
PX4：未解锁
推进器：确认不会意外转动
output_enabled：false
```

如需在岸上打开 `output_enabled: true` 做架空测试，必须确认推进器离水、无人接触推进器、船体固定。

## 3. 硬件检查

### 3.1 电气和机械

1. 检查电池电压、供电线、保险、急停开关。
2. 检查推进器固定、舵机/推进器方向、螺旋桨周边无异物。
3. 检查 Jetson、PX4、RealSense D435i、毫米波雷达如参与测试则检查 CAN 盒连接。
4. 检查 RealSense 安装角度和视野，镜头前无遮挡、水滴和强反光。

### 3.2 网络和远程

1. 确认能通过 NoMachine 或 SSH 连接 Jetson。
2. 确认地面站或 SRT 播放端能看到 RealSense/EWaSR dashboard 流。
3. 确认遥控器和 PX4 链路稳定。

## 4. 软件启动前检查

进入容器：

```bash
cd /home/ubuntu2204/smartboat
docker exec -it smartboat_ros2_g39 bash
```

进入工作区并 source：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检查关键文件：

```bash
ls -lh /workspace/boat_ws/models/ewasr_resnet18.onnx
ls -lh /workspace/boat_ws/wheels/
```

检查 RealSense：

```bash
python3 /workspace/boat_ws/scripts/realsense_ewasr_fusion_avoidance_viz_demo.py --frames 30 --no-display
```

如果只是想看画面和推理流：

```bash
python3 /workspace/boat_ws/scripts/realsense_ewasr_fusion_avoidance_viz_demo.py --stream --no-display
```

检查 MAVLink helper 能收到 PX4 HEARTBEAT：

```bash
python3 /workspace/boat_ws/src/smartboat_mission/smartboat_mission/mavlink_autoconnect.py --log-level debug
```

若提示 `/dev/ttyACM0` 存在但收不到 `HEARTBEAT`，按 README 建议物理拔插 PX4 USB 后再启动。

## 5. 构建和静态验证

修改代码或配置后先构建：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Python 语法检查：

```bash
python3 -m py_compile \
  src/smartboat_px4_bridge/smartboat_px4_bridge/cmd_vel_to_mavlink_node.py \
  src/smartboat_control/smartboat_control/avoidance_pid_node.py \
  scripts/realsense_ewasr_fusion_avoidance_demo.py \
  scripts/realsense_ewasr_fusion_avoidance_viz_demo.py
```

## 6. 只观察不控制测试

确认 `boat_ws/src/smartboat_px4_bridge/config/cmd_vel_to_mavlink.yaml` 中：

```yaml
output_enabled: false
allowed_control_modes:
  - AUTO.MISSION
```

启动一键 launch：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smartboat_mission edge_nodes.launch.py debug:=true
```

另开终端观察：

```bash
ros2 topic echo /fusion_obstacle_detected
ros2 topic echo /fusion_obstacle_distance
ros2 topic echo /fusion_avoidance_decision
ros2 topic echo /smartboat/localization/yaw
ros2 topic echo /smartboat/localization/ground_speed
ros2 topic echo /smartboat/control/cmd_vel
ros2 topic echo /smartboat/control/avoidance_status
ros2 topic echo /smartboat/px4_bridge/status
```

预期：

```text
RealSense/EWaSR 有融合结果
yaw 持续更新
ground_speed 持续更新，单位 m/s
avoidance_pid 发布 cmd_vel
px4_bridge status 中 output_enabled=False
sent=False
```

此阶段即使检测到障碍，PX4 也不应收到避障控制输出。

## 7. PX4 模式门控测试

保持 `output_enabled: false`。

### 7.1 手动模式

遥控器保持手动模式，观察：

```bash
ros2 topic echo /smartboat/px4_bridge/status
```

预期状态包含：

```text
flight_mode=MANUAL
mode_allowed=False
sent=False
```

在 RealSense 前放置障碍物，确认感知和 PID 可以变化，但 `sent` 仍为 `False`。

### 7.2 航点模式

切到航点模式 `AUTO.MISSION`，仍保持未解锁或安全状态。观察：

```text
flight_mode=AUTO.MISSION
mode_allowed=True
sent=False
```

因为 `output_enabled=false`，即使 `mode_allowed=True`，也仍不发送控制。

### 7.3 其他模式

如现场需要，可依次观察 `POSCTL`、`ALTCTL`、`OFFBOARD` 等模式。预期：

```text
mode_allowed=False
sent=False
```

## 8. 避障感知和 PID 测试

保持 `output_enabled: false`。

在船体前方 0.8m、1.5m、2.5m、3.5m 分别放置障碍物，观察：

```bash
ros2 topic echo /fusion_obstacle_detected
ros2 topic echo /fusion_obstacle_distance
ros2 topic echo /fusion_avoidance_decision
ros2 topic echo /smartboat/localization/ground_speed
ros2 topic echo /smartboat/control/avoidance_status
ros2 topic echo /smartboat/control/cmd_vel
```

预期：

- 距离小于 `safe_distance` 时进入避障。
- 距离小于 `stop_distance` 时目标速度接近 0。
- `TURN_LEFT` / `TURN_RIGHT` 只生成一次目标航向，不持续累加。
- 障碍短暂消失时 `clear_delay_sec` 内避障状态短暂保持，避免抖动。
- 有 PX4 GPS/速度输入时，避障状态下 `/smartboat/localization/ground_speed` 应保持新鲜，`avoidance_status` 中 `ground_speed` 应显示真实速度；否则避障地速限速不会闭环生效。

## 9. 岸上或架空实控测试

只有满足以下条件才进入本阶段：

```text
船体固定或推进器离水
遥控手就位
周围无人接触推进器
手动模式接管验证正常
```

将 `cmd_vel_to_mavlink.yaml` 中：

```yaml
output_enabled: true
```

重新 build 或至少重新 source 安装后的配置，启动：

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch smartboat_mission edge_nodes.launch.py debug:=true
```

### 9.1 手动模式安全确认

PX4 保持 `MANUAL`，观察：

```text
output_enabled=True
flight_mode=MANUAL
mode_allowed=False
sent=False
```

在 RealSense 前放障碍物，确认不发控制。

### 9.2 航点模式输出确认

切到 `AUTO.MISSION`，观察：

```text
output_enabled=True
flight_mode=AUTO.MISSION
mode_allowed=True
```

放置障碍物，确认 `sent=True` 只在航点模式出现。若切回 `MANUAL`，应立即回到：

```text
mode_allowed=False
sent=False
```

## 10. 系泊下水测试

船下水但保持系泊绳，航点距离短、速度低。

1. `output_enabled=false` 启动，先确认画面、yaw、MAVLink、模式状态。
2. 切 `AUTO.MISSION`，观察 `mode_allowed=True` 但 `sent=False`。
3. 如状态稳定，停船，改 `output_enabled=true` 并重新启动 bridge/launch。
4. 航点低速前进，人工放置或靠近安全障碍。
5. 观察避障转向是否符合 dashboard 和 `/smartboat/control/avoidance_status`。
6. 遥控器切回 `MANUAL`，确认桥接立即 `sent=False`，遥控器可接管。

## 11. 短航点自由航行测试

测试区域必须开阔，航点短，低速，人员保持安全距离。

1. 先跑无障碍短航点，确认船能按任务正常航行。
2. 加入远距离障碍，确认不误触发。
3. 加入 `safe_distance` 内障碍，确认避障输出。
4. 切手动模式接管，确认避障关闭。
5. 重复左侧、右侧、正前方障碍三种场景。

每轮记录：

```text
日期时间
电池电压
天气/光照/水面反光
PX4 模式
障碍物位置和距离
/fusion_avoidance_decision
/smartboat/control/avoidance_status
/smartboat/px4_bridge/status
人工接管是否正常
```

## 12. 异常处理

### 12.1 立刻切手动

出现以下任一情况，遥控手立即切 `MANUAL`：

- 船体转向和预期相反。
- 船速异常升高。
- 视频流明显卡顿或画面失真。
- yaw 不更新。
- PX4 bridge status 中模式显示异常。
- 船体接近人员、岸边、绳索或其他不可碰撞目标。

### 12.2 停止 ROS

电脑手可停止 launch：

```bash
Ctrl+C
```

或另开终端检查节点：

```bash
ros2 node list
ros2 topic echo /smartboat/px4_bridge/status
```

### 12.3 恢复保守配置

如不继续实控测试，将配置改回：

```yaml
output_enabled: false
```

重新启动前再次确认遥控器手动模式、PX4 未解锁或已上锁。

## 13. 测试通过标准

下水前必须同时满足：

- `MANUAL` 模式下 `mode_allowed=False` 且 `sent=False`。
- `AUTO.MISSION` 模式下 `mode_allowed=True`。
- `output_enabled=false` 时任何模式都 `sent=False`。
- yaw topic 连续稳定。
- ground speed topic 连续稳定，单位 m/s。
- RealSense/EWaSR topic 连续稳定。
- `/smartboat/control/cmd_vel` 在障碍出现时符合预期。
- 遥控器切手动后可立即接管。

只有这些项目都通过，才进入更长距离、更高速度或复杂障碍的测试。
