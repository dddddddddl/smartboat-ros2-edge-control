# ROS 2 workspace source guide

The workspace includes real YSI EXO acquisition/upload, ROS-native PX4 bridging and local navigation/control integration. See the [current project status](../docs/architecture.md) for completed areas, remaining work and verification limits.

The table maps available source files to their scope.

| Directory | Source contents | Presentation status |
|---|---|---|
| [smartboat_drivers](src/smartboat_drivers/README.md) | Telemetry client, EXO provider and SR73F radar code | Real EXO acquisition/upload completed; radar hardware validation not claimed |
| [smartboat_perception](src/smartboat_perception/README.md) | EWaSR/depth and ROS dashboard modules | Test/inference tools and ROS source included; hardware execution not repeated here |
| [smartboat_msgs](src/smartboat_msgs/README.md) | Four message definitions | Four definitions included; further integration/validation remains |
| [smartboat_control](src/smartboat_control/README.md) | MAVLink/IMU yaw and speed-input modules | Local control integration completed |
| [smartboat_navigation](src/smartboat_navigation/README.md) | Safety and avoidance-execution models/nodes | Local navigation integration completed |
| [smartboat_px4_bridge](src/smartboat_px4_bridge/README.md) | Command bridge and event recorder | ROS-native bridge completed |
| [smartboat_mission](src/smartboat_mission/README.md) | Main launch and MAVLink routing helper | Routing and integrated navigation/control launch included |
| [smartboat_bringup](src/smartboat_bringup/README.md) | Package skeleton and camera-only PID reference | Reference source; no separate completion claim |
| [smartboat_localization](src/smartboat_localization/README.md) | Directory scaffold | Localization under development |
| [smartboat_tools](src/smartboat_tools/README.md) | Directory scaffold | No implemented packaged tools |

Eight directories contain `package.xml` manifests. `smartboat_localization` and `smartboat_tools` do not contain package manifests.

`boat_ws/scripts/` holds manual diagnostics, D435i smoke/inference tools, radar utilities and vendor references. See its [guide](scripts/README.md).

The container workspace is `/workspace/boat_ws`. Restore the ONNX model and Python 3.10/aarch64 wheels using [runtime-assets.md](../docs/runtime-assets.md). Offline checks are recorded separately from hardware and integration verification.
