# ROS 2 workspace reading guide

Read the packages in data-flow order. Each maintained package has a short English guide describing its sources, configuration and boundaries.

| Package | Responsibility | Source status |
|---|---|---|
| [smartboat_drivers](src/smartboat_drivers/README.md) | SR73F radar and YSI EXO acquisition/upload | Implemented modules and protocol tests |
| [smartboat_perception](src/smartboat_perception/README.md) | RealSense, EWaSR/depth fusion and dashboard stream | Maintained ROS nodes and pure fusion/pipeline tests |
| [smartboat_msgs](src/smartboat_msgs/README.md) | Typed fusion/radar messages | Four message definitions |
| [smartboat_control](src/smartboat_control/README.md) | MAVLink/IMU yaw and speed inputs | Implemented input nodes |
| [smartboat_navigation](src/smartboat_navigation/README.md) | Safety assessment and avoidance execution | Maintained nodes and independently testable models |
| [smartboat_px4_bridge](src/smartboat_px4_bridge/README.md) | Single vehicle command bridge and event recording | Implemented module; ROS/hardware operation unverified here |
| [smartboat_mission](src/smartboat_mission/README.md) | Main launch composition and MAVLink helper | Implemented launch and helper |
| [smartboat_bringup](src/smartboat_bringup/README.md) | Package skeleton and retired PID reference | Legacy, outside maintained control path |
| [smartboat_localization](src/smartboat_localization/README.md) | Future localization/state estimation | Directory scaffold only |
| [smartboat_tools](src/smartboat_tools/README.md) | Future packaged diagnostics | Directory scaffold only |

The workspace contains eight `package.xml` manifests. The last two entries are not installed ROS packages simply because their directories exist.

`boat_ws/scripts/` contains manual diagnostics, benchmark tools and vendor references. They are preserved but are not the production launch API. The maintained launch resolves installed package executables, not the demo files.

Large files expected outside Git are `models/ewasr_resnet18.onnx` and two Python 3.10/aarch64 wheels in `wheels/`. Restore them using [runtime-assets.md](../docs/runtime-assets.md). The container uses `/workspace/boat_ws` as its workspace path.
