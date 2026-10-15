# SmartBoat: Water-Quality Monitoring and Autonomous Obstacle Avoidance

A SURF team project at Xi'an Jiaotong-Liverpool University, developing a Jetson-based edge-computing and telemetry platform for an unmanned surface vessel.

Our platform combines ROS 2 Humble in Docker, camera streaming, water-quality telemetry and a PX4/MAVLink mission link. D435i test tools combine EWaSR segmentation with depth information for obstacle perception.

![SmartBoat on the water](assets/field_trial.jpeg)

*Project field photograph from the English SURF poster.*

## Start here

| To understand… | Open… |
|---|---|
| Project purpose, methods and reported field activities | [English SURF poster](docs/presentation/smartboat_surf_project_poster.pptx) |
| System architecture and project status | [Architecture](docs/architecture.md) |
| Source files and package guides | [Workspace guide](boat_ws/README.md) |
| Photographs, hardware concept and interface screenshots | [Project evidence](docs/evidence.md) |
| Jetson prerequisites and model/wheel downloads | [Build guide](docs/deployment.md) and [runtime assets](docs/runtime-assets.md) |
| Checks that can run without a boat | [Offline checks](tests/README.md) and [validation record](docs/validation.md) |
| Team members and acknowledgments | [Project team](docs/team.md) |

## Approach

- **Edge platform:** Jetson Orin NX runs ROS 2 Humble in Docker with CUDA-capable runtime access.
- **Video and perception:** Hikvision RTSP video is forwarded by FFmpeg to an external SRT service. D435i test tools acquire camera/depth/IMU data and use EWaSR ResNet-18 ONNX inference for water and obstacle segmentation.
- **Water-quality telemetry:** the YSI EXO probe is connected through RS-232 using `ExoSensorProvider`; timestamped measurements are uploaded by WebSocket with station and vessel identity.
- **Mission link:** a helper requests a dedicated server port, and `mavlink-routerd` connects the PX4/MAVLink stream to the shore side. Operators use the shore interface for waypoint missions. The ROS-native PX4 bridge connects local navigation and control requests to vehicle output.

## Project status

The implemented stack includes real YSI EXO acquisition and upload, the ROS-native PX4 bridge, and local navigation/control integration. Software completion is reported separately from the checks performed during repository preparation.

| Current status | Scope |
|---|---|
| Completed | SRT camera streaming; real YSI EXO acquisition and WebSocket upload; MAVLink dynamic-port routing; D435i smoke tests and inference tools; ROS-native PX4 bridge; local navigation/control integration |
| Remaining work | Dedicated localization; custom-message integration and validation beyond the supplied definitions |
| Further evaluation | Quantified accuracy, endurance and avoidance success rate |

The real water-quality path is implemented in `exo_serial.py` and `water_quality_node.py`. Local navigation and control use the supervisor/executor modules and `cmd_vel_to_mavlink_node.py`. The original poster is retained unchanged; the current status above incorporates the project owner's completion update.

## Reported field activities

The poster reports repeated trials at Dushu Lake and the campus pond covering manual control, waypoint missions, obstacle avoidance and water-quality workflows. See the [poster](docs/presentation/smartboat_surf_project_poster.pptx) and [evidence gallery](docs/evidence.md) for the original descriptions, photographs and interface screenshots.

These activities demonstrate the intended integrated workflow. No measured accuracy, endurance, avoidance-success or latency figures are supplied here.

## Review the source without hardware

```bash
python3 -m venv /tmp/smartboat-host-checks
source /tmp/smartboat-host-checks/bin/activate
python -m pip install -r tests/requirements-host.txt
bash tests/run_host_checks.sh
```

**65 offline tests passed** during repository preparation. They inspect the supplied source without starting ROS nodes, model inference, hardware or shore connections. They are separate from the project's completion status and the poster's reported field activities.

## Jetson build and runtime assets

The supplied build targets Jetson/aarch64, Ubuntu 22.04, ROS 2 Humble and Python 3.10. The model and two architecture-specific wheels are available from this repository's [public Release](https://github.com/dddddddddl/smartboat-ros2-edge-control/releases/tag/portfolio-v1.0), with [download and restoration instructions](docs/runtime-assets.md).

Use the [build guide](docs/deployment.md) to inspect the supplied source on the target platform. It is not a reconstruction of the poster's exact trial configuration. The supplied main launch enables the navigation/control nodes and a command bridge configured with `output_enabled: true`; review those settings before connecting hardware.

## Limitations and next steps

Further work includes dedicated localization, additional interface validation and controlled performance evaluation. The included local avoidance stack is not a complete global mission planner. The repository does not contain the complete shore application, PX4 firmware, model training data or raw quantitative field-test records.

Jetson/ROS builds, GPU inference and physical vessel operation were not repeated during repository preparation. Existing Chinese interface labels and original reference manuals are retained; the English guides explain the project and source layout.

## Project team and acknowledgments

Our team comprises **Yuxiao Xie, Kaihan Ouyang, Siyuan Lin, Ruoxi Zhu, Shengzhe Li and Yijia Cao**, supervised by **Xiaohui Zhu and Yong Yue** at Xi'an Jiaotong-Liverpool University.

The poster reports a **National Second Prize in the 15th National Marine Vehicle Design and Manufacturing Competition**. A separate award certificate is not included in this repository.

See [team and acknowledgments](docs/team.md) for credits and third-party components.
