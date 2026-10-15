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
| Available source files and development modules | [Workspace guide](boat_ws/README.md) |
| Photographs, hardware concept and interface screenshots | [Project evidence](docs/evidence.md) |
| Jetson prerequisites and model/wheel downloads | [Build guide](docs/deployment.md) and [runtime assets](docs/runtime-assets.md) |
| Checks that can run without a boat | [Offline checks](tests/README.md) and [validation record](docs/validation.md) |
| Team members and acknowledgments | [Project team](docs/team.md) |

## Approach

- **Edge platform:** Jetson Orin NX runs ROS 2 Humble in Docker with CUDA-capable runtime access.
- **Video and perception:** Hikvision RTSP video is forwarded by FFmpeg to an external SRT service. D435i test tools acquire camera/depth/IMU data and use EWaSR ResNet-18 ONNX inference for water and obstacle segmentation.
- **Water-quality telemetry:** a `SensorProvider` interface supplies timestamped name/value records, which are uploaded by WebSocket with station and vessel identity.
- **Mission link:** a helper requests a dedicated server port, and `mavlink-routerd` connects the PX4/MAVLink stream to the shore side. Operators use the shore interface for waypoint missions.

## Project status

The supplied SURF poster is the reference for the project's reported completion status. Source files for additional modules are retained as development work; their presence and offline test results do not establish that the poster's pending milestones were completed.

| Status in the poster | Scope |
|---|---|
| Implemented / runnable | SRT camera streaming; WebSocket water-quality upload with mock data; MAVLink dynamic-port routing; D435i smoke tests and inference tools |
| Under development | Real water-quality `SensorProvider`; ROS-native PX4 bridge and custom messages; navigation, localization and control packages |
| Further evaluation | Quantified accuracy, endurance and avoidance success rate |

The poster describes water-quality upload with `MockSensorProvider` and the D435i pipeline as test/inference tools, rather than a completed packaged ROS 2 navigation system. This repository does not present the additional sensor/control modules as completed project outcomes.

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

**65 offline tests passed** during repository preparation. They inspect the supplied source, including development modules, without starting ROS nodes, model inference, hardware or shore connections. They are separate from the poster's reported field activities and do not change its completion status.

## Jetson build and runtime assets

The supplied build targets Jetson/aarch64, Ubuntu 22.04, ROS 2 Humble and Python 3.10. The model and two architecture-specific wheels are available from this repository's [public Release](https://github.com/dddddddddl/smartboat-ros2-edge-control/releases/tag/portfolio-v1.0), with [download and restoration instructions](docs/runtime-assets.md).

Use the [build guide](docs/deployment.md) to inspect the supplied source on the target platform. It is not a reconstruction of the poster's exact trial configuration. The supplied main launch also enables development control modules and a command bridge configured with `output_enabled: true`; review those settings before connecting hardware.

## Limitations and next steps

Following the poster, the next steps are real water-quality sensor integration, ROS-native messages and the PX4 bridge, navigation/localization/control integration, and controlled performance evaluation. The repository does not contain the complete shore application, PX4 firmware, model training data or raw quantitative field-test records.

Jetson/ROS builds, GPU inference and physical vessel operation were not repeated during repository preparation. Existing Chinese interface labels and original reference manuals are retained; the English guides explain the project and source layout.

## Project team and acknowledgments

Our team comprises **Yuxiao Xie, Kaihan Ouyang, Siyuan Lin, Ruoxi Zhu, Shengzhe Li and Yijia Cao**, supervised by **Xiaohui Zhu and Yong Yue** at Xi'an Jiaotong-Liverpool University.

The poster reports a **National Second Prize in the 15th National Marine Vehicle Design and Manufacturing Competition**. A separate award certificate is not included in this repository.

See [team and acknowledgments](docs/team.md) for credits and third-party components.
