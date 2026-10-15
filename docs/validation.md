# Validation record

Checks in this record were performed for the portfolio edition on **October 15, 2026**, using a macOS host. A source-to-document audit and offline test rerun were completed on **October 16, 2026**. These checks are separate from the historical field activities described in the supplied poster.

## Completed checks

| Check | Result | What it establishes |
|---|---|---|
| Existing offline test selection | **65 passed** across eight test files | Sensor protocol, fusion, safety/execution model, synchronization, text-cache and event-recording checks |
| Python 3.10 grammar parsing | **71 files passed** | Syntax compatibility inspection of project Python sources; not execution under Python 3.10 |
| Shell syntax | **7 scripts passed** | `bash -n`; scripts were not executed |
| YAML parsing | **12 YAML files passed** | YAML structure, including Compose YAML; not validation of ROS parameters or Compose runtime semantics |
| Package XML parsing | **8 manifests passed** | XML structure and package inventory |
| Runtime archive integrity | Passed | 315,950,592 bytes and SHA-256 match the recorded archive checksum; three files at expected model/wheel paths |
| Poster integrity | Passed | Uploaded presentation is byte-identical to the supplied PPTX |
| Source preservation | Checked against the pre-organization file inventory | Runtime source, tests, messages, configurations and Docker recipes retained without functional changes |

The host test command is [tests/run_host_checks.sh](../tests/run_host_checks.sh). It invokes existing assertions, does not launch ROS nodes or substitute fake ROS bindings, and opens no vehicle/network connection.

The review environment used Python 3.12, pytest 9.1.1, NumPy 1.26.4, opencv-python 4.11.0.86, Pillow 12.3.0, pyserial 3.5 and PyYAML 6.0.3. These host-only pins are recorded in `tests/requirements-host.txt`. They do not migrate the Jetson/Python 3.10 runtime or change its ONNX Runtime/CUDA/RealSense dependencies.

## Checks not run

This host has no ROS 2 Humble environment, generated ROS message bindings, Docker/Jetson runtime, RealSense camera, EXO probe, SR73F radar or PX4 controller. The following were therefore not performed:

- target `docker compose config` or image build/start;
- `colcon build --symlink-install --packages-skip librealsense2`;
- ROS-dependent yaw, dashboard-node, bridge and ament test suites;
- model inference, TensorRT conversion/cache generation or camera acquisition;
- serial/CAN hardware tests, shore uploads or MAVLink communication;
- arming, mode changes, actuator output or on-water testing.

The Python syntax and pure-model checks do not validate integration, field reliability, sensor accuracy, avoidance success rate or latency. The text-cache test checks compositing behavior, not independent verification of Chinese glyph appearance on the deployed Jetson.

## Original project evidence

The poster and extracted figures document the project's reported platform, hardware and field activities. The project owner has confirmed real EXO integration and ROS bridge/local navigation/control integration as completed. Offline checks assess the supplied source but do not independently verify hardware integration or field performance. Source configuration contains reported braking/turning values, but no raw experiment logs establishing those values were supplied in this repository. No new performance numbers or per-person contribution claims were generated for the portfolio.
