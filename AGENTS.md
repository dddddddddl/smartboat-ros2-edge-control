# AGENTS.md - Smartboat Codex Guide

This file is the short, high-priority guide for Codex or other coding agents working in this repository. Keep it concise. Use `README.md` for the English project guide and `docs/reference/maintenance_guide.zh-CN.md` for the original maintenance manual.

## Project presentation

The project owner has explicitly updated the status: real YSI EXO acquisition/upload, ROS-native PX4 bridging and local navigation/control integration are completed. This update supersedes the corresponding pending-status text in the supplied poster. Do not infer completed dedicated localization, further custom-message integration, new hardware tests or quantitative performance results. Preserve actual runtime settings and existing verification limits.

## Repository Root

The Git repository root is:

```bash
/home/ubuntu2204/smartboat
```

The ROS 2 workspace is:

```bash
/home/ubuntu2204/smartboat/boat_ws
```

Inside the Docker container it is mounted as:

```bash
/workspace/boat_ws
```

Run repository-level commands from `~/smartboat`. Run ROS build/test commands from `~/smartboat/boat_ws` or `/workspace/boat_ws`.

## What To Read First

1. Read this file first.
2. Read `README.md` and the original maintenance manual only when you need detailed launch, hardware, topic, Docker, or troubleshooting context.
3. Prefer inspecting the actual source files before making changes.

## Runtime Assumptions

The primary runtime target is Ubuntu 22.04 with ROS 2 Humble inside Docker.

Assume the maintained Python runtime is Python 3.10 unless the user explicitly requests otherwise.

The primary embedded/runtime hardware target is Jetson/aarch64. Runtime assets for Jetson are intentionally kept outside normal Git history and are expected to be supplied as release assets or local files.

Do not upgrade or change the ROS distribution, Ubuntu base version, Python major/minor version, CUDA/TensorRT stack, ONNX Runtime version, RealSense SDK/runtime expectation, or Jetson/aarch64 wheel versions unless the user explicitly asks for that migration.

Current pinned runtime asset expectations include:

```text
boat_ws/models/ewasr_resnet18.onnx
boat_ws/wheels/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl
```

If a local development machine differs from the target runtime, prefer changes that remain compatible with the Docker/Jetson target instead of optimizing for the local machine only.

## Current Runtime Units

The maintained ROS/runtime code is:

- `boat_ws/src/smartboat_perception/smartboat_perception/ewasr_fusion_node.py`
  - Formal RealSense D435i + EWaSR safety-perception node.
- `boat_ws/src/smartboat_perception/smartboat_perception/dashboard_stream_node.py`
  - Independent RealSense/EWaSR dashboard to SRT stream via `ffmpeg`; Chinese
    labels are pre-rendered once and composited with OpenCV/NumPy at runtime.
    Complete visualization frames are timestamp-synchronized and handled by a
    latest-only worker; do not reintroduce a fixed-rate render timer or block
    ROS callbacks on rendering/stream writes.
- `boat_ws/src/smartboat_drivers/smartboat_drivers/water_quality_node.py`
  - YSI EXO water-quality upload over the existing WebSocket protocol; serial
    acquisition is injected through the independent `exo_serial.py` provider.
- `boat_ws/src/smartboat_drivers/smartboat_drivers/sr73f_radar_node.py`
  - SR73F SocketCAN radar driver; uses 0x60A scan status for freshness and
    publishes front 0x60B targets whose raw radar distance is 3-10m. It brings
    up a DOWN interface itself through passwordless `sudo -n ip link` and must
    not call the manual scripts for production startup.
- `boat_ws/src/smartboat_navigation/smartboat_navigation/avoidance_supervisor_node.py`
  - Radar + D435i safety assessment and avoidance recommendation output.
- `boat_ws/src/smartboat_navigation/smartboat_navigation/avoidance_executor_node.py`
  - Production mode-aware avoidance execution: mission slowdown/detour/stop and
    manual slowdown/brake/reverse/hold.
- `boat_ws/src/smartboat_px4_bridge/smartboat_px4_bridge/cmd_vel_to_mavlink_node.py`
  - The single production MAVLink actuator bridge. It keeps
    `COM_RC_IN_MODE=2`, uses PX4 differential-rover Offboard body-frame vx/vy
    velocity vectors, and gives RC MANUAL/Kill priority; do not launch a second output
    instance.
- `boat_ws/src/smartboat_mission/smartboat_mission/mavlink_autoconnect.py`
  - Requests a dynamic MAVLink port and starts `mavlink-routerd`; the main
    launch runs its installed console entry as a respawning non-ROS process so
    helper failure does not shut down the other edge nodes.
- `boat_ws/src/smartboat_mission/launch/edge_nodes.launch.py`
  - Main launch entry for installed edge nodes; it must not execute source-tree
    files from `boat_ws/scripts`.
- `boat_ws/src/smartboat_mission/config/edge_nodes.yaml`
  - Main runtime config for edge nodes.
- `boat_ws/scripts/realsense_ewasr_fusion_avoidance_demo.py`
  - Long-term RealSense D435i + EWaSR command-line fusion test script.
- `boat_ws/scripts/realsense_ewasr_fusion_avoidance_viz_demo.py`
  - Long-term RealSense D435i + EWaSR visualization/fusion test script.
  - Both scripts are self-contained and do not import other project demo files.
  - They are manual test/demo programs and are not production dependencies.

Main launch command:

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smartboat_mission edge_nodes.launch.py
```

Debug launch:

```bash
ros2 launch smartboat_mission edge_nodes.launch.py debug:=true
```

## Package Boundaries

Put code in the package that matches its responsibility:

- `smartboat_perception`: cameras, image processing, AI inference, perception models/config.
- `smartboat_drivers`: sensor drivers, hardware clients, peripheral IO.
- `smartboat_mission`: launch/orchestration and whole-boat composition only.
- `smartboat_msgs`: maintained typed `msg`/`srv`/`action` interfaces.
- `smartboat_px4_bridge`: PX4/MAVLink ROS bridge; hardware output remains gated.
- `smartboat_navigation`: safety supervision, planning, waypoints, path following.
- `smartboat_localization`: future GPS/RTK/IMU fusion and transforms.
- `smartboat_control`: yaw inputs and future low-level control algorithms.
- `smartboat_tools`: future development, diagnostics, offline tools.
- `smartboat_bringup`: skeleton/legacy bringup package. The retired camera-only
  avoidance PID is archived under `legacy/avoidance_pid`; do not add new
  business logic here unless explicitly requested.

Do not pile new scripts into `boat_ws/src` unless the user is explicitly asking for a temporary demo. Prefer a real package module and console script for maintained functionality.

### Production And Demo Boundary

- Maintained nodes, launch files, package modules, and configs must never import,
  execute, source, or resolve runtime assets relative to `boat_ws/scripts`.
- Files under `boat_ws/scripts` are manual test/demo/diagnostic tools only. A
  production launch must continue to work if that directory is absent.
- Demo files may remain self-contained. Do not make formal program behavior
  depend on their functions, CLI arguments, filenames, node names, or topics.
- Shared production behavior belongs in the responsible `smartboat_*` package;
  demos may copy or independently exercise it, but never become its runtime API.

## Generated Files And Large Assets

Do not commit or package these as normal source:

- `boat_ws/build/`
- `boat_ws/install/`
- `boat_ws/log/`
- `boat_ws/trt_engine_cache/`
- `__pycache__/`, `*.pyc`, `.pytest_cache/`
- `boat_ws/src/librealsense/`
- `boat_ws/models/*.onnx`
- `boat_ws/src/*.onnx`
- `boat_ws/wheels/*.whl`
- `*.bundle`, `*.tar`, `*.tar.gz`, `*.zip`

Models and wheels are runtime assets. The formal RealSense/EWaSR node and manual demos resolve the model from `SMARTBOAT_EWASR_MODEL`, `/workspace/boat_ws/models/ewasr_resnet18.onnx`, or `/home/ubuntu2204/smartboat/boat_ws/models/ewasr_resnet18.onnx`. These assets are intentionally ignored by Git and should be shipped as Release assets, currently expected in:

```text
boat_ws/models/ewasr_resnet18.onnx
boat_ws/wheels/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl
```

Current asset archive name:

```text
smartboat-runtime-assets.tar.gz
```

## Line Endings

The repository uses `.gitattributes` to keep text files LF-only, including Python, shell scripts, Dockerfile, YAML, XML, launch files, and Markdown. Do not convert scripts to CRLF. Binary assets are marked as binary.

## Build And Test

Typical build inside the container:

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip librealsense2
source install/setup.bash
```

The checked-out `boat_ws/src/librealsense/` tree is a local SDK source/runtime
asset, not a maintained ROS package for routine workspace builds. Always skip
its colcon package `librealsense2`; otherwise its optional GLFW examples can
fail on headless images because RandR development headers are absent. Build it
separately only when the user explicitly asks to rebuild the RealSense SDK.

Existing lightweight tests/checks include:

```bash
cd /workspace/boat_ws
python3 -m py_compile scripts/realsense_ewasr_fusion_avoidance_demo.py scripts/realsense_ewasr_fusion_avoidance_viz_demo.py
PYTHONPATH=src/smartboat_drivers:src/smartboat_navigation python3 -m pytest -q \
  src/smartboat_drivers/test/test_sr73f_protocol.py \
  src/smartboat_navigation/test/test_safety_model.py
```

For ROS package tests, use `colcon test` only when the environment has ROS sourced and dependencies available.

## Validation Matrix

Run the smallest relevant check first, then run broader checks when the change can affect runtime behavior. If a check cannot be run in the current environment, report the exact command that was skipped and why.

| Change type | Required validation |
| --- | --- |
| Python runtime node or script | `python3 -m py_compile <changed .py files>` |
| ROS package code, launch files, package metadata, or config that affects launch/runtime | `cd /workspace/boat_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-skip librealsense2` |
| `smartboat_mission` launch composition or `edge_nodes.yaml` | Build the workspace, then run `source install/setup.bash && ros2 launch smartboat_mission edge_nodes.launch.py debug:=true` when hardware/network dependencies are available or safely mockable. |
| RealSense/EWaSR scripts | `cd /workspace/boat_ws && python3 -m py_compile scripts/realsense_ewasr_fusion_avoidance_demo.py scripts/realsense_ewasr_fusion_avoidance_viz_demo.py` |
| Docker Compose or Dockerfile changes | From `~/smartboat`, run `docker compose config`. Build or restart containers only when the user explicitly asks or the change requires it. |
| Runtime asset packaging/export changes | Verify the expected model/wheel paths, list archive contents, and confirm generated archives do not include `build/`, `install/`, `log/`, caches, secrets, or unintended large assets. |
| Documentation-only changes | No ROS build is required unless the documented workflow, commands, paths, or runtime assumptions changed in a way that should be verified. |

Before declaring a coding task complete, summarize which validations were run and which were not run.

## Docker

Project-level Docker files live at:

```text
docker-compose.yml
docker/Dockerfile
Dockerfile.jetson_ros2_gpu
scripts/start_smartboat_ros2.sh
```

`scripts/start_smartboat_ros2.sh` now launches `edge_nodes.launch.py` by
default; pass `--shell` when only an interactive container shell is wanted.

The compose service/container names commonly used in this project are:

```text
service: boat_ros2
container: smartboat_ros2_g39
```

The container mounts `./boat_ws` to `/workspace/boat_ws`.

The runtime images install `iproute2`, `can-utils`, and the PyPI package
`pymavlink==2.4.49` (Jammy arm64 has no `python3-pymavlink` apt package).
They install `python3-serial` for the YSI EXO PL2303GT/RS-232 provider; the
matching `pl2303` kernel module must be loaded on the Jetson host because
containers share the host kernel.
They also install `fonts-noto-cjk` and `python3-pil` for Chinese dashboard
labels in the RealSense/EWaSR visualization stream.
The SR73F node uses `ip link` to configure SocketCAN, while the MAVLink
autoconnect, yaw, and PX4 command bridge nodes import `pymavlink`.

## Git Rules

Read-only Git inspection commands such as `git status`, `git diff`, `git log`, and `git show` are allowed when useful.

Do not run `git add`, `git commit`, `git push`, create tags, create branches, change remotes, rewrite history, or run destructive cleanup commands such as `git reset --hard` or `git clean -fdx` unless the user explicitly asks for that exact Git operation.

Before editing, inspect the current state with `git status --short` when a Git checkout is available. After editing, inspect `git diff --stat` and relevant `git diff` output so the final summary can describe exactly what changed.

Do not modify `.gitignore` or `.gitattributes` unless the change is directly required by the task. If either file is changed, explain why.

Never introduce new secrets, credentials, camera passwords, tokens, identity files, private keys, build caches, TensorRT caches, models, or wheels into normal Git history. If an existing config already contains credentials, preserve behavior but avoid duplicating or exposing them.

## Cleanup And Packaging Rules

For GitHub source export, keep source, config, launch files, tests, Docker files, and startup scripts. Exclude generated files and runtime assets listed above.

Current GitHub-transfer outputs, when generated, are:

```text
smartboat-github-ready.bundle
smartboat-github-ready.tar.gz
smartboat-runtime-assets.tar.gz
```

If regenerating these archives, delete the old ones first so the user does not accidentally upload stale files.

## Editing Rules

- Preserve existing behavior unless the user explicitly asks for cleanup or migration.
- Do not delete files unless the user has confirmed the deletion list.
- Keep ROS node names, topic names, launch arguments, and config parameter names stable unless asked to change them.
- Prefer small, focused edits.
- If you touch Docker or runtime asset handling, update `README.md` and this file when the workflow changes.
- Never commit secrets, camera passwords beyond existing config, identity files, build caches, TensorRT caches, models, or wheels to normal Git history.
