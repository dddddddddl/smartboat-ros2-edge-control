# Jetson deployment guide

The runtime targets Jetson/aarch64 with Ubuntu 22.04, ROS 2 Humble and Python 3.10. The poster identifies a Jetson Orin NX and D435i. The Docker recipes use the L4T JetPack R36 family, while the host-specific version in the poster is historical platform information. This edition does not change that software stack.

## Before a target build

1. Restore the original model and aarch64 wheels using [runtime-assets.md](runtime-assets.md).
2. Review `docker-compose.yml` and `docker/Dockerfile` against the actual Jetson installation. The recipes require NVIDIA runtime access and bind host device/router/identity resources.
3. Provide the host `mavlink-routerd` binary and `/etc/mavlink-router` configuration expected by Compose. Those host resources are not bundled in the repository.
4. Supply site-specific `name.txt` and `boat_name.txt` identity files at the mounted location, and review server addresses in configuration and the MAVLink helper.
5. Match the EXO USB serial identity, SocketCAN adapter and camera to the intended physical vessel. No real identities, sensor configuration or server reachability are established by the offline tests.

The repository's root is wherever you cloned it. Several deployment scripts retain the original host path `/home/ubuntu2204/smartboat`, and the container workspace is `/workspace/boat_ws`. The original target user/container settings are retained rather than rewritten for the reviewer's computer.

## Build on the prepared target

From the repository root on the Jetson:

```bash
docker compose config
docker compose build boat_ros2
docker compose up -d boat_ros2
docker exec -it smartboat_ros2_g39 bash
```

The NVIDIA base image, GPU libraries and architecture-specific wheels make this a target deployment workflow, not a Docker Desktop/macOS quick start. The Compose service uses host networking, device mounts and privileged hardware access in the deployment configuration. It was not started during this preparation.

Inside the container:

```bash
cd /workspace/boat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip librealsense2
source install/setup.bash
```

The `librealsense` SDK checkout, if restored locally, is not a routine maintained ROS package. The source guide calls for skipping its `librealsense2` colcon package; rebuilding the SDK is a separate operation.

## Main launch and individual entries

On the prepared target, the main entry is:

```bash
ros2 launch smartboat_mission edge_nodes.launch.py
```

`debug:=true` changes maintained node logging. The launch includes perception, streaming, EXO upload, radar, yaw, supervision, execution, the PX4 bridge and the MAVLink helper. Defaults enable control-related nodes and the bridge configuration enables hardware output subject to additional gates. Review [PX4 command ownership](../boat_ws/src/smartboat_px4_bridge/README.md) before a live launch.

For inspecting individual components after a successful target build, the registered entry points include:

```text
smartboat_perception: dashboard_stream_node, ewasr_fusion_node
smartboat_drivers: water_quality_node, sr73f_radar_node
smartboat_mission: mavlink_autoconnect
smartboat_control: yaw_from_imu_node, yaw_from_mavlink_node
smartboat_navigation: avoidance_supervisor_node, avoidance_executor_node
smartboat_px4_bridge: cmd_vel_to_mavlink_node
```

Use `ros2 pkg executables <package>` in the built environment to inspect the installed entries. `mavlink_autoconnect` is installed as a console executable but runs as a non-ROS helper process in the main launch. The older maintenance guide's claim that it has no console entry is outdated.

## Original operator material

The [Chinese maintenance guide](reference/maintenance_guide.zh-CN.md) retains detailed target commands, NoMachine/display setup, sensor notes and troubleshooting. The [older field-test procedure](reference/field_test_procedure.zh-CN.md) contains historical PID and mode assumptions that differ from the current source. They remain useful project references, but their older default-output and mode statements should not be copied into a current live test without reconciling the code/configuration.

No Docker image build, ROS build, cloud/shore connection or hardware startup was performed on this macOS host. See [validation.md](validation.md) for what was checked instead.
