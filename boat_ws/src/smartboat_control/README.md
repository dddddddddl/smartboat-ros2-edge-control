# Yaw and ground-speed input nodes

The maintained modules are `yaw_from_imu_node.py` and `yaw_from_mavlink_node.py`. They provide yaw inputs and, for MAVLink, ground-speed/validity signals used by safety assessment and control. Configuration and individual launch files are in `config/` and `launch/`.

Read `yaw_from_mavlink_node.py` for heartbeat filtering, source-system selection and plausibility handling. The main edge launch includes the MAVLink reader. The dedicated `smartboat_localization` directory remains a scaffold; these inputs are not a complete fused GPS/RTK/IMU localization implementation.

The old camera-only avoidance PID is retained under `smartboat_bringup/legacy/avoidance_pid`, outside this maintained package. Its field-test assumptions differ from the current executor/bridge behavior.

The yaw tests are preserved but import ROS-dependent modules, so they were not executed on this macOS host. No live IMU or MAVLink stream was opened.
