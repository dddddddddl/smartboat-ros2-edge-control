# Integrated control-input source

**Current project status:** local control integration is completed. Dedicated localization remains under development.

The source includes `yaw_from_imu_node.py` and `yaw_from_mavlink_node.py`, with configuration and individual launch files in `config/` and `launch/`. The MAVLink reader contains yaw, ground-speed/validity, heartbeat filtering and source-selection logic.

These modules are available for source review. They supply yaw and speed inputs for local control; they do not implement a fused GPS/RTK/IMU localization system. The dedicated `smartboat_localization` directory remains a scaffold.

The camera-only PID reference is under `smartboat_bringup/legacy/avoidance_pid`. ROS-dependent yaw tests were not run, and no live IMU/MAVLink stream was opened during repository preparation.
