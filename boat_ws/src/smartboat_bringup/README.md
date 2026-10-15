# Bringup and PID reference source

**Current project status:** local navigation/control integration is completed in the navigation, control and PX4-bridge packages. This package retains the legacy PID reference.

This package contains the camera-only PID reference under `legacy/avoidance_pid/`, including its README, launch, YAML and node. It is retained for inspection and is not a separately claimed completed project result.

The supplied whole-workspace launch is in `smartboat_mission`; navigation and bridge modules are in their respective packages. Read their guides before interpreting package names as completion claims.

ROS/ament style and copyright tests were not run on the review host.
