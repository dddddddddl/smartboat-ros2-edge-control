# ROS-native PX4 command bridge

**Current project status:** the ROS-native PX4 bridge is completed and integrates local navigation/control requests with PX4 output. The external routing helper provides a separate MAVLink communication link.

`smartboat_px4_bridge/cmd_vel_to_mavlink_node.py` and `safety_event_recorder.py` are retained for source inspection. They contain Offboard velocity-setpoint, RC/manual-priority, command-gating and event-recording logic. The project completion update does not constitute a new field-validation result.

The manual-control helper retains GPS-speed PI settings. The Offboard output path sends velocity setpoints without calling that helper.

## Actual launch configuration

The supplied `config/cmd_vel_to_mavlink.yaml` sets **`output_enabled: true`** and is referenced by the main source launch. These actual settings must be reviewed before connecting hardware; output remains subject to the source's control gates. The shore-test launch/YAML are separate files.

Inputs and outputs include `/smartboat/control/cmd_vel`, `/smartboat/control/avoidance_active` and `/smartboat/px4_bridge/status`. Read the source and YAML for the complete interfaces and gates.

No bridge process, PX4 connection, arming, mode change or actuator output was executed during repository preparation. ROS-dependent bridge tests were not run; independent event-recorder tests passed offline.
