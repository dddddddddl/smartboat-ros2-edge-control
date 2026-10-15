# PX4 bridge development source

**Project status from the poster:** the ROS-native PX4 bridge remains under development. The reported implemented mission link is the external MAVLink routing helper.

`smartboat_px4_bridge/cmd_vel_to_mavlink_node.py` and `safety_event_recorder.py` are retained for source inspection. They contain Offboard velocity-setpoint, RC/manual-priority, command-gating and event-recording logic. Their presence does not establish a completed or field-validated bridge milestone.

The manual-control helper retains GPS-speed PI settings. The Offboard output path sends velocity setpoints without calling that helper.

## Actual launch configuration

The supplied `config/cmd_vel_to_mavlink.yaml` sets **`output_enabled: true`** and is referenced by the main source launch. These actual settings must be reviewed before connecting hardware, even though the project presentation classifies bridge integration as development work. The shore-test launch/YAML are separate files.

Inputs and outputs include `/smartboat/control/cmd_vel`, `/smartboat/control/avoidance_active` and `/smartboat/px4_bridge/status`. Read the source and YAML for the complete interfaces and gates.

No bridge process, PX4 connection, arming, mode change or actuator output was executed during repository preparation. ROS-dependent bridge tests were not run; independent event-recorder tests passed offline.
