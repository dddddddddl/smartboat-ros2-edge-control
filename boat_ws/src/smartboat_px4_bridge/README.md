# PX4 command bridge and event recording

`smartboat_px4_bridge/cmd_vel_to_mavlink_node.py` is the maintained vehicle output bridge. `safety_event_recorder.py` records state transitions to a local JSONL file without blocking the bridge's main path.

## Read the bridge

The important boundaries are output ownership, valid PX4 heartbeat, flight/armed state, fresh command/activation input and RC/manual takeover. An exclusive lock prevents two enabled bridge instances from controlling the same output path.

The current implementation uses PX4 Offboard body-frame velocity setpoints. It retains RC input mode 2 and separate behavior for manual release, activation timeout, physical Kill and emergency handoff. The configuration retains GPS-speed PI-controller settings in the manual-control helper. The active Offboard output path sends velocity setpoints and does not call that helper. These are source behaviors; their physical timing and performance were not verified here.

## Configuration and output default

`config/cmd_vel_to_mavlink.yaml` is the production configuration. It sets **`output_enabled: true`** and is loaded by the main edge launch. Additional gates still apply, but the default is not a passive inspection profile. The shore-test YAML and launch are separate artifacts and should not be confused with the production defaults.

Key interfaces include:

- input `/smartboat/control/cmd_vel`;
- activation `/smartboat/control/avoidance_active`;
- status `/smartboat/px4_bridge/status`;
- PX4 mode/armed state topics;
- manual-fallback and emergency-Kill request topics.

No bridge process, PX4 connection, mode switch, arming operation or output was executed during this repository preparation. The existing ROS-dependent bridge tests remain in `test/`; they require the ROS/message environment and were not run on the review host. The independent event-recorder tests passed.
