# Architecture and implementation status

The edge stack separates sensor acquisition, perception, safety assessment, command execution and visualization. This separation lets a dashboard/streaming failure remain outside the primary perception and control callbacks.

## Implementation status

| Area | Current implementation | Boundary |
|---|---|---|
| Water-quality acquisition | `ExoSensorProvider` parses YSI EXO serial samples and supplies the WebSocket client | Real sensor integration exists in code; no probe was attached here |
| Camera perception | `ewasr_fusion_node` acquires D435i frames and runs EWaSR/depth fusion | Model/Jetson dependencies are external runtime assets |
| Visualization | `dashboard_stream_node` synchronizes color/depth/mask/detections and streams a composed dashboard | Preserves the Chinese interface text and SRT identity conventions |
| Radar | `sr73f_radar_node` manages SocketCAN and parses scan/target messages | Requires the intended CAN adapter, driver and radar |
| Safety assessment | `SafetyModel` and ROS supervisor combine range, geometry, speed and freshness | Model tests are not proof of physical detection reliability |
| Avoidance execution | `AvoidanceExecutionModel` and node handle mission/manual phases | Not a complete global route planner |
| Vehicle bridge | `cmd_vel_to_mavlink_node` owns output and implements activation/RC/fallback gates | Live PX4 transport/output was not exercised here |
| Mission composition | `edge_nodes.launch.py` starts installed nodes and the routing helper | Requires the shore service, router configuration and local device identity |
| Localization/tools folders | Scaffolds | No dedicated state-estimation stack or packaged tools implementation |

The supplied poster describes an earlier implementation stage. Its mock-provider and unimplemented ROS-bridge statements do not describe the current implementation.

## Perception and data interfaces

The RealSense node aligns color/depth inputs, performs EWaSR inference and publishes a typed `FusionDetection`. The pure `fusion_model.py` calculates obstacle evidence and a recommended straight/left/right decision. Its configuration bounds valid depth, obstacle pixel count and near-depth sampling. These are configuration choices, not measured accuracy claims.

The dashboard consumes synchronized visualization inputs. It pairs exact timestamps and uses a latest-only worker for composition and stream writes, so it does not build a queue of obsolete frames or block the ROS callbacks on FFmpeg.

The SR73F driver uses scan status `0x60A` for freshness and target messages `0x60B` for geometry. Its configured raw target window is 3–10 m ahead within the forward field of view. Source geometry applies the radar-to-bow offset before reporting an obstacle range. An empty fresh scan and a stale/no-frame sensor are distinct states.

The EXO provider validates the configured sensor parameter order before mapping samples to the existing server field labels. Sampling/reconnection is independent of WebSocket upload. The payload carries timestamp, station, boat and a list of name/value observations. Labels are intentionally unchanged Chinese strings because the existing shore server renders those names directly.

## Main ROS topics

| Topic | Use |
|---|---|
| `/smartboat/perception/fusion/detection` | Typed camera fusion result |
| `/smartboat/perception/radar/closest_obstacle_distance` | Bow-relative radar range |
| `/smartboat/perception/radar/objects` and `/status` | Radar diagnostics and target data |
| `/smartboat/localization/yaw` | Yaw input from a maintained reader |
| `/smartboat/localization/ground_speed` and `/ground_speed_valid` | Speed estimate and validity |
| `/smartboat/navigation/avoidance_cmd_vel` | Supervisor recommendation |
| `/smartboat/navigation/avoidance_supervisor_status` | Safety assessment status |
| `/smartboat/control/cmd_vel` | Executor request to the command bridge |
| `/smartboat/control/avoidance_active` | Executor activation heartbeat |
| `/smartboat/control/manual_fallback_requested` | Manual fallback request |
| `/smartboat/px4_bridge/status`, `/flight_mode`, `/armed` | Bridge/PX4 state |

Topic types and exact defaults remain in the source/YAML; this table is a reading map, not a replacement message specification. Some legacy topics remain for compatibility.

## Control ownership

The supervisor recommends a response. The executor turns that assessment into mode-aware control requests. A single MAVLink command bridge applies output-enabled, armed/mode/heartbeat, activation and command-freshness gates before attempting PX4 output. An exclusive output lock prevents a second enabled bridge instance from sharing command ownership.

The current bridge uses body-frame Offboard velocity setpoints and explicit RC/manual priority behavior. Several older comments and the preserved field-test document describe earlier manual-input or PID behavior. Read the actual bridge source and configuration for the current implementation.

The main YAML has `output_enabled: true`; automatic reverse in the executor is separately disabled by default because the maintained stack has no rear-clearance sensing. These settings were preserved. This edition did not activate the control chain, change modes, arm a controller or run a vehicle.

## External systems

The repository implements the vessel edge side. It expects a shore WebSocket service, SRT receiver, dynamic MAVLink-port service, identity files and a host `mavlink-routerd` setup. It does not include the shore dashboard's source, PX4 firmware, vessel electronics CAD, training data or a complete independent deployment of those external services.

Source configuration includes specific reported braking/turning measurements, but the corresponding raw experimental logs were not supplied in this repository. They are not presented here as independently reproduced performance results.
