# Sensor and telemetry source

**Project status from the poster:** WebSocket upload with mock data is implemented; real water-quality sensor integration remains under development.

| Source | Inspection guide |
|---|---|
| `water_quality_node.py` | Sensor-provider interface, identity loading and WebSocket payload/client |
| `exo_serial.py` | EXO serial protocol, field mapping and provider development source |
| `sr73f_protocol.py` | Radar CAN decoding and target-window helpers |
| `sr73f_radar_node.py` | SocketCAN and ROS radar development source |

The supplied node references `ExoSensorProvider`. Its presence is retained for code review and is not presented as completion of the poster's pending real-sensor milestone. The poster does not establish completed SR73F integration, so the radar files are not claimed as a demonstrated project outcome.

`smartboat_mission/config/edge_nodes.yaml` contains sensor/upload settings, and `config/sr73f_radar.yaml` contains radar settings. The JSON upload structure uses `timestamp`, `station`, `boat` and `data`; Chinese measurement labels follow `exo_serial.py`.

Existing protocol/mapping tests passed offline. No physical probe, radar, serial/CAN device or shore upload service was exercised during repository preparation.
