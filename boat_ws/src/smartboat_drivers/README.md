# Sensor and telemetry source

**Current project status:** real YSI EXO sensor acquisition and WebSocket upload are completed.

| Source | Inspection guide |
|---|---|
| `water_quality_node.py` | Sensor-provider interface, identity loading and WebSocket payload/client |
| `exo_serial.py` | EXO serial protocol, field mapping and real sensor provider |
| `sr73f_protocol.py` | Radar CAN decoding and target-window helpers |
| `sr73f_radar_node.py` | SocketCAN and ROS radar development source |

The water-quality node starts `ExoSensorProvider`, which reads the EXO probe through the configured RS-232 adapter and supplies measurements to the telemetry client. SR73F source remains available for inspection; this status update does not add a separate radar hardware-validation claim.

`smartboat_mission/config/edge_nodes.yaml` contains sensor/upload settings, and `config/sr73f_radar.yaml` contains radar settings. The JSON upload structure uses `timestamp`, `station`, `boat` and `data`; Chinese measurement labels follow `exo_serial.py`.

Existing protocol/mapping tests passed offline. No physical probe, radar, serial/CAN device or shore upload service was exercised during repository preparation.
