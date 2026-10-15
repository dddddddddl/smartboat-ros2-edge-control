# Radar and water-quality drivers

This package contains the maintained SR73F radar driver and YSI EXO water-quality acquisition/upload path.

| Source | Responsibility |
|---|---|
| `sr73f_protocol.py` | Pure CAN target decoding and target-window helpers |
| `sr73f_radar_node.py` | SocketCAN interface management, scan freshness and ROS radar output |
| `exo_serial.py` | USB serial selection, EXO command/sample protocol and sensor provider |
| `water_quality_node.py` | Device identity, asynchronous WebSocket client and provider injection |

Read the protocol helpers and `test/` first, then the hardware-facing nodes. The radar uses scan status `0x60A` and target data `0x60B`; its maintained configuration is `config/sr73f_radar.yaml`. It can configure a down SocketCAN interface using the upstream passwordless `sudo -n ip link` setup. It does not depend on a demo script for production startup.

The water-quality node instantiates `ExoSensorProvider`, not a mock provider. Serial selection and parameter order are configured in `smartboat_mission/config/edge_nodes.yaml`. The default sample rate is 0.5 Hz; the WebSocket client has a separate upload/reconnect loop.

The existing JSON payload uses `timestamp`, `station`, `boat` and `data` with name/value observations. Chinese measurement labels and their units remain unchanged because the shore interface uses those names. A WebSocket failure does not deliberately stop the provider's serial acquisition.

Radar parsing and EXO record/mapping tests were run offline. The physical radar, EXO probe, adapters, permissions and remote upload service were not exercised. The edge repository does not include the shore web application's source.
