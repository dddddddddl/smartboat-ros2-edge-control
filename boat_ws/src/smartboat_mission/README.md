# Mission link and launch source

**Project status from the poster:** MAVLink dynamic-port routing is implemented; the ROS-native PX4 bridge and navigation/control integration remain under development.

`smartboat_mission/mavlink_autoconnect.py` reads vessel identity, requests a dedicated server port and starts `mavlink-routerd` with host-provided configuration. The poster describes it as an external process. The source registers a `mavlink_autoconnect` console entry and launches it through `ExecuteProcess` with respawning.

`launch/edge_nodes.launch.py` is the supplied whole-workspace launch. In addition to routing/telemetry, it references ROS perception, radar, yaw, supervision, execution and a PX4 command bridge. Those additional entries are development-source contents, not extra completed outcomes reported by the poster. `debug:=true` changes logging.

`config/edge_nodes.yaml` contains upload/sensor settings. The supplied water-quality node references EXO development source; the poster's reported telemetry milestone remains mock-data upload.

The shore service, host router installation/configuration and deployment identity files are not bundled. No live launch or shore connection was performed. Read the [build guide](../../../docs/deployment.md) before using source launch files on a target vessel.
