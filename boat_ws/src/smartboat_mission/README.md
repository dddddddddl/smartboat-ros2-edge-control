# Mission link and launch source

**Current project status:** MAVLink routing, the ROS-native PX4 bridge and local navigation/control integration are completed.

`smartboat_mission/mavlink_autoconnect.py` reads vessel identity, requests a dedicated server port and starts `mavlink-routerd` with host-provided configuration. The poster describes it as an external process. The source registers a `mavlink_autoconnect` console entry and launches it through `ExecuteProcess` with respawning.

`launch/edge_nodes.launch.py` is the supplied whole-workspace launch. In addition to routing/telemetry, it references ROS perception, radar, yaw, supervision, execution and a PX4 command bridge. The launch composes these installed modules; independent target execution was not performed during repository preparation. `debug:=true` changes logging.

`config/edge_nodes.yaml` contains upload/sensor settings. The water-quality node starts the real EXO provider for sensor acquisition and upload.

The shore service, host router installation/configuration and deployment identity files are not bundled. No live launch or shore connection was performed. Read the [build guide](../../../docs/deployment.md) before using source launch files on a target vessel.
