# Edge launch composition and MAVLink helper

`launch/edge_nodes.launch.py` is the main maintained entry point. Read it to see the actual node composition and launch arguments. It resolves installed package executables and configuration, rather than running the manual demos from `boat_ws/scripts`.

The default composition includes EXO upload, D435i/EWaSR perception, dashboard streaming, SR73F radar, MAVLink yaw/speed, safety supervision, avoidance execution, the PX4 command bridge and a routing helper. Logging defaults to warnings; `debug:=true` enables detailed node logging. Individual enable arguments are declared in the launch file.

`smartboat_mission/mavlink_autoconnect.py` reads vessel identity, requests a dedicated server port and starts the host-provided `mavlink-routerd` configuration. It is registered as the `mavlink_autoconnect` console entry, but the main launch executes it as a respawning non-ROS process. Older notes saying that it has no installed console entry are outdated.

`config/edge_nodes.yaml` configures the EXO provider and upload service. The repository does not contain the shore service, host router installation/configuration or real deployment identity files. Existing endpoint defaults are preserved and were not contacted during preparation.

Build/deployment details are in the [target guide](../../../docs/deployment.md). The source was parsed and launch/package references inspected, but a complete ROS build or live launch was not performed here.
