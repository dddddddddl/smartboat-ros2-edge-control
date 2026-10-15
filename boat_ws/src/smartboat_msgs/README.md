# ROS interfaces

This `ament_cmake` package declares four maintained messages:

| Definition | Use |
|---|---|
| `FusionDetection.msg` | Camera sensor readiness, obstacle/range/decision evidence and inference timing |
| `RadarStatus.msg` | Radar status interface |
| `RadarTarget.msg` | Individual radar target |
| `RadarTargetArray.msg` | Radar target collection |

`CMakeLists.txt` registers these definitions with `rosidl_generate_interfaces` and declares the `std_msgs` dependency. Empty `srv/` and `action/` scaffolds do not represent implemented services/actions.

Not every diagnostic topic uses these messages: some radar status/target topics are still JSON strings. Inspect publishers/subscribers for actual wire types rather than assuming a message definition is used everywhere.

The message text, manifest and CMake references were inspected; generated message bindings were not built on the host.
