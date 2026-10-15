# Safety supervision and avoidance execution

This package provides local safety assessment and a separate execution state machine. Its name does not imply that a complete global waypoint planner is included.

| Source | Responsibility |
|---|---|
| `safety_model.py` | Pure safety logic using sensor freshness, range, speed and vessel geometry |
| `avoidance_supervisor_node.py` | ROS inputs and safety/recommendation outputs |
| `avoidance_execution_model.py` | Pure mission/manual avoidance phases and transition logic |
| `avoidance_executor_node.py` | ROS execution wrapper and activation/command publication |

Read the two pure models with their test files before the ROS wrappers. The supervisor publishes recommendations at `/smartboat/navigation/avoidance_cmd_vel` and status at `/smartboat/navigation/avoidance_supervisor_status`. The executor publishes the maintained command request at `/smartboat/control/cmd_vel` and activation at `/smartboat/control/avoidance_active`.

`config/avoidance_supervisor.yaml` contains vessel dimensions, sensing ranges, timeout assumptions and source-reported motion measurements. The raw experiments behind those measurements are not included in this source snapshot. `config/avoidance_executor.yaml` configures mission/manual speeds and timeouts. Automatic reverse is disabled by default because rear clearance is not sensed by the maintained stack.

The command bridge owns final PX4 output. Recommendations, an executor request and actual actuator output are distinct stages. This edition preserved the source/configuration and ran the existing pure safety/execution tests; it did not run live ROS or move a vessel.
