# Navigation development source

**Project status from the poster:** navigation, localization and control integration remain under development; quantified avoidance performance remains future evaluation work.

| Source | Inspection guide |
|---|---|
| `safety_model.py` | Sensor freshness, ranges, speed and vessel-geometry logic |
| `avoidance_supervisor_node.py` | ROS assessment inputs and recommendation outputs |
| `avoidance_execution_model.py` | Mission/manual execution-state model |
| `avoidance_executor_node.py` | ROS wrapper for execution requests |

The two pure models and their existing tests provide a way to inspect development logic without ROS. Their offline tests passed; this is not a completed navigation-system or on-water avoidance result.

The source includes supervisor and executor YAML configurations. The executor's automatic reverse setting is false, with a comment that rear-clearance sensing is absent. Existing parameter values are retained, not promoted to independently verified performance measurements.

This directory does not contain a complete global mission planner. No ROS navigation launch or vessel movement was performed during repository preparation.
