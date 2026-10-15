# Manual diagnostics and vendor references

This directory contains manual programs for RealSense/EWaSR inspection, inference benchmarks, radar visualization, CAN adapter setup and vessel-dynamics measurement. The poster reports D435i smoke/inference tooling as implemented; radar and dynamics utilities are retained as additional source for review, not as extra completed milestones. The supplied ROS launch resolves package executables under `boat_ws/src`.

Start with the [workspace guide](../README.md) for source layout and current project status. The original detailed [Chinese diagnostic manual](../../docs/reference/diagnostics_guide.zh-CN.md) is preserved for target commands and troubleshooting. Some startup instructions in that document differ from the current self-configuring radar node.

Hardware/driver installation scripts can modify a Jetson host, and active dynamics modes can request physical motion. They were not executed during this portfolio preparation. The vendor USB-CAN tree retains its original paths and vendor notices.
