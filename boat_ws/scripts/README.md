# Manual diagnostics and vendor references

This directory contains manual programs for RealSense/EWaSR inspection, inference benchmarks, radar visualization, CAN adapter setup and vessel-dynamics measurement. The maintained main ROS launch resolves package executables under `boat_ws/src`; these demos are not its runtime API.

Start with the [workspace guide](../README.md) for maintained code. The original detailed [Chinese diagnostic manual](../../docs/reference/diagnostics_guide.zh-CN.md) is preserved for target commands and troubleshooting. Some older startup statements in that document predate the current self-configuring radar node.

Hardware/driver installation scripts can modify a Jetson host, and active dynamics modes can request physical motion. They were not executed during this portfolio preparation. The vendor USB-CAN tree retains its original paths and vendor notices.
