# Legacy bringup package

This package is retained from the upstream workspace. The retired camera-only avoidance PID lives in `legacy/avoidance_pid/` with its original README, launch, YAML and node source. It is outside the maintained main control chain.

Current whole-vessel composition is in `smartboat_mission`. Current local safety/execution logic is in `smartboat_navigation`, and vehicle output belongs to `smartboat_px4_bridge`. Do not treat the legacy PID's interfaces or the older field-test document as the current default deployment.

The ament copyright/style/docstring tests in `test/` are preserved. They require the ROS/ament environment and were not run here.
