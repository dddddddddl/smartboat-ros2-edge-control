# Offline host checks

`run_host_checks.sh` selects eight existing test files that can run without ROS, hardware, a model or shore services. No test assertions or runtime algorithms were changed for this portfolio edition. The checks include development modules; passing them does not change the completion status reported in the [SURF poster](../docs/presentation/smartboat_surf_project_poster.pptx).

From the repository root in an isolated virtual environment:

```bash
python -m pip install -r tests/requirements-host.txt
bash tests/run_host_checks.sh
```

The recorded result is **65 passed**. The dependency pins describe the host review environment and do not modify the Jetson runtime, its Python 3.10 expectation, ONNX Runtime or TensorRT stack.

| Test area | Existing test file |
|---|---|
| Radar payload and target window | `smartboat_drivers/test/test_sr73f_protocol.py` |
| EXO serial records and telemetry field mapping | `smartboat_drivers/test/test_exo_serial.py` |
| Sensor freshness and safety decisions | `smartboat_navigation/test/test_safety_model.py` |
| Mode-aware avoidance execution | `smartboat_navigation/test/test_avoidance_execution_model.py` |
| Segmentation/depth fusion | `smartboat_perception/test/test_fusion_model.py` |
| Complete-frame synchronization and latest-only worker | `smartboat_perception/test/test_dashboard_pipeline.py` |
| Cached text sprite compositing | `smartboat_perception/test/test_dashboard_text.py` |
| Transition event persistence | `smartboat_px4_bridge/test/test_safety_event_recorder.py` |

Paths in the table are relative to `boat_ws/src/`. The cached-text test checks the rendering/cache mechanism; it does not independently verify Chinese font appearance on Jetson.

ROS-dependent bridge, yaw and dashboard-node tests and the ament linters are preserved in their original packages. They were not run on this macOS host because ROS message bindings and `rclpy` were unavailable. Use the ROS/colcon workflow in the target environment for those checks. No serial/CAN/GPU/vehicle test scripts were executed.
