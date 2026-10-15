# Perception source and development modules

**Source scope:** D435i smoke tests and EWaSR inference tools are available alongside ROS perception and dashboard modules. Local navigation/control completion is described in the navigation and bridge guides.

Manual tools are under `boat_ws/scripts/`, including `realsense_ewasr_fusion_avoidance_demo.py` and `realsense_ewasr_fusion_avoidance_viz_demo.py`.

| Package source | Inspection guide |
|---|---|
| `ewasr_core.py` | Model resolution and ONNX Runtime session creation |
| `realsense_source.py` | Colour/depth acquisition and alignment |
| `fusion_model.py` | Segmentation/depth calculations |
| `ewasr_fusion_node.py` | ROS perception integration module |
| `dashboard_pipeline.py` | Timestamp synchronization and latest-only worker |
| `dashboard_text.py` | Cached Chinese label rendering |
| `dashboard_stream_node.py` | ROS dashboard/FFmpeg/SRT module |

The package includes ROS message publication and dashboard code for inspection. The perception node uses `smartboat_msgs/FusionDetection`; configuration is in `config/`.

The model and target wheels are available as [runtime assets](../../../docs/runtime-assets.md). Offline fusion, synchronization and text-cache checks passed. Model inference, camera acquisition and streaming were not executed during repository preparation.
