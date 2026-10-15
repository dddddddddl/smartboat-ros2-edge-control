# Perception and dashboard streaming

This package contains the maintained camera/perception path and an independent visualization/streaming path.

| Source | Responsibility |
|---|---|
| `ewasr_core.py` | Model path resolution and ONNX Runtime session creation |
| `realsense_source.py` | RealSense acquisition and frame handling |
| `fusion_model.py` | Pure segmentation/depth fusion calculations |
| `ewasr_fusion_node.py` | ROS node publishing fusion results and synchronized visualization inputs |
| `dashboard_pipeline.py` | Exact-timestamp synchronization and latest-only background work |
| `dashboard_text.py` | Pre-rendered Chinese label sprites |
| `dashboard_stream_node.py` | Dashboard composition and FFmpeg/SRT output |

Start with `fusion_model.py` and its tests, then inspect the ROS node. The node publishes `smartboat_msgs/FusionDetection` on `/smartboat/perception/fusion/detection`. `config/ewasr_fusion.yaml` records the supplied depth/pixel/decision thresholds.

The dashboard does not acquire the camera or publish safety decisions. It consumes complete matching color/depth/mask/detection inputs and streams frames independently. Display text, font expectations and the upstream SRT station/boat identity format are preserved.

The default model is `boat_ws/models/ewasr_resnet18.onnx`, resolved through configuration, `SMARTBOAT_EWASR_MODEL` or target workspace paths. The model and architecture-specific wheels are supplied separately as [runtime assets](../../../docs/runtime-assets.md). No model inference, camera capture or stream connection was run in this preparation. Offline fusion, synchronization and text-cache tests passed.
