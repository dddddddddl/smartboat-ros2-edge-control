# Model and Jetson runtime assets

The source repository intentionally keeps the EWaSR model and two architecture-specific wheels outside normal Git history. This portfolio edition mirrors the original upstream runtime archive as a Release attachment, without altering its contents.

[Download smartboat-runtime-assets.tar.gz](https://github.com/dddddddddl/smartboat-ros2-edge-control/releases/download/portfolio-v1.0/smartboat-runtime-assets.tar.gz)

The [upstream release](https://github.com/xyx-chn/2026SURF_SmartBoat_ROS2_EdgeSide/releases/tag/untagged-6e6ef4dd12e3e2f0c3a8) remains the provenance source. The downloaded archive was checked against its upstream GitHub SHA-256 digest before mirroring.

```text
Archive size: 315,950,592 bytes
SHA-256: 72e25bfbc07b9caa696996f544dd1e87aeb37cd7b59b4f2f2d2c8b9ba6a38dd0
```

## Contents and target paths

| Restored path | Uncompressed bytes | Purpose |
|---|---:|---|
| `boat_ws/models/ewasr_resnet18.onnx` | 241,076,632 | Supplied EWaSR segmentation model |
| `boat_ws/wheels/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl` | 145,022,838 | Python 3.10/aarch64 inference runtime wheel |
| `boat_ws/wheels/pyrealsense2-2.58.2-cp310-cp310-linux_aarch64.whl` | 5,675,266 | Python 3.10/aarch64 RealSense wheel |

The archive contains only the model/wheel directories and these three files. It contains no build/install/log/cache directory. Individual file hashes are in [runtime-assets-manifest.json](runtime-assets-manifest.json).

After downloading, verify it before restoring at the repository root:

```bash
shasum -a 256 smartboat-runtime-assets.tar.gz
tar -tzf smartboat-runtime-assets.tar.gz
tar -xzf smartboat-runtime-assets.tar.gz
```

The wheels are target runtime assets, not packages to install into a macOS review environment. The Docker recipe expects those exact paths. The model's training process, original training dataset and independent model-quality evaluation are not supplied by this asset archive. The inherited components retain their upstream authorship and licensing status.

Restoring files does not build the ROS workspace, install the host router/driver services or validate GPU/camera compatibility. Follow [deployment.md](deployment.md) on the intended target.
