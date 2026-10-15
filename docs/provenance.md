# Provenance and portfolio preparation

## Upstream source

This public repository is a curated source snapshot of [xyx-chn/2026SURF_SmartBoat_ROS2_EdgeSide](https://github.com/xyx-chn/2026SURF_SmartBoat_ROS2_EdgeSide), inspected at commit:

```text
8728b8be6ead61c0110a7790d3042cdc4a029612
```

The source repository is private, so the upstream link may require authorized access. The portfolio owner explicitly confirmed permission to publish the source. This independent public repository includes the code, poster and a mirrored runtime archive so reviewers can use the published materials without access to that private repository.

Public Git history starts with the curated import; it does not reproduce the private repository history. A separate private fork retains that history for traceability. The source revision above identifies the imported implementation. Inclusion does not establish that the portfolio owner individually authored every module.

## Changes in this edition

- Added an English project README, architecture/deployment/asset guides and short package reading guides.
- Added the supplied English SURF poster unchanged, plus selected original figures extracted from it. The original poster's embedded authorship and references remain intact.
- Preserved the original long Chinese maintenance and field-test documents under `docs/upstream/`, with clear notes about stale implementation statements.
- Added an offline runner for existing upstream tests and a separately documented host-only dependency file. No test assertions or runtime algorithms were rewritten.
- Updated agent/document pointers to the new main README and marked the PPTX as binary under the existing line-ending policy. The ignore-file comment now points to the asset guide; ignore behavior is unchanged.

The source-export conversation transcript (`declaration.txt`) is excluded from the current tree because it records an earlier packaging conversation, not project implementation. It is retained locally and in the private upstream history, outside this public repository. No runtime endpoints, topic names, parameters, motion limits, control gates, Docker target versions, telemetry labels or source algorithms were changed.

## Team credit

The supplied SURF 0634 poster credits Yuxiao Xie, Kaihan Ouyang, Siyuan Lin, Ruoxi Zhu, Shengzhe Li and Yijia Cao. It names Xiaohui Zhu and Yong Yue as supervisors at Xi'an Jiaotong-Liverpool University. Per-person module ownership was not specified, so no individual contribution breakdown is invented.

The poster reports a National Second Prize in the 15th National Marine Vehicle Design and Manufacturing Competition. It is retained as an attributed project statement; no separate award certificate was provided for independent inspection here.

## License and third-party content

No top-level project license was found upstream; several package metadata entries contain `TODO` license declarations. This edition preserves those entries and does not assign a new license or relicense the upstream material. Existing Open Source Robotics Foundation headers and vendor notices remain unchanged.

The USB-CAN directory contains upstream vendor reference code. Its Chinese path and original instructions are preserved because scripts may resolve vendor resources by their existing paths. The project also depends on externally licensed ROS 2, PX4/MAVLink, OpenCV, RealSense, EWaSR model/runtime components and NVIDIA platform packages. Their presence in the stack does not make them portfolio-authored work.

## Evidence boundary

The supplied poster is a historical artifact. Its mock-water-provider and incomplete-bridge descriptions predate the inspected implementation. Code-level existence, offline tests and an original field photograph are different kinds of evidence; this edition does not equate them with newly measured reliability, sensor accuracy or successful complete missions.
