# Team and acknowledgments

## Project team

SmartBoat is a SURF 0634 team project at Xi'an Jiaotong-Liverpool University. The project poster credits **Yuxiao Xie, Kaihan Ouyang, Siyuan Lin, Ruoxi Zhu, Shengzhe Li and Yijia Cao**, with **Xiaohui Zhu and Yong Yue** as supervisors.

This repository presents the team's vessel edge software and supporting project materials. Individual module ownership is not itemized. The [English SURF poster](presentation/smartboat_surf_project_poster.pptx) provides the original project overview and team credits.

The poster reports a National Second Prize in the 15th National Marine Vehicle Design and Manufacturing Competition. A separate award certificate is not included in this repository.

## Third-party acknowledgments

The stack uses ROS 2, PX4/MAVLink, OpenCV, RealSense, EWaSR and NVIDIA platform components. The USB-CAN directory contains vendor reference code. Existing author headers, copyright notices and package license metadata are preserved; these components retain their respective terms.

## Documentation and validation notes

The English reading guides, offline test runner and validation record were added while preparing this repository for review. They document the implementation without changing its runtime algorithms, control settings or existing test assertions.

The poster and the Chinese [maintenance guide](reference/maintenance_guide.zh-CN.md), [field-test procedure](reference/field_test_procedure.zh-CN.md) and [diagnostic guide](reference/diagnostics_guide.zh-CN.md) are retained as project references. Some describe earlier implementation stages; the architecture guide identifies differences from the current code.

Photographs and screenshots document historical project activities. The repository's host checks do not establish new field-performance measurements. See the [validation record](validation.md) for the checks performed and their limits.
