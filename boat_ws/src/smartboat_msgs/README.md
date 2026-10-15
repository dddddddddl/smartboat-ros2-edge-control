# ROS message development source

**Project status from the poster:** custom ROS messages remain under development.

This directory contains four definitions for inspection: `FusionDetection.msg`, `RadarStatus.msg`, `RadarTarget.msg` and `RadarTargetArray.msg`. `CMakeLists.txt` lists them in `rosidl_generate_interfaces` with a `std_msgs` dependency.

Definitions and build declarations are source contents, not evidence of completed system integration. Some diagnostic topics use JSON strings rather than these messages; inspect the publisher/subscriber code for actual types. Empty `srv/` and `action/` directories do not represent implemented interfaces.

Message generation and a ROS build were not run on the review host.
