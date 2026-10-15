#!/usr/bin/env bash
# Run only existing offline tests. Do not launch ROS nodes or hardware tools.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root/boat_ws/src/smartboat_drivers:$project_root/boat_ws/src/smartboat_navigation:$project_root/boat_ws/src/smartboat_perception:$project_root/boat_ws/src/smartboat_px4_bridge${PYTHONPATH:+:$PYTHONPATH}"
python -m pytest -q \
  boat_ws/src/smartboat_drivers/test/test_sr73f_protocol.py \
  boat_ws/src/smartboat_drivers/test/test_exo_serial.py \
  boat_ws/src/smartboat_navigation/test/test_safety_model.py \
  boat_ws/src/smartboat_navigation/test/test_avoidance_execution_model.py \
  boat_ws/src/smartboat_perception/test/test_fusion_model.py \
  boat_ws/src/smartboat_perception/test/test_dashboard_pipeline.py \
  boat_ws/src/smartboat_perception/test/test_dashboard_text.py \
  boat_ws/src/smartboat_px4_bridge/test/test_safety_event_recorder.py
