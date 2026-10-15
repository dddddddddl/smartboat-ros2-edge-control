import math

from smartboat_control.yaw_from_mavlink_node import (
    ground_speed_is_plausible,
    is_px4_autopilot_heartbeat,
)


class FakeHeartbeat:
    def __init__(self, system=1, component=1, autopilot=12):
        self.system = system
        self.component = component
        self.autopilot = autopilot

    def get_type(self):
        return 'HEARTBEAT'

    def get_srcSystem(self):
        return self.system

    def get_srcComponent(self):
        return self.component


def test_yaw_reader_only_accepts_px4_autopilot_heartbeat():
    assert is_px4_autopilot_heartbeat(FakeHeartbeat())
    assert not is_px4_autopilot_heartbeat(FakeHeartbeat(system=0))
    assert not is_px4_autopilot_heartbeat(FakeHeartbeat(component=190))
    assert not is_px4_autopilot_heartbeat(FakeHeartbeat(autopilot=8))


def test_yaw_reader_honors_explicit_system_id():
    assert is_px4_autopilot_heartbeat(FakeHeartbeat(system=2), 2)
    assert not is_px4_autopilot_heartbeat(FakeHeartbeat(system=2), 1)


def test_ground_speed_plausibility_rejects_invalid_estimates():
    assert ground_speed_is_plausible(0.0, 5.0)
    assert ground_speed_is_plausible(2.0, 5.0)
    assert not ground_speed_is_plausible(19.7, 5.0)
    assert not ground_speed_is_plausible(-0.1, 5.0)
    assert not ground_speed_is_plausible(math.nan, 5.0)
