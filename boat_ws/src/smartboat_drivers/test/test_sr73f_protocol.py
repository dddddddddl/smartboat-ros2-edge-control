import pytest

from smartboat_drivers.sr73f_protocol import (
    RADAR_OBJECT_CAN_ID,
    distance_from_bow,
    object_in_detection_window,
    parse_object_general,
    recent_hits_confirm_target,
)


def encode_object(
    *,
    object_id=7,
    dist_long=5.0,
    dist_lat=0.0,
    vrel_long=-1.0,
    vrel_lat=0.0,
    dyn_prop=0,
    rcs=10.0,
):
    long_raw = round((dist_long + 500.0) / 0.2)
    lat_raw = round((dist_lat + 204.6) / 0.2)
    vlong_raw = round((vrel_long + 128.0) / 0.25)
    vlat_raw = round((vrel_lat + 64.0) / 0.25)
    return bytes([
        object_id,
        (long_raw >> 5) & 0xFF,
        ((long_raw & 0x1F) << 3) | ((lat_raw >> 8) & 0x07),
        lat_raw & 0xFF,
        (vlong_raw >> 2) & 0xFF,
        ((vlong_raw & 0x03) << 6) | ((vlat_raw >> 3) & 0x3F),
        ((vlat_raw & 0x07) << 5) | (dyn_prop & 0x07),
        round((rcs + 64.0) / 0.5),
    ])


def parse(**values):
    return parse_object_general(RADAR_OBJECT_CAN_ID, encode_object(**values))


def in_window(obj, minimum=3.0, maximum=10.0):
    return object_in_detection_window(
        obj,
        minimum_distance_m=minimum,
        maximum_distance_m=maximum,
        field_of_view_deg=112.0,
        minimum_rcs=-64.0,
    )


def test_decode_object_fields():
    obj = parse(dist_long=5.0, dist_lat=1.0, vrel_long=-1.5)
    assert obj['id'] == 7
    assert obj['dist_long'] == pytest.approx(5.0, abs=0.11)
    assert obj['dist_lat'] == pytest.approx(1.0, abs=0.11)
    assert obj['vrel_long'] == pytest.approx(-1.5, abs=0.13)


@pytest.mark.parametrize(
    ('distance', 'expected'),
    [(2.8, False), (3.0, True), (7.0, True), (10.0, True), (10.2, False)],
)
def test_detection_window_is_three_to_ten_metres(distance, expected):
    assert in_window(parse(dist_long=distance)) is expected


def test_rear_object_is_rejected():
    assert not in_window(parse(dist_long=-5.0))


def test_object_outside_fov_is_rejected():
    assert not in_window(parse(dist_long=3.0, dist_lat=5.0))


def test_distance_is_reported_from_bow():
    obj = parse(dist_long=5.0, dist_lat=0.0)
    assert distance_from_bow(obj, 0.45) == pytest.approx(4.55, abs=0.11)


def test_radar_target_requires_three_recent_actual_frames():
    values = {
        'now_sec': 10.0,
        'minimum_frames': 3,
        'confirmation_window_sec': 0.5,
    }
    assert not recent_hits_confirm_target([9.8], **values)
    assert not recent_hits_confirm_target([9.0, 9.8, 9.9], **values)
    assert recent_hits_confirm_target([9.6, 9.8, 9.9], **values)
