"""Decode SR73F object frames used by the Smartboat radar."""

import math


RADAR_STATUS_CAN_ID = 0x60A
RADAR_OBJECT_CAN_ID = 0x60B


def parse_object_general(frame_id, data):
    """Decode one SR73F Obj_1_General CAN frame."""
    if frame_id != RADAR_OBJECT_CAN_ID or len(data) < 8:
        return None

    object_id = data[0]
    dist_long_raw = (data[1] << 5) | ((data[2] >> 3) & 0x1F)
    dist_lat_raw = ((data[2] & 0x07) << 8) | data[3]
    vrel_long_raw = (data[4] << 2) | ((data[5] >> 6) & 0x03)
    vrel_lat_raw = ((data[5] & 0x3F) << 3) | ((data[6] >> 5) & 0x07)

    dist_long = dist_long_raw * 0.2 - 500.0
    dist_lat = dist_lat_raw * 0.2 - 204.6
    vrel_long = vrel_long_raw * 0.25 - 128.0
    vrel_lat = vrel_lat_raw * 0.25 - 64.0
    distance = math.hypot(dist_long, dist_lat)
    bearing_deg = math.degrees(math.atan2(dist_lat, dist_long))

    return {
        'id': int(object_id),
        'dist_long': float(dist_long),
        'dist_lat': float(dist_lat),
        'vrel_long': float(vrel_long),
        'vrel_lat': float(vrel_lat),
        'dyn_prop': int(data[6] & 0x07),
        'rcs': float(data[7] * 0.5 - 64.0),
        'distance': float(distance),
        'bearing_deg': float(bearing_deg),
    }


def object_in_detection_window(
    obj,
    *,
    minimum_distance_m,
    maximum_distance_m,
    field_of_view_deg,
    minimum_rcs,
):
    """Return whether an object is usable by the forward avoidance radar."""
    if obj is None or obj['dist_long'] <= 0.0:
        return False
    if obj['distance'] < minimum_distance_m:
        return False
    if obj['distance'] > maximum_distance_m:
        return False
    if abs(obj['bearing_deg']) > field_of_view_deg * 0.5:
        return False
    if obj['rcs'] < minimum_rcs:
        return False
    return True


def distance_from_bow(obj, radar_to_bow_m):
    """Calculate obstacle range from the bow rather than the radar origin."""
    longitudinal = obj['dist_long'] - max(0.0, float(radar_to_bow_m))
    return math.hypot(longitudinal, obj['dist_lat'])


def recent_hits_confirm_target(
    hit_times_sec,
    *,
    now_sec,
    minimum_frames,
    confirmation_window_sec,
):
    """Require multiple actual target frames inside a bounded time window."""
    now = float(now_sec)
    window = float(confirmation_window_sec)
    required = int(minimum_frames)
    if not math.isfinite(now) or not math.isfinite(window):
        return False
    if window <= 0.0 or required <= 0:
        return False
    recent = 0
    for timestamp in hit_times_sec:
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            continue
        age = now - timestamp
        if 0.0 <= age <= window:
            recent += 1
    return recent >= required
