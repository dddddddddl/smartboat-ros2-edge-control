from smartboat_perception.dashboard_stream_node import (
    DashboardStreamer,
    avoidance_type_chinese,
    build_srt_url,
    decision_chinese,
)
from smartboat_msgs.msg import FusionDetection


class Logger:
    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def test_srt_url_preserves_identity_rule(tmp_path):
    (tmp_path / "name.txt").write_text("station-a\n", encoding="utf-8")
    (tmp_path / "boat_name.txt").write_text("boat-7\n", encoding="utf-8")
    url = build_srt_url(
        "111.229.242.175",
        10000,
        str(tmp_path),
        "test",
        "test01",
    )
    assert url == (
        "srt://111.229.242.175:10000"
        "?streamid=publish:station-a/boat-7&pkt_size=1316&mode=caller"
    )


def test_explicit_stream_url_is_not_changed(tmp_path):
    expected = "srt://example:9000?streamid=publish:x/y"
    assert build_srt_url(
        "ignored",
        1,
        str(tmp_path),
        "test",
        "test01",
        stream_url=expected,
    ) == expected


def test_ffmpeg_command_preserves_low_latency_transport():
    streamer = DashboardStreamer(
        target_url="srt://example:10000?streamid=publish:a/b",
        ffmpeg_path="ffmpeg",
        fps=15,
        width=1280,
        height=960,
        codec="libx264",
        bitrate="1800k",
        loglevel="warning",
        logger=Logger(),
    )
    command = streamer.build_command()
    assert "rawvideo" in command
    assert "-use_wallclock_as_timestamps" in command
    assert command[command.index("-vsync") + 1] == "vfr"
    assert "zerolatency" in command
    assert "resend_headers+pat_pmt_at_frames" in command
    assert command[-1] == "srt://example:10000?streamid=publish:a/b"


def test_dashboard_status_labels_are_chinese():
    assert decision_chinese(FusionDetection.TURN_LEFT) == "左转绕行"
    assert decision_chinese(FusionDetection.TURN_RIGHT) == "右转绕行"
    label, _color = avoidance_type_chinese(True, True)
    assert label == "雷达与视觉均探测到障碍"
