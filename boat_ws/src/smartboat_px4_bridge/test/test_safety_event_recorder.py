import json

from smartboat_px4_bridge.safety_event_recorder import SafetyEventRecorder


def test_recorder_persists_ordered_jsonl(tmp_path):
    path = tmp_path / 'events.jsonl'
    recorder = SafetyEventRecorder(path)
    assert recorder.record('mode_change', mode='MANUAL', armed=False)
    assert recorder.record('kill', engaged=True, pwm=1933)
    recorder.close()

    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert [record['event'] for record in records] == [
        'mode_change',
        'kill',
    ]
    assert records[0]['mode'] == 'MANUAL'
    assert records[1]['pwm'] == 1933


def test_disabled_recorder_does_not_create_file(tmp_path):
    path = tmp_path / 'disabled.jsonl'
    recorder = SafetyEventRecorder(path, enabled=False)
    assert not recorder.record('ignored')
    recorder.close()
    assert not path.exists()
