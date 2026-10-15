import pytest

from smartboat_drivers.exo_serial import (
    ExoProtocolError,
    build_telemetry_data,
    extract_parameter_codes,
    parse_sample_record,
    validate_parameter_codes,
)


PARAMETER_CODES = [5, 1, 18, 212, 48, 193, 215, 37]


def test_extract_parameter_codes_from_para_response():
    response = 'para\r\n#\r\n5 1 18 212 48 193 215 37\r\n'

    assert extract_parameter_codes(response) == PARAMETER_CODES


def test_parse_sample_and_add_units_to_server_field_names():
    sample = parse_sample_record(
        '2501.2 22.4 7.18 8.03 0.42 12.6 4200 9.7',
        PARAMETER_CODES,
    )

    assert sample is not None
    assert build_telemetry_data(sample) == [
        {'name': '叶绿素（μg/L）', 'value': 12.6},
        {'name': '电导率（μS/cm）', 'value': 2501.2},
        {'name': '溶解氧（mg/L）', 'value': 8.03},
        {'name': '铵态氮（mg/L-N）', 'value': 0.42},
        {'name': '总藻-PC（cells/mL）', 'value': 4200.0},
        {'name': '浊度（NTU）', 'value': 9.7},
        {'name': '温度（°C）', 'value': 22.4},
        {'name': 'pH（无量纲）', 'value': 7.18},
    ]


def test_parameter_order_must_match_configuration():
    with pytest.raises(ExoProtocolError, match='parameter order'):
        validate_parameter_codes(list(reversed(PARAMETER_CODES)), PARAMETER_CODES)


def test_negative_sensor_values_are_preserved():
    telemetry = build_telemetry_data({193: -0.08, 37: -0.52})

    assert telemetry == [
        {'name': '叶绿素（μg/L）', 'value': -0.08},
        {'name': '浊度（NTU）', 'value': -0.52},
    ]


def test_malformed_record_is_rejected_instead_of_mislabeled():
    with pytest.raises(ExoProtocolError, match='record length'):
        parse_sample_record('1 2 3', PARAMETER_CODES)
