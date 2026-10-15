import asyncio
import re
import threading
from time import monotonic, sleep
from typing import Callable, Dict, List, Optional

import serial
from serial.tools import list_ports


PROLIFIC_VID = 0x067B
PROLIFIC_PID = 0x23A3
NUMBER_PATTERN = re.compile(
    r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
)

# The existing server renders the provider's `name` directly, so include each
# measurement unit in the label while keeping the name/value payload shape.
PARAMETER_FIELD_NAMES = {
    1: '温度（°C）',
    5: '电导率（μS/cm）',
    18: 'pH（无量纲）',
    37: '浊度（NTU）',
    48: '铵态氮（mg/L-N）',
    193: '叶绿素（μg/L）',
    212: '溶解氧（mg/L）',
    215: '总藻-PC（cells/mL）',
}

# Preserve the former UI ordering, but reference EXO parameter codes so field
# labels have a single source of truth.
TELEMETRY_PARAMETER_ORDER = (193, 5, 212, 48, 215, 37, 1, 18)


class ExoProtocolError(RuntimeError):
    pass


def find_exo_port(
    requested_port: str = 'auto',
    serial_number: str = '',
) -> str:
    if requested_port.lower() != 'auto':
        return requested_port

    ports = list(list_ports.comports())
    matches = [
        port
        for port in ports
        if port.vid == PROLIFIC_VID and port.pid == PROLIFIC_PID
    ]
    if serial_number:
        matches = [
            port for port in matches
            if (port.serial_number or '').casefold() == serial_number.casefold()
        ]
    if len(matches) == 1:
        return matches[0].device
    if not matches:
        available = ', '.join(port.device for port in ports) or 'none'
        serial_detail = (
            f' with serial {serial_number}' if serial_number else ''
        )
        raise serial.SerialException(
            'PL2303GT USB serial adapter 067b:23a3'
            f'{serial_detail} was not found; '
            f'available ports: {available}'
        )
    raise serial.SerialException(
        'More than one PL2303GT adapter was found; set serial_port '
        f'explicitly: {", ".join(port.device for port in matches)}'
    )


def extract_parameter_codes(text: str) -> List[int]:
    for raw_line in text.splitlines():
        line = raw_line.strip().lstrip('#').strip()
        if not line:
            continue
        tokens = re.split(r'[\s,]+', line)
        if not all(re.fullmatch(r'\d+', token) for token in tokens):
            continue
        codes = [int(token) for token in tokens]
        if codes and all(0 <= code <= 249 for code in codes):
            return codes
    return []


def validate_parameter_codes(actual: List[int], expected: List[int]) -> None:
    if actual != expected:
        raise ExoProtocolError(
            'EXO parameter order does not match the configured order: '
            f'expected={expected}, actual={actual}'
        )
    unsupported = [code for code in actual if code not in PARAMETER_FIELD_NAMES]
    if unsupported:
        raise ExoProtocolError(
            f'EXO parameters have no server field mapping: {unsupported}'
        )


def parse_sample_record(
    line: str,
    parameter_codes: List[int],
) -> Optional[Dict[int, float]]:
    line = line.strip()
    if not line or line == '#' or line.lower() in {
        'data', 'run', 'para', 'ok', 'ver', 'sn', 'ssn'
    }:
        return None

    values = NUMBER_PATTERN.findall(line)
    if not values:
        return None
    if len(values) != len(parameter_codes):
        raise ExoProtocolError(
            'EXO record length does not match parameter count: '
            f'codes={len(parameter_codes)}, values={len(values)}, line={line!r}'
        )
    return {
        code: float(value)
        for code, value in zip(parameter_codes, values)
    }


def build_telemetry_data(values: Dict[int, float]) -> List[Dict[str, float]]:
    return [
        {'name': PARAMETER_FIELD_NAMES[code], 'value': values[code]}
        for code in TELEMETRY_PARAMETER_ORDER
        if code in values
    ]


class ExoSerialConnection:
    def __init__(
        self,
        port: str,
        baud_rate: int,
        serial_number: str = '',
        timeout: float = 0.1,
    ) -> None:
        self.port = find_exo_port(port, serial_number)
        self.baud_rate = int(baud_rate)
        self.serial = serial.Serial()
        self.serial.port = self.port
        self.serial.baudrate = self.baud_rate
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.parity = serial.PARITY_NONE
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.timeout = float(timeout)
        self.serial.write_timeout = 2
        self.serial.xonxoff = False
        self.serial.rtscts = False
        self.serial.dsrdtr = False
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.open()
        self._buffer = ''

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def send(self, command: str) -> None:
        command = command.strip()
        if not command:
            raise ValueError('EXO command cannot be empty')
        self.serial.write((command + '\r').encode('ascii'))
        self.serial.flush()

    def read_response(self, duration: float = 5.0, idle_timeout: float = 0.4) -> str:
        deadline = monotonic() + float(duration)
        last_data_at = None
        data = bytearray()
        while monotonic() < deadline:
            waiting = self.serial.in_waiting
            chunk = self.serial.read(waiting or 1)
            if chunk:
                data.extend(chunk)
                last_data_at = monotonic()
            elif (
                last_data_at is not None
                and monotonic() - last_data_at >= idle_timeout
            ):
                break
        return bytes(data).decode('utf-8', 'replace')

    def query(self, command: str, duration: float = 5.0) -> str:
        self.serial.reset_input_buffer()
        self.send(command)
        return self.read_response(duration)

    def stop_continuous_output(self) -> None:
        self.send('0')
        self.read_response(duration=2.0, idle_timeout=0.3)
        self.serial.reset_input_buffer()
        sleep(0.5)

    def wait_for_parameter_codes(
        self,
        timeout: float,
        should_stop: Callable[[], bool],
    ) -> List[int]:
        deadline = monotonic() + float(timeout)
        while monotonic() < deadline and not should_stop():
            self.stop_continuous_output()
            remaining = deadline - monotonic()
            if remaining <= 0:
                break
            response = self.query('para', duration=min(2.0, remaining))
            codes = extract_parameter_codes(response)
            if codes:
                return codes
            sleep(min(0.5, max(0.0, deadline - monotonic())))
        return []

    def set_sample_rate(self, rate_hz: float, timeout: float) -> int:
        rate_hz = float(rate_hz)
        if not 1 / 30 <= rate_hz <= 4:
            raise ValueError('EXO sample rate must be between 0.0333 and 4 Hz')

        period_ms = round(1000 / rate_hz)
        deadline = monotonic() + float(timeout)
        last_response = ''
        while monotonic() < deadline:
            remaining = deadline - monotonic()
            last_response = self.query(
                f'setperiod {period_ms}', duration=min(2.0, remaining)
            )
            if 'OK' in last_response.upper():
                return period_ms
            sleep(min(0.5, max(0.0, deadline - monotonic())))
        raise serial.SerialException(
            f'EXO setperiod {period_ms} timed out; '
            f'last response={last_response.strip()!r}'
        )

    def start_stream(self) -> None:
        self.serial.reset_input_buffer()
        self._buffer = ''
        self.send('run')

    def read_sample(
        self,
        parameter_codes: List[int],
        timeout: float,
    ) -> Optional[Dict[int, float]]:
        deadline = monotonic() + float(timeout)
        while monotonic() < deadline:
            chunk = self.serial.read(self.serial.in_waiting or 1)
            if not chunk:
                continue
            self._buffer += chunk.decode('utf-8', 'replace')
            lines = self._buffer.split('\n')
            self._buffer = lines.pop()
            for line in lines:
                sample = parse_sample_record(line.rstrip('\r'), parameter_codes)
                if sample is not None:
                    return sample
        return None


StatusCallback = Callable[[str, str], None]


class ExoSensorProvider:
    def __init__(
        self,
        port: str,
        baud_rate: int,
        serial_number: str,
        sample_rate_hz: float,
        expected_parameter_codes: List[int],
        startup_timeout_sec: float,
        reconnect_delay_sec: float,
        sample_max_age_sec: float,
        sample_wait_timeout_sec: float,
        status_callback: Optional[StatusCallback] = None,
    ) -> None:
        self._port = port
        self._baud_rate = baud_rate
        self._serial_number = serial_number
        self._sample_rate_hz = sample_rate_hz
        self._expected_parameter_codes = expected_parameter_codes
        self._startup_timeout_sec = startup_timeout_sec
        self._reconnect_delay_sec = reconnect_delay_sec
        self._sample_max_age_sec = sample_max_age_sec
        self._sample_wait_timeout_sec = sample_wait_timeout_sec
        self._status_callback = status_callback
        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._thread: Optional[threading.Thread] = None
        self._latest_data: Optional[List[Dict[str, float]]] = None
        self._latest_at = 0.0
        self._last_error: Optional[BaseException] = None

    async def read(self) -> List[Dict[str, float]]:
        self._ensure_started()
        return await asyncio.to_thread(self._wait_for_fresh_data)

    def start(self) -> None:
        self._ensure_started()

    async def close(self) -> None:
        await asyncio.to_thread(self._close_sync)

    def _ensure_started(self) -> None:
        with self._condition:
            if self._thread is not None:
                return
            self._thread = threading.Thread(
                target=self._worker,
                name='exo-serial-reader',
                daemon=True,
            )
            self._thread.start()

    def _wait_for_fresh_data(self) -> List[Dict[str, float]]:
        deadline = monotonic() + self._sample_wait_timeout_sec
        with self._condition:
            while not self._stop_event.is_set():
                if (
                    self._latest_data is not None
                    and monotonic() - self._latest_at <= self._sample_max_age_sec
                ):
                    return [dict(item) for item in self._latest_data]
                remaining = deadline - monotonic()
                if remaining <= 0:
                    detail = f': {self._last_error}' if self._last_error else ''
                    raise serial.SerialException(
                        'Timed out waiting for a fresh EXO sample' + detail
                    )
                self._condition.wait(timeout=remaining)
        raise serial.SerialException('EXO provider is stopping')

    def _worker(self) -> None:
        consecutive_failures = 0
        while not self._stop_event.is_set():
            connection = None
            received_sample = False
            try:
                connection = ExoSerialConnection(
                    self._port,
                    self._baud_rate,
                    self._serial_number,
                )
                codes = connection.wait_for_parameter_codes(
                    self._startup_timeout_sec,
                    self._stop_event.is_set,
                )
                if not codes:
                    raise serial.SerialException(
                        'EXO did not return parameter codes during startup'
                    )
                validate_parameter_codes(codes, self._expected_parameter_codes)
                period_ms = connection.set_sample_rate(
                    self._sample_rate_hz,
                    self._startup_timeout_sec,
                )
                connection.start_stream()
                self._status(
                    'info',
                    f'EXO connected on {connection.port}; parameters={codes}; '
                    f'period_ms={period_ms}',
                )
                while not self._stop_event.is_set():
                    values = connection.read_sample(codes, timeout=1.0)
                    if values is None:
                        continue
                    received_sample = True
                    consecutive_failures = 0
                    with self._condition:
                        self._latest_data = build_telemetry_data(values)
                        self._latest_at = monotonic()
                        self._last_error = None
                        self._condition.notify_all()
            except (OSError, ValueError, ExoProtocolError) as exc:
                consecutive_failures += 1
                with self._condition:
                    self._last_error = exc
                    self._condition.notify_all()
                level = (
                    'warning'
                    if consecutive_failures == 1 or consecutive_failures % 6 == 0
                    else 'debug'
                )
                self._status(
                    level,
                    f'EXO serial reader failed: {exc}; retrying in '
                    f'{self._reconnect_delay_sec:g}s',
                )
            finally:
                if connection is not None:
                    try:
                        connection.stop_continuous_output()
                    except (OSError, ValueError):
                        pass
                    connection.close()
            if received_sample:
                consecutive_failures = 0
            self._stop_event.wait(self._reconnect_delay_sec)

        with self._condition:
            self._condition.notify_all()

    def _close_sync(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=self._startup_timeout_sec + 5.0)

    def _status(self, level: str, message: str) -> None:
        if self._status_callback is not None:
            self._status_callback(level, message)
