import asyncio
from pathlib import Path
from time import time
from typing import Any, Dict, List, Optional, Protocol

import aiohttp
import rclpy
from rclpy.node import Node

from smartboat_drivers.exo_serial import ExoSensorProvider


def read_identity(
    identity_dir: str,
    station_default: str,
    boat_default: str,
) -> tuple[str, str]:
    base = Path(identity_dir)
    return (
        _read_text(base / 'name.txt', station_default),
        _read_text(base / 'boat_name.txt', boat_default),
    )


def _read_text(path: Path, default: str) -> str:
    try:
        value = path.read_text(encoding='utf-8').strip()
        if value:
            return value
    except OSError:
        pass
    return default


class SensorProvider(Protocol):
    async def read(self) -> List[Dict[str, Any]]:
        ...


class TelemetryClient:
    WARN_AFTER_CONSECUTIVE_SEND_FAILURES = 2

    def __init__(self, node: Node, provider: SensorProvider) -> None:
        self._node = node
        self._provider = provider
        self._consecutive_send_failures = 0
        self._station, self._boat = read_identity(
            node.get_parameter('identity_dir').value,
            node.get_parameter('station_default').value,
            node.get_parameter('boat_default').value,
        )
        self._url = (
            f'ws://{node.get_parameter("server_host").value}:'
            f'{node.get_parameter("server_port").value}'
            f'/uav/{self._station}/{self._boat}'
        )
        node.get_logger().info(
            f'Device identity loaded: station={self._station}, boat={self._boat}'
        )

    async def run(self) -> None:
        period = float(self._node.get_parameter('publish_period_sec').value)
        reconnect_delay = float(self._node.get_parameter('reconnect_delay_sec').value)

        while rclpy.ok():
            try:
                self._node.get_logger().info(f'Connecting to {self._url}')
                async with aiohttp.ClientSession() as session:
                    async with session.ws_connect(self._url) as ws:
                        self._node.get_logger().info('Water quality websocket connected')
                        while rclpy.ok():
                            try:
                                data = await self._provider.read()
                            except Exception as exc:
                                self._node.get_logger().debug(
                                    f'Water quality sensor read failed: {exc}'
                                )
                                await asyncio.sleep(period)
                                continue
                            try:
                                await ws.send_json({
                                    'timestamp': time(),
                                    'station': self._station,
                                    'boat': self._boat,
                                    'data': data,
                                })
                                self._consecutive_send_failures = 0
                            except Exception as exc:
                                self._consecutive_send_failures += 1
                                if (
                                    self._consecutive_send_failures
                                    > self.WARN_AFTER_CONSECUTIVE_SEND_FAILURES
                                ):
                                    self._node.get_logger().warning(
                                        'Water quality send failed '
                                        f'{self._consecutive_send_failures} times in a row: {exc}'
                                    )
                                else:
                                    self._node.get_logger().debug(
                                        'Water quality send failed '
                                        f'{self._consecutive_send_failures} time(s); '
                                        f'will retry: {exc}'
                                    )
                                raise
                            await asyncio.sleep(period)
            except aiohttp.ClientConnectorError:
                self._node.get_logger().warning(
                    f'Connection failed; retrying in {reconnect_delay}s'
                )
                await asyncio.sleep(reconnect_delay)
            except Exception as exc:
                self._node.get_logger().debug(
                    f'Water quality client error: {exc}; reconnecting'
                )
                await asyncio.sleep(reconnect_delay)


class WaterQualityNode(Node):
    def __init__(self) -> None:
        super().__init__('water_quality')
        self.declare_parameter('server_host', '111.229.242.175')
        self.declare_parameter('server_port', 11452)
        self.declare_parameter('identity_dir', '/host_boot')
        self.declare_parameter('station_default', 'test')
        self.declare_parameter('boat_default', 'test01')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('reconnect_delay_sec', 5.0)
        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('serial_number', 'CRADb2A6710')
        self.declare_parameter('serial_baud_rate', 9600)
        self.declare_parameter('sensor_sample_rate_hz', 0.5)
        self.declare_parameter(
            'expected_parameter_codes', [5, 1, 18, 212, 48, 193, 215, 37]
        )
        self.declare_parameter('sensor_startup_timeout_sec', 10.0)
        self.declare_parameter('sensor_reconnect_delay_sec', 5.0)
        self.declare_parameter('sensor_sample_max_age_sec', 5.0)
        self.declare_parameter('sensor_sample_wait_timeout_sec', 20.0)


def _provider_status(node: Node, level: str, message: str) -> None:
    logger = node.get_logger()
    log_method = getattr(logger, level, logger.info)
    log_method(message)


def create_sensor_provider(node: Node) -> ExoSensorProvider:
    return ExoSensorProvider(
        port=str(node.get_parameter('serial_port').value),
        baud_rate=int(node.get_parameter('serial_baud_rate').value),
        serial_number=str(node.get_parameter('serial_number').value),
        sample_rate_hz=float(
            node.get_parameter('sensor_sample_rate_hz').value
        ),
        expected_parameter_codes=list(
            node.get_parameter('expected_parameter_codes').value
        ),
        startup_timeout_sec=float(
            node.get_parameter('sensor_startup_timeout_sec').value
        ),
        reconnect_delay_sec=float(
            node.get_parameter('sensor_reconnect_delay_sec').value
        ),
        sample_max_age_sec=float(
            node.get_parameter('sensor_sample_max_age_sec').value
        ),
        sample_wait_timeout_sec=float(
            node.get_parameter('sensor_sample_wait_timeout_sec').value
        ),
        status_callback=lambda level, message: _provider_status(
            node, level, message
        ),
    )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaterQualityNode()
    provider = create_sensor_provider(node)
    provider.start()
    client = TelemetryClient(node, provider)

    async def run() -> None:
        try:
            await client.run()
        finally:
            await provider.close()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
