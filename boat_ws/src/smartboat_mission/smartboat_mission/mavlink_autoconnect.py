#!/usr/bin/env python3
import argparse
import logging
import os
import signal
import subprocess
import sys
import time

import requests

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


NAME_FILES = (
    '/boot/name.txt',
    '/host_boot/name.txt',
)
ROUTER_CONF = '/etc/mavlink-router/main.conf'
SERVER = '111.229.242.175'
PORT = 11451
API = f'http://{SERVER}:{PORT}/connect'
LOG_LEVELS = {
    'debug': logging.DEBUG,
    'info': logging.INFO,
    'warn': logging.WARNING,
    'warning': logging.WARNING,
    'error': logging.ERROR,
    'critical': logging.CRITICAL,
}
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--log-level',
        default='warning',
        type=str.lower,
        choices=tuple(LOG_LEVELS.keys()),
        help='Console log level.',
    )
    parser.add_argument(
        '--restart-delay-sec',
        default=5.0,
        type=float,
        help='Delay before retrying if mavlink-routerd exits.',
    )
    parser.add_argument(
        '--heartbeat-timeout-sec',
        default=8.0,
        type=float,
        help='Time to wait for PX4 MAVLink HEARTBEAT on local TCP.',
    )
    return parser.parse_args()


def read_name():
    errors = []
    for path in NAME_FILES:
        try:
            with open(path, 'r', encoding='utf-8') as file:
                name = file.read().strip()
        except OSError as exc:
            errors.append(f'{path}: {exc}')
            continue

        if name:
            return name
        errors.append(f'{path}: empty file')

    raise RuntimeError('failed to read boat name: ' + '; '.join(errors))


def request_mavlink_port(name):
    url = f'{API}?name={name}'
    response = requests.get(url, timeout=5)
    response.raise_for_status()
    result = response.json()

    if not isinstance(result, dict):
        raise RuntimeError('API result is not a dict')

    if not result.get('success'):
        raise RuntimeError(f"API error: {result.get('msg', 'No msg field')}")

    try:
        return int(result['data']['port'])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f'malformed API data: {exc}') from exc


def wait_for_local_heartbeat(timeout_sec):
    if mavutil is None:
        raise RuntimeError(
            'pymavlink is not installed; cannot verify PX4 MAVLink HEARTBEAT.'
        )

    connection = mavutil.mavlink_connection(
        'tcp:127.0.0.1:5760',
        autoreconnect=False,
    )
    try:
        heartbeat = connection.wait_heartbeat(timeout=timeout_sec)
    finally:
        connection.close()

    if heartbeat is None:
        raise TimeoutError(
            'PX4 USB serial is present but no MAVLink HEARTBEAT was received. '
            'Please physically unplug and replug the PX4 flight controller USB, '
            'then start again.'
        )


def main():
    args = parse_args()

    logging.basicConfig(
        level=LOG_LEVELS[args.log_level],
        format='%(asctime)s %(levelname)s %(message)s',
    )
    if not os.path.exists(ROUTER_CONF):
        logging.error(
            'mavlink-router config is missing: %s. '
            'Mount the host /etc/mavlink-router directory into the ROS2 container.',
            ROUTER_CONF,
        )
        return 1

    while True:
        try:
            name = read_name()
            port = request_mavlink_port(name)
        except Exception as exc:
            logging.error('%s', exc)
            time.sleep(max(args.restart_delay_sec, 0.0))
            continue

        endpoint = f'{SERVER}:{port}'
        command = [
            '/usr/bin/mavlink-routerd',
            '-c',
            ROUTER_CONF,
            '-r',
            '-p',
            endpoint,
        ]
        logging.info('Executing: %s', ' '.join(command))
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            wait_for_local_heartbeat(max(args.heartbeat_timeout_sec, 0.1))
        except Exception as exc:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            logging.error('%s', exc)
            return 1

        try:
            return_code = process.wait()
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT)
            try:
                return process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    return process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    return process.wait()

        logging.warning(
            'mavlink-routerd exited with code %s; restarting in %.1fs',
            return_code,
            args.restart_delay_sec,
        )
        time.sleep(max(args.restart_delay_sec, 0.0))


if __name__ == '__main__':
    sys.exit(main())
