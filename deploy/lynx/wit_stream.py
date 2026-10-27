"""Exclusive WIT reader; query mode only, never calibrate or change settings."""

import argparse
import json
import signal
import socket
import time
import uuid

from outdoornav.config import load_robot_config
from outdoornav.serial_input import SerialInput
from outdoornav.wit_imu import WitImuDecoder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--duration', type=float)
    args = parser.parse_args()
    config = load_robot_config(args.config)
    if config['heading_source'] != 'wit_imu':
        parser.error('WIT input is not selected')
    if args.duration is not None and not 0 < args.duration <= 3600:
        parser.error('duration must be in (0, 3600]')
    cfg = config['wit_imu']
    decoder = WitImuDecoder(cfg)
    generation = uuid.uuid4().hex
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    forbidden = ('/dev/ttyACM0', '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        with SerialInput(cfg['port'], cfg['baud'], forbidden=forbidden, read_requests=True) as serial:
            # Discard pre-existing buffered replies before correlating READADDR.
            drain_until = time.monotonic()+0.3
            while time.monotonic() < drain_until:
                serial.read(timeout=0.05)
            queried = time.monotonic()
            serial.query_imu_algorithm()
            mode = None
            print('WIT input: %s -> localhost:%d; READADDR query only, no settings' %
                  (serial.port, cfg['udp_port']), flush=True)
            while not stopping and (args.duration is None or time.monotonic()-queried < args.duration):
                raw = serial.read()
                if not raw:
                    continue
                now = time.monotonic()
                decoder.feed(raw, now)
                if mode is None and decoder.registers is not None and decoder.register_at >= queried:
                    value = decoder.registers[0]
                    if value not in (0, 1):
                        raise RuntimeError('Unknown IMU AXIS6 register value: '+str(value))
                    mode = value
                    print('Queried AXIS6=%d (%s)' % (mode,'nine-axis magnetic' if mode == 0 else 'six-axis relative'),flush=True)
                packet = {'schema_version':1,'source':'wit_imu','generation':generation,
                          'axis6_register':mode,'received':now,'raw_hex':raw.hex()}
                udp.sendto(json.dumps(packet, allow_nan=False).encode(),('127.0.0.1',cfg['udp_port']))


if __name__ == '__main__':
    main()
