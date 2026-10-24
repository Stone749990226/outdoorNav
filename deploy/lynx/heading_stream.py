"""Read only the dual-antenna serial port; forward heading data, never corrections."""

import argparse
import signal
import socket
import struct
import time

from outdoornav.config import load_robot_config
from outdoornav.serial_input import SerialInput


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--duration', type=float)
    args = parser.parse_args()
    config = load_robot_config(args.config)
    if config['heading_source'] != 'dual_antenna':
        parser.error('The selected heading source is not dual_antenna')
    if args.duration is not None and not 0 < args.duration <= 3600:
        parser.error('duration must be in (0, 3600]')
    heading = config['dual_heading']
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    # Never fall back to the old receiver; it supplies position independently.
    forbidden = ('/dev/ttyACM0', '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        with SerialInput(heading['port'], heading['baud'], forbidden=forbidden) as serial:
            started = time.monotonic()
            print('Heading reader: %s -> localhost:%d; no receiver commands' %
                  (serial.port, heading['udp_port']), flush=True)
            while not stopped and (args.duration is None or time.monotonic()-started < args.duration):
                raw = serial.read()
                if raw:
                    udp.sendto(struct.pack('!d', time.monotonic())+raw,
                               ('127.0.0.1', heading['udp_port']))


if __name__ == '__main__':
    main()
