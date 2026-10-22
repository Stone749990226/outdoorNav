"""Capture explicit RTK serial device, or replay a log; never sends receiver commands."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import termios
import time

from outdoornav.dual_rtk import DualRtkDecoder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/rtk')
    parser.add_argument('--baud', type=int, choices=(9600, 38400, 57600, 115200, 230400, 460800, 921600), default=115200)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--baseline-to-body-yaw-deg', type=float)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 600:
        parser.error('duration must be in (0, 600]')
    import math
    offset = None if args.baseline_to_body_yaw_deg is None else math.radians(args.baseline_to_body_yaw_deg)
    decoder = DualRtkDecoder(baseline_to_body_yaw_rad=offset)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    raw_path = args.report.with_suffix('.nmea')
    started, now = time.monotonic(), time.monotonic()
    count = 0
    if args.replay:
        for line in args.replay.read_bytes().splitlines(keepends=True):
            decoder.feed(line, now)
            count += len(line)
            now += .001
    else:
        # Never fall back to ttyACM0: it can be the existing u-blox receiver.
        port = Path(args.port).resolve(strict=True)
        fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = None
        stopped = False

        def stop(*_):
            nonlocal stopped
            stopped = True

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            # Linux exclusive-open blocks subsequent opens, and checks existing
            # users before configuring termios (TIOCEXCL alone is not retroactive).
            fcntl.ioctl(fd, termios.TIOCEXCL)
            holders = []
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit() or int(proc.name) == os.getpid():
                    continue
                try:
                    for entry in (proc/'fd').iterdir():
                        try:
                            if entry.resolve() == port:
                                holders.append(proc.name)
                        except OSError:
                            pass
                except (OSError, PermissionError):
                    pass
            if holders:
                raise RuntimeError('Serial device already held by PIDs: ' + ','.join(sorted(set(holders))))
            attrs = termios.tcgetattr(fd)
            new = termios.tcgetattr(fd)
            new[0], new[1], new[2], new[3] = 0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0
            new[4] = new[5] = getattr(termios, 'B'+str(args.baud))
            new[6][termios.VMIN] = new[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, new)
            with raw_path.open('xb') as raw:
                print(f'Reading {port} at {args.baud}; raw log: {raw_path}', flush=True)
                next_display = started
                while not stopped and time.monotonic()-started < args.duration:
                    if select.select([fd], [], [], .1)[0]:
                        data = os.read(fd, 8192)
                        if not data:
                            raise RuntimeError('Serial device disconnected or returned EOF')
                        count += len(data)
                        raw.write(data)
                        decoder.feed(data, time.monotonic())
                    now = time.monotonic()
                    if now >= next_display:
                        current = decoder.snapshot(now)
                        position = current['position'] or {}
                        heading = current['heading'] or {}
                        print(json.dumps({'elapsed_s': round(now-started, 1), 'bytes': count,
                                          'solution': position.get('solution', 'no_fix'),
                                          'satellites': position.get('satellites', 0),
                                          'true_heading_deg': heading.get('true_heading_deg'),
                                          'heading_fresh': current['heading_fresh']}, ensure_ascii=False), flush=True)
                        next_display = now+2
        finally:
            try:
                if attrs is not None:
                    termios.tcsetattr(fd, termios.TCSANOW, attrs)
            finally:
                try:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
                finally:
                    os.close(fd)
    report = {**decoder.snapshot(now), 'bytes_received': count, 'replay': bool(args.replay),
              'duration_s': time.monotonic()-started, 'receiver_commands_sent': False}
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    if not count:
        raise SystemExit('No serial data received; inspect device/baud/output configuration')


if __name__ == '__main__':
    main()
