"""Exclusive, read-only serial acquisition shared by hardware input bridges."""

import fcntl
import os
from pathlib import Path
import select
import termios


class SerialInput:
    def __init__(self, port, baud, *, forbidden=(), read_requests=False):
        self.port = Path(port).resolve(strict=True)
        if self.port in [Path(p).resolve() for p in forbidden]:
            raise ValueError('Heading input must not open the position receiver')
        if not self.port.is_char_device():
            raise ValueError('Serial input must be a character device')
        self.fd = None
        self.attrs = None
        self.read_requests = read_requests
        mode = os.O_RDWR if read_requests else os.O_RDONLY
        fd = os.open(self.port, mode | os.O_NOCTTY | os.O_NONBLOCK)
        self.fd = fd
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            holders = set()
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit() or int(proc.name) == os.getpid():
                    continue
                try:
                    for entry in (proc/'fd').iterdir():
                        try:
                            if entry.resolve() == self.port:
                                holders.add(proc.name)
                        except OSError:
                            pass
                except (OSError, PermissionError):
                    # Kernel exclusion prevents subsequent opens. Operators must
                    # also stop any pre-existing privileged acquisition process.
                    pass
            if holders:
                raise RuntimeError('Serial input is already held by PIDs: ' + ','.join(sorted(holders)))
            self.attrs = termios.tcgetattr(fd)
            attrs = termios.tcgetattr(fd)
            attrs[0], attrs[1], attrs[2], attrs[3] = 0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0
            attrs[4] = attrs[5] = getattr(termios, 'B' + str(baud))
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except Exception:
            self.close()
            raise

    def query_imu_algorithm(self):
        if not self.read_requests:
            raise RuntimeError('This reader cannot send register queries')
        # READADDR only. Never unlock, set AXIS6, calibrate or save settings.
        if os.write(self.fd, bytes.fromhex('ff aa 27 24 00')) != 5:
            raise RuntimeError('Incomplete IMU mode read request')

    def read(self, timeout=0.1):
        if select.select([self.fd], [], [], timeout)[0]:
            data = os.read(self.fd, 8192)
            if not data:
                raise RuntimeError('Heading receiver disconnected')
            return data
        return b''

    def close(self):
        if self.fd is None:
            return
        try:
            if self.attrs is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.attrs)
        finally:
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            finally:
                os.close(self.fd)
                self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
