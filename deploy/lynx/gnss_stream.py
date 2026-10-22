"""Run the existing NTRIP client under its lock; tee receiver bytes locally."""

import fcntl
import os
from pathlib import Path
import runpy
import socket
import sys
import struct
import time

from pygnssutils.gnssstreamer import GNSSStreamer


def main():
    root = Path(os.environ.get("LYNX_NTRIP_DIR", "/home/nvidia/ntrip"))
    lock = (root / "client.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = ("127.0.0.1", int(os.environ.get("LYNX_GNSS_PORT", "18765")))
    original_init = GNSSStreamer.__init__

    def init(self, *args, **kwargs):
        output = kwargs.get("outputhandler") or GNSSStreamer.do_output

        def receive(raw, formatted, queue, **extra):
            # One callback, one serial reader. The ROS process never opens the port.
            if raw.startswith((b"$", b"\xb5\x62")):
                stamped = struct.pack("!d", time.monotonic()) + raw
                sock.sendto(stamped, destination)
            return output(raw, formatted, queue, **extra)

        kwargs["outputhandler"] = receive
        original_init(self, *args, **kwargs)

    GNSSStreamer.__init__ = init
    sys.argv = [str(root / "client.py"), "--config", str(root / "ntrip.conf")]
    try:
        runpy.run_path(str(root / "client.py"), run_name="__main__")
    finally:
        sock.close()
        lock.close()


if __name__ == "__main__":
    main()
