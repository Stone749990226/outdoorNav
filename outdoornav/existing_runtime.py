"""Shared HTTP/task worker; reuses the running robot deployment without booting it."""

import argparse
import json
from pathlib import Path
import signal
import socket
import struct
import threading
import time

from .api import serve
from .bundle import load_bundle
from .existing_navigation import ExistingNavigation
from .gnss import FixDecoder
from .navigation_client import NavigationClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--places', default='scenes/north_campus/places.json')
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--observe-only', action='store_true')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if config.get('integration') != 'existing_navigation':
        raise ValueError('This worker only supports the existing Navigation Service')
    for key in ('navigation_provider_id', 'mapping_provider_id', 'odometry_provider_id', 'scan_topic'):
        if not isinstance(config.get(key), str) or not config[key]:
            raise ValueError('Missing existing deployment binding: ' + key)
    antenna = config.get('antenna_xy_m')
    import math
    if not isinstance(antenna, list) or len(antenna) != 2 or not all(isinstance(v, (int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in antenna):
        raise ValueError('antenna_xy_m must contain two finite measurements')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from .existing_ros import ExistingObservations
    rclpy.init(args=None)
    # Atlas only brokers connections for registered consumers; register this
    # process identity for its lifetime and never reuse the packaged service ID.
    import uuid
    from robonix_api import ATLAS
    probe_id = 'outdoor_runtime_' + uuid.uuid4().hex[:8]
    if args.observe_only:
        config['motion_enabled'] = False
    config['consumer_id'] = probe_id
    ATLAS.register_service(probe_id, 'robonix/service/outdoor_runtime')
    client = NavigationClient(config['navigation_provider_id'], config.get('consumer_id', 'outdoor_navigation'))
    backend = ExistingNavigation(load_bundle(), args.places, config, client,
                                 Path(args.run_dir) / 'tasks.json')
    observer = ExistingObservations(backend, config)
    executor = SingleThreadedExecutor()
    executor.add_node(observer.node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', config['gnss_udp_port']))
    receiver.setblocking(False)
    decoder = FixDecoder()
    http = serve(backend, config.get('http_port', 8765))
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    try:
        while not stopping.wait(0.1):
            for _ in range(100):
                try:
                    raw = receiver.recv(65535)
                except BlockingIOError:
                    break
                if len(raw) < 8:
                    continue
                received = struct.unpack('!d', raw[:8])[0]
                if not 0 <= time.monotonic() - received < 1:
                    continue
                fix = decoder.feed(raw[8:], received)
                if fix:
                    backend.on_fix(fix, received)
            backend.tick()
    finally:
        backend.set_active(False)
        deadline = time.monotonic() + 15
        while backend.worker_task and time.monotonic() < deadline:
            backend.tick()
            time.sleep(0.1)
        if backend.worker_task:
            print('WARNING: owned navigation stop NOT confirmed; operator inspection required', flush=True)
        http.shutdown()
        http.server_close()
        receiver.close()
        executor.shutdown()
        ros_thread.join(timeout=3)
        observer.close()
        client.close()
        rclpy.shutdown()
        if probe_id:
            ATLAS.unregister(probe_id)


if __name__ == '__main__':
    main()
