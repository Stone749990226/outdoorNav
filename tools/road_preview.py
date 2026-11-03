"""Preview weighted reference routes in a browser without ROS or a robot."""

import argparse
import threading

from outdoornav.api import ApiError, serve
from outdoornav.bundle import ROOT, load_bundle
from outdoornav.outdoor import OutdoorService
from outdoornav.tasks import TaskStore


class RoadPreview:
    obstacles = None
    places = None

    def __init__(self, start=None):
        self.bundle = load_bundle()
        self.start = start or self.bundle['network']['roads'][0]['points'][0]
        self.tasks = TaskStore()
        self.outdoor = OutdoorService(self.bundle, None, self.tasks, self.status,
                                     {'road_preference_enabled': True})
        self.outdoor.preference.check_point(self.start)

    def status(self, *_):
        return {'schema_version': 1, 'mode': 'lynx', 'run_id': 'road-preview-only',
                'scene_version': self.bundle['scene_version'],
                'route_version': self.bundle['route_version'],
                'ready': False, 'diagnostic': True, 'health': {},
                'localization': 'synthetic_preview', 'task': None,
                'pose': {**self.start, 'yaw': 0., 'age_s': 0., 'stamp_s': 0., 'frame_id': 'map'},
                'planning_policy': self.outdoor.planning_policy(),
                'not_ready_reasons': [{'code': 'preview_only',
                    'message': '仅规划预览：A 为固定模拟起点，无实机连接，不支持执行导航。'}]}

    def preview(self, payload):
        return self.outdoor.preview(payload)

    def submit(self, _):
        raise ApiError(403, 'Preview only: navigation execution is disabled')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8877)
    parser.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'))
    args = parser.parse_args()
    root = ROOT / 'web/dist'
    if not (root / 'index.html').is_file():
        parser.error('Build the frontend first: cd web && npm run build')
    start = dict(zip(('x', 'y'), args.start)) if args.start else None
    server = serve(RoadPreview(start), args.port, web_root=root)
    print(f'Planning preview only: http://127.0.0.1:{args.port} (no robot, no ROS)', flush=True)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
