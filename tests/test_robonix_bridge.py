"""Exercise generated Robonix requests against the same HTTP task owner."""

import importlib.util
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import pytest

from outdoornav.api import serve
from outdoornav.bundle import load_bundle
from outdoornav.outdoor import OutdoorService
from outdoornav.places import Places
from outdoornav.tasks import TaskStore

ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / 'integrations/robonix_outdoor/rbnx-build/codegen'
pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('robonix_api') is None or not GENERATED.is_dir(),
    reason='requires the robot SDK and rbnx codegen')


def test_robonix_and_http_use_one_task_and_owned_cancel(monkeypatch):
    bundle = load_bundle()
    places = Places(ROOT / 'scenes/north_campus/places.json', bundle)
    places.items[1]['enabled'] = True
    tasks = TaskStore()
    status = lambda *_: {'mode': 'lynx', 'ready': True, 'localization_grade': 'float',
                         'pose': {**bundle['network']['nodes'][0], 'age_s': 0}}
    outdoor = OutdoorService(bundle, places, tasks, status)
    backend = SimpleNamespace(places=places, tasks=tasks, obstacles=None, status=status,
                              map_snapshot=lambda: bundle, submit=outdoor.submit, preview=outdoor.preview)
    active = {'value': False}
    backend.set_active = lambda value: active.update(value=value)
    http = serve(backend, port=0)
    monkeypatch.setenv('OUTDOORNAV_URL', f'http://127.0.0.1:{http.server_port}')
    monkeypatch.syspath_prepend(str(GENERATED / 'proto_gen'))
    monkeypatch.syspath_prepend(str(GENERATED / 'robonix_mcp_types'))
    try:
        bridge = runpy.run_path(str(ROOT / 'integrations/robonix_outdoor/bridge.py'))
        bridge['init']({})
        assert not active['value']
        bridge['activate']()
        assert active['value']
        bridge['deactivate']()
        assert not active['value']
        backend.status = lambda *_: {'mode': 'lynx', 'ready': False, 'diagnostic': True, 'task': None,
                                    'not_ready_reasons': [{'code': 'lidar', 'message': '雷达无有效新数据'}]}
        diagnostic = bridge['status'](bridge['Status_Request'](run_id=''))
        assert not diagnostic.known and '底盘输出已隔离' in diagnostic.detail and '雷达' in diagnostic.detail
        backend.status = status
        request = bridge['Navigate_Request'](place_id='J', blocked_road_ids=[], request_id='a' * 32)
        response = bridge['navigate'](request)
        assert response.accepted and response.run_id == tasks.current_id
        assert bridge['navigate'](request).run_id == response.run_id
        assert len(tasks.tasks) == 1 and tasks.snapshot()['arrival_tolerance_m'] == 1.5
        result = bridge['status'](bridge['Status_Request'](run_id=response.run_id))
        assert result.known and result.state == 'RUNNING'
        bridge['cancel'](bridge['Cancel_Request'](run_id=response.run_id))
        assert tasks.snapshot()['state'] == 'canceling'
    finally:
        http.shutdown()
        http.server_close()
