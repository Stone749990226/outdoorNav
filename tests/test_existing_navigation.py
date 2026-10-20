"""Transport-independent tests of the existing-chain integration, not robot motion."""
import math

import pytest

from outdoornav.api import ApiError
from outdoornav.bundle import load_bundle
from outdoornav.coordinates import local_radii
from outdoornav.existing_navigation import ExistingNavigation
from outdoornav.georeference import MapAlignment


class Client:
    def __init__(self):
        self.calls = []
        self.generation = 'nav-v1'
        self.result = {'known': True, 'state': 'RUNNING', 'detail': 'running'}
        self.fail_dispatch = False
        self.available = True

    def inspect(self):
        return {'active': self.available, 'registration_id': self.generation}

    def navigate(self, goal):
        self.calls.append(('navigate', goal))
        if self.fail_dispatch:
            raise TimeoutError('accepted or not?')
        return {'accepted': True, 'run_id': 'original-service-owned-id', 'detail': 'queued'}

    def cancel(self, run_id):
        self.calls.append(('cancel', run_id))
        return {'accepted': True, 'detail': 'requested'}

    def status(self, run_id):
        assert run_id == 'original-service-owned-id'
        return self.result


def backend():
    now = [100.0]
    client = Client()
    config = {'navigation_provider_id': 'nav2', 'antenna_xy_m': [0, 0],
              'antenna_offset_confirmed': True, 'motion_enabled': True}
    service = ExistingNavigation(load_bundle(), 'scenes/north_campus/places.json', config,
                                 client, clock=lambda: now[0])
    service.service_active = True
    service.observe({'x': 10, 'y': 20, 'yaw': 0, 'stamp_s': 1, 'age_s': 0}, (0, 0), True, 'map-v1')
    service.alignment.transform = (math.pi/2, 10, 20)
    service.alignment.fixed_since = 90
    service.alignment.fix_at = now[0]
    service.stopped_since = 98
    service.tick()
    return service, client, now


def payload(service, x=25.3, y=-91.8, **extra):
    return {'frame_id': 'map', 'scene_version': service.bundle['scene_version'],
            'route_version': service.bundle['route_version'], 'goal': {'x': x, 'y': y}, **extra}


def refresh(service, now, dt=0.1, speed=(0, 0)):
    now[0] += dt
    service.observe({**service.map_pose, 'age_s': 0}, speed, True, 'map-v1')
    service.alignment.fix_at = now[0]


def test_free_point_has_no_road_or_place_dependency_and_preview_is_not_a_path():
    service, _, _ = backend()
    request = payload(service, x=-123.45, y=67.89)
    preview = service.preview(request)
    assert preview['reference_path'] == []
    assert preview['length_m'] is None
    assert preview['planning'] == 'deferred_to_existing_navigation'
    task = service.submit(request)
    assert task['goal'] == request['goal']
    assert task['reference_path'] == []
    with pytest.raises(ValueError, match='Road closures'):
        service.submit({**request, 'blocked_road_ids': ['loop-a']})


def test_dispatch_transforms_goal_and_uses_original_service_not_velocity():
    service, client, _ = backend()
    service.submit(payload(service, x=4, y=2))
    service.tick()
    assert len(client.calls) == 1
    goal = client.calls[0][1]
    assert goal == pytest.approx({'x': 8, 'y': 24, 'yaw': 0})
    assert service.tasks.snapshot()['state'] == 'navigating'


def test_request_retry_does_not_send_twice_even_after_pose_changes_or_fix_loss():
    service, client, _ = backend()
    request = payload(service, request_id='a'*32)
    task = service.submit(request)
    service.map_pose['yaw'] = 1.0
    service.alignment.fixed_since = None
    assert service.submit(request)['task_id'] == task['task_id']
    with pytest.raises(ApiError, match='another goal'):
        service.submit({**request, 'goal': {'x': 7, 'y': 8}})
    assert not client.calls


def test_cancel_acceptance_does_not_mean_robot_stopped():
    service, client, now = backend()
    task = service.submit(payload(service))
    service.tick()
    service.tasks.cancel(task['task_id'])
    refresh(service, now, speed=(0.2, 0))
    service.tick()
    assert ('cancel', 'original-service-owned-id') in client.calls
    assert service.tasks.snapshot()['state'] == 'canceling'
    client.result = {'known': True, 'state': 'CANCELED', 'detail': 'done'}
    refresh(service, now, speed=(0.2, 0))
    service.tick()
    assert service.tasks.snapshot()['state'] == 'canceling'
    refresh(service, now)
    service.tick()
    refresh(service, now, 1.1)
    service.tick()
    assert service.tasks.snapshot()['state'] == 'canceled'


def test_replacement_waits_for_owned_run_terminal_and_measured_stop():
    service, client, now = backend()
    first = service.submit(payload(service))
    service.tick()
    second = service.submit(payload(service, x=5, y=6))
    refresh(service, now, speed=(0.2, 0))
    service.tick()
    assert len([c for c in client.calls if c[0] == 'navigate']) == 1
    client.result = {'known': True, 'state': 'CANCELED', 'detail': 'done'}
    refresh(service, now)
    service.tick()
    refresh(service, now, 1.1)
    service.tick()
    assert service.tasks.snapshot(first['task_id'])['state'] == 'canceled'
    service.tick()
    assert len([c for c in client.calls if c[0] == 'navigate']) == 2
    assert service.tasks.snapshot(second['task_id'])['state'] == 'navigating'


def test_loss_cancels_only_owned_goal_and_stale_odometry_never_confirms_stop():
    service, client, now = backend()
    service.submit(payload(service))
    service.tick()
    now[0] += 2
    service.tick()
    assert ('cancel', 'original-service-owned-id') in client.calls
    client.result = {'known': True, 'state': 'CANCELED', 'detail': 'done'}
    service.tick()
    assert service.tasks.snapshot()['state'] != 'failed'
    now[0] += 16
    service.tick()
    assert service.fault
    assert service.worker_task is not None
    with pytest.raises(ApiError, match='not ready'):
        service.submit(payload(service, x=1, y=2))


def test_unknown_dispatch_outcome_is_never_retried():
    service, client, _ = backend()
    client.fail_dispatch = True
    service.submit(payload(service))
    service.tick()
    for _ in range(3):
        service.tick()
    assert len(client.calls) == 1
    assert 'unknown' in service.fault


def test_success_requires_standstill_and_independent_goal_distance():
    service, client, now = backend()
    # Inverse alignment puts (10,20) map at (0,0) ENU.
    service.submit(payload(service, x=0, y=0))
    service.tick()
    client.result = {'known': True, 'state': 'SUCCEEDED', 'detail': 'done'}
    refresh(service, now, speed=(0.1, 0))
    service.tick()
    assert service.tasks.snapshot()['state'] == 'navigating'
    refresh(service, now)
    service.tick()
    refresh(service, now, 1.1)
    service.tick()
    assert service.tasks.snapshot()['state'] == 'succeeded'


def test_provider_restart_never_replays_goal():
    service, client, now = backend()
    service.submit(payload(service))
    service.tick()
    client.generation = 'nav-v2'
    client.result = {'known': False, 'state': 'UNKNOWN', 'detail': 'unknown run'}
    refresh(service, now)
    service.tick()
    assert 'restarted' in service.fault
    assert len([c for c in client.calls if c[0] == 'navigate']) == 1


def test_motion_and_measured_antenna_are_explicit_gates():
    service, _, _ = backend()
    for key in ('motion_enabled', 'antenna_offset_confirmed'):
        service.config[key] = False
        with pytest.raises(ApiError, match='not ready'):
            service.submit(payload(service))
        service.config[key] = True


def gnss(x, y, epoch):
    east, north = local_radii(30)
    return {'latitude': 30 + math.degrees(y/north), 'longitude': 120 + math.degrees(x/east),
            'tow_ms': epoch, 'valid': True, 'fixed': True, 'differential': True,
            'gga_quality': 4, 'hacc_m': 0.1, 'correction_age_s': 1}


def test_passive_alignment_observes_rotation_translation_and_antenna_offset():
    alignment = MapAlignment({'latitude': 30, 'longitude': 120}, (1, 0))
    alignment.generation_changed('map-v1')
    for i in range(81):
        # ENU antenna(x,0) corresponds to map antenna(10,20+x).
        x = i/10
        pose = {'x': 10, 'y': 19+x, 'yaw': math.pi/2, 'age_s': 0}
        alignment.update(gnss(x, 0, i), pose, i/10)
    assert alignment.ready(8)
    assert alignment.to_map({'x': 3, 'y': 2}) == pytest.approx({'x': 8, 'y': 23}, abs=1e-6)
    assert alignment.to_enu({'x': 8, 'y': 23, 'yaw': math.pi/2}) == pytest.approx({'x': 3, 'y': 2, 'yaw': 0}, abs=1e-6)
    alignment.generation_changed('map-v2')
    assert not alignment.ready(8)
    assert alignment.transform is None


def test_stationary_fixes_cannot_invent_absolute_heading_and_duplicate_epoch_expires():
    alignment = MapAlignment({'latitude': 30, 'longitude': 120})
    for i in range(81):
        alignment.update(gnss(0, 0, i), {'x': 0, 'y': 0, 'yaw': 0, 'age_s': 0}, i/10)
    assert alignment.transform is None
    assert not alignment.ready(8)
    alignment.transform = (0, 0, 0)
    alignment.update(gnss(0, 0, 80), {'x': 0, 'y': 0, 'yaw': 0, 'age_s': 0}, 10)
    assert not alignment.ready(10)


def test_gnss_map_disagreement_latches_and_invalid_fix_revokes_readiness():
    alignment = MapAlignment({'latitude': 30, 'longitude': 120})
    alignment.transform = (0, 0, 0)
    alignment.update(gnss(0, 0, 1), {'x': 0, 'y': 0, 'yaw': 0, 'age_s': 0}, 1)
    alignment.update(gnss(3, 0, 2), {'x': 0, 'y': 0, 'yaw': 0, 'age_s': 0}, 2)
    assert alignment.fault
    alignment.update({**gnss(0, 0, 3), 'fixed': False}, {'x': 0, 'y': 0, 'yaw': 0, 'age_s': 0}, 3)
    assert alignment.fixed_since is None
