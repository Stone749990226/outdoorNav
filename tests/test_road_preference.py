import math

import numpy as np
import pytest

from outdoornav.road_preference import RoadPreference
from outdoornav.outdoor import OutdoorService
from outdoornav.tasks import TaskStore


def bundle():
    points = [{'x': 0., 'y': 0.}, {'x': 0., 'y': 6.}, {'x': 10., 'y': 6.}, {'x': 10., 'y': 0.}]
    return {'grid': {'origin': [-2, -2], 'width': 32, 'height': 28, 'resolution': .5},
            'network': {'nodes': [{'id':'a',**points[0]}, {'id':'b',**points[-1]}],
                        'roads': [{'id':'main','from':'a','to':'b','points':points}]},
            'road_widths_m': {'main': 1.5}, 'scene_version':'scene', 'route_version':'roads'}


def test_soft_grid_uses_road_zero_and_offroad_finite():
    p = RoadPreference(bundle())
    grid = p.values()
    assert np.all((grid >= 0) & (grid <= 50))
    assert len(np.unique(grid)) > 2
    assert grid[p.cell({'x':0,'y':4}, grid)] < grid[p.cell({'x':.6,'y':4}, grid)]
    assert grid[p.cell({'x':5,'y':0}, grid)] == 50


def test_prefer_longer_main_road_but_allow_an_offroad_goal():
    b = bundle()
    b['road_widths_m']['main'] = 2.
    p = RoadPreference(b)
    route = p.plan({'x':0,'y':0}, {'x':13.1,'y':1.2})
    assert route['goal'] == {'x':13.1,'y':1.2}
    assert max(v['y'] for v in route['reference_path']) >= 5.
    assert route['length_m'] > math.hypot(13.1,1.2)
    assert route['reference_path'][0] == {'x':0.,'y':0.}
    assert route['reference_path'][-1] == route['goal']


def test_no_road_connection_is_not_a_hard_barrier():
    b=bundle()
    b['network']['roads']=[
        {'id':'r1','from':'a','to':'b','points':[{'x':0,'y':0},{'x':3,'y':0}]},
        {'id':'r2','from':'c','to':'d','points':[{'x':7,'y':0},{'x':10,'y':0}]}]
    b['road_widths_m']={'r1':1.5,'r2':1.5}
    assert RoadPreference(b).plan({'x':0,'y':0},{'x':10,'y':0})['length_m'] < 12


def test_keepouts_and_closed_roads_remain_hard_and_bounds_are_checked():
    p=RoadPreference(bundle(),keepouts=[[[4,-2],[6,-2],[6,12],[4,12]]])
    with pytest.raises(ValueError,match='No path'):
        p.plan({'x':0,'y':0},{'x':10,'y':0})
    with pytest.raises(ValueError,match='keepout'):
        p.check_point({'x':5,'y':0})
    with pytest.raises(ValueError,match='outside'):
        p.check_point({'x':100,'y':0})
    with pytest.raises(ValueError,match='closed road'):
        RoadPreference(bundle()).plan({'x':5,'y':2},{'x':0,'y':3},['main'])


def test_small_keepout_is_not_lost_between_cell_centres():
    p=RoadPreference(bundle(),keepouts=[[[4.99,-.01],[5.01,-.01],[5.01,.01],[4.99,.01]]])
    assert np.count_nonzero(p.values() == 100) > 0
    with pytest.raises(ValueError):p.check_point({'x':5,'y':0})


def test_api_preview_and_task_keep_original_offroad_goal():
    b=bundle();status=lambda *_:{'ready':True,'pose':{'x':0,'y':0,'age_s':0},'localization_grade':'fixed'}
    service=OutdoorService(b,None,TaskStore(),status,{'road_preference_enabled':True})
    body={'scene_version':'scene','route_version':'roads','frame_id':'map','goal':{'x':13.1,'y':1.2}}
    preview=service.preview(body);task=service.submit(body)
    assert preview['goal']==body['goal']
    assert task['goal']['x']==13.1 and task['goal']['y']==1.2
    assert task['reference_path']==preview['reference_path']
    assert service.planning_policy()['mode']=='weighted'


def test_guidance_window_does_not_overshoot_observed_horizon_on_diagonals():
    from outdoornav.routing import RouteGuidance
    start={'x':-.44,'y':-.15};goal={'x':5.75,'y':-9.25}
    guide=RouteGuidance([start,goal],start,goal)
    window=guide.window(start)
    assert math.dist(tuple(start.values()),tuple(guide.points[window[-1]].values())) <= 6.


def test_linear_profile_reaches_offroad_cost_without_a_boundary_jump():
    b = bundle()
    b['grid'] = {'origin': [-5.05, -1.05], 'width': 120, 'height': 90, 'resolution': .1}
    b['network']['roads'][0]['points'] = [{'x': 0., 'y': 0.}, {'x': 0., 'y': 7.}]
    b['road_widths_m']['main'] = 4.
    for cost in (1, 50, 99):
        p = RoadPreference(b, offroad_cost=cost, resolution=.1)
        grid = p.values()
        at = lambda x: float(grid[p.cell({'x': x, 'y': 4.}, grid)])
        assert at(0) == pytest.approx(0, abs=1e-5)
        assert at(1) == pytest.approx(cost / 2, abs=1e-5)
        assert at(1.9) == pytest.approx(cost * .95, abs=1e-5)
        assert at(2) == pytest.approx(cost, abs=1e-5)
        assert at(2.1) == pytest.approx(cost, abs=1e-5)


def test_slider_cost_is_used_by_preview_submission_and_execution_policy():
    b = bundle()
    status = lambda *_: {'ready': True, 'pose': {'x': 0, 'y': 0, 'age_s': 0}}
    service = OutdoorService(b, None, TaskStore(), status, {'road_preference_enabled': True})
    body = {'scene_version':'scene', 'route_version':'roads', 'frame_id':'map',
            'goal':{'x':13.1,'y':1.2}, 'road_offroad_cost':99, 'request_id':'a'*32}
    preview = service.preview(body)
    task = service.submit(body)
    actual = service.plan_from(task['reference_path'][0], task['goal'],
                               task['blocked_road_ids'], task['road_offroad_cost'])
    assert preview['road_offroad_cost'] == task['road_offroad_cost'] == 99
    assert actual['reference_path'] == preview['reference_path'] == task['reference_path']
    assert service.preference.offroad_cost == 50  # Another request's default is unchanged.
    assert service.submit(body)['task_id'] == task['task_id']
    with pytest.raises(ValueError, match='another goal'):
        service.submit({**body, 'road_offroad_cost':1})
    cheap = service.preview({**body, 'road_offroad_cost':1})
    assert cheap['reference_path'] != preview['reference_path']
    for invalid in (0, 100, True, 1.5, '50'):
        with pytest.raises(ValueError, match='road_offroad_cost'):
            service.preview({**body, 'road_offroad_cost':invalid})


def test_long_road_route_moves_away_from_edge_toward_centre():
    b = bundle()
    b['grid']['width'] = 74
    b['road_widths_m']['main'] = 4.
    b['network']['roads'][0]['points'] = [{'x': 0., 'y': 0.}, {'x': 30., 'y': 0.}]
    start, goal = {'x': 0., 'y': 1.8}, {'x': 30., 'y': 1.8}
    p = RoadPreference(b)
    route = p.plan(start, goal)
    assert route['reference_path'][0] == start
    assert route['reference_path'][-1] == goal
    middle = [point for point in route['reference_path'] if 5 < point['x'] < 25]
    assert middle and max(abs(point['y']) for point in middle) <= .5
    assert route['weighted_cost'] < p._line_cost(start, goal, p.values())
