"""Explicit deployment policy, independent of stricter generic/test defaults."""
import copy
from pathlib import Path

import pytest

from outdoornav.bundle import load_bundle
from outdoornav.config import DEFAULTS, load_robot_config
from outdoornav.gnss import fix_grade
from outdoornav.outdoor import OutdoorService
from outdoornav.tasks import TaskStore


@pytest.fixture
def config():
    return load_robot_config(Path('deploy/lynx/robot.json'))


@pytest.mark.parametrize('grade',['fixed','float'])
def test_hacc_three_meter_boundary(config,grade):
    fix={'valid':True,'differential':True,'fixed':grade=='fixed','float':grade=='float',
         'gga_quality':4 if grade=='fixed' else 5,'latitude':30.,'longitude':120.,
         'correction_age_s':1.,'hacc_m':3.0}
    assert fix_grade(fix,config)==grade
    assert fix_grade({**fix,'hacc_m':3.001},config) is None
    assert fix_grade({**fix,'valid':False},config) is None
    assert fix_grade({**fix,'correction_age_s':6.0},config) is None
    assert fix_grade({**fix,'gga_quality':2,'fixed':False,'float':False},config) is None


@pytest.mark.parametrize('grade',['fixed','float'])
def test_submitted_task_uses_three_meter_arrival_policy(config,grade):
    bundle=load_bundle()
    start,goal=bundle['network']['roads'][0]['points'][:2]
    tasks=TaskStore()
    status=lambda *_:{'ready':True,'localization_grade':grade,'pose':{**start,'age_s':0.0}}
    service=OutdoorService(bundle,None,tasks,status,config)
    result=service.submit({'scene_version':bundle['scene_version'],
        'route_version':bundle['route_version'],'frame_id':'map','goal':goal})
    assert result['arrival_tolerance_m']==3.0
    assert result['localization_grade']==grade


def test_demo_policy_does_not_enable_motion_or_relax_other_gates(config):
    assert config['motion_enabled'] is False
    assert config['feedback_confirmed'] is False
    assert config['wit_imu']['mounting_confirmed'] is False
    assert config['gnss_speed_accuracy_mps']==DEFAULTS['gnss_speed_accuracy_mps']
    assert config['sensor_timeout_s']==DEFAULTS['sensor_timeout_s']
    assert config['correction_max_age_s']==DEFAULTS['correction_max_age_s']
    assert DEFAULTS['gnss_fixed_hacc_m']==0.2 and DEFAULTS['fixed_arrival_m']==0.5
