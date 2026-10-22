"""Core topology and static inspection regressions; no physical robot required."""

import json
import math
import subprocess
from pathlib import Path

import pytest

from outdoornav.diagnostics import StaticFrames, json_safe, readiness_reasons


def test_static_sensor_chain_rejects_competing_localization_and_cycles():
    frames = StaticFrames()
    assert frames.matrix('lidar') is None
    frames.add('base_link', 'mount', (0, 0, 0.4), (0, 0, 0, 1))
    frames.add('mount', 'lidar', (0.2, 0, 0), (0, 0, 0, 1))
    assert frames.matrix('lidar')[:3, 3] == pytest.approx([0.2, 0, 0.4])
    # Quaternion sign alone is not a different mounting calibration.
    frames.add('mount', 'lidar', (0.2, 0, 0), (0, 0, 0, -1))
    assert not frames.faults
    frames.add('mount', 'lidar', (0.3, 0, 0), (0, 0, 0, 1))
    assert frames.matrix('lidar') is None
    frames = StaticFrames()
    frames.add('map', 'base_link', (0, 0, 0), (0, 0, 0, 1))
    assert any('static_localization_owner' in f for f in frames.faults)
    frames = StaticFrames()
    frames.add('a', 'b', (0, 0, 0), (0, 0, 0, 1))
    frames.add('b', 'a', (0, 0, 0), (0, 0, 0, 1))
    assert frames.faults


def test_nonfinite_diagnostics_remain_json_and_multiple_reasons_survive():
    value = json_safe({'sample': [math.nan, math.inf, -math.inf, 1.0]})
    assert json.loads(json.dumps(value, allow_nan=False))['sample'] == [None, None, None, 1.0]
    reasons = readiness_reasons({'localization': False, 'sensor_extrinsics': False,
                                  'motion_enabled': False, 'production_mode': False}, {})
    assert {r['code'] for r in reasons} == {'localization', 'sensor_extrinsics', 'motion_enabled', 'production_mode'}


def test_failed_child_cleans_its_grandchildren(tmp_path):
    # Exercise real process groups: the failed leader leaves a live grandchild.
    helper = Path('deploy/lynx/processes.sh').resolve()
    script = tmp_path / 'supervise.sh'
    marker = tmp_path / 'pid'
    script.write_text('set -e\nsource "$1"\nstart_child bash -c \'sleep 100 & echo $! > "$1"; exit 7\' bash "$2"\nwait -n "${task_children[@]}"\n')
    result = subprocess.run(['bash', str(script), str(helper), str(marker)], timeout=30)
    assert result.returncode == 7
    pid = int(marker.read_text())
    status = Path(f'/proc/{pid}/stat')
    assert not status.exists() or status.read_text().split()[2] == 'Z'


def test_no_default_odometry_and_unreliable_source_cannot_be_reenabled(tmp_path):
    from outdoornav.config import load_robot_config
    config = json.loads(Path('deploy/lynx/robot.json').read_text())
    assert config['odom_topic'] == '' and not config['odometry_confirmed']
    reasons = readiness_reasons({'odometry': False}, {'fault': 'odometry_source_unconfigured'})
    assert any(reason['message'] == '连续里程计来源尚未配置' for reason in reasons)
    target = tmp_path / 'robot.json'
    for source in ('/ODOM', '/outdoor/odom'):
        target.write_text(json.dumps({**config, 'odom_topic': source}))
        with pytest.raises(ValueError, match='cannot be localization sources'):
            load_robot_config(target)
    target.write_text(json.dumps({**config, 'odometry_confirmed': True}))
    with pytest.raises(ValueError, match='must be configured'):
        load_robot_config(target)
