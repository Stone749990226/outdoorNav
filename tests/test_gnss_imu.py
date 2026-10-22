"""Feasibility checks: independent ENU trajectory, rotating antenna and outages."""

import math

import numpy as np
import pytest

from outdoornav.coordinates import local_radii
from outdoornav.gnss_imu import GnssImuProbe

ANCHOR = {'latitude': 30.0, 'longitude': 120.0}


def estimator(**kwargs):
    return GnssImuProbe(ANCHOR, initial_heading_rad=0.4, heading_std_rad=0.05,
                        gyro_bias_rps=0.002, gyro_noise_rps=0.005,
                        bias_std_rps=0.001, **kwargs)


def measurement(t, position, velocity, *, fixed=True):
    east, north = local_radii(ANCHOR['latitude'])
    return {'latitude': ANCHOR['latitude'] + math.degrees(position[1] / north),
            'longitude': ANCHOR['longitude'] + math.degrees(position[0] / east),
            'valid': True, 'differential': True, 'fixed': fixed, 'float': not fixed,
            'gga_quality': 4 if fixed else 5, 'hacc_m': 0.1, 'correction_age_s': 1.0,
            'vel_e_mps': velocity[0], 'vel_n_mps': velocity[1], 'speed_accuracy_mps': 0.03,
            'tow_ms': int(round(t * 1000))}


def test_synthetic_turn_uses_gnss_velocity_and_corrects_rotating_antenna():
    probe = estimator(antenna_xy_m=(0.8, 0.2))
    random = np.random.default_rng(42)
    position = np.array([4.0, -2.0])
    errors = []
    for i in range(2001):
        t, yaw = i / 100, 0.4 + 0.1*i/100
        velocity = 0.15*np.array([math.cos(yaw), math.sin(yaw)])
        if i:
            position += velocity * 0.01
        assert probe.imu(t, 0.102)
        if i % 10 == 0:
            arm = np.array([[math.cos(yaw), -math.sin(yaw)],
                            [math.sin(yaw), math.cos(yaw)]]) @ np.array([0.8, 0.2])
            antenna_velocity = velocity + 0.1*np.array([-arm[1], arm[0]])
            fix = measurement(t, position + arm + random.normal(0, 0.03, 2), antenna_velocity)
            assert probe.gnss(fix, t)
        errors.append(float(np.linalg.norm(probe.state[:2]-position)))
    result = probe.snapshot(20)
    assert result['inputs_stable'] and not result['motion_allowed']
    assert max(errors[100:]) < 0.12
    assert result['yaw_rad'] == pytest.approx(2.4, abs=0.01)
    assert result['yaw_std_rad'] > 0.05  # GNSS position/velocity don't observe body heading.
    assert np.linalg.eigvalsh(probe.covariance).min() > 0
    assert not probe.snapshot(21.1)['inputs_stable']


def test_duplicate_fix_jump_imu_gap_and_invalid_quality_cannot_be_hidden():
    probe = estimator()
    probe.imu(0, 0.002)
    fix = measurement(0, [0, 0], [0, 0])
    assert probe.gnss(fix, 0)
    probe.imu(0.1, 0.002)
    assert not probe.gnss(fix, 0.1)
    assert probe.fix_at == 0
    assert not probe.gnss(measurement(0.1, [100, 100], [0, 0]), 0.1)
    assert probe.snapshot(0.1)['fault'] == 'gnss_innovation_rejected'
    probe = estimator()
    probe.imu(0, 0)
    assert not probe.imu(1.1, 0)
    assert not probe.snapshot(1.1)['inputs_stable']
    probe = estimator()
    probe.imu(0, 0)
    assert not probe.gnss({**fix, 'valid': False}, 0)
    assert not probe.initialized


def test_low_speed_does_not_invent_heading_and_fixed_downgrade_restabilizes():
    probe = estimator()
    for i in range(71):
        t = i/10
        probe.imu(t, 0.002)
        probe.gnss(measurement(t, [0, 0], [0, 0]), t)
    assert probe.snapshot(7)['inputs_stable']
    assert probe.snapshot(7)['yaw_rad'] == pytest.approx(0.4)
    probe.imu(7.1, 0.002)
    probe.gnss(measurement(7.1, [0, 0], [0, 0], fixed=False), 7.1)
    assert not probe.snapshot(7.1)['inputs_stable']
    assert probe.grade == 'float'
    assert not probe.snapshot(7.1)['motion_allowed']


def test_receiver_doppler_units_and_timestamp_alignment():
    import struct
    from outdoornav.gnss import FixDecoder
    payload = bytearray(92)
    struct.pack_into('<iii', payload, 48, -1234, 567, 0)
    struct.pack_into('<I', payload, 68, 45)
    body = bytes([1, 7, 92, 0]) + payload
    a = b = 0
    for byte in body:
        a, b = (a + byte) & 255, (b + a + byte) & 255
    value = FixDecoder().feed(b'\xb5\x62' + body + bytes([a, b]), 0)
    assert value['vel_n_mps'] == pytest.approx(-1.234)
    assert value['vel_e_mps'] == pytest.approx(0.567)
    assert value['speed_accuracy_mps'] == pytest.approx(0.045)
    probe = estimator()
    probe.imu(1, 0)
    assert not probe.gnss(measurement(0.9, [0, 0], [0, 0]), 0.9)
    assert probe.fault == 'gnss_imu_time_mismatch'
