"""Offline planar GNSS/gyro feasibility prototype, not a navigation provider.

Position/velocity are ENU, gyro is already expressed about base_link's up axis.
Yaw is unobservable here: an explicitly measured initial heading and gyro bias
are required. Accelerometers and motion-derived heading are deliberately unused.
"""

import math

import numpy as np

from .config import DEFAULTS
from .coordinates import enu
from .gnss import fix_grade
from .localization import wrap


class GnssImuProbe:
    def __init__(self, anchor, *, initial_heading_rad, heading_std_rad,
                 gyro_bias_rps, gyro_noise_rps, bias_std_rps,
                 antenna_xy_m=(0.0, 0.0), config=None):
        values = (initial_heading_rad, heading_std_rad, gyro_bias_rps,
                  gyro_noise_rps, bias_std_rps, *antenna_xy_m)
        if len(antenna_xy_m) != 2:
            raise ValueError('Antenna offset must have two measurements')
        if not all(math.isfinite(v) for v in values) or min(
                heading_std_rad, gyro_noise_rps, bias_std_rps) <= 0:
            raise ValueError('Measured heading/bias and positive uncertainty are required')
        self.anchor, self.config = anchor, {**DEFAULTS, **(config or {})}
        self.state = np.zeros(4)  # east, north, east velocity, north velocity
        self.covariance = np.diag([100.0, 100.0, 4.0, 4.0])
        self.yaw, self.heading_variance = wrap(initial_heading_rad), heading_std_rad**2
        self.gyro_bias, self.gyro_noise, self.bias_std = gyro_bias_rps, gyro_noise_rps, bias_std_rps
        self.antenna = np.array(antenna_xy_m, dtype=float)
        self.elapsed = 0.0
        self.stamp = None
        self.fix_at, self.stable_since = None, None
        self.last_tow, self.grade, self.fault = None, None, None
        self.initialized = False
        self.yaw_rate = 0.0

    def imu(self, stamp, base_yaw_rate):
        if not all(math.isfinite(v) for v in (stamp, base_yaw_rate)):
            self.fault = 'invalid_imu'
            return False
        if self.stamp is None:
            self.stamp = stamp
            self.yaw_rate = base_yaw_rate - self.gyro_bias
            return True
        dt = stamp - self.stamp
        if not 0 < dt < self.config['sensor_timeout_s']:
            self.fault = 'imu_time_reversed_or_gap'
            return False
        rate = base_yaw_rate - self.gyro_bias
        self.yaw = wrap(self.yaw + 0.5 * (rate + self.yaw_rate) * dt)
        self.yaw_rate = rate
        # White rate noise plus a constant, uncertain residual gyro bias.
        self.heading_variance += self.gyro_noise**2 * dt + self.bias_std**2 * (2*self.elapsed*dt + dt*dt)
        self.elapsed += dt
        transition = np.eye(4)
        transition[0, 2] = transition[1, 3] = dt
        gain = np.array([[dt*dt/2, 0], [0, dt*dt/2], [dt, 0], [0, dt]])
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + gain @ gain.T * 0.5**2
        self.stamp = stamp
        return not self.fault

    def gnss(self, fix, stamp):
        """Offline measurements must be aligned to the same IMU epoch.

        Receiver velocity is antenna Doppler ENU velocity, not pose differences.
        It remains optional for old logs; absent velocity is never a zero reading.
        """
        if self.stamp is None or not math.isfinite(stamp) or abs(self.stamp - stamp) > 1e-6:
            self.fault = 'gnss_imu_time_mismatch'
            return False
        grade = fix_grade(fix, self.config)
        if grade is None:
            self.grade, self.stable_since = None, None
            return False
        tow = fix['tow_ms']
        if tow == self.last_tow:
            return False
        if self.last_tow is not None and tow < self.last_tow and not (self.last_tow >= 604799000 and tow < 1000):
            self.fault = 'gnss_epoch_reversed'
            return False
        self.last_tow = tow
        position = np.array(enu(fix['latitude'], fix['longitude'], self.anchor))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        arm = np.array([[c, -s], [s, c]]) @ self.antenna
        measurement = position - arm
        h = np.eye(4)[:2]
        # Heading uncertainty also affects antenna-to-body correction.
        variance = max(fix['hacc_m'], 0.05)**2 + float(arm @ arm) * self.heading_variance
        noise = np.eye(2) * variance
        velocity = (fix.get('vel_e_mps'), fix.get('vel_n_mps'), fix.get('speed_accuracy_mps'))
        if all(v is not None for v in velocity):
            if not all(math.isfinite(v) for v in velocity) or velocity[2] <= 0:
                self.fault = 'invalid_gnss_velocity'
                return False
            # Antenna velocity includes angular motion about the robot origin.
            body_velocity = np.array(velocity[:2]) - self.yaw_rate * np.array([-arm[1], arm[0]])
            measurement = np.r_[measurement, body_velocity]
            h = np.eye(4)
            velocity_variance = max(velocity[2], 0.03)**2 + (self.gyro_noise**2 + self.bias_std**2) * float(arm @ arm)
            noise = np.diag([variance, variance, velocity_variance, velocity_variance])
        residual = measurement - h @ self.state
        innovation = h @ self.covariance @ h.T + noise
        if self.initialized and float(residual @ np.linalg.solve(innovation, residual)) > 25:
            self.fault = 'gnss_innovation_rejected'
            self.stable_since = None
            return False
        if not self.initialized:
            self.state[:2] = measurement[:2]
            if len(measurement) == 4:
                self.state[2:] = measurement[2:]
            residual = measurement - h @ self.state
        kalman = np.linalg.solve(innovation, h @ self.covariance).T
        self.state += kalman @ residual
        identity = np.eye(4) - kalman @ h
        self.covariance = identity @ self.covariance @ identity.T + kalman @ noise @ kalman.T
        if self.grade != grade or self.fix_at is None or stamp - self.fix_at >= self.config['sensor_timeout_s']:
            self.stable_since = stamp
        self.grade, self.fix_at, self.initialized = grade, stamp, True
        return not self.fault

    def snapshot(self, now):
        fresh = (self.stamp is not None and self.fix_at is not None
                 and 0 <= now - self.stamp < self.config['sensor_timeout_s']
                 and 0 <= now - self.fix_at < self.config['sensor_timeout_s'])
        return {'experimental': True, 'motion_allowed': False, 'fault': self.fault,
                'inputs_stable': bool(fresh and self.grade and not self.fault and self.stable_since is not None
                                      and now - self.stable_since >= self.config['gnss_stable_s']),
                'grade': self.grade, 'position_enu_m': self.state[:2].tolist() if self.initialized else None,
                'velocity_enu_mps': self.state[2:].tolist() if self.initialized else None,
                'yaw_rad': self.yaw, 'yaw_std_rad': math.sqrt(self.heading_variance),
                'position_covariance': self.covariance[:2, :2].tolist()}
