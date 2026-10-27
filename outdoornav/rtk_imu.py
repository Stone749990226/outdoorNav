"""RTK position/Doppler + measured IMU yaw/rate; no external odometry input.

Local pose integrates Doppler velocity and gyro only. Global pose is the latest
accepted GNSS position, short-extrapolated with Doppler and corrected for the
antenna arm. Their transform separates GNSS corrections from continuous odom.
No accelerometer integration or commanded-velocity feedback is used.
"""

import math

from .coordinates import enu
from .gnss import fix_grade
from .localization import wrap


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c*x-s*y, s*x+c*y


class RtkImuFeedback:
    def __init__(self, anchor, config):
        self.anchor, self.config = anchor, config
        self.fix = None
        self.grade = None
        self.fix_at = -math.inf
        self.last_tow = None
        self.stable_since = None
        self.previous_fix = None
        self.position = self.antenna_velocity = None
        self.velocity_valid = False
        self.imu = {}
        self.fault = None
        self.initialized = False
        self.x = self.y = self.yaw = 0.0
        self.last_gyro_at = None
        self.last_rate = None
        self.last_velocity = None
        self.last_map_rotation = None
        self.output = None
        self.reason = 'waiting_for_rtk_and_imu'

    def update_fix(self, fix, received):
        self.fix = dict(fix)
        grade = fix_grade(fix, self.config)
        velocity = tuple(fix.get(k) for k in ('vel_e_mps','vel_n_mps','speed_accuracy_mps'))
        velocity_ok = all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in velocity)
        velocity_ok = velocity_ok and 0 < velocity[2] <= self.config['gnss_speed_accuracy_mps']
        if grade is None or not velocity_ok:
            self.grade, self.stable_since, self.velocity_valid = None, None, False
            self.reason = 'gnss_quality_unusable' if grade is None else 'gnss_doppler_unusable'
            return  # Invalid receiver time must not seed the usable-fix replay guard.
        tow = fix['tow_ms']
        if tow == self.last_tow:
            return
        if self.last_tow is not None and tow < self.last_tow:
            if not (self.last_tow >= 604799000 and tow < 1000):
                self.fault = 'gnss_epoch_reversed; inspect receiver and restart'
                return
        self.last_tow = tow
        position = enu(fix['latitude'],fix['longitude'],self.anchor)
        if self.previous_fix:
            old, old_velocity, old_at = self.previous_fix
            dt = received-old_at
            predicted = (old[0]+old_velocity[0]*dt,old[1]+old_velocity[1]*dt)
            limit = .75 if grade == 'fixed' else max(.75,2*fix['hacc_m'])
            if dt < 0 or math.dist(position,predicted) > limit:
                self.fault = 'gnss_position_jump; inspect RTK and restart'
                return
        if self.grade != grade or received-self.fix_at >= self.config['sensor_timeout_s']:
            self.stable_since = received
        self.grade, self.fix_at, self.velocity_valid = grade, received, True
        self.position, self.antenna_velocity = position, velocity[:2]
        self.previous_fix = (position, velocity[:2], received)

    def course_yaw(self, now):
        """Travel direction from measured Doppler; None while too slow to trust."""
        if not (self.velocity_valid and self.position_ready(now)):
            return None
        east, north = self.antenna_velocity
        if math.hypot(east, north) < 0.25:
            return None
        return math.atan2(north, east)

    def position_ready(self, now):
        return bool(self.grade and self.velocity_valid and self.position is not None
                    and 0 <= now-self.fix_at < self.config['sensor_timeout_s'])

    def heading_ready(self, now):
        cfg = self.config['wit_imu']
        return bool(self.imu.get('ready') and self.imu.get('body_yaw_enu_rad') is not None
                    and 0 <= now-self.imu.get('angle_at',-math.inf) < cfg['timeout_s'])

    def update_imu(self, observation, now):
        self.imu = dict(observation)
        if observation.get('fault'):
            self.fault = observation['fault']
        t, rate = observation.get('gyro_at'), observation.get('body_yaw_rate_rps')
        if (not isinstance(t,(int,float)) or not math.isfinite(t) or
                not isinstance(rate,(int,float)) or not math.isfinite(rate) or
                not 0 <= now-t < self.config['wit_imu']['timeout_s']):
            return
        if self.last_gyro_at is not None and t <= self.last_gyro_at:
            return
        if not self.initialized:
            if not (self.position_ready(t) and self.heading_ready(t)):
                return
            self.initialized = True
            self.last_map_rotation = observation['body_yaw_enu_rad']
            self.last_gyro_at, self.last_rate = t, rate
        else:
            dt = t-self.last_gyro_at
            if dt > self.config['wit_imu']['timeout_s']:
                self.fault = 'imu_time_gap; inspect input and restart'
                return
            self.yaw = wrap(self.yaw+.5*(self.last_rate+rate)*dt)
        dt = t-self.last_gyro_at
        self.last_gyro_at, self.last_rate = t, rate
        if not self.position_ready(t):
            self.last_velocity = None
            return
        if self.heading_ready(t):
            # Pair angle and gyro by receive time; bound the short extrapolation.
            skew = t-observation['angle_at']
            if abs(skew) > self.config['wit_imu']['timeout_s']:
                return
            if self.config['wit_imu'].get('course_heading'):
                # Magnetic heading only anchors the start; the measured travel
                # direction re-anchors it while moving, gyro carries it at rest.
                absolute_yaw = wrap(self.yaw+self.last_map_rotation)
                course = self.course_yaw(t)
                if course is not None:
                    error = wrap(course-absolute_yaw)
                    if abs(error) < 1.2:
                        self.last_map_rotation = wrap(
                            self.last_map_rotation+max(-0.4, min(0.4, error)))
                        absolute_yaw = wrap(self.yaw+self.last_map_rotation)
            else:
                absolute_yaw = wrap(observation['body_yaw_enu_rad']+rate*skew)
                expected = wrap(self.yaw+self.last_map_rotation)
                limit = max(.35, 3*math.radians(self.config['wit_imu']['yaw_std_deg'] or 1.0))
                if self.output is not None and abs(wrap(absolute_yaw-expected)) > limit:
                    self.fault = 'imu_heading_jump; inspect magnetic interference and restart'
                    return
                self.last_map_rotation = wrap(absolute_yaw-self.yaw)
        else:
            # Keep measured velocity/rate feedback for stopping, never global readiness.
            absolute_yaw = wrap(self.yaw+self.last_map_rotation)
        ax, ay = self.config['antenna_xy_m']
        arm_x, arm_y = rotate(ax,ay,absolute_yaw)
        ve, vn = self.antenna_velocity
        vx, vy = rotate(ve+rate*arm_y,vn-rate*arm_x,-absolute_yaw)
        local_velocity = rotate(vx,vy,self.yaw)
        if dt > 0:
            old = self.last_velocity or local_velocity
            self.x += .5*(old[0]+local_velocity[0])*dt
            self.y += .5*(old[1]+local_velocity[1])*dt
        self.last_velocity = local_velocity
        prediction = t-self.fix_at
        gx = self.position[0]+ve*prediction-arm_x+self.config['map_offset_xy_m'][0]
        gy = self.position[1]+vn*prediction-arm_y+self.config['map_offset_xy_m'][1]
        self.output = {'stamp':t,'odom_pose':{'x':self.x,'y':self.y,'yaw':self.yaw},
                       'map_pose':{'x':gx,'y':gy,'yaw':absolute_yaw},
                       'twist':{'linear_x':vx,'linear_y':vy,'angular_z':rate},
                       'prediction_s':prediction,'velocity_accuracy_mps':self.fix['speed_accuracy_mps']}
        self.reason = None

    def feedback_ready(self, now):
        return bool(not self.fault and self.output and self.position_ready(now)
                    and 0 <= now-self.output['stamp'] < self.config['wit_imu']['timeout_s'])

    def ready(self, now):
        return bool(self.feedback_ready(now) and self.heading_ready(now)
                    and self.config['calibrated'] and self.config['antenna_offset_confirmed']
                    and self.config['feedback_confirmed'] and self.stable_since is not None
                    and now-self.stable_since >= self.config['gnss_stable_s'])

    def sample(self, now):
        return self.output if self.feedback_ready(now) else None
