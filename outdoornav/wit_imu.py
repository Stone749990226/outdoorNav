"""WIT 0x55 packets and measured sensor-to-body/magnetic-to-true conversion."""

import json
import math
import struct

import numpy as np

from .lio import checked_matrix


class WitImuDecoder:
    def __init__(self, config):
        self.config = config
        self.buffer = bytearray()
        self.counts = {}
        self.bad_checksums = self.resync_bytes = 0
        self.angles = self.gyro = self.acceleration = self.magnetic = None
        self.angle_at = self.gyro_at = -math.inf
        self.registers = None
        self.register_at = -math.inf
        self.mode = None
        self.packet_at = -math.inf
        self.generation = None
        self.fault = None

    def feed(self, raw, received):
        self.buffer.extend(raw)
        if len(self.buffer) > 65536:
            self.buffer.clear()
            self.fault = 'imu_input_overflow'
            return
        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                self.resync_bytes += 1
                continue
            frame = bytes(self.buffer[:11])
            if sum(frame[:10]) & 255 != frame[10]:
                del self.buffer[0]
                self.bad_checksums += 1
                continue
            del self.buffer[:11]
            kind = frame[1]
            self.counts[hex(kind)] = self.counts.get(hex(kind), 0) + 1
            values = struct.unpack('<hhhh', frame[2:10])
            if kind == 0x51:
                self.acceleration = [v/32768*16*9.80665 for v in values[:3]]
            elif kind == 0x52:
                self.gyro = [math.radians(v/32768*2000) for v in values[:3]]
                self.gyro_at = received
            elif kind == 0x53:
                self.angles = [math.radians(v/32768*180) for v in values[:3]]
                self.angle_at = received
            elif kind == 0x54:
                self.magnetic = list(values[:3])  # raw counts, not tesla
            elif kind == 0x5f:
                self.registers = struct.unpack('<HHHH', frame[2:10])
                self.register_at = received

    def packet(self, data, now):
        """Mode metadata is from this collector's documented AXIS6 read request."""
        value = json.loads(data)
        if value.get('schema_version') != 1 or value.get('source') != 'wit_imu':
            raise ValueError('Not a WIT collector packet')
        received = value['received']
        if (isinstance(received, bool) or not isinstance(received, (int, float)) or
                not math.isfinite(received) or not 0 <= now-received < self.config['timeout_s']):
            return
        generation = value.get('generation')
        if not isinstance(generation, str) or not 1 <= len(generation) <= 64:
            raise ValueError('WIT collector generation required')
        if self.generation is not None and generation != self.generation:
            self.fault = 'imu_source_restarted; inspect and restart localization'
        self.generation = generation
        mode = value.get('axis6_register')
        if mode is not None and (type(mode) is not int or mode not in (0, 1)):
            raise ValueError('Invalid queried IMU algorithm mode')
        self.mode = mode
        self.feed(bytes.fromhex(value['raw_hex']), received)
        self.packet_at = received
        return True

    def snapshot(self, now):
        cfg = self.config
        angles_fresh = self.angles is not None and 0 <= now-self.angle_at < cfg['timeout_s']
        gyro_fresh = self.gyro is not None and 0 <= now-self.gyro_at < cfg['timeout_s']
        body_yaw = body_rate = None
        mounted = cfg['mounting_confirmed'] and cfg['base_from_imu'] is not None
        if mounted:
            rotation = checked_matrix(cfg['base_from_imu'])[:3, :3]
            if gyro_fresh:
                body_rate = float((rotation @ np.array(self.gyro))[2])
            course = cfg.get('course_heading', False)
            if angles_fresh and (course or (cfg['reference_confirmed'] and cfg['magnetic_declination_rad'] is not None)):
                roll, pitch, yaw = self.angles
                cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
                world_sensor = np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                                         [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                                         [-sp,cp*sr,cp*cr]])
                world_body = world_sensor @ rotation.T
                # East-positive declination rotates magnetic ENU clockwise in true ENU.
                declination = 0.0 if cfg['magnetic_declination_rad'] is None else cfg['magnetic_declination_rad']
                angle = math.atan2(world_body[1, 0], world_body[0, 0])-declination
                body_yaw = math.atan2(math.sin(angle), math.cos(angle))
        quality = bool(cfg.get('course_heading', False) or (cfg['quality_confirmed'] and cfg['yaw_std_deg'] is not None))
        # The READADDR query can go unanswered while the data stream stays
        # healthy; in course_heading demo mode the magnetic anchor is corrected
        # by measured travel direction, so an unverified mode still passes.
        mode_ok = self.mode == 0 or (self.mode is None and cfg.get('course_heading', False))
        heading_ready = bool(angles_fresh and body_yaw is not None and mode_ok
                             and quality and not self.fault)
        return {'source':'wit_imu', 'ready':heading_ready, 'heading_fresh':heading_ready,
                'angles_fresh':angles_fresh,'gyro_fresh':gyro_fresh,
                'raw_euler_rad':self.angles,'raw_gyro_rps':self.gyro,
                'acceleration_mps2':self.acceleration,'magnetic_raw':self.magnetic,
                'body_yaw_enu_rad':body_yaw,'body_yaw_rate_rps':body_rate,
                'angle_at':self.angle_at,'gyro_at':self.gyro_at,
                'axis6_register':self.mode,'nine_axis_verified':self.mode == 0,
                'mounting_confirmed':bool(mounted),'reference_confirmed':cfg['reference_confirmed'],
                'quality_confirmed':cfg['quality_confirmed'],'generation':self.generation,
                'fault':self.fault,'counts':dict(self.counts),'bad_checksums':self.bad_checksums,
                'time_basis':'host_monotonic_receive; device acquisition time not validated'}
