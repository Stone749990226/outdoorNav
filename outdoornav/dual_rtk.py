"""Read-only dual-antenna NMEA observations, separate from navigation readiness.

THS A indicates valid true heading. HDT has no validity/solution flag, so it is
reported as an unverified observation. Neither HDOP nor RTK fix class is hAcc.
"""

import math

from .receiver import degrees


class DualRtkDecoder:
    def __init__(self, *, timeout_s=1.0, baseline_to_body_yaw_rad=None):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Positive input timeout required')
        if baseline_to_body_yaw_rad is not None and not math.isfinite(baseline_to_body_yaw_rad):
            raise ValueError('Heading mounting offset must be finite')
        self.timeout = timeout_s
        self.offset = baseline_to_body_yaw_rad
        self.buffer = bytearray()
        self.counts = {}
        self.bad_checksums = self.malformed = 0
        self.position, self.position_at = None, -math.inf
        self.heading, self.heading_at = None, -math.inf
        self.hdt = None
        self.last_epoch = None
        self.fault = None

    def feed(self, raw, now):
        self.buffer.extend(raw)
        if len(self.buffer) > 65536:
            self.buffer.clear()
            self.malformed += 1
            return
        while b'\n' in self.buffer:
            line, _, remainder = self.buffer.partition(b'\n')
            self.buffer = bytearray(remainder)
            try:
                text = line.decode('ascii').strip()
                if not text.startswith('$'):
                    continue
                body, checksum = text[1:].rsplit('*', 1)
                value = 0
                for byte in body.encode('ascii'):
                    value ^= byte
                if len(checksum) != 2 or value != int(checksum, 16):
                    self.bad_checksums += 1
                    continue
                fields = body.split(',')
                name, values = fields[0], fields[1:]
                self.counts[name] = self.counts.get(name, 0)+1
                if name.endswith('GGA'):
                    self.gga(values, now)
                elif name.endswith('THS'):
                    # An explicit invalid status revokes the previous heading.
                    self.heading, self.heading_at = None, -math.inf
                    if len(values) >= 2 and values[1] == 'A':
                        angle = float(values[0])
                        if not math.isfinite(angle) or not 0 <= angle <= 360:
                            raise ValueError('Invalid THS angle')
                        self.heading = {'true_heading_deg': angle % 360, 'source': name,
                                        'status': 'A', 'reference': 'true_north_clockwise'}
                        self.heading_at = now
                elif name.endswith('HDT') and len(values) >= 2 and values[1] == 'T':
                    angle = float(values[0])
                    if math.isfinite(angle) and 0 <= angle <= 360:
                        self.hdt = {'true_heading_deg': angle % 360, 'source': name,
                                    'validity_verified': False}
                # TRA field layout is receiver-specific; never guess an index.
            except (ValueError, IndexError, UnicodeError):
                self.malformed += 1

    def gga(self, fields, now):
        quality = int(fields[5])
        if quality not in (1, 2, 4, 5):
            self.position, self.position_at = None, -math.inf
            return
        epoch = fields[0]
        hh, mm, ss = int(epoch[:2]), int(epoch[2:4]), float(epoch[4:])
        if not 0 <= hh < 24 or not 0 <= mm < 60 or not 0 <= ss < 60:
            raise ValueError('Invalid GNSS UTC')
        seconds = hh*3600+mm*60+ss
        if self.last_epoch is not None:
            if seconds == self.last_epoch:
                return  # repeated fix does not extend freshness
            if seconds < self.last_epoch and not (self.last_epoch > 86399 and seconds < 1):
                self.fault = 'gnss_epoch_reversed; inspect receiver and restart'
                return
        if fields[2] not in ('N', 'S') or fields[4] not in ('E', 'W'):
            raise ValueError('Invalid hemisphere')
        lat, lon = degrees(fields[1], fields[2]), degrees(fields[3], fields[4])
        hdop = float(fields[7]) if fields[7] else None
        altitude = float(fields[8])
        age = float(fields[12]) if len(fields) > 12 and fields[12] else None
        if (not all(math.isfinite(v) for v in (lat, lon, altitude))
                or not -90 <= lat <= 90 or not -180 <= lon <= 180
                or (hdop is not None and (not math.isfinite(hdop) or hdop <= 0))
                or (age is not None and (not math.isfinite(age) or age < 0))):
            raise ValueError('Invalid GGA measurement')
        self.last_epoch = seconds
        self.position = {'latitude': lat, 'longitude': lon, 'msl_m': altitude,
                         'gga_quality': quality, 'utc': epoch, 'satellites': int(fields[6]),
                         'solution': {1: 'single', 2: 'differential', 4: 'fixed', 5: 'float'}[quality],
                         'hdop': hdop, 'hacc_m': None, 'correction_age_s': age}
        self.position_at = now

    def snapshot(self, now):
        position_fresh = self.position is not None and 0 <= now-self.position_at < self.timeout
        heading_fresh = self.heading is not None and 0 <= now-self.heading_at < self.timeout
        yaw = None
        if heading_fresh and self.offset is not None:
            angle = math.pi/2-math.radians(self.heading['true_heading_deg'])+self.offset
            yaw = math.atan2(math.sin(angle), math.cos(angle))
        return {'schema_version': 1, 'receiver': 'dual_antenna_nmea',
                'position': self.position, 'position_fresh': position_fresh,
                'heading': self.heading, 'heading_fresh': heading_fresh,
                'unverified_hdt': self.hdt, 'body_yaw_enu_rad': yaw,
                'mounting_confirmed': self.offset is not None,
                'fault': self.fault, 'counts': dict(self.counts),
                'bad_checksums': self.bad_checksums, 'malformed': self.malformed,
                'navigation_ready': False,
                'limitations': ['NMEA GGA does not provide measured horizontal accuracy',
                                'THS A alone does not prove a fixed dual-antenna baseline solution',
                                'Antenna reference point, baseline direction and time alignment require verification']}
