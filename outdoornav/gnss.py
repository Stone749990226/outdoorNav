"""Decode receiver bytes; correction age is from GGA, never RTCM arrival."""

import math
import struct

from .receiver import nmea_records, ubx_records
from .config import DEFAULTS


class FixDecoder:
    def __init__(self, config=None):
        self.timeout = (config or DEFAULTS)["sensor_timeout_s"]
        self.gga = None

    def feed(self, raw, now):
        for name, fields in nmea_records(raw)[0]:
            if name.endswith("GGA") and len(fields) >= 14:
                try:
                    age = float(fields[12]) if fields[12] else None
                    self.gga = (now, int(fields[5]), age, fields[0])
                except ValueError:
                    self.gga = None
        for cls, ident, p in ubx_records(raw):
            if (cls, ident) != (1, 7) or len(p) < 92:
                continue
            lon, lat, height, hmsl, hacc, vacc = struct.unpack_from("<iiiiII", p, 24)
            age = None
            quality = 0
            if self.gga and 0 <= now - self.gga[0] <= self.timeout:
                quality, age = self.gga[1:3]
            return {
                "latitude": lat * 1e-7,
                "longitude": lon * 1e-7,
                "altitude_m": height / 1000,
                "msl_m": hmsl / 1000,
                "hacc_m": hacc / 1000,
                "vacc_m": vacc / 1000,
                "fixed": (p[21] >> 6) & 3 == 2,
                "float": (p[21] >> 6) & 3 == 1,
                "valid": bool(p[21] & 1) and p[20] in (3, 4),
                "differential": bool(p[21] & 2),
                "satellites": p[23],
                "vel_n_mps": struct.unpack_from("<i", p, 48)[0] / 1000,
                "vel_e_mps": struct.unpack_from("<i", p, 52)[0] / 1000,
                "speed_accuracy_mps": struct.unpack_from("<I", p, 68)[0] / 1000,
                "correction_age_s": age,
                "gga_quality": quality,
                "tow_ms": struct.unpack_from("<I", p, 0)[0],
            }
        return None


def fix_grade(fix, config=None):
    """Return a usable carrier solution, never ordinary differential GNSS."""
    config = {**DEFAULTS, **(config or {})}
    if not fix:
        return None
    age, hacc = fix.get('correction_age_s'), fix.get('hacc_m')
    age_ok = (isinstance(age, (int, float)) and not isinstance(age, bool)
              and math.isfinite(age) and 0 <= age <= config['correction_max_age_s'])
    # Explicit demo policy only: GGA age remains visible, but is not treated as
    # a reliable measure of current RTCM delivery. Fresh epochs, solution flags
    # and Doppler quality are still checked by the localization consumer.
    if not (fix.get('valid') and fix.get('differential')
            and (age_ok or not config['require_correction_age'])
            and isinstance(hacc, (int, float)) and math.isfinite(hacc) and hacc > 0
            and -90 <= fix.get('latitude', math.inf) <= 90
            and -180 <= fix.get('longitude', math.inf) <= 180):
        return None
    if fix.get('fixed') and fix.get('gga_quality') == 4 and hacc <= config['gnss_fixed_hacc_m']:
        return 'fixed'
    if fix.get('float') and fix.get('gga_quality') == 5 and hacc <= config['gnss_float_hacc_m']:
        return 'float'
    return None


def usable_fix(fix):
    return fix_grade(fix) == 'fixed'
