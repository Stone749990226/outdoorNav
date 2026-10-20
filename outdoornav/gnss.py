"""Decode receiver bytes; correction age is from GGA, never RTCM arrival."""

import math
import struct

from .receiver import nmea_records, ubx_records


class FixDecoder:
    def __init__(self):
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
            if self.gga and 0 <= now - self.gga[0] <= 1:
                quality, age = self.gga[1:3]
            return {
                "latitude": lat * 1e-7,
                "longitude": lon * 1e-7,
                "altitude_m": height / 1000,
                "msl_m": hmsl / 1000,
                "hacc_m": hacc / 1000,
                "vacc_m": vacc / 1000,
                "fixed": (p[21] >> 6) & 3 == 2,
                "valid": bool(p[21] & 1) and p[20] in (3, 4),
                "differential": bool(p[21] & 2),
                "satellites": p[23],
                "correction_age_s": age,
                "gga_quality": quality,
                "tow_ms": struct.unpack_from("<I", p, 0)[0],
            }
        return None


def usable_fix(fix):
    age = fix.get("correction_age_s")
    return bool(
        fix.get("valid")
        and fix.get("fixed")
        and fix.get("differential")
        and fix.get("gga_quality") == 4
        and math.isfinite(fix["hacc_m"])
        and 0 < fix["hacc_m"] <= 0.20
        and age is not None
        and math.isfinite(age)
        and 0 <= age <= 5
        and -90 <= fix["latitude"] <= 90
        and -180 <= fix["longitude"] <= 180
    )
