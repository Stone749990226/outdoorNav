import math
import unittest

from pathlib import Path
from outdoornav.receiver import nmea_records, read_log
from outdoornav.coordinates import enu

LOG = Path(__file__).resolve().parents[1] / "北侧路口到东侧楼前.ubx"


class PrepareDemoTests(unittest.TestCase):
    def test_checksum_rejects_corruption(self):
        line = b"$GNGGA,025119.00,3013.43580,N,12015.59004,E,4,12,0.65,9.6,M,7.5,M,1.0,0656*5A\r\n"
        records, bad = nmea_records(line)
        self.assertEqual(len(records), 1)
        self.assertEqual(bad, 0)
        records, bad = nmea_records(line.replace(b"3013.43580", b"3013.43581"))
        self.assertEqual(len(records), 0)
        self.assertEqual(bad, 1)

    def test_real_log_quality_and_jump(self):
        _, bad, fixes, pvt = read_log(LOG.read_bytes())
        self.assertEqual(bad, 0)
        self.assertEqual(sum(f["quality"] == 4 for f in fixes), 150)
        self.assertEqual(sum(f["quality"] == 5 for f in fixes), 130)
        self.assertEqual(sum(f["quality"] == 2 for f in fixes), 467)
        self.assertEqual(len(pvt), 747)
        self.assertEqual(sum(f["carrier"] == 2 for f in pvt), 150)
        jumps = [
            math.hypot(*enu(after["latitude"], after["longitude"], before))
            for before, after in zip(fixes, fixes[1:])
        ]
        self.assertGreater(max(jumps), 28)
        self.assertLess(max(jumps), 29)
