#!/usr/bin/env python3
"""只读采样机头航向；可与现场测得的真北指南针角度比较，不修改标定。"""

import argparse
import json
import math
import time
from urllib.request import urlopen


def difference(a, b):
    return (a - b + 180) % 360 - 180


def summarize(values, reference=None):
    x = sum(math.cos(math.radians(v)) for v in values)
    y = sum(math.sin(math.radians(v)) for v in values)
    if math.hypot(x, y) / len(values) < 0.1:
        raise ValueError("航向变化过大，请保持原地站稳后重测")
    mean = math.degrees(math.atan2(y, x)) % 360
    errors = [difference(v, mean) for v in values]
    result = {
        "samples": len(values),
        "compass_true_north_deg": round(mean, 2),
        "sample_std_deg": round(math.sqrt(sum(e * e for e in errors) / len(errors)), 3),
        "sample_span_deg": round(max(errors) - min(errors), 3),
        "note": "短时波动不代表绝对航向精度；需在多个朝向与独立参照比较",
    }
    if reference is not None:
        result["reference_true_north_deg"] = reference
        result["error_deg"] = round(difference(mean, reference), 2)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:19980/status")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--reference-deg", type=float, help="现场真北角度：北0、东90、南180、西270")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 15:
        parser.error("采样时长须为 1～15 秒")
    if args.reference_deg is not None and not (math.isfinite(args.reference_deg) and 0 <= args.reference_deg < 360):
        parser.error("真北角度须为 [0, 360)")
    values = []
    rejected = 0
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        with urlopen(args.url, timeout=2) as response:
            status = json.load(response)
        heading = status.get("heading") or {}
        yaw, age = heading.get("body_yaw_enu_rad"), heading.get("angle_age_s")
        if (type(yaw) in (int, float) and math.isfinite(yaw)
                and type(age) in (int, float) and 0 <= age < 0.5):
            # 定位服务：东为零、逆时针；指南针：北为零、顺时针。
            values.append((90 - math.degrees(yaw)) % 360)
        else:
            rejected += 1
        time.sleep(0.1)
    if len(values) < 5:
        raise SystemExit("新鲜航向样本不足，请检查 IMU 数据")
    print(json.dumps({**summarize(values, args.reference_deg), "rejected": rejected}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
