#!/usr/bin/env python3
"""下载 Esri World Imagery 卫星瓦片,拼成与 web 地图 SVG 坐标系精确对齐的底图。

用法: python3 tools/fetch_satellite_tiles.py [margin米=10]
输出: web/src/assets/north-campus-satellite.webp 及经纬度范围 JSON

坐标系(与 web/src/map.ts 的 drawing() 一致):
  米制坐标 X 东 Y 北;SVG 画布 = (floor(minX-10)..ceil(maxX+10), 同理 Y),scale=0.125。
  本脚本在该画布基础上四周再扩 margin 米,输出图以
  x=-margin/scale, y=-margin/scale 的 <image> 铺进 MapView 即精确对齐。
"""

import io
import json
import math
import os
import subprocess
import sys
import time

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUNDLE = os.path.join(ROOT, "scenes/north_campus/generated/bundle.json")
OUT = os.path.join(ROOT, "web/src/assets/north-campus-satellite.webp")

Z = 19  # 该区域 Esri z=20 无影像,用 z=19(约 0.26 m/px)
TILE = 256
SCALE = 0.125
PAD_M = 10  # 与 map.ts 相同的画布内边距
URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def meters_per_degree(lat_deg: float) -> tuple[float, float]:
    lat = math.radians(lat_deg)
    sin = math.sin(lat)
    denom = math.sqrt(1 - WGS84_E2 * sin * sin)
    east = (WGS84_A / denom) * math.cos(lat)
    north = (WGS84_A * (1 - WGS84_E2)) / denom**3
    return east * math.pi / 180, north * math.pi / 180


def geo_to_global_px(lon: float, lat: float) -> tuple[float, float]:
    n = TILE * 2**Z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def fetch(url: str) -> bytes:
    for attempt in range(3):
        proc = subprocess.run(
            ["curl", "-fsS", "--max-time", "20", "-A", "outdoornav-demo/1.0", url],
            capture_output=True,
        )
        if proc.returncode == 0 and proc.stdout:
            return proc.stdout
        print(f"  retry {attempt + 1}: {url}", file=sys.stderr)
        time.sleep(1 + attempt)
    raise RuntimeError(f"download failed: {url}")


def main() -> None:
    margin = float(sys.argv[1]) if len(sys.argv) > 1 else 10
    if not math.isfinite(margin) or margin < 0:
        raise ValueError("margin must be a finite, non-negative number")
    bundle = json.load(open(BUNDLE, encoding="utf-8"))
    origin = bundle["origin_wgs84"]
    points = [p for r in bundle["network"]["roads"] for p in r["points"]]

    # 与 map.ts 逐字一致: minX=floor(min-10), maxX=ceil(max+10), 同上 Y
    canvas_min_x = math.floor(min(p["x"] for p in points) - PAD_M)
    canvas_max_x = math.ceil(max(p["x"] for p in points) + PAD_M)
    canvas_min_y = math.floor(min(p["y"] for p in points) - PAD_M)
    canvas_max_y = math.ceil(max(p["y"] for p in points) + PAD_M)
    min_x, max_x = canvas_min_x - margin, canvas_max_x + margin
    min_y, max_y = canvas_min_y - margin, canvas_max_y + margin

    dlon, dlat = meters_per_degree(origin["latitude"])
    lon_min = origin["longitude"] + min_x / dlon
    lon_max = origin["longitude"] + max_x / dlon
    lat_min = origin["latitude"] + min_y / dlat
    lat_max = origin["latitude"] + max_y / dlat

    out_w = int(round((max_x - min_x) / SCALE))
    out_h = int(round((max_y - min_y) / SCALE))
    image_x = (min_x - canvas_min_x) / SCALE  # = -margin/scale
    image_y = (canvas_max_y - max_y) / SCALE  # = -margin/scale

    px_a, py_a = geo_to_global_px(lon_min, lat_max)  # 目标图左上角
    px_b, py_b = geo_to_global_px(lon_max, lat_min)  # 目标图右下角
    tx0, ty0 = math.floor(px_a / TILE), math.floor(py_a / TILE)
    tx1, ty1 = math.floor(px_b / TILE), math.floor(py_b / TILE)
    print(f"tiles x[{tx0}..{tx1}] y[{ty0}..{ty1}] -> canvas {out_w}x{out_h}")

    mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * TILE, (ty1 - ty0 + 1) * TILE))
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            data = fetch(URL.format(z=Z, y=ty, x=tx))
            try:
                tile = Image.open(io.BytesIO(data)).convert("RGB")
            except Exception as exc:  # noqa: BLE001
                raise RuntimeError(f"bad tile {tx},{ty}: {exc}") from exc
            mosaic.paste(tile, ((tx - tx0) * TILE, (ty - ty0) * TILE))
            print(f"  ok {tx},{ty}")

    # 目标图 (u,v) 采样全局像素 (px_a + u*(px_b-px_a)/out_w, py_a + v*(py_b-py_a)/out_h)
    # mosaic 的本地坐标 = 全局像素 - 瓦片起点
    src = mosaic.transform(
        (out_w, out_h),
        Image.Transform.AFFINE,
        (
            (px_b - px_a) / out_w,
            0,
            px_a - tx0 * TILE,
            0,
            (py_b - py_a) / out_h,
            py_a - ty0 * TILE,
        ),
        resample=Image.Resampling.BILINEAR,
    )
    src.save(OUT, "WEBP", quality=82, method=6)
    with open(OUT.replace(".webp", ".json"), "w", encoding="utf-8") as metadata:
        json.dump(
            {"west": lon_min, "east": lon_max, "north": lat_max, "south": lat_min},
            metadata,
            indent=2,
        )
        metadata.write("\n")
    print(f"wrote {OUT} ({out_w}x{out_h}, {os.path.getsize(OUT)//1024} KiB)")
    print(f"image x={image_x:.1f} y={image_y:.1f} width={out_w} height={out_h}")


if __name__ == "__main__":
    main()
