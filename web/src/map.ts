import bundleData from "../../scenes/north_campus/generated/bundle.json" with { type: "json" };
import type { Node, Point, Region, Road, Task } from "./types";
export const bundle = bundleData;
const metricPoints = bundle.network.roads.flatMap((r) => r.points);
const minX = Math.floor(Math.min(...metricPoints.map((p) => p.x)) - 10);
const maxX = Math.ceil(Math.max(...metricPoints.map((p) => p.x)) + 10);
const minY = Math.floor(Math.min(...metricPoints.map((p) => p.y)) - 10);
const maxY = Math.ceil(Math.max(...metricPoints.map((p) => p.y)) + 10);
export const scale = 0.125;
const drawing = (p: Point): Point => ({
  x: (p.x - minX) / scale,
  y: (maxY - p.y) / scale,
});
export const svgOrigin = drawing({ x: 0, y: 0 });
export const roads: Road[] = bundle.network.roads.map((r) => ({
  ...r,
  points: r.points.map(drawing),
}));
export const regions: Region[] = [];
export const presets: Task[] = bundle.network.nodes.map((n) => ({
  id: "p-" + n.id.toLowerCase(),
  name: n.name,
  point: drawing(n),
  dwell: 2,
}));
export const campus = {
  version: bundle.route_version,
  name: "智慧谷北区实测路网",
  source: "Measured UBX receiver logs; illustrative buildings and widths",
  width: (maxX - minX) / scale,
  height: (maxY - minY) / scale,
  navigation: { pick_tolerance_px: 12 },
  nodes: bundle.network.nodes.map((n) => ({ ...n, ...drawing(n) })) as Node[],
  roads,
  regions,
  tasks: presets,
};
export const home: Point = presets[0].point;
export const survey = bundle.survey;
export const exportFrame = {
  schema_version: 1,
  scene_version: bundle.scene_version,
  route_version: bundle.route_version,
  frame_id: "map",
  units: "meters",
  axes: "X east, Y north",
  origin_wgs84: bundle.origin_wgs84,
  quality_status: bundle.network.quality_status,
  simulation_only: true,
};
const latitude = (bundle.origin_wgs84.latitude * Math.PI) / 180;
const eccentricitySquared = 6.69437999014e-3;
const denominator = Math.sqrt(
  1 - eccentricitySquared * Math.sin(latitude) ** 2,
);
const eastRadius = (6378137 / denominator) * Math.cos(latitude);
const northRadius = (6378137 * (1 - eccentricitySquared)) / denominator ** 3;
export const toMap = (p: Point): Point => ({
  x: (p.x - svgOrigin.x) * scale,
  y: (svgOrigin.y - p.y) * scale,
});
export const toMeters = (p: Point) => {
  const result = toMap(p);
  return { x: +result.x.toFixed(3), y: +result.y.toFixed(3) };
};
export const mapToGeo = (p: Point) => ({
  latitude: +(
    bundle.origin_wgs84.latitude +
    ((p.y / northRadius) * 180) / Math.PI
  ).toFixed(9),
  longitude: +(
    bundle.origin_wgs84.longitude +
    ((p.x / eastRadius) * 180) / Math.PI
  ).toFixed(9),
});
export const geoToMap = (
  latitudeDegrees: number,
  longitudeDegrees: number,
): Point => ({
  x:
    ((longitudeDegrees - bundle.origin_wgs84.longitude) *
      Math.PI *
      eastRadius) /
    180,
  y:
    ((latitudeDegrees - bundle.origin_wgs84.latitude) * Math.PI * northRadius) /
    180,
});
export const mapToSvg = drawing;
export const toGeoreferenced = (p: Point) => {
  const map = toMap(p);
  return { ...toMeters(p), ...mapToGeo(map) };
};
export const mapYaw = (yaw: number) => Math.atan2(Math.sin(yaw), Math.cos(yaw));
export const dist = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y);
export const roadLength = (r: Road) =>
  r.points.slice(1).reduce((n, p, i) => n + dist(r.points[i], p), 0);
export const pathD = (ps: Point[]) =>
  ps.map((p, i) => `${i ? "L" : "M"}${p.x},${p.y}`).join(" ");
export function containsPoint(p: Point, polygon: Point[]): boolean {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const a = polygon[i],
      b = polygon[j];
    if (
      a.y > p.y !== b.y > p.y &&
      p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x
    )
      inside = !inside;
  }
  return inside;
}
