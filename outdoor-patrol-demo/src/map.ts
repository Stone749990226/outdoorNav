import data from "./campus.json";
import calibration from "./calibration.json";
import type { Point, Road, Region, Node, Task } from "./types";
export const campus = data as typeof data & {
  nodes: Node[];
  roads: Road[];
  regions: Region[];
  tasks: Task[];
};
export const roads: Road[] = data.roads;
export const regions: Region[] = data.regions;
export const presets: Task[] = data.tasks;
export const home: Point = presets[0].point;
export { calibration };
export const scale = calibration.meters_per_unit;
const latitude = (calibration.origin_wgs84.latitude * Math.PI) / 180;
const eccentricitySquared = 6.69437999014e-3;
const denominator = Math.sqrt(1 - eccentricitySquared * Math.sin(latitude) ** 2);
const eastRadius = (6378137 / denominator) * Math.cos(latitude);
const northRadius =
  (6378137 * (1 - eccentricitySquared)) / denominator ** 3;
export const toMap = (p: Point): Point => {
  const dx = p.x - calibration.map_origin_svg.x;
  const dy = calibration.map_origin_svg.y - p.y;
  const angle = calibration.rotation_rad;
  return {
    x: scale * (Math.cos(angle) * dx - Math.sin(angle) * dy),
    y: scale * (Math.sin(angle) * dx + Math.cos(angle) * dy),
  };
};
export const toMeters = (p: Point) => {
  const result = toMap(p);
  return { x: +result.x.toFixed(3), y: +result.y.toFixed(3) };
};
export const mapToGeo = (p: Point) => ({
  latitude: +(
    calibration.origin_wgs84.latitude +
    ((p.y / northRadius) * 180) / Math.PI
  ).toFixed(9),
  longitude: +(
    calibration.origin_wgs84.longitude +
    ((p.x / eastRadius) * 180) / Math.PI
  ).toFixed(9),
});
export const geoToMap = (latitudeDegrees: number, longitudeDegrees: number): Point => ({
  x: ((longitudeDegrees - calibration.origin_wgs84.longitude) * Math.PI * eastRadius) / 180,
  y: ((latitudeDegrees - calibration.origin_wgs84.latitude) * Math.PI * northRadius) / 180,
});
export const mapToSvg = (p: Point): Point => {
  const angle = calibration.rotation_rad;
  return {
    x: calibration.map_origin_svg.x + (Math.cos(angle) * p.x + Math.sin(angle) * p.y) / scale,
    y: calibration.map_origin_svg.y + (Math.sin(angle) * p.x - Math.cos(angle) * p.y) / scale,
  };
};
export const toGeoreferenced = (p: Point) => {
  const map = toMap(p);
  return { ...toMeters(p), ...mapToGeo(map) };
};
export const mapYaw = (yaw: number) =>
  Math.atan2(
    Math.sin(yaw + calibration.rotation_rad),
    Math.cos(yaw + calibration.rotation_rad),
  );
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
