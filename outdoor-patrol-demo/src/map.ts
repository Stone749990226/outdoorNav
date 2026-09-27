import data from "./campus.json";
import type { Point, Road, Region, Node, Task } from "./types";
export const campus = data as typeof data & {
  nodes: Node[];
  roads: Road[];
  regions: Region[];
  tasks: Task[];
};
export const roads: Road[] = data.roads;
export const regions = data.regions as Region[];
export const presets: Task[] = data.tasks;
export const home: Point = presets[0].point;
export const scale = data.meters_per_unit;
export const toMeters = (p: Point) => ({
  x: +(p.x * scale).toFixed(3),
  y: +((data.height - p.y) * scale).toFixed(3),
});
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
