import {
  exportFrame,
  campus,
  dist,
  mapYaw,
  scale,
  toGeoreferenced,
} from "./map";
import type { Plan, Point } from "./types";
export const waypointOptions = {
  turn_degrees: 25,
  max_spacing_m: 20,
  junction_radius_m: 3,
};
export type WaypointReason =
  | "start"
  | "goal"
  | "turn"
  | "junction"
  | "spacing"
  | "geometry"
  | "inspect"
  | "return";
export type KeyWaypoint = {
  point: Point;
  distance: number;
  reasons: WaypointReason[];
  name: string;
  dwell: number;
  yaw: number;
};
export const reasonNames: Record<WaypointReason, string> = {
  start: "起点",
  goal: "终点",
  turn: "转弯",
  junction: "路口",
  spacing: "间隔引导",
  geometry: "保持通行",
  inspect: "巡检停留",
  return: "返回起点",
};
export function extractWaypoints(
  points: Point[],
  options: {
    segmentSafe?: (a: Point, b: Point) => boolean;
    stops?: {
      distance: number;
      name: string;
      dwell: number;
      returning?: boolean;
    }[];
  } = {},
): KeyWaypoint[] {
  if (!points.length) return [];
  const cumulative = [0];
  for (let i = 1; i < points.length; i++)
    cumulative.push(cumulative[i - 1] + dist(points[i - 1], points[i]));
  const total = cumulative.at(-1)!;
  const locate = (s: number): Point => {
    let i = cumulative.findIndex((d, index) => index > 0 && d >= s);
    if (i < 0) return { ...points.at(-1)! };
    const len = cumulative[i] - cumulative[i - 1],
      t = len ? (s - cumulative[i - 1]) / len : 0;
    return {
      x: points[i - 1].x + (points[i].x - points[i - 1].x) * t,
      y: points[i - 1].y + (points[i].y - points[i - 1].y) * t,
    };
  };
  const entries: KeyWaypoint[] = [];
  const add = (
    s: number,
    reason: WaypointReason,
    name = reasonNames[reason],
    dwell = 0,
  ) => {
    const old = entries.find((e) => Math.abs(e.distance - s) < 1e-6);
    if (old) {
      if (!old.reasons.includes(reason)) old.reasons.push(reason);
      if (dwell || reason === "return") {
        old.name = name;
        old.dwell = dwell;
      }
      return;
    }
    entries.push({
      point: locate(s),
      distance: s,
      reasons: [reason],
      name,
      dwell,
      yaw: 0,
    });
  };
  add(0, "start");
  add(total, "goal");
  for (let i = 1; i < points.length - 1; i++) {
    const a = points[i - 1],
      b = points[i],
      c = points[i + 1],
      l1 = dist(a, b),
      l2 = dist(b, c);
    if (l1 < 1e-8 || l2 < 1e-8) continue;
    const angle =
      (Math.acos(
        Math.max(
          -1,
          Math.min(
            1,
            ((b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y)) / (l1 * l2),
          ),
        ),
      ) *
        180) /
      Math.PI;
    if (angle >= waypointOptions.turn_degrees) add(cumulative[i], "turn");
  }
  // Project known junctions to the route; preserve separate visits on a loop.
  for (const node of campus.nodes) {
    if (
      campus.roads.filter((r) => r.from === node.id || r.to === node.id)
        .length < 3
    )
      continue;
    let best: { distance: number; offset: number } | null = null;
    const flush = () => {
      if (best) add(best.offset, "junction", node.name);
      best = null;
    };
    for (let i = 1; i < points.length; i++) {
      const a = points[i - 1],
        b = points[i],
        len = dist(a, b);
      if (!len) continue;
      const t = Math.max(
        0,
        Math.min(
          1,
          ((node.x - a.x) * (b.x - a.x) + (node.y - a.y) * (b.y - a.y)) /
            (len * len),
        ),
      );
      const p = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t },
        distance = dist(p, node);
      if (distance <= waypointOptions.junction_radius_m / scale) {
        if (!best || distance < best.distance)
          best = { distance, offset: cumulative[i - 1] + t * len };
      } else flush();
    }
    flush();
  }
  for (const stop of options.stops ?? [])
    add(
      stop.distance,
      stop.returning ? "return" : "inspect",
      stop.name,
      stop.dwell,
    );
  const sorted = () => entries.sort((a, b) => a.distance - b.distance);
  let selected = [...sorted()];
  for (let i = 1; i < selected.length; i++) {
    const span = selected[i].distance - selected[i - 1].distance,
      parts = Math.ceil((span * scale) / waypointOptions.max_spacing_m);
    for (let j = 1; j < parts; j++)
      add(selected[i - 1].distance + (span * j) / parts, "spacing");
  }
  // A sparse waypoint chord must not cross a forbidden area. Road-graph exports
  // conservatively retain every non-collinear bend; area routes use geometry checks.
  const refine = (a: KeyWaypoint, b: KeyWaypoint) => {
    const safe = options.segmentSafe
      ? options.segmentSafe(a.point, b.point)
      : dist(a.point, b.point) >= b.distance - a.distance - 1e-5;
    if (safe) return;
    const indices = cumulative
      .map((s, i) => ({ s, i }))
      .filter((v) => v.s > a.distance + 1e-6 && v.s < b.distance - 1e-6);
    if (!indices.length) throw new Error("途经点之间的路径未通过通行校验");
    const vertex = indices.reduce((x, y) =>
      Math.abs(x.s - (a.distance + b.distance) / 2) <
      Math.abs(y.s - (a.distance + b.distance) / 2)
        ? x
        : y,
    );
    add(vertex.s, "geometry");
    const mid = entries.find((e) => Math.abs(e.distance - vertex.s) < 1e-6)!;
    refine(a, mid);
    refine(mid, b);
  };
  selected = [...sorted()];
  for (let i = 1; i < selected.length; i++)
    refine(selected[i - 1], selected[i]);
  for (const entry of sorted()) {
    let i = cumulative.findIndex(
      (s, index) => index > 0 && s > entry.distance + 1e-6,
    );
    if (i < 0) i = points.length - 1;
    if (i > 0)
      entry.yaw = Math.atan2(
        -(points[i].y - points[i - 1].y),
        points[i].x - points[i - 1].x,
      );
  }
  return entries;
}
export const planPoints = (plan: Plan) => [
  plan.start,
  ...plan.segments.map((s) => s.b),
];
export function serializeWaypoints(points: KeyWaypoint[]) {
  return points.map((w, i) => ({
    sequence: i + 1,
    ...toGeoreferenced(w.point),
    yaw: +mapYaw(w.yaw).toFixed(5),
    distance_m: +(w.distance * scale).toFixed(3),
    kind: w.reasons.includes("inspect")
      ? "inspect"
      : w.reasons.includes("return")
        ? "return"
        : w.reasons.includes("goal")
          ? "goal"
          : "pass",
    reasons: w.reasons,
    name: w.name,
    dwell_seconds: w.dwell,
  }));
}
export function waypointCSV(rows: ReturnType<typeof serializeWaypoints>) {
  const quote = (v: unknown) => `"${String(v).replaceAll('"', '""')}"`;
  return (
    "\uFEFF" +
    [
      "sequence,frame_id,quality_status,x,y,latitude,longitude,yaw,kind,reasons,name,dwell_seconds,distance_m",
      ...rows.map((w) =>
        [
          w.sequence,
          exportFrame.frame_id,
          exportFrame.quality_status,
          w.x,
          w.y,
          w.latitude,
          w.longitude,
          w.yaw,
          w.kind,
          w.reasons.join("|"),
          w.name,
          w.dwell_seconds,
          w.distance_m,
        ]
          .map(quote)
          .join(","),
      ),
    ].join("\r\n")
  );
}
