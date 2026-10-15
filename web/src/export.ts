import {
  campus,
  exportFrame,
  regions,
  roads,
  scale,
  toGeoreferenced,
  toMap,
} from "./map";
import type { Simulation } from "./types";
import {
  extractWaypoints,
  planPoints,
  serializeWaypoints,
  waypointCSV,
  waypointOptions,
} from "./waypoints";
const frame = exportFrame;
const corners = [
  { x: 0, y: 0 },
  { x: campus.width, y: 0 },
  { x: 0, y: campus.height },
  { x: campus.width, y: campus.height },
].map(toMap);
const bounds = {
  min_x: Math.min(...corners.map((p) => p.x)),
  min_y: Math.min(...corners.map((p) => p.y)),
  max_x: Math.max(...corners.map((p) => p.x)),
  max_y: Math.max(...corners.map((p) => p.y)),
};
export function exportMap(blocked: string[]) {
  return {
    ...frame,
    name: campus.name,
    source: campus.source,
    route_model: { type: "centerline_graph", road_width_measured: false },
    bounds_m: bounds,
    width: bounds.max_x - bounds.min_x,
    height: bounds.max_y - bounds.min_y,
    regions: regions.map((r) => ({
      ...r,
      points: r.points.map(toGeoreferenced),
      traversability: r.kind === "building" ? "forbidden" : "unknown",
    })),
    nodes: campus.nodes.map((n) => ({ ...n, ...toGeoreferenced(n) })),
    roads: roads.map((r) => ({
      ...r,
      points: r.points.map(toGeoreferenced),
      bidirectional: true,
      traversability: blocked.includes(r.id) ? "closed" : "recorded_centerline",
      field_verified: false,
    })),
    tasks: campus.tasks.map((t) => ({ ...t, point: toGeoreferenced(t.point) })),
  };
}
export function exportRoute(s: Simulation) {
  if (!s.plan || s.status === "idle" || s.status === "blocked")
    throw new Error("请先生成有效路线");
  const plan = s.plan;
  const waypoints = serializeWaypoints(
    extractWaypoints(planPoints(plan), { stops: plan.stops }),
  );
  return {
    ...frame,
    task_order: s.tasks.map((t) => t.id),
    remaining_task_order: plan.taskOrder,
    tasks: s.tasks.map((t) => ({ ...t, point: toGeoreferenced(t.point) })),
    return_home: s.returnHome,
    home: toGeoreferenced(s.home),
    traveled_path: [...s.trace, s.position].map(toGeoreferenced),
    task_names: plan.stops.map((t) => t.name),
    completed_task_ids: s.completed,
    blocked_roads: s.blocked,
    length: +(plan.length * scale).toFixed(3),
    traveled: +(s.traveled * scale).toFixed(3),
    path: [plan.start, ...plan.segments.map((seg) => seg.b)].map(
      toGeoreferenced,
    ),
    waypoint_extraction: waypointOptions,
    waypoints,
  };
}
export function exportCSV(s: Simulation): string {
  return waypointCSV(exportRoute(s).waypoints);
}

export function download(
  name: string,
  content: string,
  type = "application/json",
) {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
