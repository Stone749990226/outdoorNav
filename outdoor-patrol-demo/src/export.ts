import { campus, regions, roads, toMeters, scale } from "./map";
import {
  extractWaypoints,
  planPoints,
  serializeWaypoints,
  waypointCSV,
  waypointOptions,
} from "./waypoints";
import type { Simulation } from "./types";
const frame = {
  version: "1.0.0",
  frame_id: "demo_map",
  scale_is_mock: true,
  units: "demo_meters",
  axes: "X right, Y up",
  origin: "bottom_left",
  georeferenced: false,
};
export function exportMap(blocked: string[]) {
  return {
    ...frame,
    map_version: campus.version,
    name: campus.name,
    source: campus.source,
    traversable_area_model: {
      type: "road_corridors_minus_regions_and_closures",
      road_width_m: campus.navigation.road_width_units * scale,
      robot_radius_m: campus.navigation.robot_radius_units * scale,
      grid_resolution_m: campus.navigation.grid_resolution_units * scale,
      parameters_are_mock: true,
    },
    width: campus.width * scale,
    height: campus.height * scale,
    regions: regions.map((r) => ({
      ...r,
      points: r.points.map(toMeters),
      traversability: r.kind === "building" ? "forbidden" : "unknown",
    })),
    nodes: campus.nodes.map((n) => ({ ...n, ...toMeters(n) })),
    roads: roads.map((r) => ({
      ...r,
      points: r.points.map(toMeters),
      bidirectional: true,
      traversability: blocked.includes(r.id) ? "closed" : "presumed_passable",
      field_verified: false,
    })),
    tasks: campus.tasks.map((t) => ({ ...t, point: toMeters(t.point) })),
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
    map_version: campus.version,
    task_order: s.tasks.map((t) => t.id),
    remaining_task_order: plan.taskOrder,
    tasks: s.tasks.map((t) => ({ ...t, point: toMeters(t.point) })),
    return_home: s.returnHome,
    home: toMeters(s.home),
    traveled_path: [...s.trace, s.position].map(toMeters),
    task_names: plan.stops.map((t) => t.name),
    completed_task_ids: s.completed,
    blocked_roads: s.blocked,
    length: +(plan.length * scale).toFixed(3),
    traveled: +(s.traveled * scale).toFixed(3),
    path: [plan.start, ...plan.segments.map((seg) => seg.b)].map(toMeters),
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
