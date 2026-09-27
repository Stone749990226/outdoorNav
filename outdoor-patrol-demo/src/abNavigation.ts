import { campus, dist, scale, toMeters } from "./map";
import { areaRoute, navigation, segmentSafe } from "./freeSpace";
import {
  extractWaypoints,
  serializeWaypoints,
  waypointOptions,
} from "./waypoints";
import type { KeyWaypoint } from "./waypoints";
import type { Plan, Point } from "./types";
export type ABResult = {
  points: Point[];
  waypoints: KeyWaypoint[];
  plan: Plan | null;
  error: string | null;
  blocked: string[];
};
export function planAB(a: Point, b: Point, blocked: string[] = []): ABResult {
  const route = areaRoute(a, b, blocked);
  if (route.error)
    return { ...route, waypoints: [], plan: null, blocked: [...blocked] };
  let distance = 0;
  const segments = route.points.slice(1).map((point, i) => {
    const start = distance;
    distance += dist(route.points[i], point);
    return {
      a: route.points[i],
      b: point,
      start,
      end: distance,
      edgeId: "free-space",
    };
  });
  const waypoints = extractWaypoints(route.points, {
    segmentSafe: (p, q) => segmentSafe(p, q, blocked),
  });
  return {
    ...route,
    blocked: [...blocked],
    waypoints,
    plan: {
      start: a,
      segments,
      length: distance,
      taskOrder: ["ab-b"],
      stops: [{ id: "ab-b", name: "B 点", point: b, dwell: 0, distance }],
    },
  };
}
export function exportAB(a: Point, b: Point, result: ABResult) {
  if (!result.plan || result.error) throw new Error("请先生成有效的 A→B 路线");
  return {
    version: "1.1.0",
    map_version: campus.version,
    frame_id: "demo_map",
    scale_is_mock: true,
    georeferenced: false,
    field_verified: false,
    executable_on_robot: false,
    units: "demo_meters",
    axes: "X right, Y up",
    origin: "bottom_left",
    mode: "point_to_point",
    planner: "grid_astar_with_clearance_checked_shortcuts",
    start: toMeters(a),
    goal: toMeters(b),
    blocked_roads: result.blocked,
    length: +(result.plan.length * scale).toFixed(3),
    resolution_m: navigation.grid_resolution_units * scale,
    robot_radius_m: navigation.robot_radius_units * scale,
    road_width_m: navigation.road_width_units * scale,
    parameters_are_mock: true,
    waypoint_extraction: waypointOptions,
    path: result.points.map(toMeters),
    waypoints: serializeWaypoints(result.waypoints),
  };
}
