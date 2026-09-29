import { calibration, campus, dist, scale, toGeoreferenced } from "./map";
import { route } from "./planner";
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
  const path = route(a, b, blocked);
  if (!path)
    return { points: [], error: "A/B 不在开放中心线上，或道路已断开。", waypoints: [], plan: null, blocked: [...blocked] };
  const points = [a, ...path.map((segment) => segment.b)];
  let distance = 0;
  const segments = path.map((segment) => {
    const start = distance;
    distance += dist(segment.a, segment.b);
    return {
      a: segment.a,
      b: segment.b,
      start,
      end: distance,
      edgeId: segment.edgeId,
    };
  });
  const waypoints = extractWaypoints(points);
  return {
    points,
    error: null,
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
    frame_id: calibration.frame_id,
    calibration_version: calibration.version,
    calibration_status: calibration.status,
    scale_is_mock: false,
    georeferenced: true,
    geographic_coordinates_are_estimated: true,
    field_verified: false,
    executable_on_robot: false,
    units: "meters",
    axes: "X east, Y north",
    origin_wgs84: calibration.origin_wgs84,
    mode: "point_to_point",
    planner: "centerline_graph_astar",
    start: toGeoreferenced(a),
    goal: toGeoreferenced(b),
    blocked_roads: result.blocked,
    length: +(result.plan.length * scale).toFixed(3),
    waypoint_extraction: waypointOptions,
    path: result.points.map(toGeoreferenced),
    waypoints: serializeWaypoints(result.waypoints),
  };
}
