import { describe, expect, it } from "vitest";
import { exportAB, planAB, withCarStart } from "./abNavigation";
import { calibration, dist, home, presets, roads, scale, toGeoreferenced, toMeters } from "./map";
import { snap } from "./planner";
import { extractWaypoints, serializeWaypoints, waypointCSV } from "./waypoints";
import { advance, initial } from "./simulation";
import type { Point } from "./types";
const branch = roads.find((r) => r.id === "branch")!;
const midpoint = (a: Point, b: Point, t: number) => ({ x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) });

describe("measured centerline A/B planning", () => {
  it("plans through the confirmed junction without cutting across the ring", () => {
    const result = planAB(presets[2].point, home);
    expect(result.error).toBeNull();
    expect(result.points[0]).toEqual(presets[2].point);
    expect(result.points.at(-1)).toEqual(home);
    expect(result.plan!.segments.some((s) => s.edgeId === "branch")).toBe(true);
    expect(result.waypoints.some((w) => w.reasons.includes("junction"))).toBe(true);
    for (const p of result.points) expect(snap(p)!.distance).toBeLessThan(1e-6);
  });
  it("supports two endpoints in the same segment with exact distances", () => {
    const a = midpoint(branch.points[0], branch.points[1], 0.2);
    const b = midpoint(branch.points[0], branch.points[1], 0.8);
    const result = planAB(a, b);
    expect(result.error).toBeNull();
    expect(result.plan!.length).toBeCloseTo(dist(a, b));
    expect(result.points.at(-1)).toEqual(b);
    expect(planAB(b, a).plan!.length).toBeCloseTo(result.plan!.length);
  });
  it("uses the short ring arc and reroutes around its closure", () => {
    const direct = planAB(home, presets[1].point);
    const detour = planAB(home, presets[1].point, ["loop-a"]);
    expect(direct.plan!.length * scale).toBeCloseTo(roads[0].length_m, 3);
    expect(detour.plan!.length).toBeGreaterThan(direct.plan!.length);
    expect(detour.plan!.segments.every((s) => s.edgeId === "loop-b")).toBe(true);
  });
  it("rejects closed branch interiors, off-route points and disconnected components", () => {
    expect(planAB(home, presets[2].point, ["branch"]).error).toBeTruthy();
    expect(planAB(home, presets[1].point, ["loop-a", "loop-b"]).error).toBeTruthy();
    expect(planAB({ x: -1000, y: -1000 }, home).error).toBeTruthy();
  });
  it("completes an equal A/B task locally with a single waypoint", () => {
    const result = planAB(home, home);
    expect(result.plan!.length).toBe(0);
    expect(result.waypoints).toHaveLength(1);
    expect(result.waypoints[0].reasons).toEqual(expect.arrayContaining(["start", "goal"]));
    const state = { ...initial([{ id: "ab-b", name: "B", point: home, dwell: 0 }], home, false), plan: result.plan, status: "running" as const };
    expect(advance(state, 1).status).toBe("completed");
  });
});
describe("measured route exports", () => {
  it("exports the actual metric geometry and quality without width assumptions", () => {
    const result = planAB(home, presets[2].point);
    const data = exportAB(home, presets[2].point, result);
    expect(data.start).toEqual(toGeoreferenced(home));
    expect(data.path).toEqual(result.points.map(toGeoreferenced));
    expect(data.scale_is_mock).toBe(false);
    expect(data.calibration_status).toBe("survey_dgps_unverified");
    expect(data.calibration_version).toBe(calibration.version);
    expect(data.executable_on_robot).toBe(false);
    expect(data.planner).toBe("centerline_graph_astar");
    expect(data).not.toHaveProperty("road_width_m");
    expect(waypointCSV(serializeWaypoints(result.waypoints))).toContain("map");
    expect(() => exportAB(home, presets[2].point, planAB(home, presets[2].point, ["branch"]))).toThrow();
  });
  it("prepends the real car pose only when the car is off the route", () => {
    const result = planAB(home, presets[2].point);
    const route = exportAB(home, presets[2].point, result);
    const start = toMeters(home);
    // 车就在起点的投影上（<0.1 m）⇒ 原样返回，一个字段都不多
    expect(withCarStart(route, { ...start, yaw: 0 })).toBe(route);
    // 车离线 8.485 m ⇒ path/waypoints 前面各接一条，且 path[0]==waypoints[0]
    const car = { x: start.x + 6, y: start.y + 6, yaw: 0.3 };
    const lead = Math.hypot(6, 6);
    const data = withCarStart(route, car);
    expect(data.path).toHaveLength(route.path.length + 1);
    expect(data.waypoints).toHaveLength(route.waypoints.length + 1);
    expect(Math.abs(data.path[0].x - car.x)).toBeLessThan(1e-3);
    expect(Math.abs(data.path[0].y - car.y)).toBeLessThan(1e-3);
    expect(dist(data.path[0] as never, data.waypoints[0] as never)).toBeLessThan(0.01);
    expect(data.path[1]).toEqual(route.path[0]);
    expect(data.waypoints[0].yaw).toBeCloseTo(0.3);
    expect(data.waypoints[1].sequence).toBe(2);
    expect(data.length).toBeCloseTo(route.length + lead, 3);
    expect(data.start).toEqual(route.start);
  });
  it("retains needed bends and keeps spacing below twenty meters", () => {
    const result = planAB(home, presets[2].point);
    result.waypoints.slice(1).forEach((w, i) => expect((w.distance - result.waypoints[i].distance) * scale).toBeLessThanOrEqual(20.001));
    const shallow = extractWaypoints([{ x: 0, y: 0 }, { x: 50, y: 2 }, { x: 100, y: 0 }]);
    expect(shallow.some((w) => w.reasons.includes("geometry"))).toBe(true);
  });
});
