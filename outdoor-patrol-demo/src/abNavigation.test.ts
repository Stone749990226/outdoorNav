import { describe, expect, it } from "vitest";
import {
  clearance,
  freeGrid,
  isTraversable,
  segmentSafe,
  areaRoute,
} from "./freeSpace";
import { exportAB, planAB } from "./abNavigation";
import { dist, home, presets, scale } from "./map";
import { snap } from "./planner";
import {
  extractWaypoints,
  serializeWaypoints,
  waypointCSV,
  waypointOptions,
} from "./waypoints";
import { advance, initial } from "./simulation";
import type { Point } from "./types";
const A = { x: 869, y: 495 },
  B = { x: 478, y: 844 };
describe("free-area A/B navigation", () => {
  it("preserves exact off-center endpoints and produces safe path segments", () => {
    expect(snap(A)!.distance).toBeGreaterThan(2);
    expect(isTraversable(A)).toBe(true);
    expect(isTraversable(B)).toBe(true);
    const result = planAB(A, B);
    expect(result.error).toBeNull();
    expect(result.points[0]).toEqual(A);
    expect(result.points.at(-1)).toEqual(B);
    for (let i = 1; i < result.points.length; i++)
      expect(segmentSafe(result.points[i - 1], result.points[i])).toBe(true);
    const keys = result.waypoints;
    expect(keys[0].point).toEqual(A);
    expect(keys.at(-1)!.point).toEqual(B);
    for (let i = 1; i < keys.length; i++) {
      expect(segmentSafe(keys[i - 1].point, keys[i].point)).toBe(true);
      expect(
        (keys[i].distance - keys[i - 1].distance) * scale,
      ).toBeLessThanOrEqual(20.001);
    }
  });
  it("supports points across the road width, not just centerline graph nodes", () => {
    const a = { x: 861, y: 490 },
      b = { x: 870, y: 590 };
    const result = areaRoute(a, b);
    expect(result.error).toBeNull();
    expect(result.points).toEqual([a, b]);
  });
  it("rejects obstacles, unknown ground, exterior and insufficient footprint clearance", () => {
    for (const p of [
      { x: 600, y: 350 },
      { x: 300, y: 650 },
      { x: 20, y: 800 },
      { x: 874, y: 500 },
      { x: NaN, y: 500 },
    ]) {
      expect(isTraversable(p)).toBe(false);
      expect(areaRoute(p, A).error).toBeTruthy();
    }
    expect(areaRoute(A, { x: 600, y: 350 }).error).toMatch(/B 点/);
  });
  it("reroutes around an entire closed road width and reports a disconnected route", () => {
    const a = presets[1].point,
      b = presets[2].point;
    const normal = planAB(a, b),
      closed = planAB(a, b, ["c-e"]);
    expect(normal.error).toBeNull();
    expect(closed.error).toBeNull();
    expect(closed.plan!.length).toBeGreaterThan(normal.plan!.length);
    closed.points
      .slice(1)
      .forEach((p, i) =>
        expect(segmentSafe(closed.points[i], p, ["c-e"])).toBe(true),
      );
    expect(planAB(home, b, ["n-c", "ne-n"]).error).toBeTruthy();
    expect(planAB(A, B, ["e-ne"]).error).toMatch(/A 点/);
  });
  it("accepts equal A/B and completes zero length simulation", () => {
    const result = planAB(A, A);
    expect(result.error).toBeNull();
    expect(result.plan!.length).toBe(0);
    expect(result.waypoints).toHaveLength(1);
    expect(result.waypoints[0].reasons).toEqual(
      expect.arrayContaining(["start", "goal"]),
    );
    let state = {
      ...initial([{ id: "ab-b", name: "B", point: A, dwell: 0 }], A, false),
      plan: result.plan,
      status: "running" as const,
    };
    expect(advance(state, 1).status).toBe("completed");
  });
  it("simulates exactly to B and exposes conservative selectable cells", () => {
    const result = planAB(A, B);
    let state = {
      ...initial([{ id: "ab-b", name: "B", point: B, dwell: 0 }], A, false),
      plan: result.plan,
      status: "running" as import("./types").Status,
    };
    for (let i = 0; i < 1000 && state.status === "running"; i++)
      state = advance(state, 1);
    expect(state.status).toBe("completed");
    expect(dist(state.position, B)).toBeLessThan(1e-6);
    expect(freeGrid().count).toBeGreaterThan(100);
    expect(freeGrid().path).toContain("M");
    expect(freeGrid(["c-e"]).count).toBeLessThan(freeGrid().count);
  });
  it("refuses unsafe straight shortcuts even when both endpoints are valid", () => {
    expect(isTraversable(home)).toBe(true);
    expect(isTraversable(presets[2].point)).toBe(true);
    expect(segmentSafe(home, presets[2].point)).toBe(false);
  });
});
describe("important waypoint extraction and output", () => {
  it("reduces dense straight-line points while preserving spacing", () => {
    const line = Array.from({ length: 121 }, (_, i) => ({
      x: 865,
      y: 460 + i,
    }));
    const keys = extractWaypoints(line, { segmentSafe });
    expect(keys.length).toBeLessThan(6);
    expect(keys[0].point).toEqual(line[0]);
    expect(keys.at(-1)!.point).toEqual(line.at(-1));
    expect(keys.some((k) => k.reasons.includes("spacing"))).toBe(true);
    for (let i = 1; i < keys.length; i++)
      expect(keys[i].distance - keys[i - 1].distance).toBeLessThanOrEqual(
        waypointOptions.max_spacing_m / scale + 0.001,
      );
  });
  it("keeps sharp turns and junction passages", () => {
    const result = planAB(home, presets[2].point);
    expect(result.waypoints.some((k) => k.reasons.includes("turn"))).toBe(true);
    expect(result.waypoints.some((k) => k.reasons.includes("junction"))).toBe(
      true,
    );
  });
  it("does not remove shallow necessary bends just because their angle is small", () => {
    const points: Point[] = [
      { x: 10, y: 10 },
      { x: 100, y: 12 },
      { x: 200, y: 10 },
    ];
    const keys = extractWaypoints(points);
    expect(keys.some((k) => k.reasons.includes("geometry"))).toBe(true);
  });
  it("exports the exact computed full path and annotated keypoints, never RTK claims", () => {
    const result = planAB(A, B),
      data = exportAB(A, B, result);
    expect(data.start.x).toBe(A.x * scale);
    expect(data.start.y).toBe((920 - A.y) * scale);
    expect(data.waypoints).toEqual(serializeWaypoints(result.waypoints));
    expect(data.georeferenced).toBe(false);
    expect(data.executable_on_robot).toBe(false);
    expect(data.scale_is_mock).toBe(true);
    expect(
      data.waypoints.every(
        (w) => Number.isFinite(w.yaw) && w.reasons.length > 0,
      ),
    ).toBe(true);
    const csv = waypointCSV(data.waypoints);
    expect(csv).toContain("reasons");
    expect(csv).toContain("demo_map");
    expect(() => exportAB(A, B, planAB(A, B, ["e-ne"]))).toThrow();
  });
});
