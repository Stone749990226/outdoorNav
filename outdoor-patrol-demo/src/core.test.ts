import { describe, expect, it } from "vitest";
import {
  campus,
  home,
  presets,
  roads,
  regions,
  dist,
  scale,
  toMeters,
} from "./map";
import {
  enteredRoad,
  makePlan,
  positionAt,
  reachableRoads,
  route,
  snap,
} from "./planner";
import {
  advance,
  initial,
  planSimulation,
  startDemo,
  toggleRoad,
} from "./simulation";
import { exportCSV, exportMap, exportRoute } from "./export";
import type { Point, Simulation } from "./types";
const finish = (state: Simulation) => {
  let s = state;
  for (let i = 0; i < 3000 && s.status === "running"; i++) s = advance(s, 1);
  return s;
};
function inside(p: Point, polygon: Point[]) {
  let c = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const a = polygon[i],
      b = polygon[j];
    if (
      a.y > p.y !== b.y > p.y &&
      p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x
    )
      c = !c;
  }
  return c;
}
describe("map and road topology", () => {
  it("has consistent endpoints and no water classification", () => {
    for (const road of roads) {
      expect(road.points[0]).toMatchObject({
        x: campus.nodes.find((n) => n.id === road.from)!.x,
        y: campus.nodes.find((n) => n.id === road.from)!.y,
      });
      expect(road.points.at(-1)).toMatchObject({
        x: campus.nodes.find((n) => n.id === road.to)!.x,
        y: campus.nodes.find((n) => n.id === road.to)!.y,
      });
    }
    expect(
      regions.every((r) => ["building", "green", "unknown"].includes(r.kind)),
    ).toBe(true);
  });
  it("keeps every planned road centerline outside building footprints", () => {
    for (const road of roads)
      for (let i = 1; i < road.points.length; i++)
        for (let step = 0; step <= 50; step++) {
          const a = road.points[i - 1],
            b = road.points[i],
            p = {
              x: a.x + ((b.x - a.x) * step) / 50,
              y: a.y + ((b.y - a.y) * step) / 50,
            };
          for (const region of regions.filter((r) => r.kind === "building"))
            expect(
              inside(p, region.points),
              `${road.id} crosses ${region.id}`,
            ).toBe(false);
        }
  });
});
describe("A* and endpoint projection", () => {
  it("produces connected, ordered paths without skipping targets", () => {
    const { plan, error } = makePlan(home, presets, home, true, []);
    expect(error).toBeNull();
    expect(plan!.stops.map((t) => t.id)).toEqual([
      ...presets.map((t) => t.id),
      "__home",
    ]);
    let last = home;
    for (const segment of plan!.segments) {
      expect(dist(last, segment.a)).toBeLessThan(0.001);
      last = segment.b;
    }
    expect(last).toEqual(home);
  });
  it("supports two temporary endpoints on the same road", () => {
    const a = snap({ x: 864, y: 500 })!,
      b = snap({ x: 864, y: 550 })!;
    const result = route(a.point, b.point, [])!;
    expect(result.reduce((sum, s) => sum + dist(s.a, s.b), 0)).toBeCloseTo(50);
    expect(result[0].a).toEqual(a.point);
    expect(result.at(-1)!.b).toEqual(b.point);
  });
  it("finds a longer detour around the closed front road", () => {
    const a = presets[1].point,
      b = presets[2].point;
    const length = (blocked: string[]) =>
      route(a, b, blocked)!.reduce((sum, seg) => sum + dist(seg.a, seg.b), 0);
    expect(length(["c-e"])).toBeGreaterThan(length([]));
    expect(route(a, b, ["c-e"])!.every((s) => s.edgeId !== "c-e")).toBe(true);
  });
  it("rejects disconnected and closed-road interior goals without resnapping", () => {
    expect(route(home, presets[1].point, ["n-c", "ne-n"])).toBeNull();
    const p = snap({ x: 865, y: 500 })!.point;
    expect(route(home, p, ["e-ne"])).toBeNull();
    expect(reachableRoads(home, ["n-c", "ne-n"]).size).toBe(0);
  });
  it("splits the road at the current position for replanning", () => {
    const { plan } = makePlan(home, presets, home, true, []);
    const position = positionAt(plan!, 80).point;
    const result = makePlan(position, presets.slice(1), home, true, ["c-e"]);
    expect(result.error).toBeNull();
    expect(result.plan!.segments[0].a.x).toBeCloseTo(position.x);
    expect(result.plan!.segments[0].a.y).toBeCloseTo(position.y);
  });
});
describe("simulation lifecycle", () => {
  it("completes the entire automatic closure demonstration", () => {
    const result = finish(startDemo());
    expect(result.status).toBe("completed");
    expect(result.completed).toEqual(presets.map((p) => p.id));
    expect(result.position).toEqual(home);
    expect(result.replans).toBe(1);
    expect(result.blocked).toEqual(["c-e"]);
    expect(result.plan!.segments.every((s) => s.edgeId !== "c-e")).toBe(true);
    expect(result.elapsed / 5).toBeGreaterThan(60);
    expect(result.elapsed / 5).toBeLessThan(150);
  });
  it("holds still for the full inspection dwell and preserves pause state", () => {
    let s = planSimulation(initial(), true);
    s = advance(s, 1);
    expect(s.position).toEqual(home);
    expect(s.completed).toEqual([]);
    expect(s.dwell).toBe(1);
    s = { ...s, status: "paused" };
    expect(advance(s, 30)).toBe(s);
    s = advance({ ...s, status: "running" }, 2);
    expect(s.completed).toContain("p-n");
    expect(s.traveled * scale).toBeCloseTo(1);
  });
  it("replans at the exact position and does not revisit completed tasks", () => {
    let s = advance(planSimulation(initial(), true), 10);
    const before = s.position;
    s = toggleRoad(s, "c-e");
    expect(s.position).toEqual(before);
    expect(s.completed).toEqual(["p-n"]);
    expect(s.plan!.stops.some((t) => t.id === "p-n")).toBe(false);
    expect(s.status).toBe("running");
    expect(s.traveled * scale).toBeCloseTo(8);
  });
  it("prevents closure of the entered edge, including its intermediate bends", () => {
    let s = advance(planSimulation(initial(), true), 10);
    const edge = enteredRoad(s.position)!;
    const next = toggleRoad(s, edge);
    expect(next.blocked).toEqual([]);
    expect(next.plan).toBe(s.plan);
    expect(enteredRoad(roads[0].points[1])).toBe("n-c");
  });
  it("halts on disconnection and continues after restoration without a jump", () => {
    let s = planSimulation(initial());
    s = toggleRoad(s, "n-c");
    s = toggleRoad(s, "ne-n");
    expect(s.status).toBe("blocked");
    expect(s.plan).toBeNull();
    expect(advance(s, 50)).toBe(s);
    const position = s.position;
    s = toggleRoad(s, "n-c");
    expect(s.status).toBe("ready");
    expect(s.position).toEqual(position);
    s = finish({ ...s, status: "running" });
    expect(s.status).toBe("completed");
  });
  it("handles single-location tasks and tasks without a return leg", () => {
    let s = finish(planSimulation(initial([presets[0]], home, false), true));
    expect(s.status).toBe("completed");
    expect(s.elapsed).toBe(2);
    expect(s.traveled).toBe(0);
    s = finish(planSimulation(initial([presets[2]], home, false), true));
    expect(s.status).toBe("completed");
    expect(s.position).toEqual(presets[2].point);
  });
  it("reset restores a fresh task and map state", () => {
    const s = initial();
    expect(s.blocked).toEqual([]);
    expect(s.completed).toEqual([]);
    expect(s.traveled).toBe(0);
    expect(s.position).toEqual(home);
  });
});
describe("exports", () => {
  it("exports local meter coordinates and honest scale metadata", () => {
    const result = exportMap(["c-e"]);
    expect(result.frame_id).toBe("demo_map");
    expect(result.scale_is_mock).toBe(true);
    expect(result.georeferenced).toBe(false);
    expect(result.width).toBe(250);
    expect(result.height).toBe(230);
    expect(result.nodes[0]).toMatchObject(toMeters(home));
    expect(result.roads.find((r) => r.id === "c-e")!.traversability).toBe(
      "closed",
    );
  });
  it("matches the displayed polyline and exports inspection / pass instructions", () => {
    const s = planSimulation(initial());
    const data = exportRoute(s);
    expect(data.path).toEqual(
      [s.plan!.start, ...s.plan!.segments.map((seg) => seg.b)].map(toMeters),
    );
    expect(data.waypoints.filter((w) => w.kind === "inspect")).toHaveLength(5);
    expect(data.waypoints.some((w) => w.kind === "pass")).toBe(true);
    expect(
      data.waypoints
        .filter((w) => w.kind === "inspect")
        .every((w) => w.dwell_seconds === 2),
    ).toBe(true);
    const csv = exportCSV(s);
    expect(csv).toContain("demo_map");
    expect(csv).toContain("scale_is_mock");
  });
  it("disables stale and unreachable route exports", () => {
    expect(() => exportRoute(initial())).toThrow();
    const s = toggleRoad(toggleRoad(planSimulation(initial()), "n-c"), "ne-n");
    expect(() => exportCSV(s)).toThrow();
  });
});
