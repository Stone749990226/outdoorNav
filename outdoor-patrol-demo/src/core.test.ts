import { describe, expect, it } from "vitest";
import { campus, home, presets, roads, regions, dist, scale } from "./map";
import { enteredRoad, makePlan, reachableRoads, route } from "./planner";
import { advance, initial, planSimulation, startDemo, toggleRoad } from "./simulation";
import { exportCSV, exportMap, exportRoute } from "./export";
import type { Simulation } from "./types";
const finish = (state: Simulation) => {
  let s = state;
  for (let i = 0; i < 1000 && s.status === "running"; i++) s = advance(s, 1);
  return s;
};
describe("loop and branch topology", () => {
  it("has exactly three shared nodes and no assumed building geometry", () => {
    expect(campus.nodes).toHaveLength(3);
    expect(roads).toHaveLength(3);
    expect(regions).toEqual([]);
    for (const road of roads) {
      const from = campus.nodes.find((n) => n.id === road.from)!;
      const to = campus.nodes.find((n) => n.id === road.to)!;
      expect(dist(road.points[0], from)).toBe(0);
      expect(dist(road.points.at(-1)!, to)).toBe(0);
      const length = road.points.slice(1).reduce((sum, p, i) => sum + dist(p, road.points[i]), 0) * scale;
      expect(length).toBeCloseTo(road.length_m, 6);
    }
    expect(roads.filter((r) => r.from === "J" || r.to === "J")).toHaveLength(3);
  });
  it("connects ordered tasks and returns to the original position", () => {
    const { plan, error } = makePlan(home, presets, home, true, []);
    expect(error).toBeNull();
    let last = home;
    for (const s of plan!.segments) { expect(dist(last, s.a)).toBeLessThan(1e-6); last = s.b; }
    expect(last).toEqual(home);
    expect(plan!.stops.map((s) => s.id)).toEqual([...presets.map((p) => p.id), "__home"]);
  });
  it("keeps the junction accessible when an adjacent edge is closed", () => {
    expect(route(home, presets[1].point, ["loop-a"])).not.toBeNull();
    expect(route(home, presets[1].point, ["loop-a", "loop-b"])).toBeNull();
    expect(reachableRoads(home, ["loop-a", "loop-b"]).size).toBe(0);
  });
});
describe("centerline patrol lifecycle", () => {
  it("completes the measured patrol and return leg", () => {
    const s = finish(startDemo());
    expect(s.status).toBe("completed");
    expect(s.completed).toEqual(presets.map((p) => p.id));
    expect(s.position).toEqual(home);
    expect(s.blocked).toEqual([]);
  });
  it("honors dwell, pause and exact replanning positions", () => {
    let s = advance(planSimulation(initial(), true), 1);
    expect(s.position).toEqual(home);
    expect(s.dwell).toBe(1);
    s = { ...s, status: "paused" };
    expect(advance(s, 20)).toBe(s);
    s = advance({ ...s, status: "running" }, 10);
    const previous = s.position;
    const replanned = toggleRoad(s, "loop-b");
    expect(replanned.position).toEqual(previous);
    expect(replanned.completed).toContain("p-s");
    expect(replanned.status).toBe("running");
    const entered = enteredRoad(s.position)!;
    expect(toggleRoad(s, entered).blocked).toEqual([]);
  });
  it("halts on disconnection and recovers without changing position", () => {
    let s = toggleRoad(toggleRoad(planSimulation(initial()), "loop-a"), "loop-b");
    expect(s.status).toBe("blocked");
    expect(advance(s, 50)).toBe(s);
    const previous = s.position;
    s = toggleRoad(s, "loop-a");
    expect(s.position).toEqual(previous);
    expect(finish({ ...s, status: "running" }).status).toBe("completed");
  });
});
describe("metric exports", () => {
  it("uses measured scale and a centerline model without a free-space claim", () => {
    const map = exportMap(["branch"]);
    expect(map.scale_is_mock).toBe(false);
    expect(map.route_model.type).toBe("centerline_graph");
    expect(map).not.toHaveProperty("traversable_area_model");
    expect(map.roads.find((r) => r.id === "branch")!.traversability).toBe("closed");
    expect(map.calibration_status).toBe("survey_dgps_unverified");
    expect(map.executable_on_robot).toBe(false);
    const data = exportRoute(planSimulation(initial()));
    expect(data.waypoints.filter((p) => p.kind === "inspect")).toHaveLength(3);
    expect(exportCSV(planSimulation(initial()))).toContain("survey_dgps_unverified");
  });
});
