import { expect, test } from "@playwright/test";
import { bundle, mapToSvg, toMap, scale } from "../src/map";
import { snap } from "../src/planner";
import { choose } from "./mapHelpers";

test("weighted mode accepts an offroad click without snapping and submits the same endpoint", async ({
  page,
}) => {
  const start = bundle.network.roads[0].points[0];
  const goal = { x: start.x + 8, y: start.y - 8.123 };
  const versions = {
    schema_version: 1,
    scene_version: bundle.scene_version,
    route_version: bundle.route_version,
  };
  const bounds = [
    bundle.grid.origin[0],
    bundle.grid.origin[1],
    bundle.grid.origin[0] + bundle.grid.width * bundle.grid.resolution,
    bundle.grid.origin[1] + bundle.grid.height * bundle.grid.resolution,
  ];
  let planned: any = null,
    submitted: any = null;
  await page.route("**/api/**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    if (path === "/api/status")
      return route.fulfill({
        json: {
          ...versions,
          mode: "lynx",
          ready: true,
          health: {},
          task: null,
          localization: "ok",
          pose: { ...start, yaw: 0, frame_id: "map", age_s: 0, stamp_s: 1 },
          planning_policy: {
            mode: "weighted",
            road_cost: 0,
            offroad_cost: 50,
            bounds,
            keepouts: [],
          },
        },
      });
    if (path === "/api/places")
      return route.fulfill({
        json: { places: [], route_version: bundle.route_version },
      });
    if (path === "/api/plans") {
      planned = route.request().postDataJSON();
      return route.fulfill({
        json: {
          ...versions,
          start,
          goal: planned.goal,
          reference_path: [start, planned.goal],
          segment_roads: ["offroad"],
          road_ids: [],
          length_m: 8.1,
        },
      });
    }
    if (path === "/api/tasks") {
      submitted = route.request().postDataJSON();
      return route.fulfill({
        status: 202,
        json: {
          ...versions,
          task_id: "a".repeat(32),
          state: "accepted",
          goal: submitted.goal,
        },
      });
    }
    return route.fulfill({ json: { obstacles: [] } });
  });
  await page.goto("/");
  await expect(page.getByTestId("road-preference-overlay")).toBeAttached();
  await page.getByTestId("select-b").click();
  await choose(page, mapToSvg(goal));
  const slider = page.getByRole("slider", { name: "路外代价" });
  await slider.fill("80");
  await expect(slider).toHaveValue("80");
  await page.getByTestId("plan-ab").click();
  await expect(page.getByTestId("ab-message")).toContainText("后端参考路线");
  const marker = page.getByTestId("ab-b");
  const selected = {
    x: Number(await marker.getAttribute("data-x")),
    y: Number(await marker.getAttribute("data-y")),
  };
  // Browser mouse coordinates are integer pixels, not centimetre coordinates.
  // The exact user-selected SVG point, however, must be preserved end-to-end.
  expect(planned.road_offroad_cost).toBe(80);
  expect(planned.goal.x).toBeCloseTo(toMap(selected).x, 6);
  expect(planned.goal.y).toBeCloseTo(toMap(selected).y, 6);
  expect(snap(selected)!.distance * scale).toBeGreaterThan(2);
  expect(
    Math.hypot(planned.goal.x - goal.x, planned.goal.y - goal.y),
  ).toBeLessThan(0.75);
  await page.getByRole("button", { name: "执行导航", exact: true }).click();
  await expect.poll(() => submitted).not.toBeNull();
  expect(submitted.goal).toEqual(planned.goal);
  expect(submitted.road_offroad_cost).toBe(80);
  await slider.fill("10");
  await expect(page.getByTestId("ab-message")).toContainText("权重已改变");
  await expect(
    page.getByRole("button", { name: "执行导航", exact: true }),
  ).toBeDisabled();
  await page.getByTestId("plan-ab").click();
  await expect.poll(() => planned.road_offroad_cost).toBe(10);
  await page.getByRole("button", { name: "隐藏道路权重", exact: true }).click();
  await expect(page.getByTestId("road-preference-overlay")).toHaveCount(0);
});
