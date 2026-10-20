import { expect, test } from "@playwright/test";
import { bundle } from "../src/map";
import placeData from "../../scenes/north_campus/places.json" with { type: "json" };

test("Lynx position remains visible before readiness; confirmed place uses shared task and cancel APIs", async ({
  page,
}) => {
  let ready = false;
  let task: any = null;
  let submitted: any = null;
  const places = placeData.places.map((p) => ({ ...p, enabled: p.id === "J" }));
  const pose = {
    ...places[0],
    frame_id: "map",
    yaw: 0,
    age_s: 0.1,
    stamp_s: 10,
  };
  await page.route("**/api/**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    let response: any;
    if (path === "/api/status")
      response = {
        schema_version: 1,
        mode: "lynx",
        scene_version: bundle.scene_version,
        route_version: bundle.route_version,
        ready,
        pose,
        task,
        localization: ready ? "ok" : "waiting_for_rtk",
        health: { localization: ready },
        gnss: {
          fixed: ready,
          hacc_m: ready ? 0.1 : 20,
          satellites: 20,
          correction_age_s: 1,
        },
      };
    else if (path === "/api/places") response = { ...placeData, places };
    else if (path === "/api/plans")
      response = {
        reference_path: [pose, places[1]],
        start: pose,
        goal: places[1],
        segment_roads: ["loop-a"],
        road_ids: ["loop-a"],
        length_m: 70,
      };
    else if (path === "/api/tasks") {
      submitted = route.request().postDataJSON();
      task = {
        task_id: "a".repeat(32),
        state: "navigating",
        goal: places[1],
        blocked_road_ids: [],
        path_revision: 1,
        execution_path: [pose, places[1]],
        reference_path: [pose, places[1]],
        distance_remaining_m: 70,
        reason: null,
      };
      response = task;
    } else if (path.endsWith("/cancel"))
      response = task = { ...task, state: "canceled" };
    else throw new Error(`Unexpected hardware endpoint: ${path}`);
    await route.fulfill({ json: response });
  });
  await page.goto("/");
  await expect(page.getByTestId("navigation-robot")).toBeVisible();
  await expect(
    page.getByRole("button", { name: "执行导航", exact: true }),
  ).toBeDisabled();
  await page.getByLabel("地图目的地").selectOption("J");
  await page.getByTestId("plan-ab").click();
  ready = true;
  await expect(
    page.getByRole("button", { name: "执行导航", exact: true }),
  ).toBeEnabled();
  await page.getByRole("button", { name: "执行导航", exact: true }).click();
  await expect(page.getByTestId("execution-route")).toBeAttached();
  expect(submitted.goal.x).toBeCloseTo(places[1].x, 6);
  expect(submitted.goal.y).toBeCloseTo(places[1].y, 6);
  await page.getByTitle("取消导航并停车", { exact: true }).click();
  await expect(page.getByTestId("navigation-status")).toHaveText(
    "已取消并停车",
  );
});

test("hardware free goal does not snap to roads or require a surveyed place", async ({
  page,
}) => {
  const { choose } = await import("./mapHelpers");
  const { campus, toMap } = await import("../src/map");
  const { snap } = await import("../src/planner");
  const target = { x: campus.width / 2, y: campus.height / 2 };
  expect(snap(target)!.distance).toBeGreaterThan(12);
  let submitted: any = null;
  await page.route("**/api/**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    if (path === "/api/status")
      return route.fulfill({
        json: {
          schema_version: 1,
          mode: "lynx",
          scene_version: bundle.scene_version,
          route_version: bundle.route_version,
          ready: true,
          health: {},
          localization: "ok",
          task: null,
          pose: {
            ...placeData.places[0],
            frame_id: "map",
            yaw: 0,
            age_s: 0,
            stamp_s: 1,
          },
        },
      });
    if (path === "/api/places") return route.fulfill({ json: placeData });
    if (path === "/api/tasks") {
      submitted = route.request().postDataJSON();
      return route.fulfill({
        json: {
          task_id: "a".repeat(32),
          state: "accepted",
          goal: submitted.goal,
          blocked_road_ids: [],
          path_revision: 0,
          execution_path: [],
          reference_path: [],
          distance_remaining_m: null,
          reason: null,
        },
      });
    }
    throw new Error(`Unexpected hardware endpoint ${path}`);
  });
  await page.goto("/");
  await expect(page.getByTestId("navigation-robot")).toBeVisible();
  await choose(page, target);
  await expect(page.getByTestId("ab-message")).toContainText(
    "不要求在预设道路上",
  );
  await expect(
    page.getByRole("button", { name: "封闭或恢复道路", exact: true }),
  ).toHaveCount(0);
  await page.getByRole("button", { name: "执行导航", exact: true }).click();
  await expect.poll(() => submitted).not.toBeNull();
  // Mouse coordinates quantize at screen pixels; the point stays far off-road.
  expect(submitted.goal.x).toBeCloseTo(toMap(target).x, 0);
  expect(submitted.goal.y).toBeCloseTo(toMap(target).y, 0);
  expect(submitted.blocked_road_ids).toEqual([]);
  expect(submitted.request_id).toMatch(/^[a-f0-9]{32}$/);
});
