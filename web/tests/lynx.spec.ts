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
        diagnostic: !ready,
        not_ready_reasons: ready
          ? []
          : [
              { code: "localization", message: "无合格且稳定的 RTK 定位" },
              { code: "lidar", message: "雷达无有效新数据" },
              { code: "motion_enabled", message: "运动开关关闭" },
            ],
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
  await expect(page.getByTestId("diagnostic-mode")).toContainText(
    "底盘输出已隔离",
  );
  await expect(page.getByTestId("readiness-reasons")).toContainText("RTK");
  await expect(page.getByTestId("readiness-reasons")).toContainText(
    "雷达无有效新数据",
  );
  await expect(page.getByTestId("readiness-reasons")).toContainText(
    "运动开关关闭",
  );
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

test("hardware snaps nearby goals, rejects off-road clicks and previews a road route", async ({
  page,
}) => {
  const { choose, interpolate } = await import("./mapHelpers");
  const { campus, toMap, scale } = await import("../src/map");
  const { snap } = await import("../src/planner");
  const far = { x: campus.width / 2, y: campus.height / 2 };
  expect(snap(far)!.distance * scale).toBeGreaterThan(2);
  const road = campus.roads[0].points;
  const target = interpolate(road[0], road[1], 0.5);
  const near = { x: target.x, y: target.y + 0.5 / scale };
  let submitted: any = null;
  await page.addInitScript(() =>
    Object.defineProperty(crypto, "randomUUID", { value: undefined }),
  );
  await page.route("**/api/**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    const pose = {
      ...placeData.places[0],
      frame_id: "map",
      yaw: 0,
      age_s: 0,
      stamp_s: 1,
    };
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
          pose,
          task: null,
          gnss: {
            fixed: false,
            float: true,
            hacc_m: 0.7,
            satellites: 20,
            correction_age_s: 1,
          },
        },
      });
    if (path === "/api/places") return route.fulfill({ json: placeData });
    if (path === "/api/plans")
      return route.fulfill({
        json: {
          reference_path: [pose, toMap(target)],
          start: pose,
          goal: toMap(target),
          segment_roads: ["loop-a"],
          road_ids: ["loop-a"],
          length_m: 10,
        },
      });
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
  await choose(page, far);
  await expect(page.getByTestId("ab-message")).toContainText(
    "不在开放路线附近",
  );
  await expect(
    page.getByRole("button", { name: "封闭或恢复道路", exact: true }),
  ).toBeVisible();
  await choose(page, near);
  await page.getByTestId("plan-ab").click();
  await expect(page.getByTestId("ab-message")).toContainText("路线已生成");
  await expect(page.getByText("RTK 浮点解", { exact: false })).toBeVisible();
  await page.getByRole("button", { name: "执行导航", exact: true }).click();
  await expect.poll(() => submitted).not.toBeNull();
  expect(submitted.goal.x).toBeCloseTo(toMap(snap(near)!.point).x, 0);
  expect(submitted.goal.y).toBeCloseTo(toMap(snap(near)!.point).y, 0);
  expect(submitted.request_id).toMatch(/^[a-f0-9]{32}$/);
});
