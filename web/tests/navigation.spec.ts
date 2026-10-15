import { expect, test } from "@playwright/test";
import { bundle, toMap } from "../src/map";
import { choose, shortGoal, start } from "./mapHelpers";

const pose = {
  ...toMap(start),
  frame_id: "map",
  yaw: -1.24,
  stamp_s: 10,
  age_s: 0.02,
};
const versions = {
  schema_version: 1,
  scene_version: bundle.scene_version,
  route_version: bundle.route_version,
};

test("online planning uses robot pose; execution, replacement and cancellation use task IDs", async ({
  page,
}) => {
  const requests: { path: string; body: any }[] = [];
  let task: any = null;
  await page.route("**/api/**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    const body =
      route.request().method() === "POST"
        ? route.request().postDataJSON()
        : null;
    requests.push({ path, body });
    let response: any;
    if (path === "/api/status")
      response = {
        ...versions,
        ready: true,
        run_id: "browser-test",
        localization: "ok",
        health: {},
        pose,
        task,
      };
    else if (path === "/api/obstacles") response = { obstacles: [] };
    else if (path === "/api/plans")
      response = {
        ...versions,
        start: pose,
        goal: body.goal,
        reference_path: [pose, body.goal],
        road_ids: ["loop-a"],
        segment_roads: ["loop-a"],
        length_m: 2,
      };
    else if (path === "/api/tasks") {
      task = {
        task_id: (requests.filter((r) => r.path === path).length + "").padStart(
          32,
          "0",
        ),
        state: "navigating",
        goal: body.goal,
        blocked_road_ids: body.blocked_road_ids,
        path_revision: 1,
        execution_path: [pose, body.goal],
        reference_path: [pose, body.goal],
        distance_remaining_m: 2,
        reason: null,
      };
      response = task;
    } else if (path.endsWith("/cancel")) {
      task = { ...task, state: "canceled" };
      response = task;
    } else throw new Error(`Unexpected endpoint ${path}`);
    await route.fulfill({ json: response });
  });
  await page.goto("/");
  await page.getByRole("button", { name: "A→B 规划", exact: true }).click();
  await expect(page.getByTestId("navigation-status")).toHaveText(
    "Gazebo 导航就绪",
  );
  await expect(page.getByTestId("select-a")).toBeDisabled();
  await choose(page, shortGoal);
  await page.getByTestId("plan-ab").click();
  await expect(page.getByTestId("ab-message")).toContainText("后端参考路线");
  await page.getByRole("button", { name: "执行导航", exact: true }).click();
  await expect(page.getByTestId("execution-route")).toBeAttached();
  await page.getByRole("button", { name: "替换导航目标", exact: true }).click();
  await expect
    .poll(() => requests.filter((r) => r.path === "/api/tasks").length)
    .toBe(2);
  await page.getByTitle("取消导航并停车", { exact: true }).click();
  await expect(page.getByTestId("navigation-status")).toHaveText(
    "已取消并停车",
  );
  const submission = requests.find((r) => r.path === "/api/tasks")!.body;
  expect(submission.frame_id).toBe("map");
  expect(submission.scene_version).toBe(bundle.scene_version);
  const selected = {
    x: Number(await page.getByTestId("ab-b").getAttribute("data-x")),
    y: Number(await page.getByTestId("ab-b").getAttribute("data-y")),
  };
  expect(submission.goal.x).toBeCloseTo(toMap(selected).x, 6);
  expect(
    Math.hypot(
      submission.goal.x - toMap(shortGoal).x,
      submission.goal.y - toMap(shortGoal).y,
    ),
  ).toBeLessThan(0.1);
  expect(Object.keys(submission).sort()).toEqual([
    "blocked_road_ids",
    "frame_id",
    "goal",
    "route_version",
    "scene_version",
  ]);
  expect(
    requests.some(
      (r) => r.path === "/api/tasks/00000000000000000000000000000002/cancel",
    ),
  ).toBe(true);
});

test("mismatched scene prevents execution and explains rebuild", async ({
  page,
}) => {
  await page.route("**/api/status", (route) =>
    route.fulfill({
      json: {
        ...versions,
        scene_version: "different-scene",
        ready: true,
        pose,
        task: null,
      },
    }),
  );
  await page.goto("/");
  await page.getByRole("button", { name: "A→B 规划", exact: true }).click();
  await expect(page.getByTestId("navigation-status")).toContainText(
    "场景版本不一致",
  );
  await expect(
    page.getByRole("button", { name: "执行导航", exact: true }),
  ).toBeDisabled();
});

test("map interaction adds static and moving obstacles and deletes them", async ({
  page,
}) => {
  let items: any[] = [];
  const creations: any[] = [];
  await page.route("**/api/**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    const method = route.request().method();
    let response: any;
    if (path === "/api/status")
      response = {
        ...versions,
        ready: true,
        run_id: "obstacles-test",
        localization: "ok",
        health: {},
        pose,
        task: null,
      };
    else if (path === "/api/obstacles" && method === "GET")
      response = { obstacles: items };
    else if (path === "/api/obstacles" && method === "POST") {
      const body = route.request().postDataJSON();
      creations.push(body);
      response = {
        ...body,
        position: body.start,
        error: null,
        started_sim_s: 10,
      };
      items.push(response);
    } else if (method === "DELETE") {
      const name = path.split("/").at(-1);
      items = items.filter((item) => item.name !== name);
      response = { removed: name };
    } else throw new Error(`Unexpected ${method} ${path}`);
    await route.fulfill({ json: response });
  });
  await page.goto("/");
  await page.getByRole("button", { name: "A→B 规划", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "选障碍物位置", exact: true }),
  ).toBeEnabled();
  await page.getByRole("button", { name: "选障碍物位置", exact: true }).click();
  await choose(page, shortGoal);
  await page
    .getByRole("button", { name: "添加到 Gazebo", exact: true })
    .click();
  await expect(page.getByTestId("scene-obstacle")).toHaveCount(1);
  await page.getByLabel("障碍物类型").selectOption("moving");
  await page.getByRole("button", { name: "选障碍物位置", exact: true }).click();
  await choose(page, shortGoal);
  await page.getByRole("button", { name: "选移动终点", exact: true }).click();
  await choose(page, { x: shortGoal.x + 10, y: shortGoal.y });
  await page.getByLabel("障碍物速度").fill("0.2");
  await page
    .getByRole("button", { name: "添加到 Gazebo", exact: true })
    .click();
  await expect(page.getByTestId("scene-obstacle")).toHaveCount(2);
  expect(creations[1].kind).toBe("moving");
  expect(creations[1].speed_mps).toBe(0.2);
  expect(
    Math.hypot(
      creations[1].start.x - toMap(shortGoal).x,
      creations[1].start.y - toMap(shortGoal).y,
    ),
  ).toBeLessThan(0.15);
  expect(creations[1].end.x).toBeGreaterThan(creations[1].start.x);
  for (let i = 0; i < 2; i++)
    await page
      .getByRole("button", { name: /删除障碍物/ })
      .first()
      .click();
  await expect(page.getByTestId("scene-obstacle")).toHaveCount(0);
});
