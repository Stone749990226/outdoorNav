import { expect, test } from "@playwright/test";
import fs from "node:fs/promises";
import { campus, calibration, interpolate, planRoute, start, shortGoal } from "./mapHelpers";

test("offline measured route exports metric coordinates", async ({ page }) => {
  await page.route("**/api/status", (route) => route.abort());
  await planRoute(page, start, shortGoal);
  await expect(page.getByTestId("bridge-status")).toContainText("未连接");
  await expect(page.getByRole("button", { name: "发送仿真路径" })).toBeDisabled();
  const [download] = await Promise.all([page.waitForEvent("download"), page.getByRole("button", { name: "路线 + 途经点 JSON" }).click()]);
  const route = JSON.parse(await fs.readFile((await download.path())!, "utf8"));
  expect(route.frame_id).toBe("map"); expect(route.scale_is_mock).toBe(false);
  expect(route.calibration_status).toBe(calibration.status);
  expect(Math.abs(route.length - 2)).toBeLessThan(0.1);
  expect(route).not.toHaveProperty("road_width_m");
});

test("bridge sends the measured path, shows feedback and cancels", async ({ page }) => {
  let phase = "idle", sent: Record<string, unknown> | null = null;
  await page.route("**/api/**", async (request) => {
    const url = new URL(request.request().url());
    if (url.pathname === "/api/routes") { sent = request.request().postDataJSON(); phase = "running"; }
    if (url.pathname === "/api/cancel") phase = "canceled";
    await request.fulfill({ status: 200, contentType: "application/json", body: JSON.stringify({
      phase, message: phase, pose: { x: 0, y: 0, yaw: 0 }, distance_remaining: phase === "running" ? 2.5 : null,
      cmd_vel_count: 0, calibration_version: calibration.version, map_version: calibration.map_version, calibration_status: calibration.status,
    }) });
  });
  await planRoute(page, start, shortGoal);
  await page.getByRole("button", { name: "发送仿真路径" }).click();
  await expect(page.getByTestId("bridge-status")).toContainText("剩余 2.5 m");
  expect(sent).toMatchObject({ frame_id: "map", map_version: campus.version, calibration_status: calibration.status });
  await page.getByRole("button", { name: "取消 Nav2 仿真" }).click();
  await expect(page.getByTestId("bridge-status")).toContainText("canceled");
});

test("stale ROS map blocks submission and position overlay", async ({ page }) => {
  await page.route("**/api/status", (route) => route.fulfill({ json: { phase: "idle", message: "old map", pose: { x: 1, y: 2 }, calibration_version: "old" } }));
  await planRoute(page, start, shortGoal);
  await expect(page.getByTestId("bridge-status")).toContainText("版本不一致");
  await expect(page.getByRole("button", { name: "发送仿真路径" })).toBeDisabled();
  await expect(page.getByTestId("ros-robot-pose")).toHaveCount(0);
});

test.describe("real Nav2 controller", () => {
  test.skip(process.env.BRIDGE_REAL !== "1", "Requires ros2_bridge/run_sim.sh");
  test("tracks a short measured ring path", async ({ page }) => {
    await planRoute(page, start, shortGoal);
    await expect(page.getByRole("button", { name: "发送仿真路径" })).toBeEnabled();
    await page.getByRole("button", { name: "发送仿真路径" }).click();
    await expect.poll(() => page.evaluate(() => fetch("/api/status").then((r) => r.json()).then((s) => s.phase)), { timeout: 45000 }).toBe("completed");
    const status = await page.evaluate(() => fetch("/api/status").then((r) => r.json()));
    expect(status.cmd_vel_count).toBeGreaterThan(0);
    await page.screenshot({ path: "test-results/nav2-survey-ring.png", fullPage: true });
  });
  test("tracks a branch-to-ring path through the junction", async ({ page }) => {
    const branch = campus.roads[2].points, ring = campus.roads[0].points;
    await planRoute(page, branch.at(-2)!, interpolate(ring.at(-2)!, ring.at(-1)!, 0.5));
    await page.getByRole("button", { name: "发送仿真路径" }).click();
    await expect.poll(() => page.evaluate(() => fetch("/api/status").then((r) => r.json()).then((s) => s.phase)), { timeout: 60000 }).toBe("completed");
    await page.screenshot({ path: "test-results/nav2-survey-junction.png", fullPage: true });
  });
  test("cancels a long route and rejects mismatched map metadata", async ({ page, request }) => {
    await planRoute(page, start, campus.nodes[1]);
    const [download] = await Promise.all([page.waitForEvent("download"), page.getByRole("button", { name: "路线 + 途经点 JSON" }).click()]);
    const data = JSON.parse(await fs.readFile((await download.path())!, "utf8"));
    const invalid = await request.post("/api/routes", { data: { ...data, map_version: "old" } });
    expect(invalid.status()).toBe(400);
    const commandsBefore = (await request.get("/api/status").then((r) => r.json())).cmd_vel_count;
    await page.getByRole("button", { name: "发送仿真路径" }).click();
    await expect.poll(() => request.get("/api/status").then((r) => r.json()).then((s) => s.cmd_vel_count), { timeout: 15000 }).toBeGreaterThan(commandsBefore);
    await expect(page.getByRole("button", { name: "取消 Nav2 仿真" })).toBeEnabled();
    await page.getByRole("button", { name: "取消 Nav2 仿真" }).click();
    await expect.poll(() => page.evaluate(() => fetch("/api/status").then((r) => r.json()).then((s) => s.phase)), { timeout: 15000 }).toBe("canceled");
  });
});
