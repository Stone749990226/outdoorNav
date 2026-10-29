import { test, expect } from "@playwright/test";
import { campus, choose, planRoute, start, shortGoal } from "./mapHelpers";

test.beforeEach(async ({ page }) => {
  await page.route("**/api/status", (route) => route.abort());
});

test("measured patrol completes without old map references", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/");
  await page.clock.install();
  await page.getByRole("button", { name: "离线动画演示", exact: true }).click();
  await page.getByRole("button", { name: "10×", exact: true }).click();
  for (let i = 0; i < 40; i++) await page.clock.runFor(1000);
  await expect(page.getByTestId("completion-card")).toBeVisible();
  await expect(
    page.getByRole("button", { name: /导出本次路线|路线 JSON|途经点 CSV/ }),
  ).toHaveCount(0);
  expect(errors).toEqual([]);
});

test("closed ring arc detours, both arcs disconnect and restoration recovers", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByRole("button", { name: "巡航任务", exact: true }).click();
  await page
    .getByRole("button", { name: "封闭或恢复道路", exact: true })
    .click();
  await page.getByTestId("toggle-loop-a").click();
  await expect(page.getByTestId("status")).toHaveText("路线就绪");
  await page.getByTestId("toggle-loop-b").click();
  await expect(page.getByRole("alert")).toContainText("无法到达");
  await page.getByTestId("toggle-loop-a").click();
  await expect(page.getByTestId("status")).toHaveText("路线就绪");
});

test("A/B projection, raw comparison, zoom and local playback", async ({
  page,
}) => {
  await page.route("**/api/status", (route) => route.abort());
  await planRoute(page, { x: start.x + 5, y: start.y }, shortGoal);
  const projected = {
    x: Number(await page.getByTestId("ab-a").getAttribute("data-x")),
    y: Number(await page.getByTestId("ab-a").getAttribute("data-y")),
  };
  expect(projected).not.toEqual({ x: start.x + 5, y: start.y });
  await page.getByRole("button", { name: "原始 / 滤波" }).click();
  await expect(page.getByTestId("survey-traces")).toBeAttached();
  await page.getByRole("button", { name: "放大地图", exact: true }).click();
  await page.getByRole("button", { name: "复位视角", exact: true }).click();
  await page.getByRole("button", { name: "离线动画预览", exact: true }).click();
  await expect(page.locator(".ab-arrived")).toBeVisible();
});

test("zero length A/B completes locally and cannot be sent to Nav2", async ({
  page,
}) => {
  await planRoute(page, start, start);
  await expect(page.getByRole("button", { name: "执行导航" })).toBeDisabled();
  await page.getByRole("button", { name: "离线动画预览", exact: true }).click();
  await expect(page.locator(".ab-arrived")).toBeVisible();
});

for (const size of [
  { width: 1366, height: 768 },
  { width: 1920, height: 1080 },
  { width: 390, height: 844 },
]) {
  test(`measured map layout ${size.width}x${size.height}`, async ({ page }) => {
    await page.setViewportSize(size);
    await page.goto("/");
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await expect(page.getByTestId("centerline-loop-a")).toBeAttached();
    await expect(page.getByTestId("centerline-loop-b")).toBeAttached();
    await expect(page.getByTestId("centerline-branch")).toBeAttached();
    expect(await page.locator(".campus-svg polygon").count()).toBe(0);
    await page.getByRole("button", { name: "A→B 规划" }).click();
    await page
      .getByRole("button", { name: "显示道路网络", exact: true })
      .click();
    await expect(page.locator(".node-label")).toHaveCount(campus.nodes.length);
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await page.screenshot({
      path: `test-results/survey-${size.width}.png`,
      fullPage: true,
    });
  });
}

test("A/B planning draws intermediate route waypoints", async ({ page }) => {
  const goal = campus.roads[0].points.at(-1)!;
  await planRoute(page, start, goal);
  await expect(page.getByTestId("planned-route")).toBeAttached();
  await expect
    .poll(() => page.getByTestId("route-waypoint").count())
    .toBeGreaterThan(0);
  const marker = page.getByTestId("route-waypoint").first();
  await expect(marker).toHaveAttribute("aria-label", /途经点/);
  await marker.focus();
  await expect(page.getByTestId("ab-message")).toContainText("途经点");
  await expect(
    page.locator("details").filter({ hasText: "规划途经点" }),
  ).toBeAttached();
});
