import { test, expect } from "@playwright/test";
import fs from "node:fs/promises";

test("full automatic patrol, closure, replan and route download", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto("/");
  await page.clock.install();
  await page.getByRole("button", { name: "一键演示", exact: true }).click();
  await page.getByRole("button", { name: "10×", exact: true }).click();
  // Advance in one-second frames so the simulation follows realistic animation-frame timing.
  for (let i = 0; i < 80; i++) await page.clock.runFor(1000);
  await expect(page.getByTestId("completion-card")).toBeVisible();
  await expect(page.getByTestId("replans")).toHaveText("01次");
  await expect(page.getByTestId("status")).toHaveText("巡航已完成");
  await expect(page.getByTestId("old-route")).toBeAttached();
  await page.locator(".export-menu summary").click();
  const [download] = await Promise.all([
    page.waitForEvent("download"),
    page.getByRole("button", { name: "路线 JSON", exact: false }).click(),
  ]);
  const data = JSON.parse(await fs.readFile((await download.path())!, "utf8"));
  expect(data.scale_is_mock).toBe(true);
  expect(data.frame_id).toBe("demo_map");
  expect(data.completed_task_ids).toHaveLength(5);
  expect(data.blocked_roads).toContain("c-e");
  expect(data.path.length).toBeGreaterThan(5);
  expect(errors).toEqual([]);
});

test("pause, disconnect, recovery, stale export and point editing", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByRole("button", { name: "开始", exact: true }).click();
  await page.waitForTimeout(700);
  await page.getByRole("button", { name: "暂停", exact: true }).click();
  const x = await page.getByTestId("robot").getAttribute("data-x");
  const y = await page.getByTestId("robot").getAttribute("data-y");
  await page.waitForTimeout(300);
  expect(await page.getByTestId("robot").getAttribute("data-x")).toBe(x);
  expect(await page.getByTestId("robot").getAttribute("data-y")).toBe(y);
  await page.getByRole("button", { name: "重置巡航", exact: true }).click();
  await page
    .getByRole("button", { name: "封闭或恢复道路", exact: true })
    .click();
  await page.getByTestId("toggle-n-c").click();
  await page.getByTestId("toggle-ne-n").click();
  await expect(page.getByRole("alert")).toContainText("无法到达");
  await expect(page.getByTestId("status")).toHaveText("目标不可达");
  await page.locator(".export-menu summary").click();
  await expect(
    page.getByRole("button", { name: "路线 JSON", exact: false }),
  ).toBeDisabled();
  await page.locator(".export-menu summary").click();
  await page.getByTestId("toggle-n-c").click();
  await expect(page.getByTestId("status")).toHaveText("路线就绪");
  await page.getByRole("button", { name: "关闭道路管理", exact: true }).click();
  await page.getByRole("button", { name: "删除东侧楼前", exact: true }).click();
  await expect(page.getByTestId("status")).toHaveText("等待规划");
  await page.getByRole("button", { name: "生成巡航路线", exact: true }).click();
  await expect(page.getByTestId("status")).toHaveText("路线就绪");
});

for (const size of [
  { width: 1366, height: 768 },
  { width: 1920, height: 1080 },
]) {
  test(`layout and layers at ${size.width}×${size.height}`, async ({
    page,
  }) => {
    await page.setViewportSize(size);
    await page.goto("/");
    expect(
      await page.evaluate(
        () =>
          document.body.scrollWidth <= innerWidth &&
          document.body.scrollHeight <= innerHeight,
      ),
    ).toBe(true);
    await expect(
      page.getByRole("button", { name: "一键演示", exact: true }),
    ).toBeInViewport();
    await expect(
      page.getByRole("button", { name: "生成巡航路线", exact: true }),
    ).toBeInViewport();
    await page.getByRole("button", { name: "通行性地图", exact: true }).click();
    await page.getByRole("button", { name: "图层设置", exact: true }).click();
    await page.getByText("道路拓扑网络", { exact: true }).click();
    await expect(page.locator(".node-label")).toHaveCount(11);
    await page
      .getByRole("button", { name: "关闭图层设置", exact: true })
      .click();
    await page.screenshot({
      path: `test-results/map-${size.width}.png`,
      fullPage: true,
    });
  });
}
