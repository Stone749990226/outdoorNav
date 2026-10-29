import { expect, test } from "@playwright/test";
import { bundle } from "../src/map";

test("read-only costmap view switches frames, shows stale data and returns to goal selection", async ({
  page,
}) => {
  const requests: string[] = [];
  const mutations: string[] = [];
  await page.route("**/api/**", async (route) => {
    const request = route.request();
    const path = new URL(request.url()).pathname;
    if (request.method() !== "GET") mutations.push(path);
    if (path === "/api/status")
      return route.fulfill({
        json: {
          schema_version: 1,
          mode: "lynx",
          scene_version: bundle.scene_version,
          route_version: bundle.route_version,
          ready: false,
          health: {},
          pose: null,
          task: null,
          localization: "waiting_for_rtk",
        },
      });
    if (path === "/api/places")
      return route.fulfill({
        json: { places: [], route_version: bundle.route_version },
      });
    if (path.startsWith("/api/costmaps/")) {
      requests.push(path);
      const global = path.endsWith("/global");
      return route.fulfill({
        json: {
          schema_version: 1,
          map_from_frame: {
            x: global ? 0 : 2,
            y: global ? 0 : 3,
            yaw: 0,
            age_s: 0.1,
          },
          frame_id: global ? "map" : "odom",
          width: 10,
          height: 10,
          resolution: 0.1,
          origin: { x: 0, y: 0, yaw: 0 },
          stamp_s: 1,
          age_s: global ? 5 : 0.1,
          stale: global,
          image:
            "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+j+GQAAAAASUVORK5CYII=",
          robot: { x: 0.4, y: 0.5, yaw: 0.2, age_s: 0.1 },
          path: [
            { x: 0, y: 0 },
            { x: 1, y: 1 },
          ],
        },
      });
    }
    return route.fulfill({ json: { obstacles: [] } });
  });
  await page.goto("/");
  await page.getByTestId("toggle-costmap").click();
  await expect(page.getByTestId("costmap-layer-local")).toBeVisible();
  await expect(page.getByTestId("costmap-layer-global")).toBeVisible();
  await expect(page.getByTestId("costmap-panel")).toContainText("已过期");
  const canvas = page.getByRole("img", { name: "实时导航代价地图" });
  const box = (await canvas.boundingBox())!;
  const before = await canvas.getAttribute("viewBox");
  await canvas.hover();
  await page.mouse.wheel(0, -300);
  await expect.poll(() => canvas.getAttribute("viewBox")).not.toBe(before);
  const zoomed = await canvas.getAttribute("viewBox");
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(
    box.x + box.width / 2 + 80,
    box.y + box.height / 2 + 30,
  );
  await page.mouse.up();
  await expect.poll(() => canvas.getAttribute("viewBox")).not.toBe(zoomed);
  await page.getByLabel("局部 · 青色").uncheck();
  await expect(page.getByTestId("costmap-layer-local")).toHaveCount(0);
  await expect(page.getByTestId("costmap-layer-global")).toBeVisible();
  await page.getByTestId("toggle-costmap").click();
  await expect(page.getByTestId("costmap-panel")).toHaveCount(0);
  await expect(page.getByTestId("select-b")).toBeVisible();
  expect(requests).toContain("/api/costmaps/local");
  expect(requests).toContain("/api/costmaps/global");
  expect(mutations).toEqual([]);
});
