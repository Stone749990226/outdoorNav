import { expect, type Page } from "@playwright/test";
import campus from "../src/campus.json" with { type: "json" };
import calibration from "../src/calibration.json" with { type: "json" };
export { campus, calibration };
export type Point = { x: number; y: number };
export const interpolate = (a: Point, b: Point, t: number): Point => ({ x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) });
export async function choose(page: Page, point: Point) {
  const screen = await page.getByTestId("campus-map").evaluate((svg, coordinates) => {
    const group = svg.querySelector(":scope > g") as SVGGElement;
    const p = (svg as SVGSVGElement).createSVGPoint(); p.x = coordinates.x; p.y = coordinates.y;
    const result = p.matrixTransform(group.getScreenCTM()!);
    return { x: result.x, y: result.y };
  }, point);
  await page.mouse.click(screen.x, screen.y);
}
export async function planRoute(page: Page, a: Point, b: Point) {
  await page.goto("/");
  await page.getByRole("button", { name: "A→B 规划" }).click();
  await choose(page, a); await choose(page, b);
  await page.getByTestId("plan-ab").click();
  await expect(page.getByTestId("ab-message")).toContainText("路线已生成");
}
export const start = campus.roads[0].points[0];
export const shortGoal = interpolate(start, campus.roads[0].points[1], 2 / (Math.hypot(campus.roads[0].points[1].x - start.x, campus.roads[0].points[1].y - start.y) * calibration.meters_per_unit));
