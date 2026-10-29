import { describe, expect, it } from "vitest";
import { gridPoint } from "./costmap";

describe("costmap coordinates", () => {
  it("uses origin and resolution and flips ROS rows", () => {
    expect(
      gridPoint(
        { origin: { x: 10, y: -5, yaw: 0 }, resolution: 0.5, height: 20 },
        { x: 11, y: -4 },
      ),
    ).toEqual({ x: 2, y: 18 });
  });
  it("handles a rotated grid rather than mixing map and odom", () => {
    const result = gridPoint(
      { origin: { x: 0, y: 0, yaw: Math.PI / 2 }, resolution: 1, height: 10 },
      { x: 0, y: 2 },
    );
    expect(result.x).toBeCloseTo(2);
    expect(result.y).toBeCloseTo(10);
  });
});
