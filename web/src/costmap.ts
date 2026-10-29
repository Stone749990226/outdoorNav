import type { CostmapSnapshot } from "./navigationApi";
import type { Point } from "./types";

/** OccupancyGrid rows start at the bottom; the PNG is already flipped north-up. */
export function gridPoint(
  grid: Pick<CostmapSnapshot, "origin" | "resolution" | "height">,
  point: Point,
): Point {
  const dx = point.x - grid.origin.x,
    dy = point.y - grid.origin.y;
  const c = Math.cos(grid.origin.yaw),
    s = Math.sin(grid.origin.yaw);
  return {
    x: (c * dx + s * dy) / grid.resolution,
    y: grid.height - (-s * dx + c * dy) / grid.resolution,
  };
}

/** Frame-local point to map ENU. Never silently overlay an unaligned odom grid. */
export function mapPoint(grid: CostmapSnapshot, point: Point): Point | null {
  const tf = grid.map_from_frame;
  if (!tf) return null;
  const c = Math.cos(tf.yaw),
    s = Math.sin(tf.yaw);
  return {
    x: tf.x + c * point.x - s * point.y,
    y: tf.y + s * point.x + c * point.y,
  };
}
