import { bundle } from "./map";
import type { Point } from "./types";

export type ObstacleInput = {
  name: string;
  kind: "static" | "moving";
  start: Point;
  end: Point;
  width: number;
  depth: number;
  height: number;
  speed_mps: number;
};
export type SceneObstacle = ObstacleInput & {
  position: Point;
  error: string | null;
  started_sim_s: number;
};
export async function obstacleRequest<T>(
  suffix: string,
  method = "GET",
  body?: object,
  signal?: AbortSignal,
): Promise<T> {
  const response = await fetch(`/api/obstacles${suffix}`, {
    method,
    signal,
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error ?? `HTTP ${response.status}`);
  return payload as T;
}
export const addObstacle = (input: ObstacleInput) =>
  obstacleRequest<SceneObstacle>("", "POST", {
    ...input,
    frame_id: "map",
    scene_version: bundle.scene_version,
    route_version: bundle.route_version,
  });
export const removeObstacle = (name: string) =>
  obstacleRequest(`/${encodeURIComponent(name)}`, "DELETE");
