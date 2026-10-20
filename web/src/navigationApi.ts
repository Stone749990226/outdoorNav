import { bundle, toMap } from "./map";
import type { Point } from "./types";

export type TaskState =
  | "accepted"
  | "planning"
  | "navigating"
  | "replanning"
  | "canceling"
  | "succeeded"
  | "failed"
  | "canceled";
export type NavigationTask = {
  task_id: string;
  state: TaskState;
  goal: Point;
  blocked_road_ids: string[];
  path_revision: number;
  execution_path: Point[];
  reference_path: Point[];
  distance_remaining_m: number | null;
  reason: string | null;
};
export type NavigationStatus = {
  mode?: "lynx" | "simulation";
  gnss?: {
    fixed: boolean;
    hacc_m: number;
    satellites: number;
    correction_age_s: number | null;
  } | null;
  pose_source?: string;
  schema_version: number;
  ready: boolean;
  run_id: string;
  scene_version: string;
  route_version: string;
  health: Record<string, boolean>;
  localization: string;
  pose:
    | (Point & { frame_id: "map"; yaw: number; stamp_s: number; age_s: number })
    | null;
  task: NavigationTask | null;
};
export type RoutePreview = {
  schema_version: number;
  scene_version: string;
  route_version: string;
  start: Point;
  goal: Point;
  reference_path: Point[];
  segment_roads: string[];
  road_ids: string[];
  length_m: number | null;
};
export const terminal = (task: NavigationTask | null) =>
  !task || ["succeeded", "failed", "canceled"].includes(task.state);
export const compatible = (status: NavigationStatus | null) =>
  !!status &&
  status.schema_version === 1 &&
  status.scene_version === bundle.scene_version &&
  status.route_version === bundle.route_version;

async function request<T>(
  path: string,
  method = "GET",
  body?: object,
  signal?: AbortSignal,
): Promise<T> {
  const response = await fetch(path, {
    method,
    signal,
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error ?? `HTTP ${response.status}`);
  return result as T;
}
export type SemanticPlace = Point & {
  id: string;
  name: string;
  aliases: string[];
  yaw: number;
  enabled: boolean;
  note: string;
};
export const getPlaces = (signal?: AbortSignal) =>
  request<{ route_version: string; places: SemanticPlace[] }>(
    "/api/places",
    "GET",
    undefined,
    signal,
  );
const payload = (goal: Point, blocked: string[]) => ({
  frame_id: "map",
  scene_version: bundle.scene_version,
  route_version: bundle.route_version,
  goal: toMap(goal),
  blocked_road_ids: blocked,
  request_id: crypto.randomUUID().replaceAll("-", ""),
});
export const getStatus = (signal?: AbortSignal) =>
  request<NavigationStatus>("/api/status", "GET", undefined, signal);
export const previewRoute = (goal: Point, blocked: string[]) =>
  request<RoutePreview>("/api/plans", "POST", payload(goal, blocked));
export const executeGoal = (goal: Point, blocked: string[]) =>
  request<NavigationTask>("/api/tasks", "POST", payload(goal, blocked));
export const cancelTask = (id: string) =>
  request<NavigationTask>(`/api/tasks/${id}/cancel`, "POST");
