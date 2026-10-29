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
    valid?: boolean;
    latitude?: number;
    longitude?: number;
    fixed: boolean;
    float?: boolean;
    hacc_m: number;
    satellites: number;
    correction_age_s: number | null;
    vel_e_mps?: number | null;
    vel_n_mps?: number | null;
    speed_accuracy_mps?: number | null;
  } | null;
  heading?: {
    source: "configured" | "wit_imu";
    ready: boolean;
    raw_yaw_deg?: number | null;
    angles_fresh?: boolean;
    gyro_fresh?: boolean;
    nine_axis_verified?: boolean;
    reference_confirmed?: boolean;
    body_yaw_rate_rps?: number | null;
    heading_fresh?: boolean;
    true_heading_deg?: number | null;
    body_yaw_enu_rad?: number | null;
    mounting_confirmed?: boolean;
    quality_confirmed?: boolean;
    age_s?: number | null;
  } | null;
  localization_mode?: "external_odom" | "rtk_imu";
  velocity_source?: string;
  feedback?: {
    ready: boolean;
    confirmed: boolean;
    source: string;
    reason?: string | null;
  };
  position_source?: string;
  diagnostic?: boolean;
  command_topic?: string;
  not_ready_reasons?: { code: string; message: string }[];
  pose_source?: string;
  localization_grade?: "fixed" | "float" | null;
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
export const requestId = () =>
  Array.from(crypto.getRandomValues(new Uint8Array(16)), (value) =>
    value.toString(16).padStart(2, "0"),
  ).join("");
const payload = (goal: Point, blocked: string[], placeId?: string) => ({
  frame_id: "map",
  scene_version: bundle.scene_version,
  route_version: bundle.route_version,
  goal: toMap(goal),
  blocked_road_ids: blocked,
  request_id: requestId(),
  ...(placeId ? { place_id: placeId } : {}),
});
export const getStatus = (signal?: AbortSignal) =>
  request<NavigationStatus>("/api/status", "GET", undefined, signal);
export const previewRoute = (
  goal: Point,
  blocked: string[],
  placeId?: string,
) =>
  request<RoutePreview>("/api/plans", "POST", payload(goal, blocked, placeId));
export const executeGoal = (goal: Point, blocked: string[], placeId?: string) =>
  request<NavigationTask>(
    "/api/tasks",
    "POST",
    payload(goal, blocked, placeId),
  );
export const cancelTask = (id: string) =>
  request<NavigationTask>(`/api/tasks/${id}/cancel`, "POST");

export type CostmapSnapshot = {
  map_from_frame: (Point & { yaw: number; age_s: number }) | null;
  schema_version: number;
  frame_id: string;
  width: number;
  height: number;
  resolution: number;
  origin: Point & { yaw: number };
  stamp_s: number;
  age_s: number;
  stale: boolean;
  image: string;
  robot: (Point & { yaw: number; age_s: number }) | null;
  path: Point[];
};
export const getCostmap = (layer: "local" | "global", signal?: AbortSignal) =>
  request<CostmapSnapshot>(`/api/costmaps/${layer}`, "GET", undefined, signal);
