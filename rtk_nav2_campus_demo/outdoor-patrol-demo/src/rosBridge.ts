import type { exportAB } from "./abNavigation";

export type BridgeStatus = {
  phase: "idle" | "queued" | "running" | "canceling" | "canceled" | "completed" | "failed";
  message: string;
  pose: { x: number; y: number; yaw: number } | null;
  distance_remaining: number | null;
  cmd_vel_count: number;
  calibration_version: string;
  map_version: string;
  calibration_status: string;
};

async function request(path: string, method: "GET" | "POST", body?: object): Promise<BridgeStatus> {
  const response = await fetch(path, {
    method,
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error ?? `Bridge HTTP ${response.status}`);
  return result as BridgeStatus;
}

export const bridgeStatus = () => request("/api/status", "GET");
export const sendSimulationRoute = (route: ReturnType<typeof exportAB>) => request("/api/routes", "POST", route);
export const cancelSimulationRoute = () => request("/api/cancel", "POST");
