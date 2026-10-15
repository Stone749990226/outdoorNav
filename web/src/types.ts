export type Point = { x: number; y: number };
export type Node = Point & { id: string; name: string };
export type Road = {
  id: string;
  from: string;
  to: string;
  name: string;
  points: Point[];
  length_m: number;
  source_log: string;
};
export type Region = {
  id: string;
  kind: "building" | "green" | "unknown";
  name: string;
  points: Point[];
};
export type Task = { id: string; name: string; point: Point; dwell: number };
export type Segment = {
  a: Point;
  b: Point;
  edgeId: string;
  start: number;
  end: number;
};
export type Stop = Task & { distance: number; returning?: boolean };
export type Plan = {
  segments: Segment[];
  stops: Stop[];
  length: number;
  start: Point;
  taskOrder: string[];
};
export type Log = {
  id: number;
  time: number;
  text: string;
  kind: "info" | "success" | "warning";
};
export type Status =
  | "idle"
  | "ready"
  | "running"
  | "paused"
  | "blocked"
  | "completed";
export type Simulation = {
  status: Status;
  position: Point;
  heading: number;
  plan: Plan | null;
  previousPlan: Plan | null;
  distance: number;
  traveled: number;
  elapsed: number;
  completed: string[];
  stopIndex: number;
  dwell: number;
  dwelling: boolean;
  trace: Point[];
  blocked: string[];
  replans: number;
  logs: Log[];
  error: string | null;
  autoDemo: boolean;
  autoTriggered: boolean;
  tasks: Task[];
  home: Point;
  returnHome: boolean;
};
