import { home, presets, scale, dist, roads } from "./map";
import { makePlan, positionAt, enteredRoad } from "./planner";
import type { Simulation, Task, Point, Log } from "./types";
export function initial(
  tasks: Task[] = presets,
  start: Point = home,
  returnHome = true,
): Simulation {
  return {
    status: "idle",
    position: { ...start },
    heading: 0,
    plan: null,
    previousPlan: null,
    distance: 0,
    traveled: 0,
    elapsed: 0,
    completed: [],
    stopIndex: 0,
    dwell: 0,
    dwelling: false,
    trace: [{ ...start }],
    blocked: [],
    replans: 0,
    logs: [
      {
        id: 0,
        time: 0,
        text: "园区地图已就绪 · 通行性为图片推定",
        kind: "info",
      },
    ],
    error: null,
    autoDemo: false,
    autoTriggered: false,
    tasks,
    home: start,
    returnHome,
  };
}
export function log(
  state: Simulation,
  text: string,
  kind: Log["kind"] = "info",
): Simulation {
  return {
    ...state,
    logs: [
      ...state.logs,
      { id: (state.logs.at(-1)?.id ?? 0) + 1, time: state.elapsed, text, kind },
    ].slice(-60),
  };
}
export function planSimulation(state: Simulation, run = false): Simulation {
  const tasks = state.tasks.filter((t) => !state.completed.includes(t.id));
  const { plan, error } = makePlan(
    state.position,
    tasks,
    state.home,
    state.returnHome,
    state.blocked,
  );
  const next: Simulation = {
    ...state,
    previousPlan: state.plan ?? state.previousPlan,
    plan,
    error,
    distance: 0,
    stopIndex: 0,
    dwelling: false,
    dwell: 0,
    status: error ? "blocked" : run ? "running" : "ready",
  };
  return log(
    next,
    error ?? `路线已生成 · ${(plan!.length * scale).toFixed(0)} 演示米`,
    error ? "warning" : "success",
  );
}
export function startDemo(): Simulation {
  return log(
    planSimulation({ ...initial(), autoDemo: true }, true),
    "一键演示开始 · 将在中央路口演示前方封路",
  );
}
export function toggleRoad(state: Simulation, id: string): Simulation {
  if (state.status === "completed")
    return log(state, "本次巡航已完成，请重置后编辑道路。", "info");
  const closing = !state.blocked.includes(id);
  if (closing && enteredRoad(state.position) === id)
    return log(state, "机器狗已进入这段道路，请选择前方路段封闭。", "warning");
  const blocked = closing
    ? [...state.blocked, id]
    : state.blocked.filter((x) => x !== id);
  let next = log(
    { ...state, blocked },
    `${closing ? "人工封闭" : "恢复通行"}：${roads.find((r) => r.id === id)!.name}`,
    closing ? "warning" : "success",
  );
  if (state.status === "idle")
    return {
      ...next,
      status: "idle",
      plan: null,
      previousPlan: state.plan,
      error: null,
    };
  const wasPaused = state.status === "paused";
  next = planSimulation(
    { ...next, replans: next.replans + 1 },
    state.status === "running",
  );
  if (wasPaused && next.plan) next.status = "paused";
  return next;
}
export function advance(state: Simulation, dt: number): Simulation {
  if (state.status !== "running" || !state.plan || dt <= 0) return state;
  let s: Simulation = {
    ...state,
    trace: [...state.trace],
    completed: [...state.completed],
  };
  let remaining = dt;
  let guard = 0;
  while (remaining > 1e-8 && s.status === "running" && guard++ < 1000) {
    const stop = s.plan!.stops[s.stopIndex];
    if (!stop) {
      s = log(
        { ...s, status: "completed" },
        "巡航完成 · 所有目标已到达",
        "success",
      );
      break;
    }
    if (s.distance >= stop.distance - 1e-7) {
      if (!s.dwelling) {
        s.dwelling = true;
        s.dwell = stop.dwell;
        if (!stop.returning)
          s = log(s, `到达 ${stop.name} · 停留 ${stop.dwell} 秒`);
      }
      const use = Math.min(remaining, s.dwell);
      s.elapsed += use;
      s.dwell -= use;
      remaining -= use;
      if (s.dwell > 1e-8) break;
      if (!stop.returning && !s.completed.includes(stop.id))
        s.completed.push(stop.id);
      s.stopIndex++;
      s.dwelling = false;
      if (s.autoDemo && !s.autoTriggered && stop.id === "p-c") {
        s = toggleRoad({ ...s, autoTriggered: true }, "c-e");
        s = log(s, "已从当前位置绕行 · 已完成巡检点保持不变", "success");
      }
      if (s.stopIndex >= s.plan!.stops.length)
        s = log(
          { ...s, status: "completed" },
          "巡航完成 · 所有目标已到达",
          "success",
        );
      continue;
    }
    // Stop at every polyline vertex so the recorded trace cannot cut corners.
    const segment = s.plan!.segments.find((seg) => seg.end > s.distance + 1e-7);
    const boundary = Math.min(stop.distance, segment?.end ?? stop.distance);
    const movement = Math.min(remaining / scale, boundary - s.distance);
    const used = movement * scale;
    s.distance += movement;
    s.traveled += movement;
    s.elapsed += used;
    remaining -= used;
    const sample = positionAt(s.plan!, s.distance);
    s.position = sample.point;
    s.heading = sample.heading;
    if (
      dist(s.trace.at(-1)!, s.position) > 2 ||
      Math.abs(s.distance - boundary) < 1e-6
    )
      s.trace.push({ ...s.position });
  }
  return s;
}
