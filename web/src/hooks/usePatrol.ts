import { useEffect, useRef, useState } from "react";
import { type Layers, type Mode, type Reference } from "../MapView";
import { download, exportCSV, exportMap, exportRoute } from "../export";
import {
  containsPoint,
  dist,
  presets,
  regions,
  roads,
  scale,
  toMeters,
} from "../map";
import { enteredRoad, snap } from "../planner";
import {
  advance,
  initial,
  log,
  planSimulation,
  toggleRoad,
} from "../simulation";
import type { Point, Simulation, Task } from "../types";

export function usePatrol() {
  const [workspaceMode, setWorkspaceMode] = useState<"patrol" | "ab">("patrol");
  const [state, setState] = useState<Simulation>(() =>
    planSimulation(initial()),
  );
  const [speed, setSpeed] = useState(5);
  const [mode, setMode] = useState<Mode>("inspect");
  const [layers, setLayers] = useState<Layers>({
    basemap: true,
    semantic: true,
    access: false,
    network: false,
    route: true,
    reference: true,
  });
  const [layerMenu, setLayerMenu] = useState(false),
    [help, setHelp] = useState(false);
  const [reference, setReference] = useState<Reference | null>(null);
  const [referenceMenu, setReferenceMenu] = useState(false);
  const [roadPanel, setRoadPanel] = useState(false);
  const [toast, setToast] = useState("");
  const fileRef = useRef<HTMLInputElement>(null);
  const nextId = useRef(1);
  const speedRef = useRef(speed);
  speedRef.current = speed;
  const editable =
    ["idle", "ready"].includes(state.status) &&
    state.completed.length === 0 &&
    state.traveled === 0;
  const routeValid =
    !!state.plan && state.status !== "idle" && state.status !== "blocked";
  const target = state.plan?.stops[state.stopIndex];
  const coordinate = toMeters(state.position);
  const progress =
    state.status === "completed"
      ? 100
      : state.tasks.length
        ? Math.round((state.completed.length / state.tasks.length) * 100)
        : 0;
  const nextSegment = state.plan?.segments.find(
    (seg) => seg.end > state.distance + 0.01,
  );
  const remaining = state.plan
    ? Math.max(0, state.plan.length - state.distance) * scale
    : 0;
  const remainingDwell = state.plan
    ? state.plan.stops
        .slice(state.stopIndex)
        .reduce(
          (sum, stop, i) =>
            sum + (i === 0 && state.dwelling ? state.dwell : stop.dwell),
          0,
        )
    : 0;
  useEffect(() => {
    let id: number;
    let last = performance.now();
    const frame = (now: number) => {
      const dt = Math.min((now - last) / 1000, 0.25);
      last = now;
      if (workspaceMode === "patrol")
        setState((s) => advance(s, dt * speedRef.current));
      id = requestAnimationFrame(frame);
    };
    id = requestAnimationFrame(frame);
    return () => cancelAnimationFrame(id);
  }, [workspaceMode]);
  useEffect(() => {
    if (!toast) return;
    const id = setTimeout(() => setToast(""), 4000);
    return () => clearTimeout(id);
  }, [toast]);
  useEffect(
    () => () => {
      if (reference?.url) URL.revokeObjectURL(reference.url);
    },
    [reference?.url],
  );
  const notify = (text: string) => setToast(text);
  const reset = () => {
    setState(planSimulation(initial()));
    setMode("inspect");
    setRoadPanel(false);
    setSpeed(5);
  };
  const updateTask = (
    tasks: Task[],
    start = state.home,
    returnHome = state.returnHome,
  ) => {
    setState((s) => ({
      ...initial(tasks, start, returnHome),
      blocked: s.blocked,
      logs: [
        ...s.logs,
        {
          id: (s.logs.at(-1)?.id ?? 0) + 1,
          time: 0,
          text: "任务已修改，请重新生成路线",
          kind: "info",
        },
      ],
    }));
  };
  const moveTask = (index: number, step: number) => {
    const tasks = [...state.tasks];
    [tasks[index], tasks[index + step]] = [tasks[index + step], tasks[index]];
    updateTask(tasks);
  };
  const pick = (p: Point, tolerance: number) => {
    if (mode === "inspect" || mode === "block") return;
    if (!editable) {
      notify("请先重置巡航，再编辑起点和任务。");
      return;
    }
    if (regions.some((r) => containsPoint(p, r.points))) {
      notify("这个位置位于建筑、绿化或待核实地面内，请选择道路。");
      return;
    }
    const projected = snap(p);
    const open = projected && snap(projected.point, state.blocked);
    if (
      !projected ||
      projected.distance > tolerance ||
      !open ||
      open.distance > 0.0001
    ) {
      notify("该位置不在开放路线附近。");
      return;
    }
    if (mode === "start") {
      updateTask(state.tasks, projected.point);
      setMode("inspect");
      notify("起点已吸附到道路，请重新生成路线。");
    } else {
      if (state.tasks.some((t) => dist(t.point, projected.point) < 8)) {
        notify("这个位置附近已有巡检点。");
        return;
      }
      const id = nextId.current++;
      updateTask([
        ...state.tasks,
        {
          id: `custom-${id}`,
          name: `自定义巡检点 ${id}`,
          point: projected.point,
          dwell: 2,
        },
      ]);
      notify("巡检点已吸附到道路，支持继续添加。");
    }
  };
  const roadClick = (id: string) => {
    if (mode !== "block") {
      const r = roads.find((r) => r.id === id)!;
      notify(
        `${r.name} · ${state.blocked.includes(id) ? "人工封闭" : "实测中心线 · 绝对精度未验证"}`,
      );
      return;
    }
    if (!state.blocked.includes(id) && enteredRoad(state.position) === id) {
      notify("机器狗已进入这段道路，请封闭前方路段。");
      return;
    }
    setState((s) => toggleRoad(s, id));
  };
  const addPreset = (task: Task) => updateTask([...state.tasks, task]);
  const run = () => {
    if (state.status === "running")
      setState((s) => log({ ...s, status: "paused" }, "模拟巡航已暂停"));
    else if (state.plan && ["ready", "paused"].includes(state.status))
      setState((s) =>
        log(
          { ...s, status: "running" },
          s.status === "paused" ? "继续模拟巡航" : "开始模拟巡航",
        ),
      );
  };
  const mapDownload = () =>
    download(
      "campus-map.json",
      JSON.stringify(exportMap(state.blocked), null, 2),
    );
  const routeDownload = () =>
    download("patrol-route.json", JSON.stringify(exportRoute(state), null, 2));
  const csvDownload = () =>
    download(
      "patrol-waypoints.csv",
      exportCSV(state),
      "text/csv;charset=utf-8",
    );
  const unused = presets.filter((t) => !state.tasks.some((p) => p.id === t.id));
  return {
    workspaceMode,
    setWorkspaceMode,
    state,
    setState,
    speed,
    setSpeed,
    mode,
    setMode,
    layers,
    setLayers,
    layerMenu,
    setLayerMenu,
    reference,
    setReference,
    referenceMenu,
    setReferenceMenu,
    roadPanel,
    setRoadPanel,
    toast,
    setToast,
    fileRef,
    nextId,
    speedRef,
    editable,
    routeValid,
    target,
    coordinate,
    progress,
    nextSegment,
    remaining,
    remainingDwell,
    notify,
    reset,
    updateTask,
    moveTask,
    pick,
    roadClick,
    addPreset,
    run,
    mapDownload,
    routeDownload,
    csvDownload,
    unused,
    setHelp,
    help,
  };
}
export type PatrolModel = ReturnType<typeof usePatrol>;
