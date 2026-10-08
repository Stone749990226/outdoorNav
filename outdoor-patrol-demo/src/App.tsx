import { useEffect, useRef, useState } from "react";
import {
  Activity,
  ArrowDown,
  ArrowUp,
  ArrowUpRight,
  Check,
  ChevronRight,
  CircleHelp,
  Download,
  Flag,
  Layers as LayersIcon,
  LocateFixed,
  Map as MapIcon,
  MousePointer2,
  Pause,
  Play,
  Plus,
  RotateCcw,
  Route,
  ShieldCheck,
  Sparkles,
  Target,
  Upload,
  X,
  Ban,
  Settings2,
  Crosshair,
} from "lucide-react";
import { ABWorkspace } from "./ABWorkspace";
import { MapView, type Layers, type Mode, type Reference } from "./MapView";
import {
  presets,
  campus,
  calibration,
  roads,
  regions,
  containsPoint,
  scale,
  toMeters,
  dist,
} from "./map";
import { enteredRoad, snap } from "./planner";
import {
  advance,
  initial,
  log,
  planSimulation,
  startDemo,
  toggleRoad,
} from "./simulation";
import { download, exportCSV, exportMap, exportRoute } from "./export";
import type { Point, Simulation, Task } from "./types";
const statusText = {
  idle: "等待规划",
  ready: "路线就绪",
  running: "巡航进行中",
  paused: "巡航已暂停",
  blocked: "目标不可达",
  completed: "巡航已完成",
};
const fmtTime = (t: number) =>
  `${Math.floor(t / 60)
    .toString()
    .padStart(2, "0")}:${Math.floor(t % 60)
      .toString()
      .padStart(2, "0")}`;
export default function App() {
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
    if (!projected || projected.distance > tolerance || !open || open.distance > 0.0001) {
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
  return (
    <div className="app-shell">
      <header className="app-header">
        <a className="brand" href="#" onClick={(e) => e.preventDefault()}>
          <span className="brand-icon">
            <Route size={24} />
          </span>
          <span className="brand-name">
            <span>FIELD / NAV</span>
          </span>
        </a>
        <div className="header-divider" />
        <div className="workspace-title">
          园区巡航任务台 <span>OUTDOOR AUTONOMY LAB</span>
        </div>
        <nav className="header-nav">
          <span className="nav-active">
            <MapIcon size={15} />
            任务工作台
          </span>
          <button onClick={() => setHelp(true)}>
            <CircleHelp size={15} />
            演示指南
          </button>
        </nav>
        <span className="simulation-badge">
          <span className="live-dot" />
          SIMULATION
        </span>
      </header>
      <div className="page-heading">
        <div>
          <div className="eyebrow">
            MISSION CONTROL <span>/ 01</span>
          </div>
          <h1>
            <span className="heading-dot"></span>
            智慧谷北区
          </h1>
          <p>
            实测环线 <span> / </span> 中央支路
          </p>
        </div>
        <div className="heading-actions">
          <div className="workspace-switch">
            <button
              className={workspaceMode === "patrol" ? "active" : ""}
              onClick={() => setWorkspaceMode("patrol")}
            >
              巡航任务
            </button>
            <button
              className={workspaceMode === "ab" ? "active" : ""}
              onClick={() => {
                setWorkspaceMode("ab");
                setState((s) =>
                  s.status === "running" ? { ...s, status: "paused" } : s,
                );
              }}
            >
              A→B 规划
            </button>
          </div>
          {workspaceMode === "patrol" && (
            <details className="export-menu">
              <summary>
                <Download size={16} />
                导出数据
                <ChevronRight size={14} />
              </summary>
              <div className="dropdown">
                <button onClick={mapDownload}>
                  地图 JSON<span>实测中心线路网</span>
                </button>
                <button onClick={routeDownload} disabled={!routeValid}>
                  路线 JSON<span>完整折线与任务顺序</span>
                </button>
                <button onClick={csvDownload} disabled={!routeValid}>
                  途经点 CSV<span>局部坐标与参考朝向</span>
                </button>
              </div>
            </details>
          )}
          <button
            className="primary demo-button"
            onClick={() => {
              setWorkspaceMode("patrol");
              setState(startDemo());
              setSpeed(5);
              setMode("inspect");
              setRoadPanel(false);
            }}
          >
            <Sparkles size={17} />
            一键演示
            <ArrowUpRight size={17} />
          </button>
        </div>
      </div>
      {workspaceMode === "ab" ? (
        <ABWorkspace />
      ) : (
        <>
          <main className="workspace">
            <aside className="left-panel panel">
              <div className="panel-heading">
                <div>
                  <span className="eyebrow">MISSION / 任务</span>
                  <h2>园区巡检计划</h2>
                </div>
                <span className="small-badge">01</span>
              </div>
              <div className="mission-summary">
                <div className="mission-icon">
                  <Flag size={18} />
                </div>
                <div>
                  <strong>园区日常巡航</strong>
                  <span>
                    {state.tasks.length} 个巡检点 ·{" "}
                    {state.returnHome ? "闭环返回" : "单程任务"}
                  </span>
                </div>
                <span className="muted-arrow">
                  <ChevronRight size={17} />
                </span>
              </div>
              <div className="section-line">
                <span>
                  巡检顺序 <small> / WAYPOINTS</small>
                </span>
                <span className="point-count">
                  {state.tasks.length.toString().padStart(2, "0")}
                </span>
              </div>
              <div className="task-list" data-testid="task-list">
                {state.tasks.map((task, i) => {
                  const done = state.completed.includes(task.id),
                    active = target?.id === task.id;
                  return (
                    <div
                      key={task.id}
                      className={`task-row ${done ? "done" : ""} ${active ? "active" : ""}`}
                    >
                      <div className="task-order">
                        {done ? (
                          <Check size={13} />
                        ) : (
                          String(i + 1).padStart(2, "0")
                        )}
                      </div>
                      <div className="task-text">
                        <strong>{task.name}</strong>
                        <span>
                          {done
                            ? "巡检完成"
                            : active && state.dwelling
                              ? "到点停留中"
                              : `巡检停留 ${task.dwell}s`}
                          <i>演示点</i>
                        </span>
                      </div>
                      <div className="task-actions">
                        <button
                          title={`上移${task.name}`}
                          disabled={!editable || i === 0}
                          onClick={() => moveTask(i, -1)}
                        >
                          <ArrowUp size={12} />
                        </button>
                        <button
                          title={`下移${task.name}`}
                          disabled={!editable || i === state.tasks.length - 1}
                          onClick={() => moveTask(i, 1)}
                        >
                          <ArrowDown size={12} />
                        </button>
                        <button
                          title={`删除${task.name}`}
                          disabled={!editable}
                          onClick={() =>
                            updateTask(
                              state.tasks.filter((t) => t.id !== task.id),
                            )
                          }
                        >
                          <X size={12} />
                        </button>
                      </div>
                    </div>
                  );
                })}
                {!state.tasks.length && (
                  <div className="empty-tasks">
                    从地图添加巡检点，或恢复预置点。
                  </div>
                )}
              </div>
              <button
                className={`add-task ${mode === "target" ? "selected" : ""}`}
                disabled={!editable}
                onClick={() =>
                  setMode(mode === "target" ? "inspect" : "target")
                }
              >
                <Plus size={15} />
                在地图上添加巡检点
              </button>
              {unused.length > 0 && (
                <div className="restore-presets">
                  恢复：
                  {unused.map((t) => (
                    <button
                      key={t.id}
                      disabled={!editable}
                      onClick={() => addPreset(t)}
                    >
                      {t.name}
                    </button>
                  ))}
                </div>
              )}
              <label className="return-toggle">
                <span>
                  <RotateCcw size={14} />
                  完成后返回起点
                </span>
                <input
                  type="checkbox"
                  checked={state.returnHome}
                  disabled={!editable}
                  onChange={(e) =>
                    updateTask(state.tasks, state.home, e.target.checked)
                  }
                />
                <i />
              </label>
              <button
                className="plan-button"
                disabled={
                  ["running", "paused"].includes(state.status) ||
                  !state.tasks.length
                }
                onClick={() => setState((s) => planSimulation(s))}
              >
                <Route size={17} />
                生成巡航路线
                <ChevronRight size={16} />
              </button>
              <div className="mission-note">
                <ShieldCheck size={16} />
                <p>
                  沿已标注道路规划
                  <br />
                  <span>仅沿已记录路线规划</span>
                </p>
              </div>
              <div className="left-bottom">
                <span className="eyebrow">MAP SOURCE</span>
                <p>卫星影像 + 路网辅助重绘</p>
                <span className="source-tag">DGPS 实测</span>
                <span className="source-tag">{campus.version}</span>
              </div>
            </aside>
            <section className="map-panel panel">
              <div className="map-toolbar">
                <div className="map-tabs">
                  <button
                    className={!layers.access ? "active" : ""}
                    onClick={() =>
                      setLayers((l) => ({
                        ...l,
                        access: false,
                        semantic: true,
                      }))
                    }
                  >
                    <LayersIcon size={14} />
                    实测路线
                  </button>
                  <button
                    className={layers.access ? "active" : ""}
                    onClick={() => setLayers((l) => ({ ...l, access: true }))}
                  >
                    通行性地图
                  </button>
                </div>
                <div className="map-tools">
                  <button
                    title="设置起点"
                    className={mode === "start" ? "active" : ""}
                    disabled={!editable}
                    onClick={() =>
                      setMode(mode === "start" ? "inspect" : "start")
                    }
                  >
                    <LocateFixed size={16} />
                  </button>
                  <button
                    title="封闭或恢复道路"
                    className={mode === "block" ? "active warning" : ""}
                    onClick={() => {
                      setMode(mode === "block" ? "inspect" : "block");
                      setRoadPanel(mode !== "block");
                    }}
                  >
                    <Ban size={16} />
                  </button>
                  <button
                    title="图层设置"
                    className={layerMenu ? "active" : ""}
                    onClick={() => setLayerMenu(!layerMenu)}
                  >
                    <Settings2 size={16} />
                  </button>
                </div>
              </div>
              <MapView
                state={state}
                layers={layers}
                mode={mode}
                reference={reference}
                onPick={pick}
                onRoad={roadClick}
                onInfo={notify}
              />
              {layerMenu && (
                <div className="floating-panel layer-panel">
                  <div className="floating-title">
                    地图图层
                    <button
                      onClick={() => setLayerMenu(false)}
                      aria-label="关闭图层设置"
                    >
                      <X size={14} />
                    </button>
                  </div>
                  {(
                    [
                      ["basemap", "园区示意底图"],
                      ["semantic", "巡检点名称"],
                      ["access", "路线可达性"],
                      ["network", "道路拓扑网络"],
                      ["route", "路线与途经点"],
                      ["reference", "导入的参考图片"],
                    ] as const
                  ).map(([key, label]) => (
                    <label key={key}>
                      <span>{label}</span>
                      <input
                        type="checkbox"
                        checked={layers[key]}
                        onChange={(e) =>
                          setLayers((l) => ({ ...l, [key]: e.target.checked }))
                        }
                      />
                    </label>
                  ))}
                  <button
                    className="upload-button"
                    onClick={() => fileRef.current?.click()}
                  >
                    <Upload size={14} />
                    导入本地参考图片
                  </button>
                  {reference && (
                    <button
                      className="upload-button"
                      onClick={() => {
                        setReferenceMenu(true);
                        setLayerMenu(false);
                      }}
                    >
                      <Settings2 size={14} />
                      调整参考图片
                    </button>
                  )}
                </div>
              )}
              {referenceMenu && reference && (
                <div className="floating-panel reference-panel">
                  <div className="floating-title">
                    参考图片对齐
                    <button
                      aria-label="关闭参考图片设置"
                      onClick={() => setReferenceMenu(false)}
                    >
                      <X size={14} />
                    </button>
                  </div>
                  <p>仅调整参考图片，不改变地图坐标。</p>
                  {(
                    [
                      {
                        key: "opacity",
                        label: "透明度",
                        min: 0,
                        max: 1,
                        step: 0.05,
                      },
                      {
                        key: "scale",
                        label: "缩放",
                        min: 0.2,
                        max: 3,
                        step: 0.05,
                      },
                      {
                        key: "x",
                        label: "水平偏移",
                        min: -1000,
                        max: 1000,
                        step: 5,
                      },
                      {
                        key: "y",
                        label: "垂直偏移",
                        min: -920,
                        max: 920,
                        step: 5,
                      },
                    ] as const
                  ).map((item) => (
                    <label key={item.key}>
                      <span>{item.label}</span>
                      <input
                        aria-label={item.label}
                        type="range"
                        min={item.min}
                        max={item.max}
                        step={item.step}
                        value={reference[item.key]}
                        onChange={(e) =>
                          setReference({
                            ...reference,
                            [item.key]: Number(e.target.value),
                          })
                        }
                      />
                      <output>{reference[item.key]}</output>
                    </label>
                  ))}
                  <button
                    className="text-button"
                    onClick={() => {
                      setReference(null);
                      setReferenceMenu(false);
                    }}
                  >
                    移除图片
                  </button>
                </div>
              )}
              {roadPanel && (
                <div className="floating-panel road-panel">
                  <div className="floating-title">
                    道路通行管理
                    <button
                      onClick={() => {
                        setRoadPanel(false);
                        setMode("inspect");
                      }}
                      aria-label="关闭道路管理"
                    >
                      <X size={14} />
                    </button>
                  </div>
                  <p>点击道路或列表切换。封路后自动重新规划。</p>
                  <div className="road-list">
                    {roads.map((r) => (
                      <button
                        key={r.id}
                        data-testid={`toggle-${r.id}`}
                        className={state.blocked.includes(r.id) ? "closed" : ""}
                        onClick={() => roadClick(r.id)}
                      >
                        <span>{r.name}</span>
                        <span>
                          {state.blocked.includes(r.id) ? "已封闭" : "开放"}
                        </span>
                      </button>
                    ))}
                  </div>
                </div>
              )}
              {mode !== "inspect" && (
                <div
                  className={`mode-hint ${mode === "block" ? "warning" : ""}`}
                >
                  <MousePointer2 size={14} />
                  {mode === "start"
                    ? "点击道路设置起点 · 自动吸附"
                    : mode === "target"
                      ? "点击道路添加巡检点 · 可连续添加"
                      : "点击前方道路，模拟封闭或恢复"}
                  <button
                    title="退出编辑模式"
                    onClick={() => {
                      setMode("inspect");
                      setRoadPanel(false);
                    }}
                  >
                    <X size={14} />
                  </button>
                </div>
              )}
              {state.error && (
                <div className="error-banner" role="alert">
                  <Ban size={18} />
                  <div>
                    <strong>目标暂不可达</strong>
                    <p>{state.error}</p>
                  </div>
                </div>
              )}
              {state.status === "completed" && (
                <div className="completion-card" data-testid="completion-card">
                  <div className="completion-check">
                    <Check size={25} />
                  </div>
                  <span className="eyebrow">MISSION ACCOMPLISHED</span>
                  <h2>巡航任务完成</h2>
                  <p>
                    到访 {state.completed.length} 个巡检点 · 行走{" "}
                    {(state.traveled * scale).toFixed(0)} 米 · 重规划{" "}
                    {state.replans} 次
                  </p>
                  <button onClick={routeDownload}>
                    导出本次路线 <ArrowUpRight size={14} />
                  </button>
                </div>
              )}
              <div className="map-status">
                <span>
                  <span
                    className={`live-dot ${state.status === "blocked" ? "orange" : ""}`}
                  />
                  <b data-testid="status">{statusText[state.status]}</b>
                </span>
                <span>
                  <Crosshair size={12} /> {coordinate.x.toFixed(1)},{" "}
                  {coordinate.y.toFixed(1)} <small>米</small>
                </span>
                <span className="frame-tag">{calibration.frame_id}</span>
              </div>
            </section>
            <aside className="right-panel panel">
              <div className="panel-heading">
                <div>
                  <span className="eyebrow">LIVE / 模拟状态</span>
                  <h2>巡航概览</h2>
                </div>
                <Activity size={18} className="mint" />
              </div>
              <div className="robot-card">
                <div className="robot-visual">
                  <svg viewBox="0 0 120 80">
                    <ellipse cx="60" cy="65" rx="39" ry="8" fill="#081510" />
                    <path
                      d="M30 36 L27 55 L19 66 M76 37 L79 57 L88 65 M42 40 L45 55 L36 65 M86 35 L95 51 L94 63"
                      fill="none"
                      stroke="#92b29d"
                      strokeWidth="5"
                      strokeLinecap="round"
                    />
                    <path
                      d="M27 26 L72 20 L90 29 L83 43 L38 46 L25 38 Z"
                      fill="#8fad98"
                    />
                    <path d="M27 26 L70 20 L80 27 L38 34Z" fill="#cce4cf" />
                    <path d="M38 34 L83 27 L83 43 L38 46Z" fill="#9cbba4" />
                    <path d="M82 26 L98 28 L101 37 L84 39Z" fill="#e0f2d8" />
                    <path d="M94 30 L99 31 L99 34 L94 34Z" fill="#324c3c" />
                  </svg>
                </div>
                <div>
                  <strong>巡检机器狗</strong>
                  <span>
                    <i /> 虚拟执行器
                  </span>
                </div>
                <span className="sim-pill">SIM</span>
              </div>
              <div className="current-target">
                <span className="eyebrow">
                  {state.status === "completed"
                    ? "任务已结束"
                    : "当前目标 / NEXT TARGET"}
                </span>
                <strong>
                  {state.status === "completed"
                    ? "全部巡检点已到达"
                    : (target?.name ??
                      (state.error ? "等待道路恢复" : "等待生成路线"))}
                  <Target size={18} />
                </strong>
                <span>
                  {state.dwelling
                    ? `到点停留 · 剩余 ${state.dwell.toFixed(1)} s`
                    : nextSegment
                      ? `下一途经点：${toMeters(nextSegment.b).x.toFixed(0)}, ${toMeters(nextSegment.b).y.toFixed(0)}`
                      : "通过地图规划下一次巡航"}
                </span>
              </div>
              <div className="metric-grid">
                <div>
                  <span>已走里程</span>
                  <strong data-testid="traveled">
                    {(state.traveled * scale).toFixed(1)}
                    <small>m*</small>
                  </strong>
                </div>
                <div>
                  <span>剩余路程</span>
                  <strong>
                    {state.plan ? remaining.toFixed(0) : "—"}
                    <small>m*</small>
                  </strong>
                </div>
                <div>
                  <span>模拟用时</span>
                  <strong className="time-metric">
                    {fmtTime(state.elapsed)}
                  </strong>
                </div>
                <div>
                  <span>重规划</span>
                  <strong data-testid="replans">
                    {state.replans.toString().padStart(2, "0")}
                    <small>次</small>
                  </strong>
                </div>
              </div>
              <div className="progress-section">
                <div>
                  <span>任务进度</span>
                  <strong>
                    {state.completed.length}{" "}
                    <small>/ {state.tasks.length}</small>
                  </strong>
                </div>
                <div className="progress-track">
                  <i style={{ width: `${progress}%` }} />
                </div>
                <span>
                  {state.status === "completed"
                    ? "全部完成"
                    : `预计剩余 ${Math.ceil((remaining + remainingDwell) / speed)} 秒播放时间`}
                  <small>* 距离使用演示比例尺</small>
                </span>
              </div>
              <div className="event-section">
                <div className="section-line">
                  <span>
                    运行事件 <small>/ EVENTS</small>
                  </span>
                  <span className="live-dot" />
                </div>
                <div className="event-list" aria-live="polite">
                  {state.logs
                    .slice(-5)
                    .reverse()
                    .map((event) => (
                      <div
                        key={event.id}
                        className={`event-item ${event.kind}`}
                      >
                        <i />
                        <div>
                          <p>{event.text}</p>
                          <time>{fmtTime(event.time)}</time>
                        </div>
                      </div>
                    ))}
                </div>
              </div>
            </aside>
          </main>
          <footer className="playback-bar">
            <div className="playback-controls">
              <button
                className={`play-button ${state.status === "running" ? "playing" : ""}`}
                title={
                  state.status === "running"
                    ? "暂停"
                    : state.status === "paused"
                      ? "继续"
                      : "开始"
                }
                disabled={!routeValid || state.status === "completed"}
                onClick={run}
              >
                {state.status === "running" ? (
                  <Pause size={19} />
                ) : (
                  <Play size={19} />
                )}
              </button>
              <button className="reset-button" title="重置巡航" onClick={reset}>
                <RotateCcw size={17} />
              </button>
              <div className="playback-label">
                <strong>{statusText[state.status]}</strong>
                <span>
                  {state.autoDemo ? "自动封路演示已启用" : "自由探索模式"}
                </span>
              </div>
            </div>
            <div className="timeline">
              <span>{fmtTime(state.elapsed)}</span>
              <div>
                <i
                  style={{
                    width: `${state.plan && state.traveled + remaining / scale > 0 ? Math.min(100, (state.traveled / (state.traveled + remaining / scale)) * 100) : 0}%`,
                  }}
                />
              </div>
              <span>
                {state.plan
                  ? `${(state.traveled * scale + remaining).toFixed(0)} m*`
                  : "—"}
              </span>
            </div>
            <div className="speed-control">
              <span>播放速度</span>
              {[1, 5, 10].map((n) => (
                <button
                  key={n}
                  className={speed === n ? "active" : ""}
                  onClick={() => setSpeed(n)}
                >
                  {n}×
                </button>
              ))}
            </div>
            <div className="footer-note">
              <span className="live-dot" />
              DGPS 实测 · 仿真巡航 · 绝对精度未验证
            </div>
          </footer>
        </>
      )}
      {toast && (
        <div className="toast" role="status">
          <CircleHelp size={17} />
          {toast}
          <button aria-label="关闭提示" onClick={() => setToast("")}>
            <X size={14} />
          </button>
        </div>
      )}
      <input
        ref={fileRef}
        type="file"
        accept="image/png,image/jpeg,image/webp"
        className="hidden-input"
        aria-label="导入参考图片"
        onChange={(e) => {
          const f = e.target.files?.[0];
          if (!f) return;
          if (!["image/png", "image/jpeg", "image/webp"].includes(f.type)) {
            notify("请选择 PNG、JPEG 或 WebP 图片。");
            return;
          }
          if (f.size > 20 * 1024 * 1024) {
            notify("请选择小于 20 MB 的图片。");
            return;
          }
          const url = URL.createObjectURL(f);
          const img = new Image();
          img.onload = () => {
            setReference({ url, opacity: 0.55, x: 0, y: 0, scale: 1 });
            setReferenceMenu(true);
            setLayerMenu(false);
            setLayers((l) => ({ ...l, reference: true }));
          };
          img.onerror = () => {
            URL.revokeObjectURL(url);
            notify("图片无法解码，请选择有效图片。");
          };
          img.src = url;
          e.target.value = "";
        }}
      />
      {help && (
        <div className="modal-backdrop" onClick={() => setHelp(false)}>
          <section
            className="help-modal"
            role="dialog"
            aria-modal="true"
            aria-label="演示指南"
            onClick={(e) => e.stopPropagation()}
          >
            <button
              className="modal-close"
              aria-label="关闭指南"
              onClick={() => setHelp(false)}
            >
              <X size={20} />
            </button>
            <span className="eyebrow">YOUR FIRST MISSION</span>
            <h2>三分钟，走完一次园区巡航。</h2>
            <div className="guide-steps">
              <p>
                <b>01</b>
                <span>
                  <strong>认识地图</strong>
                  查看实测路线、道路连接与当前可达状态。
                </span>
              </p>
              <p>
                <b>02</b>
                <span>
                  <strong>点击「一键演示」</strong>
                  从北侧入口出发，访问五个巡检点并返回。默认 5×
                  播放，约两分钟完成。
                </span>
              </p>
              <p>
                <b>03</b>
                <span>
                  <strong>观察自动绕行</strong>
                  到达中央路口后模拟封闭前方连接道，橙色虚线保留上次路线，绿色显示新路线。
                </span>
              </p>
              <p>
                <b>04</b>
                <span>
                  <strong>探索与导出</strong>
                  重置后修改任务、点选起点或封路。导出局部米制地图、路线与关键途经点。
                </span>
              </p>
            </div>
            <div className="guide-note">
              地图来自两段差分解轨迹，采用米制坐标。绝对位置精度尚未独立验证；当前连接 Nav2 仿真机器人。
            </div>
            <button className="primary" onClick={() => setHelp(false)}>
              开始探索 <ArrowUpRight size={16} />
            </button>
          </section>
        </div>
      )}
    </div>
  );
}
