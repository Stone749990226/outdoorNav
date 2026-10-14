import { useEffect, useRef, useState } from "react";
import {
  ArrowRight,
  ArrowRightLeft,
  Ban,
  Check,
  Crosshair,
  Download,
  Flag,
  LocateFixed,
  Map as MapIcon,
  Pause,
  Play,
  RotateCcw,
  Route,
  Send,
  Square,
  X,
} from "lucide-react";
import { MapView, type Layers, type Mode } from "./MapView";
import { snap } from "./planner";
import { exportAB, planAB, withCarStart, type ABResult } from "./abNavigation";
import { download, exportMap } from "./export";
import { calibration, dist, home, mapToSvg, scale, toMeters, roads } from "./map";
import { bridgeStatus, cancelSimulationRoute, sendSimulationRoute, type BridgeStatus } from "./rosBridge";
import { advance, initial } from "./simulation";
import { reasonNames, serializeWaypoints, waypointCSV } from "./waypoints";
import type { Point, Simulation } from "./types";
const emptyLayers: Layers = {
  basemap: true,
  semantic: true,
  access: false,
  network: false,
  route: true,
  reference: false,
};
const coordinate = (p: Point | null) =>
  p
    ? `${toMeters(p).x.toFixed(2)}, ${toMeters(p).y.toFixed(2)}`
    : "点击地图选择";
// ★★ 起点 A ＝ **小车当前位置**（业主口径 2026-10-14）✓
//   ★ 来源：`GET /api/status` 的 `pose{x,y}`（**米**，frame `map` ✓）
//     ⇒ `mapToSvg()` 投到图上的坐标 ✓ ⇒ `snap()` 投到实测中心线 ✓
//   ★ 为什么必须这样：车的起始点由**车自己**决定（适配器吞掉 `/outdoor_nav/sim_reset`
//     ⇒ 车不会被瞬移 ✓）⇒ ★ 若 A 由人手点，路径起点就不是车的地方 ⇒
//     ★ 实测过：**路径不经过车（横向 2.91 m）⇒ nav2 中止（status 6）、车只挪 1.235 m** ✓
//   ★ ★ **不设距离上限** ✗（reviewer 2026-10-14 更正 ✓）：A 一律取**离车最近的路网点** ✓；
//     车离线那段由 `withCarStart()` 接在 `path`／`waypoints` **前面** ✓（见 abNavigation.ts ✓）
const ACTIVE_PHASES = ["queued", "running", "canceling"] as const;
const isActivePhase = (phase: string) =>
  (ACTIVE_PHASES as readonly string[]).includes(phase);
export function ABWorkspace() {
  const [a, setA] = useState<Point | null>(null),
    [b, setB] = useState<Point | null>(null);
  const [selection, setSelection] = useState<"a" | "b" | null>("a");
  const [blocked, setBlocked] = useState<string[]>([]),
    [blockMode, setBlockMode] = useState(false);
  const [result, setResult] = useState<ABResult | null>(null),
    [message, setMessage] = useState("等待选择 A/B");
  const [layers, setLayers] = useState(emptyLayers),
    [showKeys, setShowKeys] = useState(true),
    [showSurvey, setShowSurvey] = useState(false);
  const [bridge, setBridge] = useState<BridgeStatus | null>(null),
    [bridgeError, setBridgeError] = useState("");
  const [preview, setPreview] = useState<Simulation | null>(null);
  const [busy, setBusy] = useState(false);
  const [sending, setSending] = useState(false);
  const generation = useRef(0);
  useEffect(() => {
    let id: number,
      last = performance.now();
    const tick = (now: number) => {
      const dt = Math.min((now - last) / 1000, 0.25);
      last = now;
      setPreview((s) => (s ? advance(s, dt * 5) : s));
      id = requestAnimationFrame(tick);
    };
    id = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(id);
  }, []);
  useEffect(
    () => () => {
      generation.current++;
    },
    [],
  );
  useEffect(() => {
    let active = true;
    const refresh = () => {
      bridgeStatus()
        .then((status) => {
          if (!active) return;
          setBridge(status);
          setBridgeError("");
        })
        .catch(() => {
          if (!active) return;
          setBridge(null);
        });
    };
    refresh();
    const timer = window.setInterval(refresh, 1000);
    return () => {
      active = false;
      window.clearInterval(timer);
    };
  }, []);
  // ★ A 的来源（★ 每次 1 s 轮询到的新 pose 都会重算 ✓）—— **两种形态** ✓：
  //   ① pose 有 ⇒ **锁定**：A＝`snap(车位置)`（★ **不设距离上限** ✓），点地图只设 B ✓
  //   ② pose 没有（桥没连／版本不一致）⇒ **离线兜底**：保留原来的手动选 A/B ✓
  //      （★ 他们的 `tests/bridge.spec.ts` 有一条"未连桥也能规划/导出"的用例，就是走这条 ✓）
  const compatibleBridge = bridge?.calibration_version === calibration.version;
  const livePose =
    compatibleBridge && bridge?.pose
      ? mapToSvg({ x: bridge.pose.x, y: bridge.pose.y })
      : null;
  const poseSnap = livePose ? snap(livePose) : null;
  const poseOffsetM = poseSnap ? poseSnap.distance * scale : null;
  const aLocked = poseSnap !== null;
  const aSource: "pose" | "offline" = aLocked ? "pose" : "offline";
  useEffect(() => {
    if (!aLocked || !poseSnap) return;
    const next = poseSnap.point;
    if (a && dist(a, next) < 1e-9) return;
    generation.current++;
    setA(next);
    setSelection((s) => (s === "a" ? "b" : s));
    // ★ 车动了 ⇒ A 跟着动 ⇒ 已生成的路线**就地按新 A 重算** ✓
    //   ✗ **不清空** ✗ —— 清空会让"发送仿真路径"按钮变灰 ⇒ ★ 业主口径的
    //   "车在跑时再发新路线 ⇒ 直接替换"就**没法从界面做** ✗（★ 覆盖那一路要的就是车在跑 ✓）
    //   ★ 重算只是把起点换成车**此刻**的位置 ✓；`planAB` 不动 ✓
    setResult((current) => {
      if (!current?.plan || !b) return current;
      try {
        const refreshed = planAB(next, b, blocked);
        return refreshed.error ? current : refreshed;
      } catch {
        return current;
      }
    });
  }, [aLocked, poseSnap?.point.x, poseSnap?.point.y, a, b, blocked]);
  const invalidate = () => {
    generation.current++;
    setResult(null);
    setPreview(null);
    setBusy(false);
  };
  const pick = (p: Point, tolerance: number) => {
    if (blockMode) return;
    // ★ A 锁定（车在路网上）⇒ 点地图**只设 B** ✓（起点由车决定，不可改 ✓）
    const target: "a" | "b" | null = aLocked ? "b" : selection;
    if (!target) return;
    const projected = snap(p);
    const open = projected && snap(projected.point, blocked);
    if (!projected || projected.distance > tolerance || !open || open.distance > 0.0001) {
      setMessage("该位置不在开放路线附近。");
      return;
    }
    invalidate();
    if (target === "a") {
      setA(projected.point);
      setSelection("b");
      setMessage("A 点已投影到中心线。");
    } else {
      setB(projected.point);
      // ★ 锁定时保持 "b" ⇒ 可以连续点、反复改终点 ✓（离线兜底时与原来一致：设完变 null ✓）
      setSelection(aLocked ? "b" : null);
      setMessage(aLocked ? "B 点已投影到中心线（起点＝小车当前位置，已锁定）。" : "A/B 已投影到中心线。");
    }
  };
  const compute = () => {
    if (!a || !b) return;
    setBusy(true);
    const token = ++generation.current;
    // Let the button paint before the synchronous, cached grid search.
    setTimeout(() => {
      if (token !== generation.current) return;
      try {
        const next = planAB(a, b, blocked);
        setResult(next);
        setPreview(null);
        setMessage(
          next.error ??
            `路线已生成 · ${next.waypoints.length} 个重要途经点`,
        );
      } catch {
        setResult(null);
        setMessage("规划校验失败，请重新选择目标。");
      } finally {
        setBusy(false);
      }
    }, 30);
  };
  const closeRoad = (id: string) => {
    invalidate();
    setBlocked((xs) =>
      xs.includes(id) ? xs.filter((x) => x !== id) : [...xs, id],
    );
    setMessage("道路状态已变化，旧路线已失效，请重新规划。");
  };
  const reset = () => {
    invalidate();
    setA(null); // ★ 锁定时下一次轮询（≤1 s）会把它填回车位置 ✓
    setB(null);
    setBlocked([]);
    setSelection(aLocked ? "b" : "a");
    setBlockMode(false);
    setMessage("等待选择 A/B");
  };
  const swap = () => {
    invalidate();
    setA(b);
    setB(a);
    setSelection(null);
    setMessage("A/B 已交换，请重新规划。");
  };
  const valid = !!result?.plan;
  const bridgeActive =
    compatibleBridge && bridge !== null && isActivePhase(bridge.phase);
  // ★ 覆盖式发送 ✓（业主口径：车在跑时再发新路线 ⇒ **直接替换**，✗ 不要 409 ✓）
  //   ★ 做法选在**前端**（✗ 不在他们的 `bridge.py` ✗）—— 因为那份在 `.external/` 里，
  //     ✗ 不进交付 ✗；改成"先 `/api/cancel`、等它到终态、再发新的"就全是本仓的代码 ✓
  //   ★ 判据落在 **phase 上** ✓（✗ 不落在"我发过那条取消"✗ —— 那只是"发出信号"✓）
  const replaceActiveRoute = async () => {
    try {
      setBridge(await cancelSimulationRoute());
    } catch {
      // 桥在 phase=canceling 时会对 /api/cancel 回 409 ⇒ 忽略，直接去等终态 ✓
    }
    const deadline = Date.now() + 15000;
    for (;;) {
      const status = await bridgeStatus();
      setBridge(status);
      if (!isActivePhase(status.phase)) return;
      if (Date.now() > deadline)
        throw new Error(`取消旧路线超时（phase=${status.phase}）`);
      await new Promise((resolve) => setTimeout(resolve, 300));
    }
  };
  const sendToNav2 = async () => {
    if (!a || !b || !valid || sending) return;
    setSending(true);
    try {
      setBridgeError("");
      // ★ 发之前现取一次 pose：★ 路径起点必须是**车此刻**的位置 ✓
      //   （覆盖那一路尤其要紧 —— 车在跑，1 s 轮询拿到的 A 可能已经落后好几米 ✓）
      const latest = await bridgeStatus();
      setBridge(latest);
      const startSnap = latest.pose
        ? snap(mapToSvg({ x: latest.pose.x, y: latest.pose.y }))
        : null;
      const start = startSnap ? startSnap.point : a;
      const next = planAB(start, b, blocked);
      if (next.error || !next.plan) throw new Error(next.error ?? "规划失败");
      setA(start);
      setResult(next);
      setPreview(null);
      // ★ 车离线 > 0.1 m ⇒ 把**车的真实位姿**接在 path／waypoints 最前面 ✓
      //   （✗ 不接 ⇒ 车到路径最近点超过控制器的搜索窗口 ⇒ `0 poses` ⇒ 立刻中止 ✓）
      const payload = withCarStart(exportAB(start, b, next), latest.pose ?? null);
      // ★ 旧路线还在跑 ⇒ 先取消并等终态，再发新的 ✓（否则桥回 409 ✓）
      if (isActivePhase(latest.phase)) await replaceActiveRoute();
      setBridge(await sendSimulationRoute(payload));
    } catch (error) {
      setBridgeError(error instanceof Error ? error.message : "发送失败");
    } finally {
      setSending(false);
    }
  };
  const cancelNav2 = async () => {
    try {
      setBridgeError("");
      setBridge(await cancelSimulationRoute());
    } catch (error) {
      setBridgeError(error instanceof Error ? error.message : "取消失败");
    }
  };
  const state: Simulation = preview ?? {
    ...initial(
      b ? [{ id: "ab-b", name: "B 点 / 终点", point: b, dwell: 0 }] : [],
      a ?? home,
      false,
    ),
    blocked,
    plan: result?.plan ?? null,
    status: result?.error ? "blocked" : valid ? "ready" : "idle",
    position: a ?? home,
  };
  const overlay = (markerScale: number) => (
    <g pointerEvents="none">
      {compatibleBridge && bridge?.pose && (
        <g
          transform={`translate(${mapToSvg(bridge.pose).x} ${mapToSvg(bridge.pose).y}) scale(${markerScale})`}
          data-testid="ros-robot-pose"
        >
          <circle r="9" fill="#6cb5e7" stroke="#e7f7ff" strokeWidth="2" />
          <circle r="2" fill="#09202d" />
        </g>
      )}
      {showKeys &&
        result?.waypoints.filter((w) => !w.reasons.includes("start") && !w.reasons.includes("goal")).map((w) => (
          <g
            key={w.distance}
            transform={`translate(${w.point.x} ${w.point.y}) scale(${markerScale})`}
            data-testid="important-waypoint"
          >
            <circle r={w.reasons.every((r) => r === "geometry") ? 2 : 4} fill="#101e19" stroke="#f3d999" strokeWidth="1.5" />
            {w.reasons.some((r) => ["junction", "turn", "spacing"].includes(r)) && (!a || dist(w.point, a) > markerScale * 28) && (!b || dist(w.point, b) > markerScale * 28) && <text x="8" y="-8" className="key-label">K{result!.waypoints.indexOf(w) + 1}</text>}
          </g>
        ))}
      {a && (
        <g
          transform={`translate(${a.x} ${a.y}) scale(${markerScale})`}
          data-testid="ab-a"
          data-x={a.x}
          data-y={a.y}
        >
          <circle r="12" fill="#afd899" stroke="#e6ffd5" strokeWidth="2" />
          <text
            textAnchor="middle"
            y="5"
            fill="#122918"
            fontSize="14"
            fontWeight="700"
          >
            A
          </text>
        </g>
      )}
      {b && (
        <g
          transform={`translate(${b.x} ${b.y}) scale(${markerScale})`}
          data-testid="ab-b"
          data-x={b.x}
          data-y={b.y}
        >
          <circle r="12" fill="#e8c989" stroke="#fff0c6" strokeWidth="2" />
          <text
            textAnchor="middle"
            y="5"
            fill="#3a2c13"
            fontSize="14"
            fontWeight="700"
          >
            B
          </text>
        </g>
      )}
    </g>
  );
  const exportJSON = () => {
    if (a && b && result)
      download(
        "ab-route.json",
        JSON.stringify(exportAB(a, b, result), null, 2),
      );
  };
  const exportCSV = () => {
    if (result)
      download(
        "ab-waypoints.csv",
        waypointCSV(serializeWaypoints(result.waypoints)),
        "text/csv;charset=utf-8",
      );
  };
  return (
    <>
      <main className="workspace ab-workspace">
        <aside className="left-panel panel ab-controls">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">POINT TO POINT</span>
              <h2>A → B 路线规划</h2>
            </div>
            <Route size={20} className="mint" />
          </div>
          <p className="ab-description">
            智慧谷北区 · 环线与中央支路
          </p>
          {aLocked ? (
            <div
              className="endpoint-card locked"
              data-testid="select-a"
              data-locked="true"
            >
              <i>A</i>
              <span>
                <strong>起点 · 小车当前位置（只读）</strong>
                <small>{coordinate(a)}</small>
              </span>
              <LocateFixed size={15} />
            </div>
          ) : (
            <button
              className={`endpoint-card ${selection === "a" ? "selected" : ""}`}
              data-testid="select-a"
              data-locked="false"
              onClick={() => {
                setSelection("a");
                setBlockMode(false);
              }}
            >
              <i>A</i>
              <span>
                <strong>设置起点</strong>
                <small>{coordinate(a)}</small>
              </span>
              <LocateFixed size={15} />
            </button>
          )}
          <small className={`ab-a-source ${aSource}`} data-testid="a-source">
            {aSource === "pose" &&
              `起点＝小车当前位置在路网上的投影（离车 ${poseOffsetM!.toFixed(2)} m）` +
                (poseOffsetM! > 0.1
                  ? "；发路线时会先接一段「车 → 起点」的引导段"
                  : "")}
            {aSource === "offline" &&
              "ROS2 桥接未连接 ⇒ 起点改用离线手动选点（兜底；发送仿真路径不可用）"}
          </small>
          <button
            className="swap-points"
            disabled={aLocked || !a || !b}
            title={aLocked ? "起点锁定为小车当前位置，不能交换" : undefined}
            onClick={swap}
          >
            <ArrowRightLeft size={13} />
            交换 A / B
          </button>
          <button
            className={`endpoint-card goal ${selection === "b" ? "selected" : ""}`}
            data-testid="select-b"
            onClick={() => {
              setSelection("b");
              setBlockMode(false);
            }}
          >
            <i>B</i>
            <span>
              <strong>设置终点</strong>
              <small>{coordinate(b)}</small>
            </span>
            <Flag size={15} />
          </button>
          <button
            className="primary ab-plan"
            disabled={!a || !b || busy}
            onClick={compute}
            data-testid="plan-ab"
          >
            <Route size={16} />
            {busy ? "规划中…" : "规划 A→B"}
            <ArrowRight size={16} />
          </button>
          <div className="ab-bridge">
            <span className="eyebrow">NAV2 / 仿真</span>
            <div className="ab-bridge-actions">
              <button
                disabled={
                  !valid ||
                  !compatibleBridge ||
                  sending ||
                  result!.plan!.length === 0
                }
                title={bridgeActive ? "直接替换当前正在跑的路线" : undefined}
                onClick={sendToNav2}
              >
                <Send size={14} />
                发送仿真路径
              </button>
              <button title="取消 Nav2 仿真" disabled={!bridgeActive} onClick={cancelNav2}>
                <Square size={14} />
              </button>
            </div>
            <small role="status" data-testid="bridge-status">
              {bridgeError || (bridge ? !compatibleBridge ? "地图版本不一致，请更新 ROS2 地图并重启桥接" : bridge.message : "ROS2 桥接未连接")}
              {!bridgeError && compatibleBridge && bridge?.distance_remaining != null && <span style={{ whiteSpace: "nowrap" }}> · 剩余 {bridge.distance_remaining.toFixed(1)} m</span>}
              {!bridgeError && bridgeActive && <span> · 再次发送将直接替换当前路线</span>}
              {sending && <span> · 发送中…</span>}
            </small>
            {compatibleBridge && bridge?.pose && <small>仿真位置 {bridge.pose.x.toFixed(2)}, {bridge.pose.y.toFixed(2)} m</small>}
          </div>
          <div className="ab-model">
            <span className="eyebrow">实测路网</span>
            <dl>
              <div>
                <dt>环线</dt>
                <dd>{roads.filter((r) => r.id !== "branch").reduce((sum, r) => sum + r.length_m, 0).toFixed(1)} m</dd>
              </div>
              <div>
                <dt>中央支路</dt>
                <dd>{roads.find((r) => r.id === "branch")!.length_m.toFixed(1)} m</dd>
              </div>
              <div>
                <dt>采集解状态</dt>
                <dd>DGPS · 质量 2</dd>
              </div>
            </dl>
            <small>绝对位置精度与道路宽度未验证。</small>
          </div>
          <div className="ab-export">
            <button disabled={!valid} onClick={exportJSON}>
              <Download size={14} />
              路线 + 途经点 JSON
            </button>
            <button disabled={!valid} onClick={exportCSV}>
              <Download size={14} />
              重要途经点 CSV
            </button>
            <button
              onClick={() =>
                download(
                  "campus-map.json",
                  JSON.stringify(exportMap(blocked), null, 2),
                )
              }
            >
              <Download size={14} />
              地图 JSON
            </button>
          </div>
        </aside>
        <section className="map-panel panel">
          <div className="map-toolbar">
            <div className="map-tabs">
              <button
                className={layers.route ? "active" : ""}
                onClick={() => setLayers((l) => ({ ...l, route: !l.route }))}
              >
                规划路线
              </button>
              <button
                className={showKeys ? "active" : ""}
                onClick={() => setShowKeys(!showKeys)}
              >
                重要途经点
              </button>
              <button
                className={showSurvey ? "active" : ""}
                onClick={() => setShowSurvey(!showSurvey)}
              >
                原始 / 滤波
              </button>
            </div>
            <div className="map-tools">
              <button title="显示园区底图" aria-pressed={layers.basemap}
                className={layers.basemap ? "active" : ""}
                onClick={() => setLayers((l) => ({ ...l, basemap: !l.basemap }))}>
                <MapIcon size={16} />
              </button>
              <button
                title="A/B 道路封闭管理"
                className={blockMode ? "active warning" : ""}
                onClick={() => {
                  setBlockMode(!blockMode);
                  setSelection(null);
                }}
              >
                <Ban size={16} />
              </button>
              <button
                title="显示道路网络"
                className={layers.network ? "active" : ""}
                onClick={() =>
                  setLayers((l) => ({ ...l, network: !l.network }))
                }
              >
                <Crosshair size={16} />
              </button>
            </div>
          </div>
          <MapView
            state={state}
            layers={layers}
            mode={
              (blockMode
                ? "block"
                : selection === "a"
                  ? "start"
                  : selection === "b"
                    ? "target"
                    : "inspect") as Mode
            }
            reference={null}
            onPick={pick}
            onRoad={(id) =>
              blockMode
                ? closeRoad(id)
                : setMessage(
                    `${roads.find((r) => r.id === id)?.name} · 实测中心线`,
                  )
            }
            onInfo={setMessage}
            showSurvey={showSurvey}
            overlay={overlay}
            hideRobot={!preview}
            hideHome
            hideTasks
          />
          {blockMode && (
            <div className="floating-panel road-panel">
              <div className="floating-title">
                道路通行管理
                <button
                  onClick={() => setBlockMode(false)}
                  aria-label="关闭 A/B 道路管理"
                >
                  <X size={14} />
                </button>
              </div>
              <p>封路会清除旧路线，保留 A/B；重新规划后才允许导出。</p>
              <div className="road-list">
                {roads.map((r) => (
                  <button
                    key={r.id}
                    data-testid={`ab-toggle-${r.id}`}
                    className={blocked.includes(r.id) ? "closed" : ""}
                    onClick={() => closeRoad(r.id)}
                  >
                    <span>{r.name}</span>
                    <span>{blocked.includes(r.id) ? "已封闭" : "开放"}</span>
                  </button>
                ))}
              </div>
            </div>
          )}
          {preview?.status === "completed" && (
            <div className="ab-arrived">
              <Check size={16} />
              已到达 B 点
            </div>
          )}
          <div
            className={`ab-message ${result?.error ? "error" : ""}`}
            role="status"
            data-testid="ab-message"
          >
            {message}
          </div>
        </section>
        <aside className="right-panel panel ab-results">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">ROUTE / 规划结果</span>
              <h2>重要途经点</h2>
            </div>
            <span className="small-badge">{result?.waypoints.length ?? 0}</span>
          </div>
          <div className="ab-stats">
            <div>
              <small>路线长度</small>
              <strong>
                {valid ? (result!.plan!.length * scale).toFixed(1) : "—"}
                <span>m</span>
              </strong>
            </div>
            <div>
              <small>完整路径顶点</small>
              <strong>
                {result?.points.length ?? "—"}
                <span>个</span>
              </strong>
            </div>
          </div>
          <div className="keypoint-list" data-testid="keypoint-list">
            {result?.waypoints.map((w, i) => (
              <div className="keypoint-row" key={i}>
                <b>K{i + 1}</b>
                <div>
                  <strong>
                    {w.reasons.map((r) => reasonNames[r]).join(" · ")}
                  </strong>
                  <span>{coordinate(w.point)} m</span>
                  <small>
                    朝向 {((w.yaw * 180) / Math.PI).toFixed(0)}° · 累计{" "}
                    {(w.distance * scale).toFixed(1)} m
                  </small>
                </div>
              </div>
            ))}
            {!valid && (
              <div className="empty-tasks">
                选择 A/B 并规划后，在这里查看关键点的坐标、类型与参考朝向。
              </div>
            )}
          </div>
          <div className="ab-calibration">
            <strong>地图可达 ≠ 现场已验证</strong>
            <p>
              两段差分解轨迹；路口连接已确认。绝对位置精度尚未独立验证。
            </p>
          </div>
        </aside>
      </main>
      <footer className="playback-bar">
        <div className="playback-controls">
          <button
            className="play-button"
            title={
              preview?.status === "running" ? "暂停 A/B 预览" : "播放 A/B 预览"
            }
            disabled={!valid}
            onClick={() => {
              if (preview?.status === "running")
                setPreview({ ...preview, status: "paused" });
              else if (preview?.status === "paused")
                setPreview({ ...preview, status: "running" });
              else if (a && b && result?.plan)
                setPreview({
                  ...initial(
                    [{ id: "ab-b", name: "B 点", point: b, dwell: 0 }],
                    a,
                    false,
                  ),
                  plan: result.plan,
                  blocked,
                  status: "running",
                });
            }}
          >
            {preview?.status === "running" ? (
              <Pause size={18} />
            ) : (
              <Play size={18} />
            )}
          </button>
          <button className="reset-button" title="重置 A/B" onClick={reset}>
            <RotateCcw size={17} />
          </button>
          <div className="playback-label">
            <strong>
              {preview?.status === "completed"
                ? "已到达 B 点"
                : preview?.status === "running"
                  ? "沿路线预览"
                  : valid
                    ? "A→B 路线就绪"
                    : "等待选择 A/B"}
            </strong>
            <span>模拟预览 · 5×</span>
          </div>
        </div>
        <div className="ab-footer-summary">
          {valid
            ? `${result!.waypoints.length} 个重要途经点 · 中心线路径`
            : "环线 + 中央支路"}
        </div>
        <div className="footer-note">
          <span className="live-dot" />
          DGPS 实测 · 绝对精度未验证
        </div>
      </footer>
    </>
  );
}
