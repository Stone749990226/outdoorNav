import { useEffect, useMemo, useRef, useState } from "react";
import {
  ArrowRight,
  ArrowRightLeft,
  Ban,
  Check,
  Crosshair,
  Download,
  Flag,
  LocateFixed,
  Pause,
  Play,
  RotateCcw,
  Route,
  X,
} from "lucide-react";
import { MapView, type Layers, type Mode } from "./MapView";
import { freeGrid, isTraversable, navigation } from "./freeSpace";
import { exportAB, planAB, type ABResult } from "./abNavigation";
import { download, exportMap } from "./export";
import { home, scale, toMeters, roads } from "./map";
import { advance, initial } from "./simulation";
import { reasonNames, serializeWaypoints, waypointCSV } from "./waypoints";
import type { Point, Simulation } from "./types";
const emptyLayers: Layers = {
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
export function ABWorkspace() {
  const [a, setA] = useState<Point | null>(null),
    [b, setB] = useState<Point | null>(null);
  const [selection, setSelection] = useState<"a" | "b" | null>("a");
  const [blocked, setBlocked] = useState<string[]>([]),
    [blockMode, setBlockMode] = useState(false);
  const [result, setResult] = useState<ABResult | null>(null),
    [message, setMessage] = useState("选择亮绿色区域内的 A 点，再选择 B 点。");
  const [layers, setLayers] = useState(emptyLayers),
    [showArea, setShowArea] = useState(true),
    [showKeys, setShowKeys] = useState(true);
  const [preview, setPreview] = useState<Simulation | null>(null);
  const [busy, setBusy] = useState(false);
  const generation = useRef(0);
  const grid = useMemo(() => freeGrid(blocked), [blocked]);
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
  const invalidate = () => {
    generation.current++;
    setResult(null);
    setPreview(null);
    setBusy(false);
  };
  const pick = (p: Point) => {
    if (!selection || blockMode) return;
    if (!isTraversable(p, blocked)) {
      setMessage(
        "该位置不满足通行条件：请选择亮绿色区域内侧，避开建筑、未知区域与封路。",
      );
      return;
    }
    invalidate();
    if (selection === "a") {
      setA(p);
      setSelection("b");
      setMessage("A 点已保留在点击位置。现在选择 B 点。");
    } else {
      setB(p);
      setSelection(null);
      setMessage("A、B 已选好，点击「规划 A→B」。点位不会吸附到中心线。");
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
            `路线已生成 · ${next.waypoints.length} 个重要途经点 · 保留原始 A/B 坐标`,
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
    setA(null);
    setB(null);
    setBlocked([]);
    setSelection("a");
    setBlockMode(false);
    setMessage("选择亮绿色区域内的 A 点，再选择 B 点。");
  };
  const swap = () => {
    invalidate();
    setA(b);
    setB(a);
    setSelection(null);
    setMessage("A/B 已交换，请重新规划。");
  };
  const valid = !!result?.plan;
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
  const overlay = (
    <g pointerEvents="none">
      {showKeys &&
        result?.waypoints.map((w, i) => (
          <g
            key={i}
            transform={`translate(${w.point.x} ${w.point.y})`}
            data-testid="important-waypoint"
          >
            <circle r="6" fill="#101e19" stroke="#f3d999" strokeWidth="2" />
            <text x="10" y="-9" className="key-label">
              K{i + 1}
            </text>
          </g>
        ))}
      {a && (
        <g
          transform={`translate(${a.x} ${a.y})`}
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
          transform={`translate(${b.x} ${b.y})`}
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
              <h2>A → B 区域导航</h2>
            </div>
            <Route size={20} className="mint" />
          </div>
          <p className="ab-description">
            在可通行区域内任选起终点。
            <br />
            保留点击位置，不吸附中心线。
          </p>
          <button
            className={`endpoint-card ${selection === "a" ? "selected" : ""}`}
            data-testid="select-a"
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
          <button className="swap-points" disabled={!a || !b} onClick={swap}>
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
          <div className="ab-model">
            <span className="eyebrow">当前通行模型</span>
            <p>有宽度的道路区域，扣除建筑、绿化、未知地面与封闭路段。</p>
            <dl>
              <div>
                <dt>道路假设宽度</dt>
                <dd>{navigation.road_width_units * scale} m*</dd>
              </div>
              <div>
                <dt>圆形包络半径</dt>
                <dd>{navigation.robot_radius_units * scale} m*</dd>
              </div>
              <div>
                <dt>规划栅格大小</dt>
                <dd>{navigation.grid_resolution_units * scale} m*</dd>
              </div>
            </dl>
            <small>* 演示米，参数未实测。亮绿色区域已预留边界余量。</small>
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
                className={showArea ? "active" : ""}
                onClick={() => setShowArea(!showArea)}
              >
                可通行区域
              </button>
              <button
                className={showKeys ? "active" : ""}
                onClick={() => setShowKeys(!showKeys)}
              >
                重要途经点
              </button>
            </div>
            <div className="map-tools">
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
                    `${roads.find((r) => r.id === id)?.name} · 图片推定可通行`,
                  )
            }
            onInfo={setMessage}
            areaMode
            areaOverlay={showArea ? grid.path : undefined}
            overlay={overlay}
            hideRobot={!preview}
            hideHome={!a}
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
          {selection && (
            <div className="mode-hint">
              <Crosshair size={14} />
              点击可通行区域选择 {selection.toUpperCase()} 点
              <button title="退出 A/B 选点" onClick={() => setSelection(null)}>
                <X size={14} />
              </button>
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
                <span>m*</span>
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
          <p className="ab-description">
            保留起终点、路口、≥25° 转弯和最长 20 m*
            间隔；必要时补点，避免简化后切入障碍。
          </p>
          <div className="keypoint-list" data-testid="keypoint-list">
            {result?.waypoints.map((w, i) => (
              <div className="keypoint-row" key={i}>
                <b>K{i + 1}</b>
                <div>
                  <strong>
                    {w.reasons.map((r) => reasonNames[r]).join(" · ")}
                  </strong>
                  <span>{coordinate(w.point)} m*</span>
                  <small>
                    朝向 {((w.yaw * 180) / Math.PI).toFixed(0)}° · 累计{" "}
                    {(w.distance * scale).toFixed(1)} m*
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
              输出为 demo_map 局部坐标。未对齐 RTK，不能作为已验证的实机路线。
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
            ? `${result!.waypoints.length} 个重要途经点 · 相邻关键点连线已做通行校验`
            : "可选择道路宽度范围内的位置，不局限于预置节点"}
        </div>
        <div className="footer-note">
          <span className="live-dot" />
          图片辅助标注 · 尺度未校准
        </div>
      </footer>
    </>
  );
}
