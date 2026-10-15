import {
  Download,
  Flag,
  LocateFixed,
  Play,
  RotateCcw,
  Route,
} from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { MapView, type Layers } from "./MapView";
import { exportAB, planAB, type ABResult } from "./abNavigation";
import { NavigationPanel } from "./components/NavigationPanel";
import {
  ObstaclePanel,
  type ObstacleSelection,
} from "./components/ObstaclePanel";
import { useObstacles } from "./hooks/useObstacles";
import { download, exportMap } from "./export";
import { useNavigation } from "./hooks/useNavigation";
import { dist, home, mapToSvg, pathD, roads, scale, toMeters } from "./map";
import { compatible, previewRoute, type RoutePreview } from "./navigationApi";
import { snap } from "./planner";
import { advance, initial } from "./simulation";
import type { Point, Simulation } from "./types";
import { extractWaypoints, serializeWaypoints, waypointCSV } from "./waypoints";

const defaultLayers: Layers = {
  basemap: true,
  semantic: true,
  access: false,
  network: false,
  route: true,
  reference: false,
};
const coordinate = (p: Point | null) =>
  p
    ? `${toMeters(p).x.toFixed(2)}, ${toMeters(p).y.toFixed(2)} m`
    : "点击地图选择";

function referenceResult(route: RoutePreview): ABResult {
  const points = route.reference_path.map(mapToSvg);
  let distance = 0;
  const segments = points.slice(1).map((b, i) => {
    const a = points[i],
      start = distance;
    distance += dist(a, b);
    return { a, b, start, end: distance, edgeId: route.segment_roads[i] };
  });
  return {
    points,
    waypoints: extractWaypoints(points),
    error: null,
    blocked: [],
    plan: {
      segments,
      length: distance,
      start: points[0],
      taskOrder: ["ab-b"],
      stops: [
        { id: "ab-b", name: "B 点", point: points.at(-1)!, dwell: 0, distance },
      ],
    },
  };
}

export function ABWorkspace() {
  const navigation = useNavigation();
  const [a, setA] = useState<Point | null>(null),
    [b, setB] = useState<Point | null>(null);
  const [selection, setSelection] = useState<"a" | "b">("a");
  const [blocked, setBlocked] = useState<string[]>([]);
  const [blockMode, setBlockMode] = useState(false),
    [busy, setBusy] = useState(false);
  const [result, setResult] = useState<ABResult | null>(null);
  const [message, setMessage] = useState(
    "选择目标；在线时 A 自动使用机器人位置",
  );
  const [layers, setLayers] = useState(defaultLayers),
    [showSurvey, setShowSurvey] = useState(false);
  const [preview, setPreview] = useState<Simulation | null>(null);
  const generation = useRef(0);
  const online =
    compatible(navigation.status) &&
    navigation.status!.ready &&
    !!navigation.status!.pose;
  const pose = online ? navigation.status!.pose : null;
  const obstacles = useObstacles(online);
  const [obstacleSelection, setObstacleSelection] =
    useState<ObstacleSelection>(null);
  const [obstacleStart, setObstacleStart] = useState<Point | null>(null);
  const [obstacleEnd, setObstacleEnd] = useState<Point | null>(null);
  const liveStart = pose ? snap(mapToSvg(pose))?.point : null;
  useEffect(() => {
    if (liveStart) {
      setA(liveStart);
      setSelection("b");
    }
  }, [liveStart?.x, liveStart?.y]);
  useEffect(() => {
    let frame: number,
      last = performance.now();
    const tick = (now: number) => {
      const dt = Math.min((now - last) / 1000, 0.25);
      last = now;
      setPreview((s) => (s ? advance(s, dt * 5) : s));
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => {
      cancelAnimationFrame(frame);
      generation.current++;
    };
  }, []);
  const invalidate = () => {
    generation.current++;
    setResult(null);
    setPreview(null);
    setBusy(false);
  };
  const pick = (point: Point, tolerance: number) => {
    if (obstacleSelection) {
      if (obstacleSelection === "start") setObstacleStart(point);
      else setObstacleEnd(point);
      setObstacleSelection(null);
      setMessage("障碍物位置已选择，可在右侧添加到 Gazebo");
      return;
    }
    if (blockMode) return;
    const projected = snap(point),
      open = projected && snap(projected.point, blocked);
    if (
      !projected ||
      projected.distance > tolerance ||
      !open ||
      open.distance > 1e-6
    ) {
      setMessage("该位置不在开放路线附近");
      return;
    }
    invalidate();
    if (!online && selection === "a") {
      setA(projected.point);
      setSelection("b");
    } else setB(projected.point);
    setMessage("目标已选择，请规划路线");
  };
  const compute = async () => {
    if (!a || !b) return;
    const token = ++generation.current;
    setBusy(true);
    try {
      const next = online
        ? referenceResult(await previewRoute(b, blocked))
        : planAB(a, b, blocked);
      if (generation.current !== token) return;
      next.blocked = [...blocked];
      setResult(next);
      setPreview(null);
      setMessage(
        next.error ??
          `路线已生成 · ${next.waypoints.length} 个重要途经点 · ${online ? "后端参考路线" : "离线预览"}`,
      );
    } catch (error) {
      if (generation.current === token)
        setMessage(error instanceof Error ? error.message : "规划失败");
    } finally {
      if (generation.current === token) setBusy(false);
    }
  };
  const closeRoad = (id: string) => {
    invalidate();
    setBlocked((ids) =>
      ids.includes(id) ? ids.filter((v) => v !== id) : [...ids, id],
    );
    setMessage("道路已改变，请重新规划；执行中的任务由后端管理");
  };
  const playback = () => {
    if (!result?.plan || !b) return;
    setPreview({
      ...initial(
        [{ id: "ab-b", name: "B 点", point: b, dwell: 0 }],
        result.plan.start,
        false,
      ),
      blocked,
      plan: result.plan,
      status: "running",
    });
  };
  const state: Simulation = preview ?? {
    ...initial([], a ?? home, false),
    blocked,
    plan: result?.plan ?? null,
    status: result?.error ? "blocked" : result?.plan ? "ready" : "idle",
    position: a ?? home,
  };
  const execution = compatible(navigation.status)
    ? (navigation.status?.task?.execution_path ?? [])
    : [];
  const overlay = (markerScale: number) => (
    <g pointerEvents="none">
      {obstacles.items.map((item) => {
        const center = mapToSvg(item.position);
        return (
          <g key={item.name} data-testid="scene-obstacle">
            {item.kind === "moving" && (
              <path
                d={pathD([mapToSvg(item.start), mapToSvg(item.end)])}
                fill="none"
                stroke="#ff994d"
                strokeDasharray="4 3"
                vectorEffect="non-scaling-stroke"
              />
            )}
            <rect
              x={center.x - item.width / scale / 2}
              y={center.y - item.depth / scale / 2}
              width={item.width / scale}
              height={item.depth / scale}
              fill="#ff994d"
              stroke="#ffc999"
              vectorEffect="non-scaling-stroke"
            />
          </g>
        );
      })}
      {[obstacleStart, obstacleEnd].map(
        (p, i) =>
          p && (
            <g
              key={`obstacle-${i}`}
              transform={`translate(${p.x} ${p.y}) scale(${markerScale})`}
            >
              <circle r="7" fill="#ff994d" />
              <text y="-10" textAnchor="middle" fill="#ffb779">
                {i ? "移动终点" : "障碍位置"}
              </text>
            </g>
          ),
      )}
      {execution.length > 1 && (
        <path
          data-testid="execution-route"
          d={pathD(execution.map(mapToSvg))}
          fill="none"
          stroke="#67b9ff"
          strokeWidth="3"
          strokeDasharray="6 3"
          vectorEffect="non-scaling-stroke"
        />
      )}
      {[a, b].map(
        (p, i) =>
          p && (
            <g
              key={i}
              data-testid={i ? "ab-b" : "ab-a"}
              data-x={p.x}
              data-y={p.y}
              transform={`translate(${p.x} ${p.y}) scale(${markerScale})`}
            >
              <circle r="12" fill={i ? "#e8c989" : "#afd899"} />
              <text y="5" textAnchor="middle" fill="#122918">
                {i ? "B" : "A"}
              </text>
            </g>
          ),
      )}
      {pose && (
        <g
          data-testid="navigation-robot"
          transform={`translate(${mapToSvg(pose).x} ${mapToSvg(pose).y}) scale(${markerScale})`}
        >
          <circle r="8" fill="#67b9ff" />
          <path
            d="M-3 -4L5 0L-3 4Z"
            fill="#09202d"
            transform={`rotate(${(-pose.yaw * 180) / Math.PI})`}
          />
        </g>
      )}
    </g>
  );
  return (
    <main className="workspace ab-workspace">
      <aside className="left-panel panel ab-controls">
        <div className="panel-heading">
          <h2>A → B 路线规划</h2>
          <Route size={20} />
        </div>
        <p>
          {online
            ? "在线导航 · A 使用机器人当前位置"
            : "离线模式 · 动画不驱动 Gazebo"}
        </p>
        <button
          className="endpoint-card"
          data-testid="select-a"
          disabled={online}
          onClick={() => {
            setSelection("a");
            setObstacleSelection(null);
            setBlockMode(false);
          }}
        >
          <LocateFixed size={16} />
          <span>
            <strong>{online ? "当前位置" : "设置起点"}</strong>
            <small>{coordinate(a)}</small>
          </span>
        </button>
        <button
          className="endpoint-card goal"
          data-testid="select-b"
          onClick={() => {
            setSelection("b");
            setObstacleSelection(null);
            setBlockMode(false);
          }}
        >
          <Flag size={16} />
          <span>
            <strong>设置终点</strong>
            <small>{coordinate(b)}</small>
          </span>
        </button>
        <button
          className="primary ab-plan"
          data-testid="plan-ab"
          disabled={!a || !b || busy}
          onClick={() => void compute()}
        >
          <Route size={16} />
          {busy ? "规划中…" : "规划 A→B"}
        </button>
        <NavigationPanel
          status={navigation.status}
          error={navigation.error}
          busy={navigation.busy}
          canExecute={!!b && !!result?.plan && result.plan.length * scale > 0}
          onExecute={() => b && void navigation.execute(b, blocked)}
          onCancel={() => void navigation.cancel()}
        />
        <button
          onClick={() => setBlockMode((v) => !v)}
          className={blockMode ? "active" : ""}
        >
          封闭或恢复道路
        </button>
        {roads.map((r) => (
          <button
            key={r.id}
            data-testid={`toggle-${r.id}`}
            onClick={() => closeRoad(r.id)}
          >
            {blocked.includes(r.id) ? "恢复" : "封闭"}
            {r.name}
          </button>
        ))}
        <button
          onClick={() => {
            invalidate();
            setB(null);
            if (!online) setA(null);
            setSelection(online ? "b" : "a");
            setBlocked([]);
          }}
        >
          <RotateCcw size={15} />
          重置选点
        </button>
      </aside>
      <section className="map-panel panel">
        <div className="map-toolbar">
          <button onClick={() => setShowSurvey((v) => !v)}>原始 / 滤波</button>
          <button
            onClick={() => setLayers((v) => ({ ...v, network: !v.network }))}
          >
            显示道路网络
          </button>
          <button
            onClick={() => setLayers((v) => ({ ...v, basemap: !v.basemap }))}
          >
            显示园区底图
          </button>
        </div>
        <MapView
          state={state}
          layers={layers}
          mode={blockMode ? "block" : "target"}
          reference={null}
          onPick={pick}
          onRoad={closeRoad}
          onInfo={setMessage}
          overlay={overlay}
          showSurvey={showSurvey}
          hideRobot={!preview}
          hideHome
          hideTasks
        />
        <p data-testid="ab-message">{message}</p>
        {preview?.status === "completed" && (
          <p className="ab-arrived">离线预览已到达</p>
        )}
      </section>
      <aside className="right-panel panel">
        <ObstaclePanel
          enabled={online}
          obstacles={obstacles}
          start={obstacleStart}
          end={obstacleEnd}
          selection={obstacleSelection}
          select={(value) => {
            setObstacleSelection(value);
            setBlockMode(false);
          }}
          clear={() => {
            setObstacleSelection(null);
            setObstacleStart(null);
            setObstacleEnd(null);
          }}
        />
        <h3>路线与导出</h3>
        <p>绿色：参考路线 · 蓝色：Nav2 执行路径</p>
        <p>
          {result?.plan
            ? `${(result.plan.length * scale).toFixed(1)} m · ${result.waypoints.length} 个途经点`
            : "尚未规划"}
        </p>
        <button
          title="播放 A/B 预览"
          disabled={!result?.plan}
          onClick={playback}
        >
          <Play size={15} />
          离线动画预览
        </button>
        <button
          disabled={!result?.plan}
          onClick={() =>
            result &&
            b &&
            download(
              "route.json",
              JSON.stringify(exportAB(result.points[0], b, result), null, 2),
            )
          }
        >
          <Download size={15} />
          路线 JSON
        </button>
        <button
          disabled={!result?.plan}
          onClick={() =>
            result &&
            download(
              "waypoints.csv",
              waypointCSV(serializeWaypoints(result.waypoints)),
              "text/csv;charset=utf-8",
            )
          }
        >
          途经点 CSV
        </button>
        <button
          onClick={() =>
            download("map.json", JSON.stringify(exportMap(blocked), null, 2))
          }
        >
          地图 JSON
        </button>
      </aside>
    </main>
  );
}
