import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { ZoomIn, ZoomOut, Maximize, Compass } from "lucide-react";
import { campus, regions, roads, pathD, scale } from "./map";
import { reachableRoads } from "./planner";
import type { Point, Simulation } from "./types";
export type Layers = {
  semantic: boolean;
  access: boolean;
  network: boolean;
  route: boolean;
  reference: boolean;
};
export type Reference = {
  url: string;
  opacity: number;
  x: number;
  y: number;
  scale: number;
};
export type Mode = "inspect" | "start" | "target" | "block";
type Props = {
  state: Simulation;
  layers: Layers;
  mode: Mode;
  reference: Reference | null;
  onPick: (p: Point) => void;
  onRoad: (id: string) => void;
  onInfo: (text: string) => void;
  areaOverlay?: string;
  overlay?: ReactNode;
  areaMode?: boolean;
  hideRobot?: boolean;
  hideHome?: boolean;
};
const routePoints = (s: NonNullable<Simulation["plan"]>) => [
  s.start,
  ...s.segments.map((seg) => seg.b),
];
const pointsAttr = (ps: Point[]) => ps.map((p) => `${p.x},${p.y}`).join(" ");
const center = (ps: Point[]) => ({
  x: ps.reduce((s, p) => s + p.x, 0) / ps.length,
  y: ps.reduce((s, p) => s + p.y, 0) / ps.length,
});
export function MapView({
  state,
  layers,
  mode,
  reference,
  onPick,
  onRoad,
  onInfo,
  areaOverlay,
  overlay,
  areaMode = false,
  hideRobot = false,
  hideHome = false,
}: Props) {
  const svg = useRef<SVGSVGElement>(null),
    world = useRef<SVGGElement>(null);
  const [view, setView] = useState({ x: 0, y: 0, zoom: 1 });
  const [pixelsPerUnit, setPixelsPerUnit] = useState(1);
  useEffect(() => {
    const element = svg.current!;
    const observer = new ResizeObserver(() => {
      const bounds = element.getBoundingClientRect();
      setPixelsPerUnit(Math.min(bounds.width / 1060, bounds.height / 965));
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, []);
  const drag = useRef<{
    x: number;
    y: number;
    ox: number;
    oy: number;
    moved: boolean;
  } | null>(null);
  const reachable = useMemo(
    () =>
      areaMode
        ? new Set(roads.map((r) => r.id))
        : reachableRoads(state.position, state.blocked),
    [state.blocked, state.position.x, state.position.y, areaMode],
  );
  const zoom = (factor: number) =>
    setView((v) => ({
      ...v,
      zoom: Math.min(3, Math.max(0.7, v.zoom * factor)),
    }));
  const localPoint = (x: number, y: number, group = true) => {
    const point = svg.current!.createSVGPoint();
    point.x = x;
    point.y = y;
    return point.matrixTransform(
      (group ? world.current! : svg.current!).getScreenCTM()!.inverse(),
    );
  };
  return (
    <div className={`map-canvas mode-${mode}`}>
      <svg
        ref={svg}
        data-testid="campus-map"
        className="campus-svg"
        viewBox="-30 -15 1060 965"
        aria-label="园区巡航交互地图"
        onWheel={(e) => zoom(e.deltaY < 0 ? 1.08 : 1 / 1.08)}
        onPointerDown={(e) => {
          if (e.button !== 0) return;
          const p = localPoint(e.clientX, e.clientY, false);
          drag.current = {
            x: p.x,
            y: p.y,
            ox: view.x,
            oy: view.y,
            moved: false,
          };
        }}
        onPointerMove={(e) => {
          const d = drag.current;
          if (!d) return;
          const p = localPoint(e.clientX, e.clientY, false);
          if (Math.hypot(p.x - d.x, p.y - d.y) > 5) {
            d.moved = true;
            svg.current?.setPointerCapture(e.pointerId);
            setView((v) => ({
              ...v,
              x: d.ox + p.x - d.x,
              y: d.oy + p.y - d.y,
            }));
          }
        }}
        onPointerUp={() => {
          setTimeout(() => {
            drag.current = null;
          }, 0);
        }}
        onPointerCancel={() => {
          drag.current = null;
        }}
        onClick={(e) => {
          if (drag.current?.moved) return;
          const p = localPoint(e.clientX, e.clientY);
          onPick({ x: p.x, y: p.y });
        }}
      >
        <defs>
          <pattern
            id="grid"
            width="40"
            height="40"
            patternUnits="userSpaceOnUse"
          >
            <path
              d="M 40 0 L 0 0 0 40"
              fill="none"
              stroke="#8eb1a4"
              strokeOpacity=".045"
              strokeWidth="1"
            />
          </pattern>
          <pattern
            id="unknown"
            width="12"
            height="12"
            patternUnits="userSpaceOnUse"
            patternTransform="rotate(30)"
          >
            <line
              x1="0"
              y1="0"
              x2="0"
              y2="12"
              stroke="#ad9d79"
              strokeWidth="3"
              strokeOpacity=".17"
            />
          </pattern>
          <pattern
            id="roof"
            width="16"
            height="16"
            patternUnits="userSpaceOnUse"
          >
            <path
              d="M 0 16 L 16 16"
              stroke="#9db3b5"
              strokeOpacity=".08"
              strokeWidth="2"
            />
          </pattern>
          <filter id="glow">
            <feGaussianBlur stdDeviation="5" />
          </filter>
          <filter id="robot-shadow">
            <feDropShadow
              dx="0"
              dy="3"
              stdDeviation="5"
              floodColor="#000"
              floodOpacity=".6"
            />
          </filter>
          <radialGradient id="ground">
            <stop stopColor="#1c302d" />
            <stop offset="1" stopColor="#152321" />
          </radialGradient>
        </defs>
        <g
          ref={world}
          transform={`translate(${500 + view.x} ${460 + view.y}) scale(${view.zoom}) translate(-500 -460)`}
        >
          <rect x="-1500" y="-1500" width="4000" height="4000" fill="#101c1d" />
          <rect
            x="-1500"
            y="-1500"
            width="4000"
            height="4000"
            fill="url(#grid)"
          />
          <path
            d="M 62 357 L 908 87 L 948 905 L 122 911 L 80 670 Z"
            fill="url(#ground)"
            stroke="#526f61"
            strokeOpacity=".3"
            strokeDasharray="6 8"
            strokeWidth="2"
          />
          <path
            d="M-100 361 L 957 23 M946 -40 L952 1010"
            fill="none"
            stroke="#233436"
            strokeWidth="43"
          />
          <path
            d="M-100 361 L 957 23 M946 -40 L952 1010"
            fill="none"
            stroke="#92a69d"
            strokeOpacity=".18"
            strokeWidth="2"
            strokeDasharray="14 14"
          />
          <path
            d="M480 290 L468 190 M100 535 L63 366 M875 875 L950 880"
            fill="none"
            stroke="#354542"
            strokeWidth="16"
          />
          <text
            x="219"
            y="239"
            transform="rotate(-18 219 239)"
            className="external-label"
          >
            丰 二 路
          </text>
          <text
            x="976"
            y="578"
            transform="rotate(90 976 578)"
            className="external-label"
          >
            永 晖 路
          </text>
          {regions
            .filter((r) => r.kind === "green")
            .map((r) => (
              <g key={r.id}>
                <polygon
                  points={pointsAttr(r.points)}
                  fill={layers.semantic ? "#264b3a" : "#243e32"}
                  stroke="#35543d"
                  strokeWidth="1"
                />
                {Array.from({ length: 12 }, (_, i) => {
                  const a = r.points[i % r.points.length],
                    c = center(r.points);
                  const t = 0.3 + (i % 3) * 0.17;
                  return (
                    <g key={i} opacity=".6">
                      <circle
                        cx={a.x + (c.x - a.x) * t}
                        cy={a.y + (c.y - a.y) * t}
                        r={10 + (i % 5)}
                        fill="#365442"
                      />
                      <circle
                        cx={a.x + (c.x - a.x) * t - 2}
                        cy={a.y + (c.y - a.y) * t - 3}
                        r={6 + (i % 5)}
                        fill="#456049"
                      />
                    </g>
                  );
                })}
              </g>
            ))}
          {/* Shadows are visual context, not obstacle classifications. */}
          {regions
            .filter((r) => r.kind === "building")
            .map((r) => (
              <polygon
                key={`shadow-${r.id}`}
                points={pointsAttr(
                  r.points.map((p) => ({ x: p.x - 33, y: p.y - 65 })),
                )}
                fill="#030c0d"
                opacity=".48"
              />
            ))}
          <path
            d="M150 671 L290 620 L404 642 L438 687 L338 702 Z"
            fill="#050e0f"
            opacity=".42"
          />
          <text x="290" y="650" className="shadow-label">
            建筑投影
          </text>
          {regions
            .filter((r) => r.kind === "unknown")
            .map((r) => (
              <polygon
                key={r.id}
                points={pointsAttr(r.points)}
                fill={layers.access ? "url(#unknown)" : "#112121"}
                fillOpacity={layers.access ? 1 : 0.3}
                stroke={layers.access ? "#897e57" : "none"}
                strokeOpacity=".45"
                strokeDasharray="4 5"
              />
            ))}
          {roads.map((r) => (
            <path
              key={`ground-${r.id}`}
              d={pathD(r.points)}
              fill="none"
              stroke="#0d181a"
              strokeWidth="29"
              strokeLinejoin="round"
              strokeLinecap="round"
            />
          ))}
          {roads.map((r) => (
            <path
              key={`road-${r.id}`}
              d={pathD(r.points)}
              fill="none"
              stroke={
                layers.access
                  ? state.blocked.includes(r.id)
                    ? "#b9714d"
                    : reachable.has(r.id)
                      ? "#345f50"
                      : "#775069"
                  : "#52605a"
              }
              strokeWidth={layers.access ? 20 : 17}
              strokeOpacity={layers.access ? 0.9 : 0.65}
              strokeLinejoin="round"
              strokeLinecap="round"
            />
          ))}
          {regions
            .filter((r) => r.kind === "building")
            .map((r) => {
              const c = center(r.points);
              return (
                <g
                  key={r.id}
                  onClick={(e) => {
                    if (mode === "inspect") {
                      e.stopPropagation();
                      onInfo(`${r.name} · 建筑区域 · 禁止穿越`);
                    }
                  }}
                >
                  <polygon
                    points={pointsAttr(
                      r.points.map((p) => ({ x: p.x + 3, y: p.y + 9 })),
                    )}
                    fill="#15232b"
                  />
                  <polygon
                    points={pointsAttr(r.points)}
                    fill={layers.access ? "#543e3c" : "#30464b"}
                    stroke={layers.access ? "#94635a" : "#597278"}
                    strokeWidth="1.5"
                  />
                  <polygon points={pointsAttr(r.points)} fill="url(#roof)" />
                  <polygon
                    points={pointsAttr(
                      r.points.map((p) => ({
                        x: c.x + (p.x - c.x) * 0.84,
                        y: c.y + (p.y - c.y) * 0.76,
                      })),
                    )}
                    fill="none"
                    stroke="#93a9a6"
                    strokeOpacity=".16"
                    strokeWidth="1.5"
                  />
                  {layers.semantic && (
                    <>
                      <text
                        x={c.x}
                        y={c.y - 1}
                        textAnchor="middle"
                        className="building-label"
                      >
                        {r.name}
                      </text>
                      <text
                        x={c.x}
                        y={c.y + 21}
                        textAnchor="middle"
                        className="building-sub"
                      >
                        BUILDING · 建筑
                      </text>
                    </>
                  )}
                </g>
              );
            })}
          {reference && layers.reference && (
            <image
              href={reference.url}
              x={reference.x}
              y={reference.y}
              width={1000 * reference.scale}
              height={920 * reference.scale}
              opacity={reference.opacity}
              preserveAspectRatio="xMidYMid meet"
              pointerEvents="none"
            />
          )}
          {areaOverlay && (
            <path
              data-testid="free-area"
              d={areaOverlay}
              fill="#8bcf98"
              fillOpacity=".36"
              pointerEvents="none"
            />
          )}
          {layers.network && (
            <g opacity=".8">
              {roads.map((r) => (
                <path
                  key={r.id}
                  d={pathD(r.points)}
                  fill="none"
                  stroke="#71af99"
                  strokeWidth="1.5"
                  strokeDasharray="5 8"
                />
              ))}
              {campus.nodes.map((n) => (
                <g key={n.id}>
                  <circle
                    cx={n.x}
                    cy={n.y}
                    r="5"
                    fill="#142b26"
                    stroke="#91bea5"
                    strokeWidth="2"
                  />
                  <text x={n.x + 10} y={n.y - 9} className="node-label">
                    {n.id}
                  </text>
                </g>
              ))}
            </g>
          )}
          {layers.route && state.previousPlan && state.replans > 0 && (
            <path
              data-testid="old-route"
              d={pathD(routePoints(state.previousPlan))}
              fill="none"
              stroke="#e4ac6c"
              strokeWidth="4"
              strokeDasharray="7 9"
              opacity=".6"
            />
          )}
          {layers.route && state.plan && (
            <g pointerEvents="none">
              <path
                d={pathD(routePoints(state.plan))}
                fill="none"
                stroke="#6affba"
                strokeWidth="12"
                opacity=".22"
                filter="url(#glow)"
              />
              <path
                data-testid="planned-route"
                d={pathD(routePoints(state.plan))}
                fill="none"
                stroke="#7dedb6"
                strokeWidth="5"
                strokeLinejoin="round"
                strokeLinecap="round"
              />
              <path
                d={pathD(routePoints(state.plan))}
                className={state.status === "running" ? "flow-line" : ""}
                fill="none"
                stroke="#e0ffde"
                strokeWidth="2"
                strokeDasharray="3 26"
                opacity=".8"
              />
              {state.plan.segments.map((seg, i) => (
                <circle
                  key={i}
                  cx={seg.b.x}
                  cy={seg.b.y}
                  r="3"
                  fill="#c9f2bd"
                />
              ))}
            </g>
          )}
          {layers.route && state.trace.length > 1 && (
            <path
              d={pathD([...state.trace, state.position])}
              fill="none"
              stroke="#5dbbcc"
              strokeWidth="3"
              strokeLinejoin="round"
              pointerEvents="none"
            />
          )}
          {roads.map((r) => (
            <path
              key={`hit-${r.id}`}
              data-testid={`road-${r.id}`}
              d={pathD(r.points)}
              fill="none"
              stroke="transparent"
              strokeWidth="25"
              className="road-hit"
              role="button"
              tabIndex={mode === "block" ? 0 : -1}
              aria-label={`${state.blocked.includes(r.id) ? "恢复" : "封闭"}${r.name}`}
              onClick={(e) => {
                if (drag.current?.moved) return;
                if (mode === "block" || mode === "inspect") {
                  e.stopPropagation();
                  onRoad(r.id);
                }
              }}
              onKeyDown={(e) => {
                if (e.key === "Enter") onRoad(r.id);
              }}
            />
          ))}
          {state.blocked.map((id) => {
            const r = roads.find((r) => r.id === id)!;
            const c = r.points[Math.floor(r.points.length / 2)];
            return (
              <g key={id} pointerEvents="none">
                <path
                  d={pathD(r.points)}
                  stroke="#ef9464"
                  strokeWidth="5"
                  strokeDasharray="9 6"
                  fill="none"
                />
                <g transform={`translate(${c.x} ${c.y})`}>
                  <rect
                    x="-14"
                    y="-14"
                    width="28"
                    height="28"
                    rx="7"
                    fill="#44291e"
                    stroke="#ffa76e"
                  />
                  <path
                    d="M-5 -5 L5 5 M5 -5 L-5 5"
                    stroke="#ffb680"
                    strokeWidth="2.5"
                  />
                </g>
              </g>
            );
          })}
          {!hideHome && (
            <g
              transform={`translate(${state.home.x} ${state.home.y})`}
              pointerEvents="none"
            >
              <circle
                r="24"
                fill="none"
                stroke="#b9e5cc"
                strokeWidth="1"
                strokeDasharray="3 4"
              />
              <text y="-34" textAnchor="middle" className="home-label">
                {areaMode ? "A / 起点" : "起点 / HOME"}
              </text>
            </g>
          )}
          {state.tasks.map((task, i) => {
            const done = state.completed.includes(task.id);
            const active = state.plan?.stops[state.stopIndex]?.id === task.id;
            const left = task.point.x > 750;
            return (
              <g
                key={task.id}
                transform={`translate(${task.point.x} ${task.point.y})`}
                className="task-marker"
                onClick={(e) => {
                  if (mode === "inspect") {
                    e.stopPropagation();
                    onInfo(
                      `${task.name} · 演示巡检点 · 到达后停留 ${task.dwell} 秒`,
                    );
                  }
                }}
              >
                {active && <circle r="25" fill="#7debba" opacity=".1" />}
                <circle
                  r="16"
                  fill={done ? "#79deb0" : "#172e2b"}
                  stroke={active || done ? "#a4f5c8" : "#7eab92"}
                  strokeWidth="2"
                />
                <text
                  y="5"
                  textAnchor="middle"
                  className="marker-number"
                  fill={done ? "#122c25" : "#ddf8e9"}
                >
                  {areaMode ? "B" : done ? "✓" : `${i + 1}`.padStart(2, "0")}
                </text>
                <rect
                  x={left ? -124 : 26}
                  y="-14"
                  width="96"
                  height="27"
                  rx="5"
                  fill="#112421"
                  fillOpacity=".9"
                />
                <text
                  x={left ? -76 : 74}
                  y="4"
                  textAnchor="middle"
                  className="task-label"
                >
                  {task.name.length > 8 ? task.name.slice(0, 8) : task.name}
                </text>
              </g>
            );
          })}
          {overlay}
          {!hideRobot && (
            <g
              data-testid="robot"
              data-x={state.position.x.toFixed(3)}
              data-y={state.position.y.toFixed(3)}
              transform={`translate(${state.position.x} ${state.position.y})`}
              pointerEvents="none"
            >
              <circle r="34" fill="#b6f6a4" opacity=".08" />
              <circle
                r="24"
                fill="#b6f6a4"
                opacity=".12"
                stroke="#bcf6b1"
                strokeOpacity=".3"
              />
              <g
                transform={`rotate(${(state.heading * 180) / Math.PI})`}
                filter="url(#robot-shadow)"
              >
                <path
                  d="M23 -7 L32 0 L23 7"
                  fill="none"
                  stroke="#d6ffb9"
                  strokeWidth="2"
                />
                <rect
                  x="-15"
                  y="-9"
                  width="29"
                  height="18"
                  rx="5"
                  fill="#cefab3"
                  stroke="#eaffdf"
                  strokeWidth="1.5"
                />
                <rect
                  x="9"
                  y="-6"
                  width="9"
                  height="12"
                  rx="3"
                  fill="#edffe2"
                />
                <path
                  d="M-9 -10 L-12 -16 M7 -10 L10 -16 M-9 10 L-12 16 M7 10 L10 16"
                  stroke="#d2f8c1"
                  strokeWidth="3"
                  strokeLinecap="round"
                />
                <path d="M-5 -4 L3 -4 L3 4 L-5 4Z" fill="#496c44" />
              </g>
            </g>
          )}
        </g>
      </svg>
      <div className="map-coordinates">
        <span className="live-dot" /> LOCAL MAP <span>250 × 230 演示米</span>
      </div>
      <div className="compass">
        <Compass size={23} />
        <span>图上方</span>
      </div>
      <div className="zoom-controls">
        <button title="放大地图" onClick={() => zoom(1.2)}>
          <ZoomIn size={17} />
        </button>
        <button title="缩小地图" onClick={() => zoom(1 / 1.2)}>
          <ZoomOut size={17} />
        </button>
        <button
          title="复位视角"
          onClick={() => setView({ x: 0, y: 0, zoom: 1 })}
        >
          <Maximize size={16} />
        </button>
      </div>
      <div className="map-legend">
        <span>
          <i className="legend-line" />
          规划路线
        </span>
        <span>
          <i className="legend-line traveled" />
          已走轨迹
        </span>
        {layers.access && !areaMode && (
          <>
            <span>
              <i className="legend-box unknown" />
              待核实
            </span>
            <span>
              <i className="legend-box unreachable" />
              不可达
            </span>
          </>
        )}
        {state.replans > 0 && (
          <span>
            <i className="legend-line old" />
            上次路线
          </span>
        )}
      </div>
      <div className="scale-bar">
        <div
          style={{ width: `${(20 / scale) * view.zoom * pixelsPerUnit}px` }}
        />
        <span>20 演示米 · 未校准</span>
      </div>
      <div className="map-footnote">
        {areaMode
          ? "亮绿色：可选区域 · 保留点击位置 · 宽度/包络均为演示假设"
          : "图片辅助重绘 / 建筑阴影不等于障碍 / 点位为演示命名"}
      </div>
    </div>
  );
}
