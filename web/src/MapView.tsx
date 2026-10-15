import { Compass, Maximize, ZoomIn, ZoomOut } from "lucide-react";
import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import basemapUrl from "./assets/north-building-basemap.svg";
import { svgOrigin, campus, pathD, roads, scale, survey, toMap } from "./map";
import { reachableRoads } from "./planner";
import type { Point, Simulation } from "./types";

export type Layers = {
  basemap: boolean;
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
  onPick: (p: Point, tolerance: number) => void;
  onRoad: (id: string) => void;
  onInfo: (text: string) => void;
  overlay?: ReactNode | ((markerScale: number) => ReactNode);
  showSurvey?: boolean;
  hideRobot?: boolean;
  hideHome?: boolean;
  hideTasks?: boolean;
};
const planPoints = (plan: NonNullable<Simulation["plan"]>) => [
  plan.start,
  ...plan.segments.map((s) => s.b),
];
const surveySvg = (points: Point[]) =>
  points.map((p) => ({
    x: svgOrigin.x + p.x / scale,
    y: svgOrigin.y - p.y / scale,
  }));

export function MapView({
  state,
  layers,
  mode,
  reference,
  onPick,
  onRoad,
  onInfo,
  overlay,
  showSurvey = false,
  hideRobot = false,
  hideHome = false,
  hideTasks = false,
}: Props) {
  const svg = useRef<SVGSVGElement>(null),
    world = useRef<SVGGElement>(null);
  const [view, setView] = useState({ x: 0, y: 0, zoom: 1 });
  const [pixelsPerUnit, setPixelsPerUnit] = useState(1);
  const width = campus.width,
    height = campus.height;
  useEffect(() => {
    const element = svg.current!;
    const observer = new ResizeObserver(() => {
      const bounds = element.getBoundingClientRect();
      setPixelsPerUnit(Math.min(bounds.width / width, bounds.height / height));
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, [width, height]);
  const drag = useRef<{
    x: number;
    y: number;
    ox: number;
    oy: number;
    moved: boolean;
  } | null>(null);
  const reachable = useMemo(
    () => reachableRoads(state.position, state.blocked),
    [state.position.x, state.position.y, state.blocked],
  );
  const zoom = (factor: number) =>
    setView((v) => ({
      ...v,
      zoom: Math.min(5, Math.max(0.5, v.zoom * factor)),
    }));
  const localPoint = (x: number, y: number, group = true) => {
    const p = svg.current!.createSVGPoint();
    p.x = x;
    p.y = y;
    return p.matrixTransform(
      (group ? world.current! : svg.current!).getScreenCTM()!.inverse(),
    );
  };
  const markerScale = 1 / Math.max(0.001, pixelsPerUnit * view.zoom);
  const grid = 10 / scale;
  const labelTransform = (p: Point) =>
    `translate(${p.x} ${p.y}) scale(${markerScale})`;
  return (
    <div className={`map-canvas mode-${mode}`}>
      <svg
        ref={svg}
        data-testid="campus-map"
        className="campus-svg"
        viewBox={`0 0 ${width} ${height}`}
        aria-label="智慧谷北区实测路网"
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
          if (Math.hypot(p.x - d.x, p.y - d.y) * pixelsPerUnit > 5) {
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
          onPick(
            { x: p.x, y: p.y },
            campus.navigation.pick_tolerance_px * markerScale,
          );
        }}
      >
        <defs>
          <pattern
            id="metric-grid"
            width={grid}
            height={grid}
            x={svgOrigin.x % grid}
            y={svgOrigin.y % grid}
            patternUnits="userSpaceOnUse"
          >
            <path
              d={`M${grid} 0H0V${grid}`}
              fill="none"
              stroke="#707a79"
              strokeOpacity=".18"
              strokeWidth="1"
            />
          </pattern>
        </defs>
        <g
          ref={world}
          transform={`translate(${width / 2 + view.x} ${height / 2 + view.y}) scale(${view.zoom}) translate(${-width / 2} ${-height / 2})`}
        >
          <rect
            x={-width * 3}
            y={-height * 3}
            width={width * 7}
            height={height * 7}
            fill="#1c2223"
          />
          <rect
            x={-width * 3}
            y={-height * 3}
            width={width * 7}
            height={height * 7}
            fill="url(#metric-grid)"
          />
          {/* Artwork is authored in the current survey's SVG frame, below every interactive layer. */}
          {layers.basemap && (
            <image
              data-testid="campus-basemap"
              href={basemapUrl}
              x="0"
              y="0"
              width="976"
              height="952"
              pointerEvents="none"
              aria-label="园区示意底图"
            />
          )}
          {reference && layers.reference && (
            <image
              href={reference.url}
              x={reference.x}
              y={reference.y}
              width={width * reference.scale}
              height={height * reference.scale}
              opacity={reference.opacity}
              preserveAspectRatio="xMidYMid meet"
              pointerEvents="none"
            />
          )}
          {showSurvey && (
            <g pointerEvents="none" data-testid="survey-traces">
              {survey.raw.map((track) => (
                <path
                  key={track.source_log}
                  d={pathD(surveySvg(track.points))}
                  fill="none"
                  stroke="#e7ad6f"
                  strokeWidth="1.5"
                  strokeDasharray="4 4"
                  vectorEffect="non-scaling-stroke"
                  opacity=".7"
                />
              ))}
              {survey.filtered.flatMap((track) =>
                track.segments.map((segment, i) => (
                  <path
                    key={`${track.source_log}-${i}`}
                    d={pathD(surveySvg(segment))}
                    fill="none"
                    stroke="#acb9d9"
                    strokeWidth="1.5"
                    vectorEffect="non-scaling-stroke"
                    opacity=".7"
                  />
                )),
              )}
            </g>
          )}
          {roads.map((road) => (
            <path
              key={road.id}
              data-testid={`centerline-${road.id}`}
              d={pathD(road.points)}
              fill="none"
              stroke={
                state.blocked.includes(road.id)
                  ? "#e6946c"
                  : layers.access && !reachable.has(road.id)
                    ? "#aa8a9d"
                    : road.id === "branch"
                      ? "#83b8de"
                      : "#8abda9"
              }
              strokeWidth="3"
              strokeDasharray={
                state.blocked.includes(road.id) ? "7 5" : undefined
              }
              strokeLinejoin="round"
              strokeLinecap="round"
              vectorEffect="non-scaling-stroke"
            />
          ))}
          {layers.network &&
            campus.nodes.map((node) => (
              <g
                key={node.id}
                transform={labelTransform(node)}
                pointerEvents="none"
              >
                <circle
                  r="4"
                  fill="#20292a"
                  stroke="#d3dfdc"
                  strokeWidth="1.5"
                />
                <text x="8" y="-10" className="node-label">
                  {node.id}
                </text>
              </g>
            ))}
          {layers.route && state.previousPlan && state.replans > 0 && (
            <path
              data-testid="old-route"
              d={pathD(planPoints(state.previousPlan))}
              fill="none"
              stroke="#e4ac6c"
              strokeWidth="2"
              strokeDasharray="7 9"
              opacity=".5"
              vectorEffect="non-scaling-stroke"
              pointerEvents="none"
            />
          )}
          {layers.route && state.plan && (
            <path
              data-testid="planned-route"
              d={pathD(planPoints(state.plan))}
              fill="none"
              stroke="#83efb7"
              strokeWidth="4"
              strokeLinecap="round"
              strokeLinejoin="round"
              vectorEffect="non-scaling-stroke"
              pointerEvents="none"
            />
          )}
          {layers.route && state.trace.length > 1 && (
            <path
              d={pathD([...state.trace, state.position])}
              fill="none"
              stroke="#6bc1d1"
              strokeWidth="2"
              vectorEffect="non-scaling-stroke"
              pointerEvents="none"
            />
          )}
          {roads.map((road) => (
            <path
              key={`hit-${road.id}`}
              data-testid={`road-${road.id}`}
              d={pathD(road.points)}
              fill="none"
              stroke="transparent"
              strokeWidth="24"
              vectorEffect="non-scaling-stroke"
              className="road-hit"
              role="button"
              tabIndex={mode === "block" ? 0 : -1}
              aria-label={`${state.blocked.includes(road.id) ? "恢复" : "封闭"}${road.name}`}
              onClick={(e) => {
                if (drag.current?.moved) return;
                if (mode === "block" || mode === "inspect") {
                  e.stopPropagation();
                  onRoad(road.id);
                }
              }}
              onKeyDown={(e) => {
                if (e.key === "Enter" && mode === "block") onRoad(road.id);
              }}
            />
          ))}
          {!hideHome && (
            <g transform={labelTransform(state.home)} pointerEvents="none">
              <circle
                r="18"
                fill="none"
                stroke="#b9e5cc"
                strokeWidth="1"
                strokeDasharray="3 4"
              />
            </g>
          )}
          {!hideTasks &&
            state.tasks.map((task, i) => (
              <g
                key={task.id}
                transform={labelTransform(task.point)}
                className="task-marker"
                onClick={(e) => {
                  if (mode === "inspect") {
                    e.stopPropagation();
                    onInfo(`${task.name} · 停留 ${task.dwell} 秒`);
                  }
                }}
              >
                <circle
                  r="11"
                  fill={
                    state.completed.includes(task.id) ? "#79deb0" : "#202c2a"
                  }
                  stroke="#a4c6b7"
                  strokeWidth="1.5"
                />
                <text y="4" textAnchor="middle" fill="#e2f5ec" fontSize="11">
                  {i + 1}
                </text>
                {layers.semantic && (
                  <text
                    x={task.point.x > width * 0.7 ? -16 : 16}
                    y="4"
                    textAnchor={task.point.x > width * 0.7 ? "end" : "start"}
                    className="task-label"
                  >
                    {task.name}
                  </text>
                )}
              </g>
            ))}
          {typeof overlay === "function" ? overlay(markerScale) : overlay}
          {!hideRobot && (
            <g
              data-testid="robot"
              data-x={state.position.x.toFixed(3)}
              data-y={state.position.y.toFixed(3)}
              transform={labelTransform(state.position)}
              pointerEvents="none"
            >
              <circle r="10" fill="#e7edda" stroke="#172426" strokeWidth="2" />
              <path
                d="M-3 -4L5 0L-3 4Z"
                fill="#284438"
                transform={`rotate(${(state.heading * 180) / Math.PI})`}
              />
            </g>
          )}
          {layers.network && (
            <g pointerEvents="none">
              {Array.from({ length: Math.ceil(width / grid) }, (_, i) => {
                const x = (svgOrigin.x % grid) + i * grid;
                if (x < grid / 2 || x > width - grid / 2) return null;
                return (
                  <text
                    key={`x${i}`}
                    transform={labelTransform({ x, y: height - 60 })}
                    fill="#879a95"
                    fontSize="10"
                    textAnchor="middle"
                  >
                    {toMap({ x, y: 0 }).x.toFixed(0)}
                  </text>
                );
              })}
            </g>
          )}
        </g>
      </svg>
      <div className="map-coordinates">
        <span className="live-dot" /> DGPS <span>X 东 / Y 北 · 米</span>
      </div>
      <div className="compass">
        <Compass size={23} />
        <span>北 N</span>
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
        {showSurvey && <span style={{ color: "#e7ad6f" }}>原始 / 滤波</span>}
      </div>
      <div className="scale-bar">
        <div style={{ width: (10 / scale) * pixelsPerUnit * view.zoom }} />
        <span>10 m</span>
      </div>
    </div>
  );
}
