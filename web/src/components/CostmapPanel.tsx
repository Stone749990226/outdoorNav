import { useEffect, useRef, useState } from "react";
import { getCostmap, type CostmapSnapshot } from "../navigationApi";
import { mapPoint } from "../costmap";

type View = { x: number; y: number; width: number };
export function CostmapPanel() {
  const [layers, setLayers] = useState<
    Partial<Record<"global" | "local", CostmapSnapshot>>
  >({});
  const [errors, setErrors] = useState("");
  const [enabled, setEnabled] = useState({ global: true, local: true });
  const [follow, setFollow] = useState(true);
  const [view, setView] = useState<View>({ x: 0, y: 0, width: 24 });
  const [aspect, setAspect] = useState(1.6);
  const [elapsed, setElapsed] = useState(0);
  const svg = useRef<SVGSVGElement>(null);
  const viewRef = useRef(view);
  viewRef.current = view;
  const arrived = useRef(performance.now());
  const drag = useRef<{ x: number; y: number; view: View } | null>(null);
  const usable = (d: CostmapSnapshot) =>
    !!d.map_from_frame && (d.frame_id === "map" || d.map_from_frame.age_s < 1);
  const robotLayer = [layers.local, layers.global].find(
    (d) => d && usable(d) && d.robot && d.robot.age_s + elapsed < 1,
  );
  const robot = robotLayer?.robot;
  const robotPoint = robotLayer && robot ? mapPoint(robotLayer, robot) : null;

  useEffect(() => {
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const refresh = async () => {
      const results = await Promise.allSettled([
        getCostmap("global", controller.signal),
        getCostmap("local", controller.signal),
      ]);
      if (controller.signal.aborted) return;
      const next: typeof layers = {};
      const problems: string[] = [];
      results.forEach((r, i) => {
        const key = i ? "local" : "global";
        if (r.status === "fulfilled") next[key] = r.value;
        else problems.push(`${i ? "局部" : "全局"}图读取失败`);
      });
      setLayers(next);
      setErrors(problems.join(" · "));
      arrived.current = performance.now();
      setElapsed(0);
      timer = setTimeout(refresh, 1000);
    };
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, []);
  useEffect(() => {
    const timer = setInterval(
      () => setElapsed((performance.now() - arrived.current) / 1000),
      250,
    );
    return () => clearInterval(timer);
  }, []);
  useEffect(() => {
    if (follow && robotPoint)
      setView((v) => ({ ...v, x: robotPoint.x, y: -robotPoint.y }));
  }, [follow, robotPoint?.x, robotPoint?.y]);
  useEffect(() => {
    const element = svg.current!;
    const resize = new ResizeObserver(() =>
      setAspect(element.clientWidth / Math.max(1, element.clientHeight)),
    );
    resize.observe(element);
    const wheel = (event: WheelEvent) => {
      event.preventDefault();
      setFollow(false);
      const box = element.getBoundingClientRect(),
        old = viewRef.current;
      const dx = event.clientX - box.left - box.width / 2,
        dy = event.clientY - box.top - box.height / 2;
      const width = Math.max(
        2,
        Math.min(500, old.width * Math.exp(event.deltaY * 0.001)),
      );
      setView({
        width,
        x: old.x + (dx * (old.width - width)) / box.width,
        y: old.y + (dy * (old.width - width)) / box.width,
      });
    };
    element.addEventListener("wheel", wheel, { passive: false });
    return () => {
      resize.disconnect();
      element.removeEventListener("wheel", wheel);
    };
  }, []);
  const fit = () => {
    const d = layers.global;
    if (!d || !usable(d)) return;
    const origin = mapPoint(d, d.origin)!;
    setFollow(false);
    const yaw = d.origin.yaw + d.map_from_frame!.yaw,
      w = d.width * d.resolution,
      h = d.height * d.resolution;
    setView({
      x: origin.x + (Math.cos(yaw) * w - Math.sin(yaw) * h) / 2,
      y: -origin.y - (Math.sin(yaw) * w + Math.cos(yaw) * h) / 2,
      width:
        Math.max(
          Math.abs(Math.cos(yaw) * w) + Math.abs(Math.sin(yaw) * h),
          aspect * (Math.abs(Math.sin(yaw) * w) + Math.abs(Math.cos(yaw) * h)),
        ) * 1.1,
    });
  };
  return (
    <div className="costmap-panel" data-testid="costmap-panel">
      <div className="costmap-controls">
        <strong>叠加代价地图</strong>
        {(["global", "local"] as const).map((key) => (
          <label key={key} className={`costmap-${key}`}>
            <input
              type="checkbox"
              checked={enabled[key]}
              onChange={(e) =>
                setEnabled((v) => ({ ...v, [key]: e.target.checked }))
              }
            />
            {key === "global" ? "全局 · 紫色" : "局部 · 青色"}
          </label>
        ))}
        <button
          onClick={() => {
            setFollow(true);
            setView((v) => ({ ...v, width: 24 }));
          }}
        >
          定位山猫
        </button>
        <button onClick={fit}>全图</button>
        <span>{follow ? "跟随中" : "自由浏览"} · 拖动平移 / 滚轮缩放</span>
      </div>
      <div className="costmap-status" role="status">
        {errors ||
          (!layers.global && !layers.local ? "等待代价地图…" : "map 坐标叠加")}
        {(["global", "local"] as const).map((key) => {
          const d = layers[key];
          return (
            d && (
              <span key={key}>
                {key === "global" ? "全局" : "局部"} {d.resolution.toFixed(2)} m
                / {(d.age_s + elapsed).toFixed(1)} 秒前
                {d.age_s + elapsed >= 3 ? "（已过期）" : ""}
                {!usable(d) ? " · 坐标未对齐，暂不叠加" : ""}
              </span>
            )
          );
        })}
      </div>
      <svg
        ref={svg}
        className="costmap-canvas"
        role="img"
        aria-label="实时导航代价地图"
        viewBox={`${view.x - view.width / 2} ${view.y - view.width / aspect / 2} ${view.width} ${view.width / aspect}`}
        onPointerDown={(e) => {
          if (e.button !== 0) return;
          e.currentTarget.setPointerCapture(e.pointerId);
          drag.current = { x: e.clientX, y: e.clientY, view };
          setFollow(false);
        }}
        onPointerMove={(e) => {
          const d = drag.current;
          if (!d) return;
          const ratio = d.view.width / e.currentTarget.clientWidth;
          setView({
            ...d.view,
            x: d.view.x - (e.clientX - d.x) * ratio,
            y: d.view.y - (e.clientY - d.y) * ratio,
          });
        }}
        onPointerUp={() => {
          drag.current = null;
        }}
        onPointerCancel={() => {
          drag.current = null;
        }}
      >
        <defs>
          <pattern
            id="costmap-grid"
            width="1"
            height="1"
            patternUnits="userSpaceOnUse"
          >
            <path
              d="M 1 0 L 0 0 0 1"
              fill="none"
              stroke="#57656b"
              strokeWidth="0.015"
            />
          </pattern>
        </defs>
        <rect
          x={view.x - view.width / 2}
          y={view.y - view.width / aspect / 2}
          width={view.width}
          height={view.width / aspect}
          fill="url(#costmap-grid)"
        />
        {(["global", "local"] as const).map((key) => {
          const d = layers[key];
          if (!d || !enabled[key] || !usable(d)) return null;
          const o = mapPoint(d, d.origin)!;
          return (
            <g
              key={key}
              data-testid={`costmap-layer-${key}`}
              transform={`translate(${o.x} ${-o.y}) rotate(${(-(d.origin.yaw + d.map_from_frame!.yaw) * 180) / Math.PI}) scale(${d.resolution})`}
              opacity={d.age_s + elapsed >= 3 ? 0.3 : 1}
            >
              <image
                href={d.image}
                x="0"
                y={-d.height}
                width={d.width}
                height={d.height}
                style={{ imageRendering: "pixelated" }}
              />
              <rect
                x="0"
                y={-d.height}
                width={d.width}
                height={d.height}
                fill="none"
                stroke={key === "global" ? "#bd79ff" : "#22d3ee"}
                strokeWidth="1.5"
                vectorEffect="non-scaling-stroke"
              />
            </g>
          );
        })}
        {robotLayer && (
          <polyline
            points={robotLayer.path
              .map((p) => {
                const v = mapPoint(robotLayer, p)!;
                return `${v.x},${-v.y}`;
              })
              .join(" ")}
            fill="none"
            stroke="#ffdd75"
            strokeWidth="2.5"
            vectorEffect="non-scaling-stroke"
          />
        )}
        {robotPoint && robot && robotLayer && (
          <g
            transform={`translate(${robotPoint.x} ${-robotPoint.y}) rotate(${(-(robot.yaw + robotLayer.map_from_frame!.yaw) * 180) / Math.PI})`}
          >
            <circle
              r="0.3"
              fill="#1576e7"
              stroke="white"
              strokeWidth="1.5"
              vectorEffect="non-scaling-stroke"
            />
            <path d="M .55 0 L -.16 -.2 L -.16 .2 Z" fill="white" />
          </g>
        )}
      </svg>
      <div className="costmap-footer">
        <span>
          紫：全局障碍影响区 · 青：局部障碍影响区 · 黄线：执行路径 · 灰：未知
        </span>
        <span>
          {!robotPoint ? "车位暂不可用 · " : ""}只读视图，返回路线地图选点
        </span>
      </div>
    </div>
  );
}
