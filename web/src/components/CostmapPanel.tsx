import { useEffect, useRef, useState } from "react";
import { getCostmap, type CostmapSnapshot } from "../navigationApi";
import { gridPoint } from "../costmap";

export function CostmapPanel() {
  const [layer, setLayer] = useState<"local" | "global">("local");
  const [data, setData] = useState<CostmapSnapshot | null>(null);
  const [error, setError] = useState("");
  const [zoom, setZoom] = useState(1);
  const [follow, setFollow] = useState(true);
  const [elapsed, setElapsed] = useState(0);
  const arrived = useRef(performance.now());
  useEffect(() => {
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    setData(null);
    setError("");
    const refresh = async () => {
      try {
        const next = await getCostmap(layer, controller.signal);
        if (!controller.signal.aborted) {
          arrived.current = performance.now();
          setElapsed(0);
          setData(next);
          setError("");
        }
      } catch (e) {
        if (!controller.signal.aborted)
          setError(e instanceof Error ? e.message : "代价图读取失败");
      }
      if (!controller.signal.aborted) timer = setTimeout(refresh, 1000);
    };
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [layer]);
  useEffect(() => {
    const timer = setInterval(
      () => setElapsed((performance.now() - arrived.current) / 1000),
      250,
    );
    return () => clearInterval(timer);
  }, []);
  const robot = data?.robot;
  const pixel = data && robot ? gridPoint(data, robot) : null;
  const width = (data?.width ?? 1) / zoom;
  const height = (data?.height ?? 1) / zoom;
  const center =
    follow && pixel
      ? pixel
      : { x: (data?.width ?? 1) / 2, y: (data?.height ?? 1) / 2 };
  const viewBox = `${center.x - width / 2} ${center.y - height / 2} ${width} ${height}`;
  const age = (data?.age_s ?? 0) + elapsed;
  return (
    <div className="costmap-panel" data-testid="costmap-panel">
      <div className="costmap-controls">
        <label>
          地图{" "}
          <select
            aria-label="代价图类型"
            value={layer}
            onChange={(e) => {
              setLayer(e.target.value as "local" | "global");
              setZoom(1);
            }}
          >
            <option value="local">局部代价图（车附近）</option>
            <option value="global">全局代价图</option>
          </select>
        </label>
        <label>
          放大{" "}
          <input
            aria-label="代价图缩放"
            type="range"
            min="1"
            max="16"
            step="0.5"
            value={zoom}
            onChange={(e) => setZoom(Number(e.target.value))}
          />
        </label>
        <label>
          <input
            type="checkbox"
            checked={follow}
            onChange={(e) => setFollow(e.target.checked)}
          />
          跟随山猫
        </label>
      </div>
      <p role="status">
        {error ||
          (!data
            ? "等待代价地图…"
            : `${data.frame_id} 坐标 · 分辨率 ${data.resolution} m · 数据 ${age.toFixed(1)} 秒前${age >= 3 ? "（已过期）" : ""}`)}
      </p>
      {data && (
        <svg
          className="costmap-canvas"
          role="img"
          aria-label="实时导航代价地图"
          viewBox={viewBox}
          preserveAspectRatio="xMidYMid meet"
          style={{ opacity: error || age >= 3 ? 0.5 : 1 }}
        >
          <image
            href={data.image}
            x="0"
            y="0"
            width={data.width}
            height={data.height}
            style={{ imageRendering: "pixelated" }}
          />
          {data.path.length > 1 && (
            <polyline
              points={data.path
                .map((p) => {
                  const v = gridPoint(data, p);
                  return `${v.x},${v.y}`;
                })
                .join(" ")}
              fill="none"
              stroke="#1498ff"
              strokeWidth={2}
              vectorEffect="non-scaling-stroke"
            />
          )}
          {robot && pixel && robot.age_s + elapsed < 1 && (
            <g
              transform={`translate(${pixel.x} ${pixel.y}) rotate(${(-(robot.yaw - data.origin.yaw) * 180) / Math.PI})`}
            >
              <circle
                r={0.3 / data.resolution}
                fill="#0869c9"
                stroke="white"
                strokeWidth={1.5}
                vectorEffect="non-scaling-stroke"
              />
              <path
                d={`M ${0.5 / data.resolution} 0 L ${-0.15 / data.resolution} ${-0.18 / data.resolution} L ${-0.15 / data.resolution} ${0.18 / data.resolution} Z`}
                fill="white"
              />
            </g>
          )}
        </svg>
      )}
      {data && (!robot || robot.age_s + elapsed >= 1) && (
        <p>机器人位置变换未就绪或已过期，暂不显示车标。</p>
      )}
      <p className="costmap-legend">
        浅色：空闲 · 黄橙：障碍影响区 · 红色：碰撞区 · 灰色：未知 ·
        蓝线：当前执行路径
      </p>
      <p>
        只读实时图，每秒更新；不代替现场观察，不在此视图下发目标。返回路线地图可选点导航。
      </p>
    </div>
  );
}
