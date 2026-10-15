import { useState } from "react";
import type { Point } from "../types";
import { toMap } from "../map";
import { addObstacle, removeObstacle } from "../obstacleApi";
import type { useObstacles } from "../hooks/useObstacles";

export type ObstacleSelection = "start" | "end" | null;
export function ObstaclePanel({
  enabled,
  obstacles,
  start,
  end,
  selection,
  select,
  clear,
}: {
  enabled: boolean;
  obstacles: ReturnType<typeof useObstacles>;
  start: Point | null;
  end: Point | null;
  selection: ObstacleSelection;
  select: (value: ObstacleSelection) => void;
  clear: () => void;
}) {
  const [kind, setKind] = useState<"static" | "moving">("static");
  const [size, setSize] = useState(0.6);
  const [speed, setSpeed] = useState(0.3);
  const coordinate = (point: Point | null) =>
    point
      ? `${toMap(point).x.toFixed(1)}, ${toMap(point).y.toFixed(1)} m`
      : "未选择";
  const create = async () => {
    if (!start || (kind === "moving" && !end)) return;
    const ok = await obstacles.operate(() =>
      addObstacle({
        name: `test_${crypto.randomUUID().replaceAll("-", "").slice(0, 12)}`,
        kind,
        start: toMap(start),
        end: toMap(end ?? start),
        width: size,
        depth: size,
        height: 0.8,
        speed_mps: speed,
      }),
    );
    if (ok) clear();
  };
  return (
    <section className="obstacle-panel" aria-label="场景障碍物">
      <h3>场景障碍物</h3>
      <p>行驶中也可添加。雷达与相机感知后触发避障；道路堵死时任务会失败。</p>
      <label>
        类型{" "}
        <select
          aria-label="障碍物类型"
          value={kind}
          onChange={(e) => {
            setKind(e.target.value as typeof kind);
            select(null);
          }}
        >
          <option value="static">静止障碍物</option>
          <option value="moving">移动障碍物（两点往返）</option>
        </select>
      </label>
      <button
        disabled={!enabled || obstacles.busy}
        aria-pressed={selection === "start"}
        onClick={() => select(selection === "start" ? null : "start")}
      >
        选障碍物位置
      </button>
      <small>{coordinate(start)}</small>
      {kind === "moving" && (
        <>
          <button
            disabled={!enabled || obstacles.busy}
            aria-pressed={selection === "end"}
            onClick={() => select(selection === "end" ? null : "end")}
          >
            选移动终点
          </button>
          <small>{coordinate(end)}</small>
        </>
      )}
      <label>
        边长（米）{" "}
        <input
          aria-label="障碍物边长"
          type="number"
          min="0.2"
          max="2"
          step="0.1"
          value={size}
          onChange={(e) => setSize(Number(e.target.value))}
        />
      </label>
      {kind === "moving" && (
        <label>
          速度（米/秒）{" "}
          <input
            aria-label="障碍物速度"
            type="number"
            min="0.05"
            max="1"
            step="0.05"
            value={speed}
            onChange={(e) => setSpeed(Number(e.target.value))}
          />
        </label>
      )}
      <button
        className="primary"
        disabled={
          !enabled || obstacles.busy || !start || (kind === "moving" && !end)
        }
        onClick={() => void create()}
      >
        添加到 Gazebo
      </button>
      <small role="status">
        {obstacles.error ||
          (!enabled
            ? "连接仿真后可添加"
            : selection
              ? "请在地图上点击；橙色为障碍物"
              : "移动障碍物按仿真时间往返运动")}
      </small>
      {obstacles.items.map((item) => (
        <div className="obstacle-row" key={item.name}>
          <span>
            {item.kind === "moving" ? "移动" : "静止"} ·{" "}
            {item.position.x.toFixed(1)}, {item.position.y.toFixed(1)}{" "}
            {item.error && ` · ${item.error}`}
          </span>
          <button
            aria-label={`删除障碍物 ${item.name}`}
            disabled={obstacles.busy}
            onClick={() =>
              void obstacles.operate(() => removeObstacle(item.name))
            }
          >
            删除
          </button>
        </div>
      ))}
    </section>
  );
}
