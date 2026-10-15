import { Pause, Play, RotateCcw } from "lucide-react";
import type { PatrolModel } from "../hooks/usePatrol";
import { scale } from "../map";
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

export function PatrolPlayback({ model }: { model: PatrolModel }) {
  const { state, speed, setSpeed, routeValid, remaining, reset, run } = model;
  return (
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
          <span>{state.autoDemo ? "自动封路演示已启用" : "自由探索模式"}</span>
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
  );
}
