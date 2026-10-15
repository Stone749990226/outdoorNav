import { Activity, Target } from "lucide-react";
import type { PatrolModel } from "../hooks/usePatrol";
import { scale, toMeters } from "../map";
const fmtTime = (t: number) =>
  `${Math.floor(t / 60)
    .toString()
    .padStart(2, "0")}:${Math.floor(t % 60)
    .toString()
    .padStart(2, "0")}`;

export function PatrolStatus({ model }: { model: PatrolModel }) {
  const {
    state,
    speed,
    target,
    progress,
    nextSegment,
    remaining,
    remainingDwell,
  } = model;
  return (
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
            : (target?.name ?? (state.error ? "等待道路恢复" : "等待生成路线"))}
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
          <strong className="time-metric">{fmtTime(state.elapsed)}</strong>
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
            {state.completed.length} <small>/ {state.tasks.length}</small>
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
              <div key={event.id} className={`event-item ${event.kind}`}>
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
  );
}
