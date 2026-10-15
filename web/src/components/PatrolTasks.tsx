import {
  ArrowDown,
  ArrowUp,
  Check,
  ChevronRight,
  Flag,
  Plus,
  RotateCcw,
  Route,
  ShieldCheck,
  X,
} from "lucide-react";
import type { PatrolModel } from "../hooks/usePatrol";
import { campus } from "../map";
import { planSimulation } from "../simulation";

export function PatrolTasks({ model }: { model: PatrolModel }) {
  const {
    state,
    setState,
    mode,
    setMode,
    editable,
    target,
    updateTask,
    moveTask,
    addPreset,
    unused,
  } = model;
  return (
    <aside className="left-panel panel">
      <div className="panel-heading">
        <div>
          <span className="eyebrow">MISSION / 任务</span>
          <h2>园区巡检计划</h2>
        </div>
        <span className="small-badge">01</span>
      </div>
      <div className="mission-summary">
        <div className="mission-icon">
          <Flag size={18} />
        </div>
        <div>
          <strong>园区日常巡航</strong>
          <span>
            {state.tasks.length} 个巡检点 ·{" "}
            {state.returnHome ? "闭环返回" : "单程任务"}
          </span>
        </div>
        <span className="muted-arrow">
          <ChevronRight size={17} />
        </span>
      </div>
      <div className="section-line">
        <span>
          巡检顺序 <small> / WAYPOINTS</small>
        </span>
        <span className="point-count">
          {state.tasks.length.toString().padStart(2, "0")}
        </span>
      </div>
      <div className="task-list" data-testid="task-list">
        {state.tasks.map((task, i) => {
          const done = state.completed.includes(task.id),
            active = target?.id === task.id;
          return (
            <div
              key={task.id}
              className={`task-row ${done ? "done" : ""} ${active ? "active" : ""}`}
            >
              <div className="task-order">
                {done ? <Check size={13} /> : String(i + 1).padStart(2, "0")}
              </div>
              <div className="task-text">
                <strong>{task.name}</strong>
                <span>
                  {done
                    ? "巡检完成"
                    : active && state.dwelling
                      ? "到点停留中"
                      : `巡检停留 ${task.dwell}s`}
                  <i>演示点</i>
                </span>
              </div>
              <div className="task-actions">
                <button
                  title={`上移${task.name}`}
                  disabled={!editable || i === 0}
                  onClick={() => moveTask(i, -1)}
                >
                  <ArrowUp size={12} />
                </button>
                <button
                  title={`下移${task.name}`}
                  disabled={!editable || i === state.tasks.length - 1}
                  onClick={() => moveTask(i, 1)}
                >
                  <ArrowDown size={12} />
                </button>
                <button
                  title={`删除${task.name}`}
                  disabled={!editable}
                  onClick={() =>
                    updateTask(state.tasks.filter((t) => t.id !== task.id))
                  }
                >
                  <X size={12} />
                </button>
              </div>
            </div>
          );
        })}
        {!state.tasks.length && (
          <div className="empty-tasks">从地图添加巡检点，或恢复预置点。</div>
        )}
      </div>
      <button
        className={`add-task ${mode === "target" ? "selected" : ""}`}
        disabled={!editable}
        onClick={() => setMode(mode === "target" ? "inspect" : "target")}
      >
        <Plus size={15} />
        在地图上添加巡检点
      </button>
      {unused.length > 0 && (
        <div className="restore-presets">
          恢复：
          {unused.map((t) => (
            <button
              key={t.id}
              disabled={!editable}
              onClick={() => addPreset(t)}
            >
              {t.name}
            </button>
          ))}
        </div>
      )}
      <label className="return-toggle">
        <span>
          <RotateCcw size={14} />
          完成后返回起点
        </span>
        <input
          type="checkbox"
          checked={state.returnHome}
          disabled={!editable}
          onChange={(e) =>
            updateTask(state.tasks, state.home, e.target.checked)
          }
        />
        <i />
      </label>
      <button
        className="plan-button"
        disabled={
          ["running", "paused"].includes(state.status) || !state.tasks.length
        }
        onClick={() => setState((s) => planSimulation(s))}
      >
        <Route size={17} />
        生成巡航路线
        <ChevronRight size={16} />
      </button>
      <div className="mission-note">
        <ShieldCheck size={16} />
        <p>
          沿已标注道路规划
          <br />
          <span>仅沿已记录路线规划</span>
        </p>
      </div>
      <div className="left-bottom">
        <span className="eyebrow">MAP SOURCE</span>
        <p>卫星影像 + 路网辅助重绘</p>
        <span className="source-tag">DGPS 实测</span>
        <span className="source-tag">{campus.version}</span>
      </div>
    </aside>
  );
}
