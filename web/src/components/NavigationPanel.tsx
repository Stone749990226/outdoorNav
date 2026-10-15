import { Send, Square } from "lucide-react";
import { compatible, terminal, type NavigationStatus } from "../navigationApi";

const labels = {
  accepted: "任务已接收",
  planning: "规划中",
  navigating: "行驶中",
  replanning: "重规划中",
  canceling: "正在停车",
  succeeded: "已到达并停车",
  failed: "导航失败",
  canceled: "已取消并停车",
};
export function NavigationPanel({
  status,
  error,
  busy,
  canExecute,
  onExecute,
  onCancel,
}: {
  status: NavigationStatus | null;
  error: string;
  busy: boolean;
  canExecute: boolean;
  onExecute: () => void;
  onCancel: () => void;
}) {
  const match = compatible(status);
  const text = !status
    ? "导航服务未连接 · 可使用离线预览"
    : !match
      ? "场景版本不一致，请重新构建并重启"
      : !status.ready
        ? "定位、传感器或导航尚未就绪"
        : status.task
          ? labels[status.task.state]
          : "Gazebo 导航就绪";
  return (
    <div className="navigation-panel">
      <span className="eyebrow">GAZEBO / 实际仿真导航</span>
      <div className="navigation-panel-actions">
        <button
          onClick={onExecute}
          disabled={busy || !canExecute || !match || !status?.ready}
        >
          <Send size={14} />
          {status?.task && !terminal(status.task) ? "替换导航目标" : "执行导航"}
        </button>
        <button
          title="取消导航并停车"
          onClick={onCancel}
          disabled={busy || terminal(status?.task ?? null)}
        >
          <Square size={14} />
        </button>
      </div>
      <small role="status" data-testid="navigation-status">
        {error || text}
      </small>
      {match && status?.task && (
        <small>
          路径版本 {status.task.path_revision} ·{" "}
          {status.task.distance_remaining_m?.toFixed(1) ?? "—"} m
          {status.task.reason && <span> · {status.task.reason}</span>}
        </small>
      )}
      {match && status?.pose && (
        <small>
          当前位置 {status.pose.x.toFixed(2)}, {status.pose.y.toFixed(2)} m
        </small>
      )}
    </div>
  );
}
