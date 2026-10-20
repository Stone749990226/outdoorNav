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
          : status.mode === "lynx"
            ? "山猫室外导航就绪"
            : "Gazebo 导航就绪";
  return (
    <div className="navigation-panel">
      <span className="eyebrow">
        {status?.mode === "lynx" ? "山猫 / 室外导航" : "GAZEBO / 实际仿真导航"}
      </span>
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
          {status.pose.age_s > 1 && " · 定位数据已过期"}
          {status.pose_source === "gnss_antenna" && " · 天线定位，尚未对齐"}
        </small>
      )}
      {status?.mode === "lynx" && (
        <small>
          {status.gnss?.fixed ? "RTK 固定解" : "等待 RTK 固定解"}
          {status.gnss &&
            ` · 估计误差 ${status.gnss.hacc_m.toFixed(2)} m · ${status.gnss.satellites} 颗卫星`}
          {!status.ready &&
            ` · 未就绪：${Object.entries(status.health)
              .filter(([, ok]) => !ok)
              .map(
                ([key]) =>
                  ({
                    localization: "定位",
                    calibration: "现场标定",
                    motion_enabled: "运动开关",
                    service_active: "导航服务未激活",
                    controller_stopped: "停车确认",
                    odometry: "里程计",
                    actions: "导航执行",
                    tf: "坐标变换",
                    lidar: "雷达",
                    global_map: "全局代价地图",
                    local_map: "局部代价地图",
                    planner_server: "规划器",
                    controller_server: "控制器",
                  })[key] ?? key,
              )
              .join("、")}`}
        </small>
      )}
    </div>
  );
}
