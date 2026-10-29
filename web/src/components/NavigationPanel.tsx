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
  blockedReason,
  onExecute,
  onCancel,
}: {
  status: NavigationStatus | null;
  error: string;
  busy: boolean;
  canExecute: boolean;
  blockedReason: string;
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
      {status?.diagnostic && (
        <small data-testid="diagnostic-mode">诊断模式 · 底盘输出已隔离</small>
      )}
      <div className="navigation-panel-actions">
        <button
          onClick={onExecute}
          disabled={busy || !canExecute || !match || !status?.ready}
          title={
            busy || !canExecute || !match || !status?.ready
              ? blockedReason
              : undefined
          }
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
      {(busy || !canExecute || !match || !status?.ready) && (
        <small data-testid="execute-blocked">执行不可用：{blockedReason}</small>
      )}
      {match &&
        status?.not_ready_reasons &&
        status.not_ready_reasons.length > 0 && (
          <ul data-testid="readiness-reasons">
            {status.not_ready_reasons.map((reason) => (
              <li key={reason.code}>{reason.message}</li>
            ))}
          </ul>
        )}
      {match && status?.policy_warnings?.length ? (
        <details className="policy-warnings">
          <summary>现场策略提示（{status.policy_warnings.length}）</summary>
          <ul>
            {status.policy_warnings.map((w) => (
              <li key={w.code}>{w.message}</li>
            ))}
          </ul>
        </details>
      ) : null}
      {match && status?.task && (
        <small>
          {status.mode === "lynx"
            ? "任务状态"
            : `路径版本 ${status.task.path_revision}`}{" "}
          · {status.task.distance_remaining_m?.toFixed(1) ?? "—"} m
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
      {status?.mode === "lynx" && status.gnss?.valid !== undefined && (
        <small data-testid="position-source-status">
          {status.gnss.valid &&
          status.gnss.latitude != null &&
          status.gnss.longitude != null
            ? `RTK 天线经纬度 ${status.gnss.latitude.toFixed(7)}, ${status.gnss.longitude.toFixed(7)}`
            : "RTK 位置无效，不使用当前经纬度"}
        </small>
      )}
      {status?.mode === "lynx" && status.heading?.source === "wit_imu" && (
        <small data-testid="wit-heading-status">
          {status.heading.angles_fresh && status.heading.raw_yaw_deg != null
            ? `新 IMU Z 角 ${status.heading.raw_yaw_deg.toFixed(1)}°（模块磁参考）`
            : "新 IMU 角度无效或已过期"}
          {!status.heading.nine_axis_verified && " · 九轴模式未确认"}
          {!status.heading.mounting_confirmed && " · 安装外参待确认"}
          {!status.heading.reference_confirmed && " · 真北参考待确认"}
          {!status.heading.quality_confirmed && " · 航向精度待验证"}
          {status.heading.heading_fresh &&
            status.heading.body_yaw_enu_rad != null &&
            ` · 车体 ENU 角 ${((status.heading.body_yaw_enu_rad * 180) / Math.PI).toFixed(1)}°`}
        </small>
      )}
      {status?.mode === "lynx" && status.localization_mode === "rtk_imu" && (
        <small data-testid="feedback-source-status">
          运动反馈：RTK 多普勒＋IMU 角速度（不使用本体里程计）
          {!status.feedback?.confirmed && " · 尚未现场验收"}
        </small>
      )}
      {status?.mode === "lynx" && (
        <small>
          {status.gnss?.fixed
            ? "RTK 固定解"
            : status.gnss?.float
              ? "RTK 浮点解"
              : "等待可用 RTK"}
          {status.gnss &&
            ` · 估计误差 ${status.gnss.hacc_m.toFixed(2)} m · ${status.gnss.satellites} 颗卫星`}
          {!status.ready &&
            !status.not_ready_reasons &&
            ` · 未就绪：${Object.entries(status.health)
              .filter(([, ok]) => !ok)
              .map(
                ([key]) =>
                  ({
                    localization: "定位",
                    calibration: "地理坐标对齐",
                    antenna_extrinsics: "天线安装偏移确认",
                    motion_enabled: "运动开关",
                    service_active: "导航服务未激活",
                    controller_stopped: "停车确认",
                    odometry: "里程计",
                    actions: "导航执行",
                    tf: "坐标变换",
                    exclusive_owners: "定位发布冲突",
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
