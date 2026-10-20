import {
  ArrowUpRight,
  ChevronRight,
  CircleHelp,
  Download,
  Map as MapIcon,
  Route,
  Sparkles,
  X,
} from "lucide-react";
import { ABWorkspace } from "./ABWorkspace";
import { PatrolMap } from "./components/PatrolMap";
import { PatrolPlayback } from "./components/PatrolPlayback";
import { PatrolStatus } from "./components/PatrolStatus";
import { PatrolTasks } from "./components/PatrolTasks";
import { usePatrol } from "./hooks/usePatrol";
import { startDemo } from "./simulation";
import { useEffect } from "react";
import { getStatus } from "./navigationApi";

export default function App() {
  const patrol = usePatrol();
  const {
    workspaceMode,
    setWorkspaceMode,
    setState,
    setSpeed,
    setMode,
    setLayers,
    setLayerMenu,
    reference,
    setReference,
    setReferenceMenu,
    setRoadPanel,
    toast,
    setToast,
    fileRef,
    routeValid,
    target,
    notify,
    mapDownload,
    routeDownload,
    csvDownload,
    setHelp,
    help,
  } = patrol;
  useEffect(() => {
    let active = true;
    void getStatus()
      .then((status) => {
        if (active && status.mode === "lynx") setWorkspaceMode("ab");
      })
      .catch(() => {});
    return () => {
      active = false;
    };
  }, [setWorkspaceMode]);
  return (
    <div className="app-shell">
      <header className="app-header">
        <a className="brand" href="#" onClick={(e) => e.preventDefault()}>
          <span className="brand-icon">
            <Route size={24} />
          </span>
          <span className="brand-name">
            <span>FIELD / NAV</span>
          </span>
        </a>
        <div className="header-divider" />
        <div className="workspace-title">
          园区巡航任务台 <span>OUTDOOR AUTONOMY LAB</span>
        </div>
        <nav className="header-nav">
          <span className="nav-active">
            <MapIcon size={15} />
            任务工作台
          </span>
          <button onClick={() => setHelp(true)}>
            <CircleHelp size={15} />
            演示指南
          </button>
        </nav>
        <span className="simulation-badge">
          <span className="live-dot" />
          FIELD NAV
        </span>
      </header>
      <div className="page-heading">
        <div>
          <div className="eyebrow">
            MISSION CONTROL <span>/ 01</span>
          </div>
          <h1>
            <span className="heading-dot"></span>
            智慧谷北区
          </h1>
          <p>
            实测环线 <span> / </span> 中央支路
          </p>
        </div>
        <div className="heading-actions">
          <div className="workspace-switch">
            <button
              className={workspaceMode === "patrol" ? "active" : ""}
              onClick={() => setWorkspaceMode("patrol")}
            >
              巡航任务
            </button>
            <button
              className={workspaceMode === "ab" ? "active" : ""}
              onClick={() => {
                setWorkspaceMode("ab");
                setState((s) =>
                  s.status === "running" ? { ...s, status: "paused" } : s,
                );
              }}
            >
              A→B 规划
            </button>
          </div>
          {workspaceMode === "patrol" && (
            <details className="export-menu">
              <summary>
                <Download size={16} />
                导出数据
                <ChevronRight size={14} />
              </summary>
              <div className="dropdown">
                <button onClick={mapDownload}>
                  地图 JSON<span>实测中心线路网</span>
                </button>
                <button onClick={routeDownload} disabled={!routeValid}>
                  路线 JSON<span>完整折线与任务顺序</span>
                </button>
                <button onClick={csvDownload} disabled={!routeValid}>
                  途经点 CSV<span>局部坐标与参考朝向</span>
                </button>
              </div>
            </details>
          )}
          <button
            className="primary demo-button"
            onClick={() => {
              setWorkspaceMode("patrol");
              setState(startDemo());
              setSpeed(5);
              setMode("inspect");
              setRoadPanel(false);
            }}
          >
            <Sparkles size={17} />
            离线动画演示
            <ArrowUpRight size={17} />
          </button>
        </div>
      </div>
      {workspaceMode === "ab" ? (
        <ABWorkspace />
      ) : (
        <>
          <main className="workspace">
            <PatrolTasks model={patrol} />
            <PatrolMap model={patrol} />
            <PatrolStatus model={patrol} />
          </main>
          <PatrolPlayback model={patrol} />
        </>
      )}
      {toast && (
        <div className="toast" role="status">
          <CircleHelp size={17} />
          {toast}
          <button aria-label="关闭提示" onClick={() => setToast("")}>
            <X size={14} />
          </button>
        </div>
      )}
      <input
        ref={fileRef}
        type="file"
        accept="image/png,image/jpeg,image/webp"
        className="hidden-input"
        aria-label="导入参考图片"
        onChange={(e) => {
          const f = e.target.files?.[0];
          if (!f) return;
          if (!["image/png", "image/jpeg", "image/webp"].includes(f.type)) {
            notify("请选择 PNG、JPEG 或 WebP 图片。");
            return;
          }
          if (f.size > 20 * 1024 * 1024) {
            notify("请选择小于 20 MB 的图片。");
            return;
          }
          const url = URL.createObjectURL(f);
          const img = new Image();
          img.onload = () => {
            setReference({ url, opacity: 0.55, x: 0, y: 0, scale: 1 });
            setReferenceMenu(true);
            setLayerMenu(false);
            setLayers((l) => ({ ...l, reference: true }));
          };
          img.onerror = () => {
            URL.revokeObjectURL(url);
            notify("图片无法解码，请选择有效图片。");
          };
          img.src = url;
          e.target.value = "";
        }}
      />
      {help && (
        <div className="modal-backdrop" onClick={() => setHelp(false)}>
          <section
            className="help-modal"
            role="dialog"
            aria-modal="true"
            aria-label="演示指南"
            onClick={(e) => e.stopPropagation()}
          >
            <button
              className="modal-close"
              aria-label="关闭指南"
              onClick={() => setHelp(false)}
            >
              <X size={20} />
            </button>
            <span className="eyebrow">YOUR FIRST MISSION</span>
            <h2>三分钟，走完一次园区巡航。</h2>
            <div className="guide-steps">
              <p>
                <b>01</b>
                <span>
                  <strong>认识地图</strong>
                  查看实测路线、道路连接与当前可达状态。
                </span>
              </p>
              <p>
                <b>02</b>
                <span>
                  <strong>点击「离线动画演示」</strong>
                  从北侧入口出发，访问五个巡检点并返回。默认 5×
                  播放，约两分钟完成。
                </span>
              </p>
              <p>
                <b>03</b>
                <span>
                  <strong>观察自动绕行</strong>
                  到达中央路口后模拟封闭前方连接道，橙色虚线保留上次路线，绿色显示新路线。
                </span>
              </p>
              <p>
                <b>04</b>
                <span>
                  <strong>探索与导出</strong>
                  重置后修改任务、点选起点或封路。导出局部米制地图、路线与关键途经点。
                </span>
              </p>
            </div>
            <div className="guide-note">
              地图来自两段差分解轨迹，采用米制坐标。绝对位置精度尚未独立验证；当前连接
              Nav2 仿真机器人。
            </div>
            <button className="primary" onClick={() => setHelp(false)}>
              开始探索 <ArrowUpRight size={16} />
            </button>
          </section>
        </div>
      )}
    </div>
  );
}
