import {
  ArrowUpRight,
  Ban,
  Check,
  Crosshair,
  Layers as LayersIcon,
  LocateFixed,
  MousePointer2,
  Settings2,
  Upload,
  X,
} from "lucide-react";
import { MapView } from "../MapView";
import type { PatrolModel } from "../hooks/usePatrol";
import { bundle, roads, scale } from "../map";
const statusText = {
  idle: "等待规划",
  ready: "路线就绪",
  running: "巡航进行中",
  paused: "巡航已暂停",
  blocked: "目标不可达",
  completed: "巡航已完成",
};

export function PatrolMap({ model }: { model: PatrolModel }) {
  const {
    state,
    mode,
    setMode,
    layers,
    setLayers,
    layerMenu,
    setLayerMenu,
    reference,
    setReference,
    referenceMenu,
    setReferenceMenu,
    roadPanel,
    setRoadPanel,
    fileRef,
    editable,
    target,
    coordinate,
    notify,
    pick,
    roadClick,
    routeDownload,
  } = model;
  return (
    <section className="map-panel panel">
      <div className="map-toolbar">
        <div className="map-tabs">
          <button
            className={!layers.access ? "active" : ""}
            onClick={() =>
              setLayers((l) => ({
                ...l,
                access: false,
                semantic: true,
              }))
            }
          >
            <LayersIcon size={14} />
            实测路线
          </button>
          <button
            className={layers.access ? "active" : ""}
            onClick={() => setLayers((l) => ({ ...l, access: true }))}
          >
            通行性地图
          </button>
        </div>
        <div className="map-tools">
          <button
            title="设置起点"
            className={mode === "start" ? "active" : ""}
            disabled={!editable}
            onClick={() => setMode(mode === "start" ? "inspect" : "start")}
          >
            <LocateFixed size={16} />
          </button>
          <button
            title="封闭或恢复道路"
            className={mode === "block" ? "active warning" : ""}
            onClick={() => {
              setMode(mode === "block" ? "inspect" : "block");
              setRoadPanel(mode !== "block");
            }}
          >
            <Ban size={16} />
          </button>
          <button
            title="图层设置"
            className={layerMenu ? "active" : ""}
            onClick={() => setLayerMenu(!layerMenu)}
          >
            <Settings2 size={16} />
          </button>
        </div>
      </div>
      <MapView
        state={state}
        layers={layers}
        mode={mode}
        reference={reference}
        onPick={pick}
        onRoad={roadClick}
        onInfo={notify}
      />
      {layerMenu && (
        <div className="floating-panel layer-panel">
          <div className="floating-title">
            地图图层
            <button
              onClick={() => setLayerMenu(false)}
              aria-label="关闭图层设置"
            >
              <X size={14} />
            </button>
          </div>
          {(
            [
              ["basemap", "园区示意底图"],
              ["semantic", "巡检点名称"],
              ["access", "路线可达性"],
              ["network", "道路拓扑网络"],
              ["route", "路线与途经点"],
              ["reference", "导入的参考图片"],
            ] as const
          ).map(([key, label]) => (
            <label key={key}>
              <span>{label}</span>
              <input
                type="checkbox"
                checked={layers[key]}
                onChange={(e) =>
                  setLayers((l) => ({ ...l, [key]: e.target.checked }))
                }
              />
            </label>
          ))}
          <button
            className="upload-button"
            onClick={() => fileRef.current?.click()}
          >
            <Upload size={14} />
            导入本地参考图片
          </button>
          {reference && (
            <button
              className="upload-button"
              onClick={() => {
                setReferenceMenu(true);
                setLayerMenu(false);
              }}
            >
              <Settings2 size={14} />
              调整参考图片
            </button>
          )}
        </div>
      )}
      {referenceMenu && reference && (
        <div className="floating-panel reference-panel">
          <div className="floating-title">
            参考图片对齐
            <button
              aria-label="关闭参考图片设置"
              onClick={() => setReferenceMenu(false)}
            >
              <X size={14} />
            </button>
          </div>
          <p>仅调整参考图片，不改变地图坐标。</p>
          {(
            [
              {
                key: "opacity",
                label: "透明度",
                min: 0,
                max: 1,
                step: 0.05,
              },
              {
                key: "scale",
                label: "缩放",
                min: 0.2,
                max: 3,
                step: 0.05,
              },
              {
                key: "x",
                label: "水平偏移",
                min: -1000,
                max: 1000,
                step: 5,
              },
              {
                key: "y",
                label: "垂直偏移",
                min: -920,
                max: 920,
                step: 5,
              },
            ] as const
          ).map((item) => (
            <label key={item.key}>
              <span>{item.label}</span>
              <input
                aria-label={item.label}
                type="range"
                min={item.min}
                max={item.max}
                step={item.step}
                value={reference[item.key]}
                onChange={(e) =>
                  setReference({
                    ...reference,
                    [item.key]: Number(e.target.value),
                  })
                }
              />
              <output>{reference[item.key]}</output>
            </label>
          ))}
          <button
            className="text-button"
            onClick={() => {
              setReference(null);
              setReferenceMenu(false);
            }}
          >
            移除图片
          </button>
        </div>
      )}
      {roadPanel && (
        <div className="floating-panel road-panel">
          <div className="floating-title">
            道路通行管理
            <button
              onClick={() => {
                setRoadPanel(false);
                setMode("inspect");
              }}
              aria-label="关闭道路管理"
            >
              <X size={14} />
            </button>
          </div>
          <p>点击道路或列表切换。封路后自动重新规划。</p>
          <div className="road-list">
            {roads.map((r) => (
              <button
                key={r.id}
                data-testid={`toggle-${r.id}`}
                className={state.blocked.includes(r.id) ? "closed" : ""}
                onClick={() => roadClick(r.id)}
              >
                <span>{r.name}</span>
                <span>{state.blocked.includes(r.id) ? "已封闭" : "开放"}</span>
              </button>
            ))}
          </div>
        </div>
      )}
      {mode !== "inspect" && (
        <div className={`mode-hint ${mode === "block" ? "warning" : ""}`}>
          <MousePointer2 size={14} />
          {mode === "start"
            ? "点击道路设置起点 · 自动吸附"
            : mode === "target"
              ? "点击道路添加巡检点 · 可连续添加"
              : "点击前方道路，模拟封闭或恢复"}
          <button
            title="退出编辑模式"
            onClick={() => {
              setMode("inspect");
              setRoadPanel(false);
            }}
          >
            <X size={14} />
          </button>
        </div>
      )}
      {state.error && (
        <div className="error-banner" role="alert">
          <Ban size={18} />
          <div>
            <strong>目标暂不可达</strong>
            <p>{state.error}</p>
          </div>
        </div>
      )}
      {state.status === "completed" && (
        <div className="completion-card" data-testid="completion-card">
          <div className="completion-check">
            <Check size={25} />
          </div>
          <span className="eyebrow">MISSION ACCOMPLISHED</span>
          <h2>巡航任务完成</h2>
          <p>
            到访 {state.completed.length} 个巡检点 · 行走{" "}
            {(state.traveled * scale).toFixed(0)} 米 · 重规划 {state.replans} 次
          </p>
          <button onClick={routeDownload}>
            导出本次路线 <ArrowUpRight size={14} />
          </button>
        </div>
      )}
      <div className="map-status">
        <span>
          <span
            className={`live-dot ${state.status === "blocked" ? "orange" : ""}`}
          />
          <b data-testid="status">{statusText[state.status]}</b>
        </span>
        <span>
          <Crosshair size={12} /> {coordinate.x.toFixed(1)},{" "}
          {coordinate.y.toFixed(1)} <small>米</small>
        </span>
        <span className="frame-tag">{bundle.frame_id}</span>
      </div>
    </section>
  );
}
