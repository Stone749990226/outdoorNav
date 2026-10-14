import { calibration, campus, dist, mapToSvg, scale, toGeoreferenced } from "./map";
import { route } from "./planner";
import {
  extractWaypoints,
  serializeWaypoints,
  waypointOptions,
} from "./waypoints";
import type { KeyWaypoint } from "./waypoints";
import type { Plan, Point } from "./types";
export type ABResult = {
  points: Point[];
  waypoints: KeyWaypoint[];
  plan: Plan | null;
  error: string | null;
  blocked: string[];
};
export function planAB(a: Point, b: Point, blocked: string[] = []): ABResult {
  const path = route(a, b, blocked);
  if (!path)
    return { points: [], error: "A/B 不在开放中心线上，或道路已断开。", waypoints: [], plan: null, blocked: [...blocked] };
  const points = [a, ...path.map((segment) => segment.b)];
  let distance = 0;
  const segments = path.map((segment) => {
    const start = distance;
    distance += dist(segment.a, segment.b);
    return {
      a: segment.a,
      b: segment.b,
      start,
      end: distance,
      edgeId: segment.edgeId,
    };
  });
  const waypoints = extractWaypoints(points);
  return {
    points,
    error: null,
    blocked: [...blocked],
    waypoints,
    plan: {
      start: a,
      segments,
      length: distance,
      taskOrder: ["ab-b"],
      stops: [{ id: "ab-b", name: "B 点", point: b, dwell: 0, distance }],
    },
  };
}
/** ★ 车的**真实位姿**（**map 帧米值** —— 就是 `GET /api/status` 的 `pose{x,y,yaw}` ✓）。 */
export type CarPose = { x: number; y: number; yaw: number };
// ★★ 2026-10-14（reviewer 更正判据 ✓）：发出去的 `path`／`waypoints`
//    **前面接一段"车的真实位置"** ✓ —— ✗ 不接会死在一个 nav2 的搜索窗口上 ✗：
//   ★ 控制器用 `max_robot_pose_search_dist`（本栈 ≈ **3.99 m** ✓）在路径上找"离车最近的点" ✓
//     ⇒ ★ 车离线超过它 ⇒ **一个点都找不到** ⇒ 报
//       `Resulting plan has 0 poses in it` ⇒ ★ 立刻中止 ✓
//     （★ reviewer 实测：★ **33.46 m** ⇒ 2 s 中止、车位移 **0.047 m** ✗；
//       ★ **13.42 m ＋ 接这一段** ⇒ `completed` ✓ 末点离目标 **0.176 m** ✓）
//   ★ 判据：★ **车到 A（＝车在路网上的投影点）> 0.1 m** 才接 ✓（在线就没什么可接的 ✓）
//   ★ ★ 只动 `path` ∧ `waypoints` 两处 ✗ **别的字段一个不动** ✗ ——
//     ★ 桥硬校验 `path[0] ≈ waypoints[0]`（≤0.01 m ✓）⇒ 两处必须同步接 ✓
//   ★ ★ 本函数**不改** `exportAB`（★ "export 的算法不动" ✓）——
//     它是"发出去之前"的一道**装配** ✓；离线导出的 JSON 仍是纯规划结果 ✓
export function withCarStart(
  route: ReturnType<typeof exportAB>,
  car: CarPose | null,
): ReturnType<typeof exportAB> {
  if (!car) return route;
  const first = route.path[0];
  const lead = Math.hypot(car.x - first.x, car.y - first.y);
  if (!(lead > 0.1)) return route;
  const carEntry = {
    ...toGeoreferenced(mapToSvg({ x: car.x, y: car.y })),
    // ★★ 这个 yaw **不再过 `mapYaw()`** ✓（★ 2026-10-14 复核过 ✓）：
    //   ★ 车 pose 的 yaw **已经是 `map` 帧** ✓ —— 适配器发 `/odom` 时直接把
    //     TF `map→base_link` 的**旋转原样**拷进去 ✓（`frontend_adapter.py::publish_odom`：
    //     `tf.lookup_transform(MAP_FRAME, BASE_FRAME, …)` ⇒ `message.pose.pose.orientation = q` ✓）
    //     ⇒ ★ 桥从那个四元数算出来的 yaw 就是 map 帧 ✓
    //   ★ 而 `mapYaw()` 是给**另一路**用的 ✓：`extractWaypoints` 算的是 `atan2(-Δy_svg, Δx_svg)`
    //     —— ★ 那是**转过 `rotation_rad` 之前**的角度 ✓ ⇒ ★ 只有它需要 `mapYaw()` 转进 map 帧 ✓
    //     （见 `waypoints.ts::serializeWaypoints` ✓）
    //   ⇒ ★ 这里再过一次 `mapYaw()` ⇒ ★ 会把 `rotation_rad` **加第二次** ✗
    //     （★ 本标定 `rotation_rad = 0` ⇒ 现在两者同值 ⇒ ✗ 看不出来 ✗ —— 所以写成注释钉住 ✓）
    yaw: +car.yaw.toFixed(5),
  };
  return {
    ...route,
    length: +(route.length + lead).toFixed(3),
    path: [carEntry, ...route.path],
    waypoints: [
      {
        sequence: 1,
        ...carEntry,
        distance_m: 0,
        kind: "pass",
        reasons: ["start"],
        name: "小车当前位置",
        dwell_seconds: 0,
      },
      ...route.waypoints.map((w, i) => ({
        ...w,
        sequence: i + 2,
        distance_m: +(w.distance_m + lead).toFixed(3),
      })),
    ],
  };
}
export function exportAB(a: Point, b: Point, result: ABResult) {
  if (!result.plan || result.error) throw new Error("请先生成有效的 A→B 路线");
  return {
    version: "1.1.0",
    map_version: campus.version,
    frame_id: calibration.frame_id,
    calibration_version: calibration.version,
    calibration_status: calibration.status,
    scale_is_mock: false,
    georeferenced: true,
    geographic_coordinates_are_estimated: true,
    field_verified: false,
    executable_on_robot: false,
    units: "meters",
    axes: "X east, Y north",
    origin_wgs84: calibration.origin_wgs84,
    mode: "point_to_point",
    planner: "centerline_graph_astar",
    start: toGeoreferenced(a),
    goal: toGeoreferenced(b),
    blocked_roads: result.blocked,
    length: +(result.plan.length * scale).toFixed(3),
    waypoint_extraction: waypointOptions,
    path: result.points.map(toGeoreferenced),
    waypoints: serializeWaypoints(result.waypoints),
  };
}
