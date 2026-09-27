import { campus, containsPoint, dist, regions, roads } from "./map";
import type { Point } from "./types";

export const navigation = campus.navigation;
const radius = navigation.road_width_units / 2;
const robot = navigation.robot_radius_units;
const step = navigation.grid_resolution_units;
const nx = Math.ceil(campus.width / step),
  ny = Math.ceil(campus.height / step);
const gridMargin = robot + step / Math.SQRT2;
export function segmentDistance(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x,
    dy = b.y - a.y,
    len = dx * dx + dy * dy;
  const t = len
    ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len))
    : 0;
  return Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
function lineDistance(p: Point, points: Point[]) {
  let best = Infinity;
  for (let i = 1; i < points.length; i++)
    best = Math.min(best, segmentDistance(p, points[i - 1], points[i]));
  return best;
}
// Keep the common junction disks open when closing an adjoining road. The
// interior closure still spans its full width, so it cannot be passed sideways.
const blockedLines = roads.map((r) => {
  const points = r.points.map((p) => ({ ...p }));
  const inset = radius + robot + step;
  for (const [i, j] of [
    [0, 1],
    [points.length - 1, points.length - 2],
  ]) {
    const a = points[i],
      b = points[j],
      length = dist(a, b),
      t = Math.min(inset / length, 0.45);
    points[i] = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
  }
  return { id: r.id, points };
});
const obstaclePolygons = regions.map((r) => ({
  points: [...r.points, r.points[0]],
  polygon: r.points,
}));
const roadSegments = roads.flatMap((r) =>
  r.points.slice(1).map((b, i) => ({ a: r.points[i], b })),
);
/** Conservative clearance to the edge of modeled free space, in drawing units. */
export function clearance(p: Point, blocked: string[] = []): number {
  if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) return -Infinity;
  let nearest = Infinity;
  for (const s of roadSegments)
    nearest = Math.min(nearest, segmentDistance(p, s.a, s.b));
  let result = Math.min(
    radius - nearest,
    p.x,
    p.y,
    campus.width - p.x,
    campus.height - p.y,
  );
  if (result < 0) return result;
  for (const r of obstaclePolygons) {
    if (containsPoint(p, r.polygon)) return -1;
    result = Math.min(result, lineDistance(p, r.points));
    if (result < 0) return result;
  }
  for (const r of blockedLines)
    if (blocked.includes(r.id))
      result = Math.min(result, lineDistance(p, r.points) - radius);
  return result;
}
export const isTraversable = (p: Point, blocked: string[] = []) =>
  clearance(p, blocked) >= robot + 0.02;
/** Lipschitz clearance bounds certify each straight segment, not only its endpoints. */
export function segmentSafe(
  a: Point,
  b: Point,
  blocked: string[] = [],
): boolean {
  const ca = clearance(a, blocked) - robot,
    cb = clearance(b, blocked) - robot;
  if (ca < 0.02 || cb < 0.02) return false;
  const check = (
    p: Point,
    q: Point,
    cp: number,
    cq: number,
    depth: number,
  ): boolean => {
    const length = dist(p, q);
    if (Math.max(cp, cq) >= length) return true;
    const m = { x: (p.x + q.x) / 2, y: (p.y + q.y) / 2 },
      cm = clearance(m, blocked) - robot;
    if (cm < 0.02) return false;
    if (cm >= length / 2) return true;
    if (depth >= 18) return false;
    return check(p, m, cp, cm, depth + 1) && check(m, q, cm, cq, depth + 1);
  };
  return check(a, b, ca, cb, 0);
}
type Grid = {
  free: Uint8Array;
  path: string;
  component: Int32Array;
  count: number;
};
let baseClearance: Float32Array | undefined;
const cache = new Map<string, Grid>();
const center = (i: number): Point => ({
  x: ((i % nx) + 0.5) * step,
  y: (Math.floor(i / nx) + 0.5) * step,
});
export function freeGrid(blocked: string[] = []): Grid {
  const key = [...blocked].sort().join("|");
  const existing = cache.get(key);
  if (existing) return existing;
  if (!baseClearance) {
    baseClearance = new Float32Array(nx * ny);
    for (let i = 0; i < baseClearance.length; i++)
      baseClearance[i] = clearance(center(i));
  }
  const closures = blockedLines.filter((r) => blocked.includes(r.id));
  const free = new Uint8Array(nx * ny),
    component = new Int32Array(nx * ny);
  component.fill(-1);
  const rows: string[] = [];
  let count = 0;
  for (let y = 0; y < ny; y++) {
    let run = -1;
    for (let x = 0; x <= nx; x++) {
      const i = y * nx + x;
      if (
        x < nx &&
        baseClearance[i] >= gridMargin &&
        closures.every(
          (r) => lineDistance(center(i), r.points) >= radius + gridMargin,
        )
      ) {
        free[i] = 1;
        count++;
        if (run < 0) run = x;
      } else if (run >= 0) {
        rows.push(
          `M${run * step},${y * step}h${(x - run) * step}v${step}h${-(x - run) * step}z`,
        );
        run = -1;
      }
    }
  }
  let label = 0;
  const queue = new Int32Array(nx * ny);
  for (let i = 0; i < free.length; i++)
    if (free[i] && component[i] < 0) {
      let head = 0,
        tail = 1;
      queue[0] = i;
      component[i] = label;
      while (head < tail) {
        const at = queue[head++],
          x = at % nx,
          y = Math.floor(at / nx);
        for (const [dx, dy] of [
          [1, 0],
          [-1, 0],
          [0, 1],
          [0, -1],
        ]) {
          const px = x + dx,
            py = y + dy,
            next = py * nx + px;
          if (
            px >= 0 &&
            px < nx &&
            py >= 0 &&
            py < ny &&
            free[next] &&
            component[next] < 0
          ) {
            component[next] = label;
            queue[tail++] = next;
          }
        }
      }
      label++;
    }
  const grid = { free, path: rows.join(""), component, count };
  if (cache.size > 5) cache.clear();
  cache.set(key, grid);
  return grid;
}
class Heap {
  private entries: { id: number; score: number }[] = [];
  push(id: number, score: number) {
    const a = this.entries;
    let i = a.length;
    a.push({ id, score });
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].score <= score) break;
      a[i] = a[p];
      i = p;
    }
    a[i] = { id, score };
  }
  pop() {
    const a = this.entries;
    if (!a.length) return null;
    const first = a[0],
      last = a.pop()!;
    if (a.length) {
      let i = 0;
      while (i * 2 + 1 < a.length) {
        let c = i * 2 + 1;
        if (c + 1 < a.length && a[c + 1].score < a[c].score) c++;
        if (a[c].score >= last.score) break;
        a[i] = a[c];
        i = c;
      }
      a[i] = last;
    }
    return first;
  }
}
function anchors(p: Point, grid: Grid, blocked: string[]) {
  const x = Math.floor(p.x / step),
    y = Math.floor(p.y / step),
    result: number[] = [];
  for (let dy = -3; dy <= 3; dy++)
    for (let dx = -3; dx <= 3; dx++) {
      const cx = x + dx,
        cy = y + dy,
        i = cy * nx + cx;
      if (
        cx >= 0 &&
        cx < nx &&
        cy >= 0 &&
        cy < ny &&
        grid.free[i] &&
        segmentSafe(p, center(i), blocked)
      )
        result.push(i);
    }
  return result;
}
export type AreaRoute = { points: Point[]; error: string | null };
export function areaRoute(
  a: Point,
  b: Point,
  blocked: string[] = [],
): AreaRoute {
  if (!isTraversable(a, blocked))
    return {
      points: [],
      error: "A 点位于禁行、未知、封闭区域或离边界过近，请重新选择。",
    };
  if (!isTraversable(b, blocked))
    return {
      points: [],
      error: "B 点位于禁行、未知、封闭区域或离边界过近，请重新选择。",
    };
  if (segmentSafe(a, b, blocked))
    return { points: dist(a, b) < 1e-8 ? [a] : [a, b], error: null };
  const grid = freeGrid(blocked),
    starts = anchors(a, grid, blocked),
    ends = anchors(b, grid, blocked);
  if (!starts.length || !ends.length)
    return {
      points: [],
      error:
        "选点附近没有满足当前分辨率与安全余量的连接，请选择可通行区域更内侧的位置。",
    };
  const components = new Set(starts.map((i) => grid.component[i]));
  if (!ends.some((i) => components.has(grid.component[i])))
    return {
      points: [],
      error: "A 与 B 的可通行区域不连通，请恢复道路或重新选择目标。",
    };
  const endSet = new Set(ends),
    g = new Float64Array(nx * ny),
    parent = new Int32Array(nx * ny),
    closed = new Uint8Array(nx * ny);
  g.fill(Infinity);
  parent.fill(-1);
  const heap = new Heap();
  for (const i of starts) {
    g[i] = dist(a, center(i));
    heap.push(i, g[i] + dist(center(i), b));
  }
  let goal = -1,
    bestGoal = Infinity;
  for (let item = heap.pop(); item; item = heap.pop()) {
    const id = item.id;
    if (closed[id]) continue;
    if (item.score >= bestGoal) break;
    closed[id] = 1;
    const p = center(id),
      x = id % nx,
      y = Math.floor(id / nx);
    if (endSet.has(id)) {
      const score = g[id] + dist(p, b);
      if (score < bestGoal) {
        bestGoal = score;
        goal = id;
      }
    }
    for (let dy = -1; dy <= 1; dy++)
      for (let dx = -1; dx <= 1; dx++) {
        if (!dx && !dy) continue;
        const xx = x + dx,
          yy = y + dy,
          next = yy * nx + xx;
        if (
          xx < 0 ||
          xx >= nx ||
          yy < 0 ||
          yy >= ny ||
          !grid.free[next] ||
          closed[next]
        )
          continue;
        if (dx && dy && (!grid.free[y * nx + xx] || !grid.free[yy * nx + x]))
          continue;
        const candidate = g[id] + Math.hypot(dx, dy) * step;
        if (candidate < g[next]) {
          g[next] = candidate;
          parent[next] = id;
          heap.push(next, candidate + dist(center(next), b));
        }
      }
  }
  if (goal < 0) return { points: [], error: "无法找到满足通行约束的路线。" };
  const raw: Point[] = [b];
  for (let id = goal; id >= 0; id = parent[id]) raw.push(center(id));
  raw.push(a);
  raw.reverse();
  // Remove grid staircase jitter only when the whole replacement chord is certified.
  const points: Point[] = [a];
  let at = 0;
  while (at < raw.length - 1) {
    let next = raw.length - 1;
    while (next > at + 1 && !segmentSafe(raw[at], raw[next], blocked)) next--;
    if (!segmentSafe(raw[at], raw[next], blocked))
      return {
        points: [],
        error: "路线在边界处校验失败，请重新选择更内侧的点。",
      };
    if (dist(points.at(-1)!, raw[next]) > 1e-8) points.push(raw[next]);
    at = next;
  }
  return { points, error: null };
}
