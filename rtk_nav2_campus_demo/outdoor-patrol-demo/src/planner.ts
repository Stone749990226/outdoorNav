import { roads, campus, dist } from "./map";
import type { Point, Plan, Segment, Stop, Task } from "./types";
export type Snap = {
  point: Point;
  edgeId: string;
  offset: number;
  distance: number;
};
export function snap(p: Point, excluded: string[] = []): Snap | null {
  let best: Snap | null = null;
  for (const road of roads) {
    if (excluded.includes(road.id)) continue;
    let offset = 0;
    for (let i = 1; i < road.points.length; i++) {
      const a = road.points[i - 1],
        b = road.points[i],
        length = dist(a, b);
      const t = Math.max(
        0,
        Math.min(
          1,
          ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) /
            (length * length),
        ),
      );
      const point = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
      const distance = dist(p, point);
      if (!best || distance < best.distance)
        best = {
          point,
          edgeId: road.id,
          offset: offset + t * length,
          distance,
        };
      offset += length;
    }
  }
  return best;
}
type Arc = { to: string; edgeId: string; length: number };
export function route(
  start: Point,
  end: Point,
  blocked: string[],
): { a: Point; b: Point; edgeId: string }[] | null {
  const startSnap = snap(start, blocked),
    endSnap = snap(end, blocked);
  // Never silently move an endpoint from a closed road to a different road.
  if (
    !startSnap ||
    !endSnap ||
    startSnap.distance > 0.000001 ||
    endSnap.distance > 0.000001
  )
    return null;
  startSnap.point = { ...start };
  endSnap.point = { ...end };
  const key = (p: Point) => `${p.x.toFixed(9)},${p.y.toFixed(9)}`;
  const points = new Map<string, Point>(),
    graph = new Map<string, Arc[]>();
  const add = (p: Point) => {
    const k = key(p);
    points.set(k, p);
    if (!graph.has(k)) graph.set(k, []);
    return k;
  };
  for (const r of roads) {
    if (blocked.includes(r.id)) continue;
    const list: { point: Point; offset: number }[] = [];
    let offset = 0;
    r.points.forEach((p, i) => {
      if (i) offset += dist(r.points[i - 1], p);
      list.push({ point: p, offset });
    });
    for (const s of [startSnap, endSnap]) if (s.edgeId === r.id) list.push(s);
    list.sort((a, b) => a.offset - b.offset);
    for (let i = 1; i < list.length; i++) {
      const a = list[i - 1].point,
        b = list[i].point,
        ka = add(a),
        kb = add(b),
        length = dist(a, b);
      if (length < 0.0001) continue;
      graph.get(ka)!.push({ to: kb, edgeId: r.id, length });
      graph.get(kb)!.push({ to: ka, edgeId: r.id, length });
    }
  }
  const begin = key(startSnap.point),
    target = key(endSnap.point);
  const open = new Set([begin]),
    g = new Map([[begin, 0]]),
    previous = new Map<string, { key: string; edgeId: string }>();
  while (open.size) {
    let current = "",
      best = Infinity;
    for (const id of open) {
      const score = g.get(id)! + dist(points.get(id)!, endSnap.point);
      if (score < best) {
        best = score;
        current = id;
      }
    }
    if (current === target) {
      const result: { a: Point; b: Point; edgeId: string }[] = [];
      while (current !== begin) {
        const prev = previous.get(current)!;
        result.unshift({
          a: points.get(prev.key)!,
          b: points.get(current)!,
          edgeId: prev.edgeId,
        });
        current = prev.key;
      }
      return result;
    }
    open.delete(current);
    for (const arc of graph.get(current) ?? []) {
      const candidate = g.get(current)! + arc.length;
      if (candidate < (g.get(arc.to) ?? Infinity) - 1e-8) {
        g.set(arc.to, candidate);
        previous.set(arc.to, { key: current, edgeId: arc.edgeId });
        open.add(arc.to);
      }
    }
  }
  return null;
}
export function makePlan(
  start: Point,
  tasks: Task[],
  home: Point,
  returnHome: boolean,
  blocked: string[],
): { plan: Plan | null; error: string | null } {
  const goals: (Task & { returning?: boolean })[] = [...tasks];
  if (returnHome)
    goals.push({
      id: "__home",
      name: "返回起点",
      point: home,
      dwell: 0,
      returning: true,
    });
  let current = start,
    distance = 0;
  const segments: Segment[] = [],
    stops: Stop[] = [];
  for (const task of goals) {
    const result = route(current, task.point, blocked);
    if (!result)
      return {
        plan: null,
        error: `无法到达「${task.name}」：道路已断开，请恢复连接后重新规划。`,
      };
    for (const segment of result) {
      const length = dist(segment.a, segment.b);
      segments.push({ ...segment, start: distance, end: distance + length });
      distance += length;
    }
    stops.push({ ...task, distance });
    current = task.point;
  }
  return {
    plan: {
      start,
      segments,
      stops,
      length: distance,
      taskOrder: tasks.map((t) => t.id),
    },
    error: null,
  };
}
export function positionAt(
  plan: Plan,
  distance: number,
): { point: Point; heading: number; edgeId: string | null } {
  const segment =
    plan.segments.find((s) => s.end > distance + 1e-7) ?? plan.segments.at(-1);
  if (!segment) return { point: plan.start, heading: 0, edgeId: null };
  const t = Math.max(
    0,
    Math.min(1, (distance - segment.start) / (segment.end - segment.start)),
  );
  return {
    point: {
      x: segment.a.x + (segment.b.x - segment.a.x) * t,
      y: segment.a.y + (segment.b.y - segment.a.y) * t,
    },
    heading: Math.atan2(segment.b.y - segment.a.y, segment.b.x - segment.a.x),
    edgeId: segment.edgeId,
  };
}
export function reachableRoads(
  position: Point,
  blocked: string[],
): Set<string> {
  const result = new Set<string>();
  for (const r of roads)
    if (!blocked.includes(r.id) && route(position, r.points[0], blocked))
      result.add(r.id);
  return result;
}
export function enteredRoad(position: Point): string | null {
  if (campus.nodes.some((n) => dist(n, position) < 0.1)) return null;
  const s = snap(position);
  return s && s.distance < 0.1 ? s.edgeId : null;
}
