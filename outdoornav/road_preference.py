"""Soft road preference, independent of sensor obstacles and confirmed keepouts."""

import heapq
import math
import numpy as np

from .routing import validate_point, snap


class RoadPreference:
    def __init__(self, bundle, offroad_cost=50, resolution=.5, keepouts=(), robot_radius=0.):
        self.bundle = bundle
        self.offroad_cost = int(offroad_cost)
        self.resolution = float(resolution)
        self.keepouts = keepouts
        self.robot_radius = float(robot_radius)
        self.grid = bundle['grid']
        self._cache = {}

    def policy(self):
        g = self.grid
        return {'mode': 'weighted', 'road_cost': 0, 'offroad_cost': self.offroad_cost,
                'resolution': self.resolution,
                'bounds': [*g['origin'], g['origin'][0]+g['width']*g['resolution'],
                           g['origin'][1]+g['height']*g['resolution']],
                'keepouts': self.keepouts, 'profile': 'linear_center_distance'}

    def _raster(self, resolution, blocked, for_planning):
        g = self.grid
        width = math.ceil(g['width']*g['resolution']/resolution)
        height = math.ceil(g['height']*g['resolution']/resolution)
        yy, xx = np.ogrid[:height, :width]
        x = g['origin'][0] + (xx+.5)*resolution
        y = g['origin'][1] + (yy+.5)*resolution
        costs = np.full((height, width), self.offroad_cost, dtype=np.float32)
        closed = np.zeros_like(costs, dtype=bool)
        for road in self.bundle['network']['roads']:
            radius = self.bundle['road_widths_m'][road['id']]/2
            for a, b in zip(road['points'], road['points'][1:]):
                dx, dy = b['x']-a['x'], b['y']-a['y']
                norm = dx*dx+dy*dy
                t = np.clip(((x-a['x'])*dx+(y-a['y'])*dy)/norm, 0, 1) if norm else 0
                distance_sq = (x-a['x']-t*dx)**2 + (y-a['y']-t*dy)**2
                if road['id'] in blocked:
                    padding = (self.robot_radius if for_planning else 0.) + resolution/math.sqrt(2)
                    closed |= (x-a['x']-t*dx)**2 + (y-a['y']-t*dy)**2 <= (radius+padding)**2
                else:
                    np.minimum(costs, self.offroad_cost*np.minimum(np.sqrt(distance_sq)/radius, 1.), out=costs)
        costs[closed] = 100
        for polygon in self.keepouts:
            inside = np.zeros_like(costs, dtype=bool)
            for a, b in zip(polygon, [*polygon[1:], polygon[0]]):
                if a[1] == b[1]:
                    continue
                inside ^= ((a[1] > y) != (b[1] > y)) & (x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])
            padding = (self.robot_radius if for_planning else 0.) + resolution/math.sqrt(2)
            for a,b in zip(polygon, [*polygon[1:], polygon[0]]):
                dx,dy=b[0]-a[0],b[1]-a[1];norm=dx*dx+dy*dy
                t=np.clip(((x-a[0])*dx+(y-a[1])*dy)/norm,0,1) if norm else 0
                inside |= (x-a[0]-t*dx)**2+(y-a[1]-t*dy)**2 <= padding**2
            costs[inside] = 100
        return costs

    def values(self, blocked=(), resolution=None, for_planning=False):
        unknown = set(blocked)-{r['id'] for r in self.bundle['network']['roads']}
        if unknown:
            raise ValueError('Unknown road IDs: ' + ', '.join(sorted(unknown)))
        resolution = self.resolution if resolution is None else resolution
        key = (float(resolution), tuple(sorted(set(blocked))), for_planning)
        if key not in self._cache:
            if len(self._cache) >= 8:
                self._cache.clear()
            self._cache[key] = self._raster(resolution, set(blocked), for_planning)
        return self._cache[key]

    def cell(self, point, values):
        p = validate_point(point)
        bounds = self.policy()['bounds']
        if not bounds[0] <= p['x'] < bounds[2] or not bounds[1] <= p['y'] < bounds[3]:
            raise ValueError('Point is outside the planning map')
        col = math.floor((p['x']-bounds[0])/self.resolution)
        row = math.floor((p['y']-bounds[1])/self.resolution)
        if values[row, col] >= 100:
            raise ValueError('Point is in a closed road or confirmed keepout')
        return row, col

    def check_point(self, point):
        self.cell(point, self.values(for_planning=True))

    def plan(self, start, goal, blocked=()):
        start, goal = validate_point(start), validate_point(goal)
        costs = self.values(blocked, for_planning=True)
        begin, end = self.cell(start, costs), self.cell(goal, costs)
        height, width = costs.shape
        queue = [(math.dist(begin, end), 0., begin)]
        best, parent = {begin: 0.}, {}
        while queue:
            _, score, current = heapq.heappop(queue)
            if score != best[current]:
                continue
            if current == end:
                cells = [current]
                while current != begin:
                    current = parent[current]
                    cells.append(current)
                cells.reverse()
                points = [{'x': self.grid['origin'][0]+(c+.5)*self.resolution,
                           'y': self.grid['origin'][1]+(r+.5)*self.resolution} for r,c in cells]
                if len(points) == 1:
                    points = [start, goal]
                else:
                    points[0], points[-1] = start, goal
                points = self._smooth(points, costs)
                edges = []
                for a, b in zip(points, points[1:]):
                    mid = {'x': (a['x']+b['x'])/2, 'y': (a['y']+b['y'])/2}
                    nearest = snap(self.bundle['network'], mid)
                    edges.append(nearest['road_id'] if nearest['distance_m'] <= self.bundle['road_widths_m'][nearest['road_id']]/2 else 'offroad')
                return {'start': start, 'goal': goal, 'start_road_id': snap(self.bundle['network'],start)['road_id'],
                        'reference_path': points, 'segment_roads': edges,
                        'road_ids': list(dict.fromkeys(e for e in edges if e != 'offroad')),
                        'length_m': sum(math.dist((a['x'],a['y']),(b['x'],b['y'])) for a,b in zip(points,points[1:])),
                        'weighted_cost': sum(self._line_cost(a,b,costs) for a,b in zip(points,points[1:])),
                        'road_offroad_cost': self.offroad_cost, 'blocked_road_ids': sorted(set(blocked)),
                        'planner': 'weighted_grid_astar'}
            r,c = current
            for dr,dc in ((0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
                nr,nc = r+dr,c+dc
                if not (0<=nr<height and 0<=nc<width) or costs[nr,nc]>=100:
                    continue
                if dr and dc and (costs[r,nc]>=100 or costs[nr,c]>=100):
                    continue
                # Zero road cost still has a distance cost. Occupancy cost 50
                # is a ~3x travel multiplier, similar to NavFn's neutral cost.
                step = math.hypot(dr,dc)*(1+(float(costs[r,c])+float(costs[nr,nc]))/50)
                candidate = score+step
                nxt = (nr,nc)
                if candidate < best.get(nxt,math.inf):
                    best[nxt],parent[nxt] = candidate,current
                    heapq.heappush(queue,(candidate+math.dist(nxt,end),candidate,nxt))
        raise ValueError('No path outside confirmed keepouts and closed roads')

    def _line_cost(self, a, b, costs):
        distance = math.dist((a['x'],a['y']),(b['x'],b['y']))
        count = max(1, math.ceil(distance/(self.resolution*.4)))
        total, previous = 0., None
        for i in range(count+1):
            p = {k:a[k]+(b[k]-a[k])*i/count for k in ('x','y')}
            try: cell = self.cell(p,costs)
            except ValueError:return math.inf
            if previous is not None and previous[0] != cell[0] and previous[1] != cell[1]:
                if costs[previous[0],cell[1]]>=100 or costs[cell[0],previous[1]]>=100:return math.inf
            previous = cell
            total += 1+float(costs[cell])/25
        return distance*total/(count+1)

    def _smooth(self, points, costs):
        result, index = [points[0]], 0
        while index < len(points)-1:
            end, budget = index+1, 0.
            for i in range(index+1,min(len(points),index+33)):
                budget += self._line_cost(points[i-1],points[i],costs)
                if self._line_cost(points[index],points[i],costs) <= budget*1.01+1e-6:
                    end = i
            if math.dist((result[-1]['x'],result[-1]['y']),(points[end]['x'],points[end]['y'])) > 1e-8:
                result.append(points[end])
            index = end
        return result
