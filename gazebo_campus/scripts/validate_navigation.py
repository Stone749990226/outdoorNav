#!/usr/bin/env python3
"""Verify saved live motion against exact scene geometry and plot the evidence."""
import argparse
import json
import math
from pathlib import Path
from PIL import Image, ImageDraw
from shapely.geometry import LineString, Point, shape, box
from shapely.affinity import rotate
from shapely.ops import unary_union

HERE=Path(__file__).resolve().parents[1]


def validate(input_path):
    report=json.loads(input_path.read_text())
    scene=json.loads((HERE/'generated/scene_geometry.json').read_text())
    if report['scene_version']!=scene['version']:
        raise ValueError('Navigation evidence is for a different scene version')
    free=shape(scene['free'])
    traces=json.loads((HERE/'generated/traces.json').read_text())
    centreline=unary_union([LineString(t['points']) for t in traces if t['name'].startswith('network/')])
    image=Image.open(HERE/'generated/site_plan.png').convert('RGB'); draw=ImageDraw.Draw(image)
    xmin,ymin,xmax,ymax=scene['bounds']; scale=image.width/(xmax-xmin)
    colors=['#fc7e37','#d8a9ff','#28dbec','#ee66cc']
    summaries=[]
    for index,case in enumerate(report['cases']):
        runtime=[rotate(box(o['x']-o['width']/2,o['y']-o['depth']/2,o['x']+o['width']/2,o['y']+o['depth']/2),
                        o['yaw'],origin=(o['x'],o['y']),use_radians=True) for o in case.get('extra_obstacles',[])]
        case_free=free.difference(unary_union(runtime)) if runtime else free
        positions=case['positions']
        motion=LineString(positions) if len(positions)>1 else Point(positions[0]) if positions else None
        clearance=motion.distance(case_free.boundary) if motion is not None else 0
        inside=motion is not None and case_free.covers(motion)
        offcentre=max((Point(p).distance(centreline) for p in positions),default=0)
        passed=bool(case['passed_status'] and not case['contacts'] and case['max_command_m_s']<=.450001)
        if case['expected_success']:
            passed=passed and inside and clearance>=scene['robot_radius'] and case['goal_error_m']<=.4 and case['laser_messages']>0
        summary={k:v for k,v in case.items() if k not in ['positions','initial_plan','contacts']}
        summary.update(min_boundary_clearance_m=clearance,max_distance_from_original_centerline_m=offcentre,
                       collision_events=len(case['contacts']),passed=passed)
        if case['name']=='plaza_diagonal':
            planter=next(shape(o['footprint']) for o in scene['obstacles'] if o['name']=='plaza_planter')
            task=next(t for t in scene['tasks'] if t['name']==case['name'])
            summary['direct_line_blocked']=LineString([task['start'][:2],task['goal'][:2]]).intersects(planter)
            summary['actual_path_avoids_planter']=motion is not None and motion.distance(planter)>=scene['robot_radius']
            summary['passed']=passed and summary['direct_line_blocked'] and summary['actual_path_avoids_planter'] and offcentre>5
        if case['name']=='road_obstacle':
            direct=LineString([positions[0],positions[-1]]) if len(positions)>1 else Point(0,0)
            summary['direct_line_blocked']=any(direct.intersects(o) for o in runtime)
            summary['passed']=passed and bool(runtime) and summary['direct_line_blocked']
        summaries.append(summary)
        if case['expected_success'] and len(positions)>1:
            color=colors[index%len(colors)]
            pixels=[((x-xmin)*scale,(ymax-y)*scale) for x,y in positions]
            draw.line(pixels,fill=color,width=4)
            for x,y in [pixels[0],pixels[-1]]: draw.ellipse((x-5,y-5,x+5,y+5),fill=color)
            draw.text((18,58+index*18),case['name'],fill=color)
    summary={'scene_version':scene['version'],'passed':all(c['passed'] for c in summaries),
             'laser_messages':report.get('laser_messages',0),'scans_with_returns':report.get('scans_with_returns',0),'cases':summaries}
    target=input_path.with_name(input_path.stem+'_validation.json')
    target.write_text(json.dumps(summary,indent=2)+'\n')
    image.save(input_path.with_name(input_path.stem+'_paths.png'))
    print(json.dumps(summary,indent=2))
    if not summary['passed']: raise SystemExit(1)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',nargs='?',type=Path,default=HERE/'evidence/navigation.json')
    validate(parser.parse_args().input)
