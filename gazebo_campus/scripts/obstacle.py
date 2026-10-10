#!/usr/bin/env python3
"""Add/remove a temporary physical box to test laser-based replanning on roads."""
import argparse
import json
import math
from pathlib import Path
import re
import time
import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import String

HERE=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['add','remove'])
    parser.add_argument('--name',default='road_box')
    parser.add_argument('--x',type=float,default=98.8)
    parser.add_argument('--y',type=float,default=-42.0)
    parser.add_argument('--width',type=float,default=1.6)
    parser.add_argument('--depth',type=float,default=1.6)
    parser.add_argument('--height',type=float,default=1.0)
    args=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_]{1,40}',args.name): parser.error('Name must contain 1-40 letters, digits or underscores')
    if not all(math.isfinite(v) for v in [args.x,args.y,args.width,args.depth,args.height]): parser.error('Coordinates and sizes must be finite')
    if not all(.1<=v<=8 for v in [args.width,args.depth,args.height]): parser.error('Each box dimension must be between 0.1 and 8 metres')
    name='test_'+args.name
    if args.action=='add':
        scene=json.loads((HERE/'generated/scene_geometry.json').read_text())
        xmin,ymin,xmax,ymax=scene['bounds']; res=scene['resolution']
        header,dims,maximum,pixels=(HERE/'generated/campus.pgm').read_bytes().split(b'\n',3)
        cols,rows=map(int,dims.split())
        lo=math.floor((args.x-args.width/2-xmin)/res); hi=math.floor((args.x+args.width/2-xmin)/res)
        top=math.floor((ymax-args.y-args.depth/2)/res); bottom=math.floor((ymax-args.y+args.depth/2)/res)
        if lo<0 or hi>=cols or top<0 or bottom>=rows or any(v!=254 for row in range(top,bottom+1) for v in pixels[row*cols+lo:row*cols+hi+1]):
            parser.error('Place the entire temporary box on an existing road or plaza')
    rclpy.init(); node=rclpy.create_node('campus_obstacle_tool')
    try:
        client=node.create_client(SpawnEntity if args.action=='add' else DeleteEntity,'/spawn_entity' if args.action=='add' else '/delete_entity')
        if not client.wait_for_service(timeout_sec=15): raise RuntimeError('Start gazebo_campus/run.sh first')
        if args.action=='add':
            request=SpawnEntity.Request(); request.name=name
            request.reference_frame='world'; request.initial_pose.position.x=args.x; request.initial_pose.position.y=args.y
            request.initial_pose.orientation.w=1.0
            size=f'{args.width} {args.depth} {args.height}'
            request.xml=f'<sdf version="1.6"><model name="{name}"><static>true</static><link name="box"><pose>0 0 {args.height/2} 0 0 0</pose><collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{size}</size></box></geometry><material><ambient>1 .35 .08 1</ambient><diffuse>1 .35 .08 1</diffuse></material></visual></link></model></sdf>'
        else:
            request=DeleteEntity.Request(); request.name=name
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=20)
        if not future.done() or not future.result().success: raise RuntimeError(future.result().status_message if future.done() else 'Gazebo service timed out')
        publisher=node.create_publisher(String,'/campus/obstacle_updates',10)
        until=time.monotonic()+3
        while publisher.get_subscription_count()==0 and time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.1)
        publisher.publish(String(data=json.dumps({**vars(args),'name':name})))
        until=time.monotonic()+.5
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
        print(f'{args.action}: {name}; {future.result().status_message}')
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
