#!/usr/bin/env python3
"""Move the Gazebo GUI camera to reproducible scene views (requires gzclient)."""
import argparse
import math
import os
import subprocess

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('view',choices=['oblique','top','plaza'])
args=parser.parse_args()
# Gazebo cameras look along local +X; positive pitch points down.
views={'oblique':(210,-255,195,.60,2.22), 'top':(49,-38,240,math.pi/2,math.pi/2),
       'plaza':(83,-102,40,.65,2.25)}
x,y,z,pitch,yaw=views[args.view]
qx=-math.sin(pitch/2)*math.sin(yaw/2)
qy=math.sin(pitch/2)*math.cos(yaw/2)
qz=math.cos(pitch/2)*math.sin(yaw/2)
qw=math.cos(pitch/2)*math.cos(yaw/2)
message=f'camera {{ name: "user_camera" pose {{ position {{ x: {x} y: {y} z: {z} }} orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }} }}'
env=os.environ.copy()
env.setdefault('GAZEBO_MASTER_URI','http://127.0.0.1:11456')
env.setdefault('GAZEBO_IP','127.0.0.1')
subprocess.run(['gz','topic','-p','/gazebo/north_campus/gui','-m',message],env=env,check=True,timeout=15)
