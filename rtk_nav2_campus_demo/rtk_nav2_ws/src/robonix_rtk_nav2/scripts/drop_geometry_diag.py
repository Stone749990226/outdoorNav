#!/usr/bin/env python3
"""One-frame debug of measured depth geometry. No scene coordinates are fed to detector."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from robonix_rtk_nav2.depth_drop import rotation, edge_columns

rclpy.init()
n = Node('drop_geometry_diag')
b = Buffer(); listener = TransformListener(b, n)
state = {}
n.create_subscription(CameraInfo, '/rtk/camera/camera_info',
                      lambda m: state.update(info=m), qos_profile_sensor_data)
n.create_subscription(Image, '/rtk/camera/depth_image',
                      lambda m: state.update(depth=m), qos_profile_sensor_data)
stop = time.time() + 35
while time.time() < stop:
    rclpy.spin_once(n, timeout_sec=.2)
    if 'info' not in state or 'depth' not in state:
        continue
    msg = state['depth']; info = state['info']
    try:
        tf = b.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
    except Exception:
        continue
    h, w = msg.height, msg.width
    z = np.frombuffer(msg.data, dtype=np.float32).reshape(h, msg.step//4)[:, :w]
    u, v = np.meshgrid(np.arange(0, w, 2), np.arange(0, h, 2))
    z = z[::2, ::2]
    valid = np.isfinite(z) & (z > .05) & (z < 7.95)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    camera = np.stack(((u-cx)*z/fx, (v-cy)*z/fy, z), axis=-1)
    q = tf.transform.rotation
    t = tf.transform.translation
    pos = np.array([t.x,t.y,t.z])
    world = camera @ rotation(q).T + pos
    edges, upper, lower, heading = edge_columns(
        np.frombuffer(msg.data, dtype=np.float32).reshape(h,msg.step//4)[:, :w],
        (fx,fy,cx,cy),tf,.19,.15,.06)
    print('image',w,h,'valid',valid.sum(),'frame',msg.header.frame_id)
    print('camera pose',pos,'heading',heading,'depth range',np.nanmin(z),np.nanmax(z))
    for name, mask in [('upper',upper),('lower',lower)]:
        rows, cols = np.where(mask)
        if len(rows):
            pts = world[mask]
            d = (pts-pos) @ heading
            print(name,'count',len(rows),'columns',len(set(cols)),'row',rows.min(),rows.max(),
                  'x range',pts[:,0].min(),pts[:,0].max(), 'y range',pts[:,1].min(),pts[:,1].max(),
                  'z range',pts[:,2].min(),pts[:,2].max(), 'forward',d.min(),d.max())
        else: print(name,'none')
    print('edges',edges.shape)
    for col in range(0, w//2, 20):
        a = np.where(upper[:,col])[0]; c = np.where(lower[:,col])[0]
        if len(a) and len(c):
            d0=(world[a,col]-pos)@heading; d1=(world[c,col]-pos)@heading
            print('col',col*2,'upper',len(a),'lower',len(c),'furthest upper',d0.max(),
                  'nearest lower',d1.min(), 'gap',d1.min()-d0.max())
    break
n.destroy_node(); rclpy.shutdown()
