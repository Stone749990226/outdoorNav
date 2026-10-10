#!/usr/bin/env python3
"""Publish ideal Gazebo world odometry and independently toggled survey overlays."""
import json
import math
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SceneNode(Node):
    def __init__(self):
        super().__init__('campus_scene')
        self.declare_parameter('scene_dir', '')
        directory=Path(self.get_parameter('scene_dir').value)
        self.odom=self.create_publisher(Odometry,'/odom',20)
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        transform=TransformStamped()
        transform.header.stamp=self.get_clock().now().to_msg()
        transform.header.frame_id='map'; transform.child_frame_id='odom'
        transform.transform.rotation.w=1.0
        self.static.sendTransform(transform)
        self.create_subscription(Odometry,'/campus/ground_truth',self.on_pose,20)
        self.scans=deque(maxlen=20)
        self.scan_publisher=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/campus/raw_scan',self.scans.append,qos_profile_sensor_data)
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.scene_publisher=self.create_publisher(MarkerArray,'/campus/scene',retained)
        self.scene_markers=self.load_scene(directory)
        self.obstacle_publisher=self.create_publisher(MarkerArray,'/campus/test_obstacles',retained)
        self.test_obstacles={}
        self.create_subscription(String,'/campus/obstacle_updates',self.on_obstacle,10)
        self.create_subscription(ModelStates,'/model_states',self.on_models,10)
        self.overlays={}
        traces=json.loads((directory/'traces.json').read_text())
        for category,color in [('raw',(1.0,0.8,0.1)),('filtered',(0.2,0.9,0.8)),('network',(0.9,0.35,0.3))]:
            publisher=self.create_publisher(MarkerArray,f'/campus/traces/{category}',retained)
            array=MarkerArray()
            for i,trace in enumerate(t for t in traces if t['name'].startswith(category+'/')):
                marker=Marker(); marker.header.frame_id='map'; marker.ns=category; marker.id=i
                marker.type=Marker.LINE_STRIP; marker.action=Marker.ADD
                marker.pose.orientation.w=1.0; marker.scale.x=0.12
                marker.color.r,marker.color.g,marker.color.b=color; marker.color.a=1.0
                marker.points=[Point(x=float(x),y=float(y),z=0.05) for x,y in trace['points']]
                array.markers.append(marker)
            self.overlays[category]=(publisher,array)
        self.create_timer(2.0,self.publish_overlays)
        self.publish_overlays()

    def on_pose(self,message):
        # gazebo_ros_p3d expresses velocity in its reference (world) frame.
        # nav_msgs/Odometry requires twist in child_frame_id, i.e. base_link.
        q=message.pose.pose.orientation
        x,y,z,w=q.x,q.y,q.z,q.w
        for vector in (message.twist.twist.linear,message.twist.twist.angular):
            vx,vy,vz=vector.x,vector.y,vector.z
            vector.x=(1-2*y*y-2*z*z)*vx+2*(x*y+z*w)*vy+2*(x*z-y*w)*vz
            vector.y=2*(x*y-z*w)*vx+(1-2*x*x-2*z*z)*vy+2*(y*z+x*w)*vz
            vector.z=2*(x*z+y*w)*vx+2*(y*z-x*w)*vy+(1-2*x*x-2*y*y)*vz
        message.header.frame_id='odom'
        message.child_frame_id='base_link'
        self.odom.publish(message)
        t=TransformStamped(); t.header=message.header; t.child_frame_id='base_link'
        t.transform.translation.x=message.pose.pose.position.x
        t.transform.translation.y=message.pose.pose.position.y
        t.transform.translation.z=message.pose.pose.position.z
        t.transform.rotation=message.pose.pose.orientation
        self.tf.sendTransform(t)
        # Humble's asynchronous laser TF filter can deadlock when a scan needs a
        # transform not received yet (Buffer::waitForTransform lock inversion).
        # Release unmodified measurements only after 0.2 s of subsequent TF has
        # been published. This keeps real scan timestamps and avoids future TF.
        now_ns=t.header.stamp.sec*1_000_000_000+t.header.stamp.nanosec
        while self.scans:
            stamp=self.scans[0].header.stamp
            age=now_ns-(stamp.sec*1_000_000_000+stamp.nanosec)
            if age<200_000_000:
                break
            scan=self.scans.popleft()
            if age<1_000_000_000:
                self.scan_publisher.publish(scan)

    def publish_overlays(self):
        self.scene_publisher.publish(self.scene_markers)
        self.publish_obstacles()
        for publisher,array in self.overlays.values():
            publisher.publish(array)

    def on_obstacle(self,message):
        try:
            spec=json.loads(message.data)
            if not spec['name'].startswith('test_'):
                return
            if spec['action']=='remove':
                self.test_obstacles.pop(spec['name'],None)
            else:
                marker=Marker(); marker.header.frame_id='map'; marker.ns=spec['name']
                marker.type=Marker.CUBE; marker.action=Marker.ADD
                marker.pose.position.x=float(spec['x']); marker.pose.position.y=float(spec['y'])
                marker.pose.position.z=spec['height']/2; marker.pose.orientation.w=1.0
                marker.scale.x=spec['width']; marker.scale.y=spec['depth']; marker.scale.z=spec['height']
                marker.color.r=1.0; marker.color.g=.35; marker.color.b=.08; marker.color.a=1.0
                self.test_obstacles[spec['name']]=marker
            self.publish_obstacles()
        except (ValueError,KeyError,TypeError) as error:
            self.get_logger().warning(f'Ignoring invalid obstacle update: {error}')

    def on_models(self,message):
        for name,pose in zip(message.name,message.pose):
            if name in self.test_obstacles:
                # Model origins are at ground level; the box is centred at h/2.
                marker=self.test_obstacles[name]
                marker.pose.position.x=pose.position.x; marker.pose.position.y=pose.position.y
                marker.pose.position.z=pose.position.z+marker.scale.z/2
                marker.pose.orientation=pose.orientation

    def publish_obstacles(self):
        clear=Marker(); clear.action=Marker.DELETEALL
        self.obstacle_publisher.publish(MarkerArray(markers=[clear,*self.test_obstacles.values()]))

    def load_scene(self,directory):
        """Render exactly the Gazebo visuals in RViz, including building heights."""
        root=ET.parse(directory/'models/campus_scene/model.sdf')
        array=MarkerArray()
        for i,visual in enumerate(root.findall('.//visual')):
            marker=Marker(); marker.header.frame_id='map'; marker.id=i
            name=visual.attrib['name']
            marker.ns='buildings' if name.startswith(('main_tower','east_podium')) else 'vegetation' if name.startswith('tree') else 'site'
            marker.action=Marker.ADD
            x,y,z,roll,pitch,yaw=map(float,visual.findtext('pose','0 0 0 0 0 0').split())
            marker.pose.position.x=x; marker.pose.position.y=y; marker.pose.position.z=z
            cr,sr=math.cos(roll/2),math.sin(roll/2)
            cp,sp=math.cos(pitch/2),math.sin(pitch/2)
            cy,sy=math.cos(yaw/2),math.sin(yaw/2)
            q=marker.pose.orientation
            q.x=sr*cp*cy-cr*sp*sy; q.y=cr*sp*cy+sr*cp*sy
            q.z=cr*cp*sy-sr*sp*cy; q.w=cr*cp*cy+sr*sp*sy
            color=list(map(float,visual.findtext('material/diffuse','0.5 0.5 0.5 1').split()))
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=color
            geometry=visual.find('geometry')
            if geometry.find('box') is not None:
                marker.type=Marker.CUBE
                marker.scale.x,marker.scale.y,marker.scale.z=map(float,geometry.findtext('box/size').split())
            elif geometry.find('cylinder') is not None:
                marker.type=Marker.CYLINDER
                marker.scale.x=marker.scale.y=2*float(geometry.findtext('cylinder/radius'))
                marker.scale.z=float(geometry.findtext('cylinder/length'))
            elif geometry.find('sphere') is not None:
                marker.type=Marker.SPHERE
                marker.scale.x=marker.scale.y=marker.scale.z=2*float(geometry.findtext('sphere/radius'))
            else:
                marker.type=Marker.MESH_RESOURCE
                uri=geometry.findtext('mesh/uri').removeprefix('model://')
                marker.mesh_resource=(directory/'models'/uri).as_uri()
                marker.mesh_use_embedded_materials=False
                marker.scale.x=marker.scale.y=marker.scale.z=1.0
            array.markers.append(marker)
        return array


if __name__=='__main__':
    rclpy.init(); node=SceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
