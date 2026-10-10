#!/usr/bin/env python3
"""Exercise live Gazebo/Nav2, saving measured motion for geometric postchecks.

Run in the simulator ROS domain with Ubuntu /usr/bin/python3. The robot is reset
between cases through Gazebo's service, never while an action is still running.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry, Path as RosPath
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker

HERE=Path(__file__).resolve().parents[1]


class Check(Node):
    def __init__(self):
        super().__init__('campus_acceptance', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pose=None; self.active=None; self.scan_count=0; self.finite_scan_count=0; self.contact_messages=0
        self.extra_obstacles=[]
        self.create_subscription(MarkerArray,'/campus/test_obstacles',self.on_obstacles,10)
        self.create_subscription(Odometry,'/odom',self.on_odom,20)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(ContactsState,'/campus/contacts',self.on_contacts,qos_profile_sensor_data)
        self.create_subscription(RosPath,'/plan',self.on_plan,10)
        self.create_subscription(Twist,'/cmd_vel',self.on_command,10)
        self.stop=self.create_publisher(Twist,'/cmd_vel',10)
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.reset=self.create_client(SetEntityState,'/set_entity_state')
        self.clear=[self.create_client(ClearEntireCostmap,f'/{name}_costmap/clear_entirely_{name}_costmap') for name in ['local','global']]

    def on_odom(self,msg):
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        self.pose=[p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)]
        if self.active is not None:
            if not self.active['positions'] or math.dist(self.pose[:2],self.active['positions'][-1])>.025:
                self.active['positions'].append(self.pose[:2])

    def on_obstacles(self,msg):
        self.extra_obstacles=[{'name':m.ns,'x':m.pose.position.x,'y':m.pose.position.y,
                               'width':m.scale.x,'depth':m.scale.y,
                               'yaw':2*math.atan2(m.pose.orientation.z,m.pose.orientation.w)}
                              for m in msg.markers if m.action==Marker.ADD and m.type==Marker.CUBE]

    def on_scan(self,msg):
        self.scan_count+=1
        if any(math.isfinite(r) and msg.range_min<=r<msg.range_max for r in msg.ranges): self.finite_scan_count+=1

    def on_contacts(self,msg):
        self.contact_messages+=1
        if self.active is not None and msg.states:
            self.active['contacts'] += [{'a':s.collision1_name,'b':s.collision2_name} for s in msg.states]

    def on_plan(self,msg):
        if self.active is not None:
            points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
            if not self.active['initial_plan'] and points: self.active['initial_plan']=points

    def on_command(self,msg):
        if self.active is not None:
            self.active['max_command_m_s']=max(self.active['max_command_m_s'],abs(msg.linear.x))

    def until(self,predicate,timeout=30):
        end=time.monotonic()+timeout
        while rclpy.ok() and not predicate():
            if time.monotonic()>end: raise TimeoutError('Timed out waiting for simulation state')
            rclpy.spin_once(self,timeout_sec=.1)

    def result(self,future,timeout=30):
        self.until(future.done,timeout)
        return future.result()

    def settle(self,seconds):
        end=time.monotonic()+seconds
        self.until(lambda:time.monotonic()>=end,seconds+2)

    def run(self,task):
        self.stop.publish(Twist()); self.settle(.3)
        req=SetEntityState.Request(); req.state.name='campus_robot'; req.state.reference_frame='world'
        x,y,yaw=task['start']
        req.state.pose.position.x=float(x); req.state.pose.position.y=float(y); req.state.pose.position.z=.02
        req.state.pose.orientation.z=math.sin(yaw/2); req.state.pose.orientation.w=math.cos(yaw/2)
        if not self.result(self.reset.call_async(req)).success: raise RuntimeError('Gazebo reset failed')
        self.until(lambda:self.pose is not None and math.dist(self.pose[:2],[x,y])<.1)
        self.settle(1.0)
        for client in self.clear:
            self.result(client.call_async(ClearEntireCostmap.Request()))
        self.settle(1.5)
        case={'name':task['name'],'expected_success':task.get('expect_success',True),'positions':[],
              'contacts':[],'initial_plan':[],'max_command_m_s':0.0}
        case['extra_obstacles']=list(self.extra_obstacles)
        self.active=case
        goal=NavigateToPose.Goal(); goal.pose=PoseStamped(); goal.pose.header.frame_id='map'
        goal.pose.header.stamp=self.get_clock().now().to_msg()
        x,y,yaw=task['goal']; goal.pose.pose.position.x=float(x); goal.pose.pose.position.y=float(y)
        goal.pose.pose.orientation.z=math.sin(yaw/2); goal.pose.pose.orientation.w=math.cos(yaw/2)
        start=time.monotonic(); scans_before=self.scan_count
        handle=self.result(self.nav.send_goal_async(goal))
        if not handle.accepted:
            case['status']='rejected'
        else:
            future=handle.get_result_async()
            try:
                response=self.result(future,240)
                case['status']={4:'succeeded',5:'canceled',6:'aborted'}.get(response.status,str(response.status))
            except TimeoutError:
                self.result(handle.cancel_goal_async())
                self.result(future,20)
                case['status']='timeout'
        case['elapsed_wall_s']=round(time.monotonic()-start,2)
        case['goal_error_m']=math.dist(self.pose[:2],task['goal'][:2])
        case['laser_messages']=self.scan_count-scans_before
        case['passed_status']=(case['status']=='succeeded') if case['expected_success'] else case['status'] in ['aborted','rejected']
        self.active=None
        self.stop.publish(Twist())
        print(json.dumps({k:v for k,v in case.items() if k not in ['positions','initial_plan','contacts']}),flush=True)
        return case


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',action='append')
    parser.add_argument('--output',type=Path,default=HERE/'evidence/navigation.json')
    args=parser.parse_args()
    scene=json.loads((HERE/'generated/scene_geometry.json').read_text())
    if args.case and 'road_obstacle' in args.case:
        scene['tasks'].append({'name':'road_obstacle','start':[99.1,-33.5,-math.pi/2],
                               'goal':[99.1,-50.5,-math.pi/2]})
    rclpy.init(); node=Check()
    report={'scene_version':scene['version'],'cases':[]}
    try:
        if not node.nav.wait_for_server(timeout_sec=60): raise RuntimeError('NavigateToPose unavailable')
        if not node.reset.wait_for_service(timeout_sec=20): raise RuntimeError('/set_entity_state unavailable')
        for client in node.clear:
            if not client.wait_for_service(timeout_sec=20): raise RuntimeError('Costmap service unavailable')
        # Gazebo contact sensors may emit only when contact exists. Require the
        # publisher and independently check the whole motion footprint offline.
        node.until(lambda:node.pose is not None and node.finite_scan_count>0
                   and bool(node.get_publishers_info_by_topic('/campus/contacts')))
        for task in scene['tasks']:
            if args.case and task['name'] not in args.case: continue
            if task['name']=='road_obstacle':
                node.until(lambda:bool(node.extra_obstacles),10)
            report['cases'].append(node.run(task))
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(report,indent=2)+'\n')
        report['laser_messages']=node.scan_count; report['scans_with_returns']=node.finite_scan_count
        report['contact_sensor_messages']=node.contact_messages
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        if not report['cases'] or not all(c['passed_status'] for c in report['cases']): raise SystemExit(1)
    finally:
        node.stop.publish(Twist()); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
