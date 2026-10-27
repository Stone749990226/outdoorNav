"""Synthetic separate-receiver integration on domain 91; no motion executable."""

import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

# Set before importing rclpy: none of these inputs can enter robot domain 0.
os.environ['ROS_DOMAIN_ID'] = '91'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from outdoornav.bundle import load_bundle
from outdoornav.config import HEADING_DEFAULTS

ROOT = Path(__file__).resolve().parents[1]


def nmea(body):
    check = 0
    for byte in body.encode():
        check ^= byte
    return f'${body}*{check:02X}\r\n'.encode()


def pvt(lat, lon, epoch):
    payload = bytearray(92)
    struct.pack_into('<I', payload, 0, epoch)
    payload[20], payload[21], payload[23] = 3, 3 | (2 << 6), 20
    struct.pack_into('<iiiiII', payload, 24, round(lon*1e7), round(lat*1e7), 10000, 10000, 100, 150)
    data = bytes([1, 7, 92, 0])+payload
    a = b = 0
    for value in data:
        a = (a+value) & 255
        b = (b+a) & 255
    return b'\xb5\x62'+data+bytes([a,b])


def main():
    config = json.loads((ROOT/'deploy/lynx/robot.json').read_text())
    config.update(localization_mode='external_odom', calibrated=True, motion_enabled=False, odom_topic='/test/continuous_odom',
                  heading_source='dual_antenna', gnss_udp_port=29765)
    config['dual_heading'] = {**HEADING_DEFAULTS, 'udp_port':29766,
                             'mounting_confirmed':True, 'quality_confirmed':True,
                             'baseline_to_body_yaw_rad':0.1}
    bundle = load_bundle()
    anchor = bundle['origin_wgs84']
    rclpy.init(args=[])
    node = rclpy.create_node('separate_heading_test')
    publisher = node.create_publisher(Odometry, config['odom_topic'], 10)
    latest = {}

    def status(msg):
        latest.clear()
        latest.update(json.loads(msg.data))

    node.create_subscription(String, '/outdoor/localization', status, 10)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    epoch = 0
    with tempfile.TemporaryDirectory(prefix='heading-smoke-') as directory:
        path = Path(directory)/'robot.json'
        path.write_text(json.dumps(config))
        log = (Path(directory)/'localizer.log').open('w')
        process = subprocess.Popen([sys.executable,'-c','from outdoornav_ros.lynx_localizer import main; main()',
            '--ros-args','-p','bundle:='+str(ROOT/'scenes/north_campus/generated/bundle.json'),
            '-p','robot_config:='+str(path),'-r','/tf:=/outdoor/tf'],
            env=os.environ.copy(),stdout=log,stderr=log,start_new_session=True)

        def pump(seconds, heading='valid', gnss=True):
            nonlocal epoch
            until = time.monotonic()+seconds
            while time.monotonic() < until:
                assert process.poll() is None, 'Localizer exited'
                message = Odometry()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id, message.child_frame_id = 'test_odom','test_base'
                message.pose.pose.position.x, message.pose.pose.position.y = 10.0,20.0
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(.3/2), math.cos(.3/2)
                publisher.publish(message)
                if gnss:
                    epoch += 100
                    for raw in (nmea('GNGGA,120000,3000.0,N,12000.0,E,4,20,0.8,10,M,0,M,1.0,0001'),
                                pvt(anchor['latitude'],anchor['longitude'],epoch)):
                        udp.sendto(struct.pack('!d',time.monotonic())+raw,('127.0.0.1',29765))
                raw = {'valid':nmea('GNGGA,120000,4000.0,N,11000.0,E,4,20,0.8,10,M,0,M,1.0,0001')+nmea('GNTHS,90,A'),
                       'invalid':nmea('GNTHS,,V'), 'hdt':nmea('GPHDT,0,T')}.get(heading)
                if raw:
                    udp.sendto(struct.pack('!d',time.monotonic())+raw,('127.0.0.1',29766))
                rclpy.spin_once(node,timeout_sec=.1)

        try:
            began = time.monotonic()
            while not latest and time.monotonic()-began < 10:
                pump(.2,heading='hdt')
            pump(2,heading='hdt')
            assert not latest['ready'] and latest['heading']['unverified_hdt']['true_heading_deg'] == 0,latest
            assert latest['pose'] is None,latest
            pump(8)
            assert latest['ready'],latest
            assert abs(latest['pose']['yaw']-.1)<1e-6,latest
            assert abs(latest['fix']['latitude']-anchor['latitude'])<1e-6,latest
            assert latest['position_source']=='ublox_nav_pvt'
            assert latest['heading']['source']=='dual_antenna'
            pump(.5,heading='invalid')
            assert not latest['ready'] and not latest['heading']['heading_fresh'],latest
            pump(1)
            assert latest['ready'],latest
            pump(2,heading='hdt')
            assert not latest['ready'] and not latest['heading']['heading_fresh'],latest
            pump(1)
            assert latest['ready'],latest
            pump(1.5,gnss=False)
            assert not latest['ready'] and latest['heading']['heading_fresh'],latest
            counts={t:node.count_publishers(t) for t in ('/cmd_vel','/outdoor/safe_command','/NAV_CMD')}
            assert not any(counts.values()),counts
            print(json.dumps({'passed':True,'domain':91,'motion_publishers':counts,
                'checks':['separate position/heading source','true-north to ENU plus mounting angle',
                          'HDT zero not accepted','invalid THS revokes','HDT does not refresh old THS',
                          'heading loss','position loss','no initial-heading fallback'],
                'last_status':latest},ensure_ascii=False,indent=2))
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            log.close()
            if process.returncode not in (0,-2):
                print((Path(directory)/'localizer.log').read_text(),file=sys.stderr)
            udp.close();node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
