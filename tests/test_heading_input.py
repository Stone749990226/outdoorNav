"""Independent position/heading inputs; synthetic data never authorizes a robot."""

import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import sys
import termios
import time

import pytest

from outdoornav.config import HEADING_DEFAULTS, load_robot_config
from outdoornav.dual_rtk import DualRtkDecoder
from outdoornav.localization import GeoAlignment
from outdoornav.serial_input import SerialInput
from test_lynx import fix

ROOT = Path(__file__).resolve().parents[1]


def sentence(body):
    check = 0
    for byte in body.encode():
        check ^= byte
    return f'${body}*{check:02X}\r\n'.encode()


def config():
    return {'calibrated': True, 'initial_heading_rad': 2.0,
            'antenna_xy_m': [0, 0], 'map_offset_xy_m': [0, 0],
            'heading_source': 'dual_antenna',
            'dual_heading': {**HEADING_DEFAULTS, 'mounting_confirmed': True,
                             'quality_confirmed': True, 'baseline_to_body_yaw_rad': 0.0}}


def observation(degrees, now, offset=0.0):
    decoder = DualRtkDecoder(baseline_to_body_yaw_rad=offset)
    decoder.feed(sentence(f'GNTHS,{degrees},A'), now)
    return decoder.snapshot(now)


def prepared():
    a = GeoAlignment({'latitude': 30.0, 'longitude': 120.0}, config())
    for i in range(71):
        now = i/10
        a.odom(10, 20, 0.3, 100+now, now)
        a.update_heading(observation(90, now), now)
        a.update_fix(fix(i), now)
    return a


def test_absolute_heading_not_the_old_configured_initial_angle():
    a = prepared()
    assert a.ready(7)
    assert a.pose()['yaw'] == pytest.approx(0)
    assert a.rotation == pytest.approx(-0.3)
    assert a.pose()['x'] == pytest.approx(0)
    assert a.pose()['y'] == pytest.approx(0)


def test_missing_heading_cannot_fall_back_to_configured_initial_angle():
    a = GeoAlignment({'latitude': 30.0, 'longitude': 120.0}, config())
    for i in range(71):
        a.odom(10,20,.3,100+i/10,i/10)
        a.update_fix(fix(i),i/10)
    assert a.rotation is None and a.pose() is None
    assert not a.ready(7)


def test_invalid_or_stale_heading_revokes_readiness_even_with_good_position():
    a = prepared()
    assert not a.ready(8.6)
    a.update_heading({'heading_fresh':False,'body_yaw_enu_rad':None},7.1)
    a.odom(10,20,.3,107.1,7.1)
    a.update_fix(fix(71),7.1)
    assert not a.ready(7.1)
    assert a.grade == 'fixed'


def test_heading_reanchor_keeps_position_continuous_and_wraps_north():
    a = prepared()
    before = a.pose()
    a.update_heading(observation(0,7),7)
    after = a.pose()
    assert after['x'] == pytest.approx(before['x'])
    assert after['y'] == pytest.approx(before['y'])
    assert after['yaw'] == pytest.approx(math.pi/2)
    a.update_heading(observation(359,7),7)
    assert a.pose()['yaw'] == pytest.approx(math.radians(91))


def test_mounting_and_quality_confirmations_cannot_be_inferred():
    a = prepared()
    for key in ('mounting_confirmed','quality_confirmed'):
        a.config['dual_heading'][key] = False
        assert not a.ready(7)
        a.config['dual_heading'][key] = True
    a.update_heading(observation(90,8),8)
    assert not a.heading_ready(8), 'No fresh corresponding odometry at time 8'


def test_new_receiver_gga_is_not_the_position_source():
    a = prepared()
    original = dict(a.fix)
    decoder = DualRtkDecoder(baseline_to_body_yaw_rad=0)
    decoder.feed(sentence('GNGGA,120000.00,4000.0,N,11000.0,E,4,20,0.8,10,M,0,M,1.0,0001')+
                 sentence('GNTHS,90,A'),7)
    a.update_heading(decoder.snapshot(7),7)
    assert a.fix == original
    assert a.fix['latitude'] == 30 and a.fix['longitude'] == 120


def test_hdt_zero_and_tra_cannot_initialize_heading():
    a = prepared()
    decoder = DualRtkDecoder(baseline_to_body_yaw_rad=0)
    decoder.feed(sentence('GPHDT,0,T')+sentence('GPTRA,90,A'),7)
    a.update_heading(decoder.snapshot(7),7)
    assert not a.heading_ready(7)


def test_config_separates_udp_ports_and_requires_a_measured_mounting_angle(tmp_path):
    data=json.loads((ROOT/'deploy/lynx/robot.json').read_text())
    assert not data['motion_enabled']
    path=tmp_path/'robot.json'
    for change,match in (({'udp_port':data['gnss_udp_port']},'separate UDP'),
                         ({'mounting_confirmed':True},'measured angle'),
                         ({'baseline_to_body_yaw_rad':float('nan')},'finite radians'),
                         ({'timeout_s':10},'timeout')):
        data['dual_heading']={**HEADING_DEFAULTS,**change}
        path.write_text(json.dumps(data))
        with pytest.raises(ValueError,match=match):load_robot_config(path)


def test_read_only_serial_exclusion_and_settings_restore():
    master,slave=pty.openpty()
    path=os.ttyname(slave)
    attrs=termios.tcgetattr(slave)
    os.close(slave)
    try:
        with SerialInput(path,115200) as reader:
            os.write(master,b'$HEADING,read-only\r\n')
            assert reader.read(1)==b'$HEADING,read-only\r\n'
            with pytest.raises(OSError):os.open(path,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        check=os.open(path,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        try:assert termios.tcgetattr(check)==attrs
        finally:os.close(check)
        with pytest.raises(ValueError,match='position receiver'):
            SerialInput(path,115200,forbidden=[path])
    finally:os.close(master)


def test_real_streamer_forwards_stamped_serial_bytes_without_commands(tmp_path):
    master,slave=pty.openpty()
    path=os.ttyname(slave)
    os.close(slave)
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1',0));udp.settimeout(3)
    data=json.loads((ROOT/'deploy/lynx/robot.json').read_text())
    data['dual_heading'].update(port=path,udp_port=udp.getsockname()[1])
    cfg=tmp_path/'robot.json';cfg.write_text(json.dumps(data))
    process=subprocess.Popen([sys.executable,str(ROOT/'deploy/lynx/heading_stream.py'),
        '--config',str(cfg),'--duration','5'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,
        env={**os.environ,'PYTHONPATH':str(ROOT)})
    try:
        assert select.select([process.stdout],[],[],3)[0], 'Reader did not start'
        assert 'Heading reader:' in process.stdout.readline().decode()
        raw=sentence('GNTHS,90,A')
        os.write(master,raw)
        packet,_=udp.recvfrom(4096)
        assert packet[8:]==raw
        received=struct.unpack('!d',packet[:8])[0]
        assert 0 <= time.monotonic()-received < 1
        # Nothing was written back to the receiver (PTY master).
        assert not select.select([master],[],[],0.05)[0]
    finally:
        if process.poll() is None:process.terminate()
        process.wait(timeout=5)
        udp.close();os.close(master)
