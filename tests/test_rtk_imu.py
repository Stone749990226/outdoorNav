import json
import math
from pathlib import Path
import struct

import pytest

from outdoornav.config import DEFAULTS, WIT_DEFAULTS, load_robot_config
from outdoornav.coordinates import local_radii
from outdoornav.rtk_imu import RtkImuFeedback
from outdoornav.wit_imu import WitImuDecoder


IDENTITY = [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]


def config():
    return {**DEFAULTS,'calibrated':True,'antenna_offset_confirmed':True,
            'feedback_confirmed':True,'antenna_xy_m':[0.,0.],'map_offset_xy_m':[0.,0.],
            'wit_imu':{**WIT_DEFAULTS,'base_from_imu':IDENTITY,'magnetic_declination_rad':0.,
                       'yaw_std_deg':1.,'mounting_confirmed':True,'reference_confirmed':True,'quality_confirmed':True}}


def frame(kind, values):
    raw=bytes([0x55,kind])+struct.pack('<hhhh',*values)
    return raw+bytes([sum(raw)&255])


def observation(yaw, rate, now, cfg=None, angle=True):
    cfg=cfg or config()['wit_imu']
    decoder=WitImuDecoder(cfg);decoder.mode=0
    raw=frame(0x52,[0,0,round(math.degrees(rate)/2000*32768),0])
    if angle:raw+=frame(0x53,[0,0,round(yaw/math.pi*32768),0])
    decoder.feed(raw,now)
    return decoder.snapshot(now)


def fix(x, y, ve, vn, epoch, **extra):
    east,north=local_radii(30)
    return {'valid':True,'fixed':True,'float':False,'differential':True,'gga_quality':4,
            'hacc_m':.1,'correction_age_s':1.,'latitude':30+math.degrees(y/north),
            'longitude':120+math.degrees(x/east),'vel_e_mps':ve,'vel_n_mps':vn,
            'speed_accuracy_mps':.02,'tow_ms':epoch,**extra}


def state():
    return RtkImuFeedback({'latitude':30.,'longitude':120.},config())


def test_signed_units_and_no_double_scaling():
    decoder=WitImuDecoder(config()['wit_imu']);decoder.mode=0
    raw=frame(0x51,[-2048,0,2048,3000])+frame(0x52,[0,0,16384,0])+frame(0x53,[0,0,8192,0])
    for i in range(0,len(raw),3):decoder.feed(raw[i:i+3],1)
    s=decoder.snapshot(1)
    assert s['acceleration_mps2']==pytest.approx([-9.80665,0,9.80665])
    assert s['raw_gyro_rps'][2]==pytest.approx(math.radians(1000))
    assert s['body_yaw_enu_rad']==pytest.approx(math.pi/4)
    assert s['ready'] and s['nine_axis_verified']
    assert not decoder.snapshot(2)['ready']


def test_mounting_rotation_and_declination_sign():
    cfg=config()['wit_imu']
    cfg['base_from_imu']=[[0.,-1.,0.,0.],[1.,0.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    cfg['magnetic_declination_rad']=.1
    s=observation(0,0,1,cfg)
    assert s['body_yaw_enu_rad']==pytest.approx(-math.pi/2-.1)


def test_mode_and_unmeasured_reference_cannot_grant_heading():
    decoder=WitImuDecoder(config()['wit_imu'])
    decoder.feed(frame(0x52,[0,0,0,0])+frame(0x53,[0,0,0,0]),1)
    assert not decoder.snapshot(1)['ready']
    decoder.mode=1;assert not decoder.snapshot(1)['ready']
    decoder.mode=0
    for key in ('mounting_confirmed','reference_confirmed','quality_confirmed'):
        decoder.config[key]=False;assert not decoder.snapshot(1)['ready'];decoder.config[key]=True


def test_envelope_mode_and_restart_are_explicit():
    decoder=WitImuDecoder(config()['wit_imu'])
    raw=frame(0x52,[0,0,0,0])+frame(0x53,[0,0,0,0])
    def packet(generation):return json.dumps({'schema_version':1,'source':'wit_imu','received':1.,
        'generation':generation,'axis6_register':0,'raw_hex':raw.hex()}).encode()
    assert decoder.packet(packet('first'),1)
    assert decoder.snapshot(1)['ready']
    decoder.packet(packet('second'),1)
    assert not decoder.snapshot(1)['ready'] and 'restarted' in decoder.fault


def test_bad_checksum_cannot_produce_data():
    d=WitImuDecoder(config()['wit_imu']);d.mode=0
    packet=bytearray(frame(0x53,[0,0,0,0]));packet[-1]^=1
    d.feed(packet,0)
    assert d.angles is None and d.bad_checksums==1


def test_feedback_without_external_odometry_uses_doppler_not_position_differences():
    s=state()
    for i in range(71):
        t=i/10
        # Position correction is intentionally noisy; reported speed stays .2 m/s.
        s.update_fix(fix(.2*t+(.05 if i%2 else -.05),0,.2,0,i),t)
        s.update_imu(observation(0,0,t),t)
    assert s.ready(7)
    out=s.sample(7)
    assert out['twist']['linear_x']==pytest.approx(.2)
    assert out['odom_pose']['x']==pytest.approx(1.4,abs=.001)
    assert out['map_pose']['x']==pytest.approx(1.35,abs=.001)


def test_body_velocity_and_rotating_antenna_arm():
    s=state();s.config['antenna_xy_m']=[1.,0.]
    # Heading north: arm=(0,1), positive yaw rate gives antenna velocity west.
    s.update_fix(fix(0,1,-.1,0,1),0)
    s.update_imu(observation(math.pi/2,.1,0),0)
    out=s.sample(0)
    assert out['map_pose']['x']==pytest.approx(0,abs=.002)
    assert out['map_pose']['y']==pytest.approx(0,abs=.002)
    assert out['twist']['linear_x']==pytest.approx(0,abs=.002)
    assert out['twist']['linear_y']==pytest.approx(0,abs=.002)


def test_missing_bad_and_duplicate_velocity_do_not_become_zero_or_fresh():
    for extra in ({'vel_e_mps':None},{'speed_accuracy_mps':0},{'speed_accuracy_mps':1.}):
        s=state();s.update_fix(fix(0,0,0,0,1,**extra),0)
        s.update_imu(observation(0,0,0),0)
        assert s.sample(0) is None and not s.ready(0)
    s=state()
    for i in range(21):
        t=i/10;s.update_fix(fix(0,0,0,0,1),t);s.update_imu(observation(0,0,t),t)
    assert not s.ready(2) and s.sample(2) is None


def test_heading_jump_and_gnss_jump_latch():
    s=state();s.update_fix(fix(0,0,0,0,1),0);s.update_imu(observation(0,0,0),0)
    s.update_fix(fix(0,0,0,0,2),.1);s.update_imu(observation(math.pi/2,0,.1),.1)
    assert 'heading_jump' in s.fault
    # A single position outlier is dropped without latching.
    s=state();s.update_fix(fix(0,0,0,0,1),0);s.update_fix(fix(5,0,0,0,2),.1)
    assert s.fault is None
    # A persistent jump streak stops the robot.
    s=state();s.update_fix(fix(0,0,0,0,1),0)
    for i in range(4):
        s.update_fix(fix(5,0,0,0,2+i),.1+i/10)
    assert 'position_jump' in s.fault


def test_dropout_does_not_publish_held_feedback_as_current():
    s=state();s.update_fix(fix(0,0,0,0,1),0);s.update_imu(observation(0,0,0),0)
    assert s.sample(0)
    assert s.sample(1) is None and not s.ready(1)


def test_direct_mode_rejects_external_odom_and_unconfirmed_motion(tmp_path):
    d=json.loads(Path('deploy/lynx/robot.json').read_text());p=tmp_path/'robot.json'
    p.write_text(json.dumps(d));c=load_robot_config(p)
    assert c['localization_mode']=='rtk_imu' and c['odom_topic']=='' and not c['motion_enabled']
    d['odom_topic']='/test/pretend_odom';p.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='no external odometry'):load_robot_config(p)
    d['odom_topic']='';d['wit_imu']['mounting_confirmed']=True;p.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='measured base_from_imu'):load_robot_config(p)


def test_invalid_boot_epoch_does_not_block_the_first_usable_fix():
    s=state()
    s.update_fix(fix(0,0,0,0,4294967295,valid=False),0)
    s.update_fix(fix(0,0,0,0,100),.1)
    s.update_imu(observation(0,0,.1),.1)
    assert s.fault is None and s.sample(.1)


def test_tilted_imu_gyro_is_rotated_to_body_not_assumed_sensor_z():
    cfg=config()['wit_imu']
    cfg['base_from_imu']=[[1.,0.,0.,0.],[0.,0.,-1.,0.],[0.,1.,0.,0.],[0.,0.,0.,1.]]
    d=WitImuDecoder(cfg);d.mode=0
    d.feed(frame(0x52,[0,16384,0,0])+frame(0x53,[0,0,0,0]),1)
    assert d.snapshot(1)['body_yaw_rate_rps']==pytest.approx(math.radians(1000))


def test_collector_only_sends_the_documented_mode_read_request(tmp_path):
    import os,pty,select,socket,subprocess,sys,time
    root=Path(__file__).resolve().parents[1]
    master,slave=pty.openpty();port=os.ttyname(slave);os.close(slave)
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('127.0.0.1',0));udp.settimeout(2)
    d=json.loads((root/'deploy/lynx/robot.json').read_text())
    d['wit_imu'].update(port=port,udp_port=udp.getsockname()[1])
    cfg=tmp_path/'robot.json';cfg.write_text(json.dumps(d))
    process=subprocess.Popen([sys.executable,str(root/'deploy/lynx/wit_stream.py'),
        '--config',str(cfg),'--duration','5'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,
        env={**os.environ,'PYTHONPATH':str(root)})
    try:
        # PTY master may report EIO until the reader opens the slave.
        request=b'';deadline=time.monotonic()+3
        while time.monotonic()<deadline and len(request)<5:
            if select.select([master],[],[],.1)[0]:
                try:request+=os.read(master,256)
                except OSError:continue
        assert request==bytes.fromhex('ffaa272400')
        os.write(master,frame(0x5f,[0,30,2,36])+frame(0x52,[0,0,0,0])+frame(0x53,[0,0,0,0]))
        found=False;deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            packet=json.loads(udp.recvfrom(8192)[0])
            if packet['axis6_register']==0:
                found=True;break
        assert found and packet['source']=='wit_imu'
        assert not select.select([master],[],[],.1)[0], 'No calibration, unlock, save or mode write allowed'
    finally:
        if process.poll() is None:process.terminate()
        process.wait(timeout=5);udp.close();os.close(master)


@pytest.mark.skipif(__import__('importlib').util.find_spec('rclpy') is None, reason='requires ROS Humble')
def test_direct_localizer_cannot_consume_external_odometry():
    from unittest.mock import Mock
    from nav_msgs.msg import Odometry
    from outdoornav_ros.lynx_localizer import LynxLocalizer
    node=object.__new__(LynxLocalizer)
    node.direct=object();node.alignment=Mock();node.odom_pub=Mock()
    node.odom(Odometry())
    node.alignment.odom.assert_not_called()
    node.odom_pub.publish.assert_not_called()
