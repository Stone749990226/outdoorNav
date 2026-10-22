import math

import pytest

from outdoornav.dual_rtk import DualRtkDecoder


def sentence(body):
    check = 0
    for byte in body.encode():
        check ^= byte
    return f'${body}*{check:02X}\r\n'.encode()


def gga(quality=4, epoch='120000.00'):
    return sentence(f'GNGGA,{epoch},3000.0,N,12000.0,E,{quality},18,0.8,10,M,0,M,1.2,0001')


def test_fragmented_position_heading_frames_and_timeout():
    decoder = DualRtkDecoder(baseline_to_body_yaw_rad=.1)
    stream = gga(5)+sentence('GNTHS,90.0,A')
    for i in range(0, len(stream), 7):
        decoder.feed(stream[i:i+7], 1)
    value = decoder.snapshot(1.1)
    assert value['position']['solution'] == 'float'
    assert value['position']['hacc_m'] is None
    assert value['position']['latitude'] == 30
    assert value['position']['correction_age_s'] == 1.2
    assert value['body_yaw_enu_rad'] == pytest.approx(.1)
    assert value['position_fresh'] and value['heading_fresh']
    assert not value['navigation_ready']
    assert not decoder.snapshot(2.1)['heading_fresh']
    assert not decoder.snapshot(2.1)['position_fresh']


def test_invalid_heading_checksum_hdt_and_tra_cannot_authorize_heading():
    decoder = DualRtkDecoder()
    decoder.feed(sentence('GNTHS,0,A'), 0)
    assert decoder.snapshot(0)['heading_fresh']
    assert decoder.snapshot(0)['body_yaw_enu_rad'] is None
    decoder.feed(sentence('GNTHS,,V'), .1)
    decoder.feed(sentence('GPHDT,20,T')+sentence('GPTRA,30,A'), .2)
    assert not decoder.snapshot(.2)['heading_fresh']
    assert decoder.snapshot(.2)['unverified_hdt']['true_heading_deg'] == 20
    decoder.feed(b'$GNTHS,10,A*00\r\n', .3)
    assert decoder.bad_checksums == 1
    decoder.feed(sentence('GNTHS,nan,A'), .4)
    assert decoder.malformed == 1
    assert not decoder.snapshot(.4)['heading_fresh']


def test_gga_invalid_replay_midnight_and_reverse():
    decoder = DualRtkDecoder()
    decoder.feed(gga(epoch='235959.50'), 0)
    decoder.feed(gga(epoch='000000.10'), .5)
    assert not decoder.fault
    decoder.feed(gga(epoch='000000.10'), 1.4)
    assert not decoder.snapshot(1.6)['position_fresh']
    decoder.feed(gga(0, epoch='000001.10'), 1.7)
    assert decoder.position is None
    decoder.feed(gga(epoch='000002.10'), 2)
    decoder.feed(gga(epoch='000001.10'), 2.1)
    assert decoder.fault


def test_true_north_clockwise_to_enu_with_baseline_mounting():
    decoder = DualRtkDecoder(baseline_to_body_yaw_rad=0)
    for heading, expected in ((0, math.pi/2), (90, 0), (180, -math.pi/2), (270, -math.pi)):
        decoder.feed(sentence(f'GNTHS,{heading},A'), 1)
        assert decoder.snapshot(1)['body_yaw_enu_rad'] == pytest.approx(expected)
