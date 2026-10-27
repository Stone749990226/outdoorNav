"""Shared probe IO and metric geometry. OccupancyGrid and costmap codes differ."""

import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

OCCUPANCY_LETHAL = frozenset({99, 100})
COSTMAP_LETHAL = 254


def seq_of(path):
    match = re.search(r"_(\d+)\.", Path(path).name)
    if not match:
        raise ValueError(f"Probe file has no sequence number: {path}")
    return int(match[1])


def densify(points, step=0.05):
    if step <= 0:
        raise ValueError("Spacing must be positive")
    result = []
    for a, b in zip(points, points[1:]):
        count = max(1, int(math.dist(a, b) / step))
        result.extend(
            tuple(x + (y - x) * i / count for x, y in zip(a, b)) for i in range(count)
        )
    if points:
        result.append(tuple(points[-1]))
    return result


def sha(path):
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def probe_files(directory, max_sequence=None):
    paths = sorted(Path(directory).glob("tick_*.json"), key=seq_of)
    paths = [
        p
        for p in paths
        if (max_sequence is None or seq_of(p) <= max_sequence)
        and (p.parent / f"map_{seq_of(p):04d}.bin").is_file()
    ]
    if not paths:
        raise ValueError("No paired probe ticks and maps")
    return paths


def load_grid(directory):
    paths = sorted(Path(directory).glob("map_*.json"), key=seq_of)
    paths = [p for p in paths if p.with_suffix(".bin").is_file()]
    if not paths:
        return None, None
    path = paths[-1]
    info, data = json.loads(path.read_text()), path.with_suffix(".bin").read_bytes()
    if len(data) != info["width"] * info["height"]:
        raise ValueError("Grid data length does not match metadata")
    return info, data


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def summary(values):
    finite = [v for v in values if math.isfinite(v)]
    return {'min': min(finite) if finite else None,
            'max': max(finite) if finite else None,
            'mean': sum(finite) / len(finite) if finite else None,
            'nonfinite_count': len(values) - len(finite)}


class StaticFrames:
    """A latched static tree; never accepts a static localization owner."""
    def __init__(self):
        self.frames = {}
        self.faults = set()

    def add(self, parent, child, xyz, quaternion):
        values = tuple(xyz) + tuple(quaternion)
        if not parent or not child or parent == child or not all(math.isfinite(v) for v in values):
            self.faults.add('invalid_static_transform:' + child)
            return
        if child in ('odom', 'base_link'):
            self.faults.add('static_localization_owner:' + parent + '->' + child)
            return
        if abs(sum(v*v for v in quaternion) - 1) > 0.05:
            self.faults.add('invalid_static_quaternion:' + child)
            return
        quaternion = tuple(np.array(quaternion) / np.linalg.norm(quaternion))
        previous = self.frames.get(child)
        if previous and (previous[0] != parent or not np.allclose(previous[1], xyz, atol=1e-6, rtol=0)
                         or abs(float(np.dot(previous[2], quaternion))) < 1 - 1e-6):
            self.faults.add('inconsistent_static_transform:' + child)
            return
        self.frames[child] = (parent, tuple(xyz), tuple(quaternion))
        seen, cursor = set(), child
        while cursor in self.frames:
            if cursor in seen:
                self.faults.add('static_cycle:' + child)
                break
            seen.add(cursor)
            cursor = self.frames[cursor][0]

    def matrix(self, child, base='base_link'):
        if self.faults or not child:
            return None
        result, seen = np.eye(4), set()
        while child != base:
            if child in seen or child not in self.frames:
                return None
            seen.add(child)
            parent, xyz, q = self.frames[child]
            x, y, z, w = np.array(q) / np.linalg.norm(q)
            step = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), xyz[0]],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), xyz[1]],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), xyz[2]],
                [0, 0, 0, 1],
            ])
            result = step @ result
            child = parent
        return result

    def receive(self, message):
        for t in message.transforms:
            p, q = t.transform.translation, t.transform.rotation
            self.add(t.header.frame_id, t.child_frame_id, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def readiness_reasons(health, geo):
    messages = {
        'heading': '航向无效、过期或尚未确认',
        'lidar': '雷达无有效新数据', 'global_map': '全局代价地图未就绪',
        'local_map': '局部代价地图未就绪', 'localization': '无合格且稳定的 RTK 定位',
        'calibration': '坐标、天线或机器人外形尚未确认', 'motion_enabled': '运动开关关闭',
        'service_active': '导航服务未激活', 'controller_stopped': '尚未确认停稳或停车失败',
        'odometry': '连续里程计来源未配置、未确认或数据过期', 'actions': '导航执行服务不可用',
        'exclusive_owners': '室外 TF 或速度发布冲突', 'tf': '室外定位坐标变换未就绪',
        'sensor_extrinsics': '雷达静态外参缺失或冲突',
        'command_guard': '命令看门狗未就绪或输出话题不一致',
        'production_mode': '诊断模式：底盘输出已隔离',
    }
    if geo.get('localization_mode') == 'rtk_imu':
        messages['odometry'] = 'RTK 多普勒／IMU 运动反馈未就绪或未确认'
        messages['heading'] = '新 IMU 航向无效、过期或尚未确认'
        messages['localization'] = 'RTK 位置、速度与新 IMU 航向尚未稳定就绪'
    reasons = [{'code': key, 'message': messages.get(key, key + ' 未就绪')}
               for key, ready in health.items() if not ready]
    if geo.get('tf_conflicts', {}).get('dynamic'):
        reasons.append({'code': 'dynamic_tf_fault', 'message': '室外动态 TF 存在其他发布者'})
    if geo.get('fault'):
        code = geo['fault'].split(';', 1)[0]
        message = {
            'odometry_source_unconfigured': '连续里程计来源尚未配置',
            'waiting_for_rtk_and_imu': '等待原 RTK 与新 IMU 的有效位置、速度和航向',
            'gnss_quality_unusable': 'RTK 解类型、位置精度或改正时效不合格',
            'gnss_doppler_unusable': 'RTK 多普勒速度缺失或速度精度不合格',
            'imu_time_gap': '新 IMU 角速度数据中断，请检查后重启',
            'imu_heading_jump': '新 IMU 航向异常跳变，请检查磁干扰',
            'imu_source_restarted': '新 IMU 采集进程已重启，请检查后重启定位',
            'invalid_odometry': '里程计数据无效',
            'odometry_stamp_or_quaternion_invalid': '里程计时间或姿态数据无效',
            'odometry_reset_or_gap': '里程计中断或重置，请检查后重启',
            'gnss_epoch_reversed': 'GNSS 时间倒退，请检查接收机',
            'gnss_position_jump': 'GNSS 定位跳变，请检查坐标对齐',
            'conflicting_tf_owner': '室外定位存在重复坐标变换发布者',
        }.get(code, '定位异常，请检查诊断记录')
        reasons.append({'code': 'localization_fault', 'message': message})
    if geo.get('extrinsics', {}).get('faults'):
        reasons.append({'code': 'static_tf_fault', 'message': '静态外参异常：' + ', '.join(geo['extrinsics']['faults'])})
    if geo.get('lio') is not None and not geo['lio'].get('ready'):
        reasons.append({'code': 'lio_not_ready', 'message': '雷达／IMU 里程计未就绪：' +
                        (geo['lio'].get('fault') or '等待同步输入及稳定输出')})
    heading = geo.get('heading') or {}
    if heading.get('source') == 'wit_imu':
        if not heading.get('nine_axis_verified'):
            reasons.append({'code':'imu_mode','message':'新 IMU 未确认九轴磁航向模式'})
        if not heading.get('angles_fresh') or not heading.get('gyro_fresh'):
            reasons.append({'code':'imu_input','message':'新 IMU 姿态角或角速度未收到／已过期'})
        for key,code,message in (
                ('mounting_confirmed','imu_mounting','新 IMU 到车体的安装外参未确认'),
                ('reference_confirmed','imu_north_reference','新 IMU 磁北到真北参考未确认'),
                ('quality_confirmed','imu_quality','新 IMU 磁校准与航向精度未确认')):
            if not heading.get(key):reasons.append({'code':code,'message':message})
    if geo.get('feedback') is not None and not geo['feedback'].get('confirmed'):
        reasons.append({'code':'feedback_confirmation','message':'RTK 多普勒／IMU 运动反馈尚未实测确认'})
    return reasons
