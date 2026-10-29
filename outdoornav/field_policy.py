"""Visible field policy; warnings are not additional motion gates."""


def field_warnings(config, heading):
    result = []
    blind = config.get('scan_range_min_m', 0.)
    if blind > config['footprint_radius_m'] + .2:
        result.append({'code': 'lidar_blind_zone', 'message':
            '雷达配置屏蔽 %.1f 米内回波，近处障碍可能看不见；这不是可靠的自车过滤。' % blind})
    if config['wit_imu'].get('course_heading'):
        if not heading.get('nine_axis_verified'):
            result.append({'code': 'heading_mode_unverified', 'message': '演示模式尚未确认九轴模式，不代表绝对航向已校准。'})
        if not heading.get('quality_confirmed'):
            result.append({'code': 'heading_accuracy_unverified', 'message': '当前按演示航向策略运行，实际航向误差尚未验收。'})
    return result
