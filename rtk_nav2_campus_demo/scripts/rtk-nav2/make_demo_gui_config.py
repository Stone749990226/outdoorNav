#!/usr/bin/env python3
"""make_demo_gui_config.py —— 生成**演示用**的 Fortress GUI 配置（★ 容器内跑 ✓）

用法（容器内）:
  make_demo_gui_config.py --out <路径> [--camera "x y z roll pitch yaw"] [--fov 度]
                          [--bg "r,g,b"] [--width 1536] [--height 904]

★ 为什么要有它（阶段九第 4 步 ✓）：
  第 1/2 步那套"**删掉 `EntityTree`／`ComponentInspector` 让 3D 视口铺满** ＋
  `<camera_pose>` 定机位 ＋ `<horizontal_fov>` ＋ 背景色补偿"当时只作用于
  **只有世界**的起法（直接 `ign gazebo --gui-config` ✓）。
  ★ 全栈走 `ros_gz_sim` 的 `gz_sim.launch.py` ✗ ⇒ 那条路**没带 GUI 配置** ✗
  ⇒ 实测截图里 **3D 视口没铺满、下方一大片空灰** ✗。
  ⇒ 现在 `demo.launch.py` 有了可选的 `gui_config:=` ✓，本脚本负责**造那份文件** ✓。

★★ 三条"值"的来历（✗ 不许凭感觉写 ✗）：
  ① **删两个停靠面板** ✓ —— 实测：Fortress 把停靠面板算进布局 ⇒ 3D 视口只剩窗口的 ~73% 宽 ✗。
  ② `<horizontal_fov>` 单位是**度** ✗ 不是弧度 ✓（★ 写弧度会让视场≈1°、相机贴脸 ✓，
     ★ 而那种坏图**照样"非黑"** ✗ ⇒ 亮度判据抓不到 ✓）。
  ③ `<background_color>` 用**世界声明值的 sRGB 反解** ✓：
     `c = ((B+0.055)/1.055)^2.4` ✓ —— 这样显示出来正好 = 世界 `<background>` 的值 ✓。

★ 基线 = 容器里**已持久化**的 `/root/.ignition/gazebo/6/gui.config` ✓（其余逐字照抄 ✓）。
"""
import argparse
import pathlib
import re
import sys

SRC = pathlib.Path('/root/.ignition/gazebo/6/gui.config')
# ★ 他们 world 的 `<scene><background>0.73 0.83 0.90 1` ✓ 的 sRGB 反解 ✓（见文件头 ③）
BG_DEFAULT = '0.4911,0.6584,0.7835'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--camera', default='17.4 -44.3 110 0 0.75 1.5708')
    ap.add_argument('--fov', type=float, default=60.0)
    ap.add_argument('--bg', default=BG_DEFAULT)
    ap.add_argument('--width', type=int, default=1536)
    ap.add_argument('--height', type=int, default=904)
    a = ap.parse_args()
    # ★ 机位允许**逗号分隔** ✓ —— 调用方从 shell 传带空格的值会被 argparse 拆散 ✗
    #   （★ 实测：`--camera 17.4 -122 78 0 0.785 1.5708` ⇒ `unrecognized arguments: -122 78 …` ✗）
    a.camera = ' '.join(a.camera.replace(',', ' ').split())

    if not SRC.is_file():
        print('✗ 找不到基线 %s' % SRC)
        return 2
    s = SRC.read_text(encoding='utf-8')
    print('基线 %s（%d B）' % (SRC, len(s.encode())))

    s = s.replace('<dialog name="quick_start" show_again="true"/>',
                  '<dialog name="quick_start" show_again="false"/>')

    # ① 删两个停靠面板（不删 ⇒ 3D 视口只占窗口 ~73% 宽 ✗）
    for plug in ('ComponentInspector', 'EntityTree'):
        s, n = re.subn(r'<plugin filename="%s".*?</plugin>\s*' % plug, '', s, flags=re.S)
        if n != 1:
            print('  ✗ 删面板 %s 命中 %d 次（应为 1）' % (plug, n))
            return 1
    print('  ★ 已删停靠面板 ComponentInspector / EntityTree ✓')

    if '<horizontal_fov>' in s:
        print('  ✗ 基线里已有 horizontal_fov（本脚本假设没有）')
        return 1
    if s.count('<camera_pose>') != 1:
        print('  ✗ camera_pose 锚点不唯一')
        return 1
    s = s.replace('<camera_pose>',
                  '<horizontal_fov>%g</horizontal_fov>\n  <camera_pose>' % a.fov, 1)

    s = re.sub(r'<background_color>[^<]*</background_color>',
               '<background_color>%s</background_color>' % a.bg.replace(',', ' '), s, count=1)
    s = re.sub(r'<width>\d+</width>', '<width>%d</width>' % a.width, s, count=1)
    s = re.sub(r'<height>\d+</height>', '<height>%d</height>' % a.height, s, count=1)
    s = re.sub(r'<camera_pose>[^<]*</camera_pose>',
               '<camera_pose>%s</camera_pose>' % a.camera, s, count=1)

    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(s, encoding='utf-8')

    # ★ 回读核对（✗ 不看写入返回值 ✗）
    back = out.read_text(encoding='utf-8')
    got = {
        'camera_pose': re.search(r'<camera_pose>([^<]*)</camera_pose>', back).group(1),
        'horizontal_fov': re.search(r'<horizontal_fov>([^<]*)</horizontal_fov>', back).group(1),
        'background_color': re.search(r'<background_color>([^<]*)</background_color>', back).group(1),
        'width': re.search(r'<width>(\d+)</width>', back).group(1),
        'height': re.search(r'<height>(\d+)</height>', back).group(1),
        'EntityTree': str(back.count('EntityTree')),
        'ComponentInspector': str(back.count('ComponentInspector')),
    }
    print('  ★ 回读：camera_pose=%s ｜ fov=%s ｜ bg=%s ｜ %sx%s ｜ 面板残留 %s/%s'
          % (got['camera_pose'], got['horizontal_fov'], got['background_color'],
             got['width'], got['height'], got['EntityTree'], got['ComponentInspector']))
    ok = (got['camera_pose'] == a.camera and got['horizontal_fov'] == '%g' % a.fov
          and got['EntityTree'] == '0' and got['ComponentInspector'] == '0')
    print('  ⇒ %s ｜ 写到 %s' % ('✓ 全部核对通过' if ok else '✗ 回读不符', out))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
