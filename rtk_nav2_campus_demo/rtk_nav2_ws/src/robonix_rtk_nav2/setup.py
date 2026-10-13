import os
from glob import glob

from setuptools import setup


def files(pattern):
    """只取文件，避免把子目录当成文件拷贝。"""
    return [p for p in glob(pattern) if os.path.isfile(p)]


package_name = 'robonix_rtk_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*.py')),
        (os.path.join('share', package_name, 'config'), files('config/*.yaml') + files('config/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), files('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'), files('models/*.xacro') + files('models/*.urdf')),
        (os.path.join('share', package_name, 'scripts'), files('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robonix',
    maintainer_email='xjy@example.com',
    description='RTK 第三阶段：复用 nav2 costmap_2d 建图与 NavFn 规划',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtk_sim = robonix_rtk_nav2.rtk_sim:main',
            'wheel_odom_bridge = robonix_rtk_nav2.odom_bridge:main',
            'absolute_localizer = robonix_rtk_nav2.absolute_localizer:main',
            # 第二阶段的可执行文件名是 costmap；本阶段它已不再建图/膨胀，
            # 只输出前向危险距离，故改名 danger（话题与 route_follower 的接口不变）。
            'danger = robonix_rtk_nav2.costmap:main',
            'route_follower = robonix_rtk_nav2.route_follower:main',
            # 第四步：导航入口（规划→跟随→实时重规划的薄层）
            'goal_navigator = robonix_rtk_nav2.goal_navigator:main',
            # 阶段七：相机与激光的传感器帧改由本节点发布（带 IMU 实测倾斜），
            # 取代原来两个 static_transform_publisher。原因见该模块 docstring：
            # 二维定位下"水平"的传感器帧会把坡上的地面点抬高 f·sin(俯仰)，
            # 两个障碍源于是把**自己脚下的地面**标成障碍、车在坡上停死（ramp7-b 实测）。
            'rtk_tilt_frames = robonix_rtk_nav2.tilt_frames:main',
            'rtk_trajectory_logger = robonix_rtk_nav2.trajectory_logger:main',
            'depth_display = robonix_rtk_nav2.depth_display:main',
            # ★ 2026-10-09：静态实体封边 → 全局图的 `static_layer`（latched `/rtk/static_map` ✓）
            #   为什么必须是"层"而不是 keepout filter：见 `config/depth_drop_nav2.yaml` 的 `static_layer` 注释 ✓
            'static_walls = robonix_rtk_nav2.static_walls:main',
            # ★ 2026-10-09：**合并 keepout 掩膜**（检测器掩膜 ∪ 静态封边块 ✓）——
            #   为什么：实体块被 `obstacle_layer` 的 Overwrite+clearing **擦掉** ✗（第一手证据见文件头 ✓）
            #   ⇒ 改走 keepout 那条"擦不掉"的通道 ✓；本节点只**订阅**检测器产物、**不动检测器** ✓
            'keepout_merge = robonix_rtk_nav2.keepout_merge:main',
            # 第五步：深度→点云（**替换 depth_image_proc/point_cloud_xyz**）。
            # 差别只有一条：无回波像素沿它自己的射线补一个远裁剪面上的点，
            # 让 nav2 能把"没看到东西"的方向清成空闲（PointCloud2 源没有 inf_is_valid）。
            'rtk_depth_points = robonix_rtk_nav2.depth_points:main',
            # 本轮（激光入口换成 PointCloud2）：`LaserScan --> PointCloud2`。
            # 与 `depth_points` 是同一族：都把"无回波"沿它自己的射线补一个
            # **落在该源标记量程之外**的端点，让 nav2 能把那些方向清成空闲。
            # **只做转换，不改几何**：投影用的是 nav2 内部同一个
            # `laser_geometry::LaserProjection`（Python 绑定），见该模块 docstring。
            'rtk_scan_points = robonix_rtk_nav2.scan_points:main',
            'rtk_depth_drop = robonix_rtk_nav2.depth_drop:main',
        ],
    },
)
