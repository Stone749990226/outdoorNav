import base64
from io import BytesIO
import json
import math
from types import SimpleNamespace as NS
from urllib.error import HTTPError
from urllib.request import urlopen

from PIL import Image
import pytest

from outdoornav.api import ApiError, serve
from outdoornav.costmap_view import grid_snapshot


def grid():
    return NS(header=NS(frame_id='odom', stamp=NS(sec=10, nanosec=0)),
              info=NS(width=3, height=2, resolution=.1,
                      origin=NS(position=NS(x=-1., y=-2.), orientation=NS(x=0., y=0., z=0., w=1.))),
              data=[0, 99, 100, -1, 10, 80])


def test_grid_encodes_ros_occupancy_values_and_flips_rows():
    result = grid_snapshot(grid(), .2)
    png = Image.open(BytesIO(base64.b64decode(result['image'].split(',')[1]))).convert('RGB')
    assert png.size == (3, 2)
    assert png.getpixel((0, 0)) == (78, 88, 98)  # unknown top row, not free
    assert png.getpixel((0, 1)) == (233, 239, 233)
    assert png.getpixel((1, 1)) == (239, 55, 69)
    assert png.getpixel((2, 1)) == (126, 12, 30)
    assert result['robot'] is None and result['path'] == []
    assert result['frame_id'] == 'odom' and not result['stale']
    assert grid_snapshot(grid(), 4.)['stale']


@pytest.mark.parametrize('change', ['size', 'resolution', 'value'])
def test_invalid_grid_is_not_rendered_as_free(change):
    msg = grid()
    if change == 'size':
        msg.info.width = 4
    if change == 'resolution':
        msg.info.resolution = 0.
    if change == 'value':
        msg.data[0] = 101
    with pytest.raises(ValueError):
        grid_snapshot(msg, 0)


def test_http_snapshot_and_waiting_response():
    class Backend:
        def costmap_snapshot(self, layer):
            if layer == 'global':
                raise ApiError(503, 'Waiting for global costmap')
            return grid_snapshot(grid(), .2)
    server = serve(Backend(), port=0)
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base + '/api/costmaps/local') as response:
            assert json.load(response)['frame_id'] == 'odom'
        with pytest.raises(HTTPError) as error:
            urlopen(base + '/api/costmaps/global')
        assert error.value.code == 503
        with pytest.raises(HTTPError) as error:
            urlopen(base + '/api/costmaps/arbitrary')
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()


def test_node_transforms_path_into_local_frame_without_running_ros():
    pytest.importorskip('rclpy')
    from outdoornav_ros.navigation import Navigation
    import threading
    import time
    node = object.__new__(Navigation)
    node.state_lock = threading.RLock()
    node.local_map = grid()
    node.global_map = None
    node.stats = {'local_map': time.monotonic()}
    node.costmap_view_cache = {}
    node.base_frame = 'base_link'
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_200_000_000))
    def transform(target, source, _):
        if target == 'map':
            assert source == 'odom'
            return NS(header=NS(stamp=NS(sec=10, nanosec=0)),
                      transform=NS(translation=NS(x=-3., y=2.), rotation=NS(x=0., y=0., z=-math.sin(math.pi/4), w=math.cos(math.pi/4))))
        assert target == 'odom'
        return NS(header=NS(stamp=NS(sec=10, nanosec=0)),
                  transform=NS(translation=NS(x=2., y=3.), rotation=NS(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4))))
    node.tf = NS(lookup_transform=transform)
    node.tasks = NS(snapshot=lambda: {'execution_path': [{'x': 1., 'y': 0.}]})
    result = node.costmap_snapshot('local')
    assert result['map_from_frame']['x'] == -3.
    assert result['map_from_frame']['yaw'] == pytest.approx(-math.pi/2)
    assert result['robot']['x'] == 2.
    assert result['path'][0]['x'] == pytest.approx(2.)
    assert result['path'][0]['y'] == pytest.approx(4.)
    assert result['age_s'] == pytest.approx(.2)
    assert node.costmap_snapshot('local')['image'] == result['image']


def test_overlay_colors_and_transparency():
    local = grid_snapshot(grid(), 0., theme='local')
    global_map = grid_snapshot(grid(), 0., theme='global')
    def image(result):
        return Image.open(BytesIO(base64.b64decode(result['image'].split(',')[1]))).convert('RGBA')
    a, b = image(local), image(global_map)
    assert a.getpixel((0, 0))[3] == 0  # Unknown does not hide the global layer.
    assert a.getpixel((0, 1))[3] == 0  # Nor does free space.
    assert a.getpixel((2, 1))[3] > 0
    assert b.getpixel((0, 0))[3] == 255
    assert a.getpixel((2, 1))[:3] != b.getpixel((2, 1))[:3]
