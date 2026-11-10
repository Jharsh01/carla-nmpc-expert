"""The sight along a lane from a depth frame (middle_layer/perception/sensors/sight.py), on synthetic frames."""
import numpy as np
import pytest

from middle_layer.perception.sensors import sight

W, H, FOV = 640, 160, 100.0


def _lane(offset_right=3.5, below=1.5, reach=150.0):
    """A straight lane beside the camera's: points every 2 m ahead, 3.5 m to its right, 1.5 m below it (CARLA's frame:
    x forward, y right, z up), the camera at the origin looking along x."""
    d = np.arange(0.0, reach + 1e-9, 2.0)
    return d, np.c_[d, np.full_like(d, offset_right), np.full_like(d, -below)]


def test_depth_from_carlas_encoding():
    bgra = np.zeros((1, 2, 4), dtype=np.uint8)
    bgra[0, 1, 2] = 255  # R: the smallest step, 1000 / (256^3 - 1) * 255
    bgra[0, 0, :3] = 255  # all: 1000 m
    depth = sight.depth_of(bgra)
    assert depth[0, 0] == pytest.approx(1000.0)
    assert depth[0, 1] == pytest.approx(1000.0 * 255 / (256 ** 3 - 1))


def test_an_open_lane_is_seen_to_its_end_and_a_wall_cuts_it_short():
    d, pts = _lane()
    open_road = np.full((H, W), 1000.0)
    assert sight.seen_along(d, pts, open_road, np.eye(4), FOV) == pytest.approx(150.0)
    # something 60 m ahead covering the lane: the lane is seen to 60 m, not past it
    wall = open_road.copy()
    u, v, fwd = sight.project(pts, np.eye(4), W, H, FOV)
    for ui, vi, fi in zip(u, v, fwd):
        if fi > 60.0 and 0 <= ui < W and 0 <= vi < H:
            wall[int(vi), int(ui)] = 60.0
    seen = sight.seen_along(d, pts, wall, np.eye(4), FOV)
    assert 60.0 <= seen <= 64.0


def test_a_lane_out_of_the_frame_is_not_seen():
    d, pts = _lane(offset_right=200.0)  # far off to the side: out of a 100 deg frame until 170 m ahead
    assert sight.seen_along(d, pts, np.full((H, W), 1000.0), np.eye(4), FOV) < 170.0


def test_the_camera_is_placed_from_the_car_in_double_precision():
    """On Town12 the car is 5000 m from the origin: the camera 0.5 m ahead and 2.5 m above it, looking its way."""
    import types
    loc = types.SimpleNamespace(x=1173.42, y=5318.53, z=371.04)
    rot = types.SimpleNamespace(yaw=90.0, pitch=0.0, roll=0.0)
    m = sight.world_to_camera_of(loc, rot, (0.5, 0.0, 2.5))
    ahead = m @ np.array([1173.42, 5318.53 + 20.5, 371.04 + 2.5, 1.0])  # 20 m in front of the camera, level with it
    assert ahead[:3] == pytest.approx([20.0, 0.0, 0.0], abs=1e-6)
    right = m @ np.array([1173.42 - 3.0, 5318.53 + 0.5, 371.04 + 2.5, 1.0])  # CARLA: yaw 90 looks along +y, right is -x
    assert right[1] == pytest.approx(3.0, abs=1e-6)
