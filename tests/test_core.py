"""The C++ core through expertctl, and the frame conversions the agents use."""
import math
import os

import pytest

from expert import frames

expertctl = pytest.importorskip("expertctl", reason="build the module first: cmake --build build")
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_path_through_waypoints():
    path = expertctl.Path.from_waypoints([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]], False)
    assert path.length == pytest.approx(20.0)
    s, e_y = path.project(5.0, 1.0)
    assert (s, e_y) == (pytest.approx(5.0), pytest.approx(1.0))  # e_y positive to the left
    assert path.curvature(5.0) == pytest.approx(0.0)


def test_vehicle_params_from_file():
    p = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "model3.json"))
    assert p.m > 1000.0 and p.L() > 2.0


def test_frames_round_trip():
    assert frames.from_carla_xy(*frames.to_carla_xy(3.0, -2.0)) == (3.0, -2.0)
    assert frames.from_carla_yaw(frames.to_carla_yaw(0.3)) == pytest.approx(0.3)


def test_state_from_carla_moves_the_origin_to_the_cg():
    # heading north in the library's frame (CARLA yaw -90 deg), 10 m/s forward, no yaw rate
    x = frames.state_from_carla((0.0, 0.0), -90.0, (0.0, -10.0), 0.0, cg_ahead=1.5, delta=0.0)
    assert x[0] == pytest.approx(0.0, abs=1e-9) and x[1] == pytest.approx(1.5)
    assert x[2] == pytest.approx(math.pi / 2) and x[3] == pytest.approx(10.0) and x[4] == pytest.approx(0.0, abs=1e-9)
