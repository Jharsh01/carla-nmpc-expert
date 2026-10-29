"""SimLingo's car-frame predictions as the NMPC's path and speed, the odometry between plans, the car's frame, and
the NMPC planning along a path that changes, and changes frame."""
import math
import os

import numpy as np
import pytest

from expert import frames, simlingo_ref

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_path_points_in_the_cars_frame():
    # SimLingo's y is to the right; the path's is to the left
    route = [[0.2, 0.0], [1.0, 0.0], [1.3, 0.0], [2.0, 0.1], [1.5, 0.1], [3.0, 0.2]]
    pts = simlingo_ref.path_points(route)
    assert pts[0] == pytest.approx([-simlingo_ref.BEHIND, 0.0])
    assert np.all(np.diff(pts[:, 0]) > 0.0)  # always on forwards
    assert pts[-3:] == pytest.approx(np.array([[1.0, 0.0], [2.0, -0.1], [3.0, -0.2]]))  # 0.2, 1.3 too close, 1.5 back


def test_speed_reference_with_and_without_the_car_first():
    v = 8.0
    ahead = [[v * simlingo_ref.WP_DT * (k + 1), 0.0] for k in range(10)]  # starting one interval on
    t, s = simlingo_ref.speed_reference(ahead)
    assert len(s) == 10 and s == pytest.approx([v] * 10)
    assert t[0] == pytest.approx(0.5 * simlingo_ref.WP_DT)
    from_car = [[0.0, 0.0]] + ahead[:-1]  # starting at the car
    assert simlingo_ref.speed_reference(from_car)[1] == pytest.approx([v] * 9)
    assert simlingo_ref.speed_reference([[0.0, 0.0]] * 11)[1] == pytest.approx([0.0] * 10)  # at rest


def test_speed_reference_of_an_older_prediction():
    """0.6 s on, the intervals ending before then are dropped but for the one now falls in, the times from now."""
    ahead = [[2.0 * (k + 1), 0.0] for k in range(10)]  # 8 m/s, 0.25 s apart
    t, s = simlingo_ref.speed_reference(ahead, age=0.6)
    assert t[0] == pytest.approx(0.375 - 0.6) and t[1] == pytest.approx(0.625 - 0.6)
    assert len(s) == 9 and s == pytest.approx([8.0] * 9)
    t, s = simlingo_ref.speed_reference(ahead, age=5.0)  # older than the whole prediction: its last speed
    assert len(s) == 1 and s == pytest.approx([8.0])


def test_carry_a_prediction_into_the_cars_frame_now():
    """Points SimLingo predicted, seen after the car has driven 5 m on and turned a quarter to the left: the car now
    at (5, 0, pi/2) in the prediction's frame."""
    pts = [[5.0, 2.0], [10.0, 0.0]]  # SimLingo's: y right
    out = simlingo_ref.carry(pts, (5.0, 0.0, 0.5 * math.pi))
    # (5, -2) y-left is 2 m to the right of the car now; facing +y, that is 2 m behind it: (-2, 0)
    # (10, 0) is 5 m on along the old x, to the car's right now: (0, 5) with y right
    assert out == pytest.approx(np.array([[-2.0, 0.0], [0.0, 5.0]]), abs=1e-9)
    assert simlingo_ref.carry(pts, (0.0, 0.0, 0.0)) == pytest.approx(np.array(pts))  # not moved: unchanged


def test_odometry_round_a_bend():
    """A quarter circle of radius 20 m to the left at 10 m/s: the compass (CARLA's, clockwise) falls by pi/2. The
    old frame as seen from the end: its origin 20 m behind and 20 m to the left, turned a quarter to the right."""
    odo = simlingo_ref.Odometry()
    v, r = 10.0, 20.0
    n = 2000
    dt = (0.5 * math.pi * r / v) / n
    odo.update(v, 0.3, 0.0)  # the compass's first reading
    for k in range(1, n + 1):
        odo.update(v, 0.3 - (v / r) * dt * k, dt)
    assert (odo.x, odo.y, odo.psi) == pytest.approx((r, r, 0.5 * math.pi), abs=1e-3)
    assert odo.old_frame() == pytest.approx((-r, r, -0.5 * math.pi), abs=1e-3)
    assert (odo.x, odo.y, odo.psi) == (0.0, 0.0, 0.0)


def test_frame_round_trip():
    f = frames.Frame(10.0, -3.0, 0.4)
    assert f.to_world(*f.to_car(4.0, 7.0)) == pytest.approx((4.0, 7.0))
    assert f.to_car(10.0 + 2.0 * math.cos(0.4), -3.0 + 2.0 * math.sin(0.4)) == pytest.approx((2.0, 0.0))  # ahead


expertctl = pytest.importorskip("expertctl", reason="build the module first: cmake --build build")


def _nmpc():
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    route = expertctl.Path.from_waypoints([[0.0, 0.0], [400.0, 0.0]], False)
    return expertctl.UrbanNMPC(route, params, expertctl.Body(), expertctl.UrbanOptions())


def _problem(t, x, v, y=0.0, psi=0.0):
    pr = expertctl.Problem()
    pr.t = t
    pr.ego = expertctl.Ego(x=x, y=y, psi=psi, v=v)
    pr.v_max = 10.0
    pr.corridor = expertctl.Corridor([0.0], [-1.75], [1.75], [0.0])
    return pr


def _reference():
    """The plan at t = 0.1 on the original route, warm from one at t = 0."""
    nmpc = _nmpc()
    nmpc.solve(_problem(0.0, 50.0, 10.0))
    return nmpc.solve(_problem(0.1, 51.0, 10.0))


def test_set_route_carries_the_plan_over():
    """The same road as a path starting elsewhere: after set_route the plan carried over is the same in the world,
    so the next solve starts warm and plans the same."""
    same = _reference()
    nmpc = _nmpc()
    assert nmpc.solve(_problem(0.0, 50.0, 10.0)).success
    nmpc.set_route(expertctl.Path.from_waypoints([[30.0, 0.0], [600.0, 0.0]], False))  # s = x - 30 now
    moved = nmpc.solve(_problem(0.1, 51.0, 10.0))
    assert moved.success
    assert moved.x[:, 0] == pytest.approx(same.x[:, 0] - 30.0, abs=0.05)
    assert moved.iterations <= same.iterations + 2


def test_set_route_into_the_cars_frame():
    """The road given in the car's frame after it has moved to x = 51 m: the old frame's origin is then 51 m behind
    it, and the plan comes out the same; and so with the car's frame turned a quarter against the old one."""
    same = _reference()
    for turn in (0.0, 0.5 * math.pi):
        nmpc = _nmpc()
        nmpc.solve(_problem(0.0, 50.0, 10.0))
        c, s = math.cos(turn), math.sin(turn)
        # the road in the new frame: along its axis at angle `turn`, from 51 m behind the car
        road = [[-51.0 * c, -51.0 * s], [349.0 * c, 349.0 * s]]
        nmpc.set_route(expertctl.Path.from_waypoints(road, False), expertctl.Pose(-51.0 * c, -51.0 * s, turn))
        moved = nmpc.solve(_problem(0.1, 0.0, 10.0, psi=turn))
        assert moved.success
        assert moved.x[:, 0] == pytest.approx(same.x[:, 0], abs=0.05)
        assert moved.iterations <= same.iterations + 2


def test_behaviour_set_route():
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.BehaviourOptions()
    options.pool = 1
    b = expertctl.Behaviour(expertctl.Path.from_waypoints([[0.0, 0.0], [400.0, 0.0]], False), params,
                            expertctl.Body(), expertctl.UrbanOptions(), options)
    scene = expertctl.Scene()
    scene.ego = expertctl.Ego(x=20.0, v=8.0)
    scene.v_max = 10.0
    assert b.step(scene).plan.success
    b.set_route(expertctl.Path.from_waypoints([[10.0, 0.5], [400.0, 0.5]], False))
    assert b.route.length == pytest.approx(390.0)
    scene.t = 0.2
    scene.ego = expertctl.Ego(x=21.6, v=8.0)
    d = b.step(scene)
    assert d.plan.success
    assert d.plan.x[0, 0] == pytest.approx(11.6, abs=0.1)  # s from the new path's start
    assert d.plan.x[0, 1] == pytest.approx(-0.5, abs=1e-6)  # the car 0.5 m right of the new path


def test_live_view_serves_a_plan():
    """A behaviour step on a small scene, its snapshot published, and the page and the stream read back."""
    import json
    import urllib.request
    from expert import live_view
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.BehaviourOptions()
    options.pool = 2
    path = expertctl.Path.from_waypoints([[-15.0, 0.0], [0.0, 0.0], [200.0, 0.0]], False)
    b = expertctl.Behaviour(path, params, expertctl.Body(), expertctl.UrbanOptions(), options)
    scene = expertctl.Scene()
    scene.ego = expertctl.Ego(x=0.3, v=6.0)
    scene.v_max = 8.0
    scene.v_ref_t, scene.v_ref = [0.125, 0.375], [6.0, 6.5]
    parked = expertctl.Actor()
    parked.id, parked.parked = 7, True
    parked.t, parked.x, parked.y, parked.psi = [0.0, 5.0], [30.0, 30.0], [2.8, 2.8], [0.0, 0.0]
    scene.actors = [parked]
    scene.stops = [expertctl.StopLine(1, expertctl.StopLine.Kind.red_light, 60.0)]
    d = b.step(scene)
    snap = live_view.snapshot(0.0, path, 15.3, 35.0, scene, d)
    assert snap["actors"][0]["id"] == 7 and snap["stops"][0]["kind"] == "red_light"
    assert len(snap["plan"]) == len(d.plan.times) and snap["corridor"]
    view = live_view.LiveView(18765)
    try:
        view.publish(snap)
        assert b"NMPC live view" in urllib.request.urlopen("http://127.0.0.1:18765/", timeout=5).read()
        stream = urllib.request.urlopen("http://127.0.0.1:18765/stream", timeout=5)
        line = stream.readline()
        assert line.startswith(b"data: ")
        assert json.loads(line[6:])["label"] == d.label
        stream.close()
    finally:
        view.close()
