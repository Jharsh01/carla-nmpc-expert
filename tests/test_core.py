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
    p = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    assert p.m > 1000.0 and p.L() > 2.0


def test_frames_round_trip():
    assert frames.from_carla_xy(*frames.to_carla_xy(3.0, -2.0)) == (3.0, -2.0)
    assert frames.from_carla_yaw(frames.to_carla_yaw(0.3)) == pytest.approx(0.3)


def test_state_from_carla_moves_the_origin_to_the_cg():
    # heading north in the library's frame (CARLA yaw -90 deg), 10 m/s forward, no yaw rate
    x = frames.state_from_carla((0.0, 0.0), -90.0, (0.0, -10.0), 0.0, cg_ahead=1.5, delta=0.0)
    assert x[0] == pytest.approx(0.0, abs=1e-9) and x[1] == pytest.approx(1.5)
    assert x[2] == pytest.approx(math.pi / 2) and x[3] == pytest.approx(10.0) and x[4] == pytest.approx(0.0, abs=1e-9)


def test_urban_nmpc_stops_at_a_gate():
    """A stop line 50 m ahead at 12 m/s: the plan brakes and its front stays behind the line."""
    route = expertctl.Path.from_waypoints([[0.0, 0.0], [400.0, 0.0]], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.horizon = 8.0  # long enough to plan to standstill
    nmpc = expertctl.UrbanNMPC(route, params, expertctl.Body(), options)
    problem = expertctl.Problem()
    problem.ego = expertctl.Ego(x=10.0, v=12.0)
    problem.v_max = 13.9
    problem.corridor = expertctl.Corridor([0.0], [-1.75], [1.75], [0.0])
    problem.gates = [expertctl.Gate(60.0)]
    plan = nmpc.solve(problem)
    assert plan.success
    body = expertctl.Body()
    front = plan.x[:, 0] + 0.5 * body.length - body.cg_to_center
    assert front.max() <= 60.0 + 0.05
    assert plan.x[-1, 3] < 0.5  # at rest by the end
    assert max(plan.slack_max) < 1e-3
    assert list(expertctl.UrbanNMPC.groups()) == ["corridor", "movers", "ahead", "behind", "grip", "headway"]


def _behaviour_and_scene(v):
    """A straight lane, 15 m of path behind the car as the agent builds it, planning as the agent does (2 s ahead)."""
    route = expertctl.Path.from_waypoints([[x, 0.0] for x in range(-15, 200, 5)], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.a_min = -7.0
    options.horizon, options.fine = 2.0, 1.0
    options.h_fine, options.h = 0.05, 0.2
    behaviour_options = expertctl.BehaviourOptions()
    behaviour_options.pool = 1
    behaviour = expertctl.Behaviour(route, params, expertctl.Body(), options, behaviour_options)
    scene = expertctl.Scene()
    scene.ego = expertctl.Ego(x=0.3, v=v)
    scene.v_max = 8.33
    return behaviour, scene


def _car(x, y, vx, vy=0.0):
    car = expertctl.Actor()
    car.id, car.kind = 1, expertctl.Actor.Kind.vehicle
    car.length, car.width = 4.7, 1.9
    car.t = [0.25 * i for i in range(21)]
    car.x = [x + vx * t for t in car.t]
    car.y = [y + vy * t for t in car.t]
    car.psi = [0.0] * len(car.t)
    return car


@pytest.mark.parametrize("gap, speed", [(18.0, 9.3), (20.0, 9.3), (22.0, 9.3), (20.0, 7.9)])
def test_a_car_behind_is_not_a_cut_in(gap, speed):
    """Bench2Drive route 0: a car following 18-22 m back, off the path's start, read as cutting in where the car's
    rear is now, and it stopped the car dead in an empty lane."""
    behaviour, scene = _behaviour_and_scene(7.9)
    scene.actors = [_car(0.3 - gap, 0.0, speed)]
    decision = behaviour.step(scene)
    assert not any("cut-in" in note for note in decision.notes)
    assert decision.plan.x[:, 4].min() > -1.0  # no braking
    assert decision.plan.x[-1, 3] > 7.0


def test_a_car_level_with_the_car_can_still_cut_in():
    """Beside the car, not behind it, and moving into the lane ahead of it: braking is the right answer."""
    behaviour, scene = _behaviour_and_scene(7.9)
    scene.actors = [_car(5.0, 3.0, 4.0, -1.5)]
    decision = behaviour.step(scene)
    assert any("cut-in" in note for note in decision.notes)
    assert decision.plan.x[:, 4].min() < -1.0
