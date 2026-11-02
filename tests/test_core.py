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


def test_a_car_that_stays_out_is_left_out_of_the_plan():
    """Bench2Drive route 1: an oncoming car in its own lane, projected across a slanted path, read as coming down the
    car's lane, and the car stopped dead. Here a car the same as the cut-in above, but known to stay in its lane."""
    behaviour, scene = _behaviour_and_scene(7.9)
    car = _car(5.0, 3.0, 4.0, -1.5)
    car.stays_out = True
    scene.actors = [car]
    decision = behaviour.step(scene)
    assert not any("cut-in" in note for note in decision.notes)
    assert decision.plan.x[:, 4].min() > -1.0
    assert not decision.candidates[0].problem.movers


def test_a_car_that_stays_out_counts_once_the_car_would_go_into_its_lane():
    """Borrowing the left lane round a blocker: the cars in that lane are kept clear of, and oncoming ones hold it back."""
    behaviour_options = expertctl.BehaviourOptions()
    route = expertctl.Path.from_waypoints([[x, 0.0] for x in range(-15, 200, 5)], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.horizon, options.fine, options.h_fine, options.h = 2.0, 1.0, 0.05, 0.2
    behaviour = expertctl.Behaviour(route, params, expertctl.Body(), options, behaviour_options)
    lane = expertctl.LaneSection(lo=-1.6, hi=1.6, left_width=3.2, left_oncoming=True, left_allowed=True, road_lo=-1.6,
                                 road_hi=4.8)

    def scene_with(others):
        scene = expertctl.Scene()
        scene.ego = expertctl.Ego(x=0.3, v=5.0)
        scene.v_max = 8.33
        scene.lanes = [lane]
        blocker = _car(25.0, 0.0, 0.0)
        blocker.id, blocker.parked = 2, True
        scene.actors = [blocker] + others
        return scene

    behaviour_scene = scene_with([])
    assert any(c.label == "borrow_left" for c in behaviour.step(behaviour_scene).candidates)  # free to go round

    oncoming = _car(40.0, 3.2, -8.0)
    oncoming.id, oncoming.stays_out, oncoming.psi = 3, True, [math.pi] * len(oncoming.t)
    decision = behaviour.step(scene_with([oncoming]))
    assert not any(c.label == "borrow_left" for c in decision.candidates)
    assert any("oncoming traffic" in note for note in decision.notes)

    alongside = _car(10.0, 3.2, 5.0)
    alongside.id, alongside.stays_out = 3, True
    decision = behaviour.step(scene_with([alongside]))
    borrow = next(c for c in decision.candidates if c.label == "borrow_left")
    base = next(c for c in decision.candidates if c.label == "base")
    assert len(borrow.problem.movers) > len(base.problem.movers)


def test_a_steering_command_keeps_to_the_rate_across_a_jump():
    from expert.mkz_actuation import rate_limited
    assert rate_limited(0.5, 0.1, 0.6, 0.05) == pytest.approx(0.13)
    assert rate_limited(-0.5, 0.1, 0.6, 0.05) == pytest.approx(0.07)
    assert rate_limited(0.11, 0.1, 0.6, 0.05) == pytest.approx(0.11)


class _Plan:
    def __init__(self, success):
        self.success = success


def test_a_failed_plan_is_not_driven():
    from expert.plan_hold import PlanHold
    hold = PlanHold(hold=0.6, brake=-2.0)
    good = _Plan(True)
    hold.update(good, 1.0)
    assert hold.plan is good and hold.a_limit(1.1) == math.inf
    hold.update(_Plan(False), 1.2)
    assert hold.plan is good and hold.t == 1.0  # the last good one, from when it was made
    assert hold.a_limit(1.7) == math.inf  # followed on for a while
    hold.update(_Plan(False), 1.4)  # still counted from the first failure
    assert hold.plan is good and hold.a_limit(1.85) == -2.0  # then braking
    better = _Plan(True)
    hold.update(better, 2.0)
    assert hold.plan is better and hold.a_limit(2.0) == math.inf


def test_with_no_good_plan_yet_the_failed_one_stands_under_the_brake():
    from expert.plan_hold import PlanHold
    hold = PlanHold(hold=0.0, brake=-2.0)
    first = _Plan(False)
    hold.update(first, 0.0)
    assert hold.plan is first and hold.a_limit(0.0) == -2.0
