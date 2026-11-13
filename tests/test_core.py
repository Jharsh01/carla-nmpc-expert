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


def _planner_and_scene(v):
    """A straight lane, 15 m of path behind the car as the agent builds it, planning as the agent does (2 s ahead)."""
    route = expertctl.Path.from_waypoints([[x, 0.0] for x in range(-15, 200, 5)], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.a_min = -4.6
    options.horizon, options.fine = 2.0, 1.0
    options.h_fine, options.h = 0.05, 0.2
    planner_options = expertctl.PlannerOptions()
    planner_options.pool = 1
    planner = expertctl.Planner(route, params, expertctl.Body(), options, planner_options)
    scene = expertctl.Scene()
    scene.ego = expertctl.Ego(x=0.3, v=v)
    scene.v_max = 8.33
    return planner, scene


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
    planner, scene = _planner_and_scene(7.9)
    scene.actors = [_car(0.3 - gap, 0.0, speed)]
    decision = planner.step(scene)
    assert not any("cut-in" in note for note in decision.notes)
    assert decision.plan.x[:, 4].min() > -1.0  # no braking
    assert decision.plan.x[-1, 3] > 7.0


def test_a_car_level_with_the_car_can_still_cut_in():
    """Beside the car, not behind it, and moving into the lane ahead of it: braking is the right answer."""
    planner, scene = _planner_and_scene(7.9)
    scene.actors = [_car(5.0, 3.0, 4.0, -1.5)]
    decision = planner.step(scene)
    assert any("cut-in" in note for note in decision.notes)
    assert decision.plan.x[:, 4].min() < -1.0


def test_a_car_that_stays_out_is_left_out_of_the_plan():
    """Bench2Drive route 1: an oncoming car in its own lane, projected across a slanted path, read as coming down the
    car's lane, and the car stopped dead. Here a car the same as the cut-in above, but known to stay in its lane."""
    planner, scene = _planner_and_scene(7.9)
    car = _car(5.0, 3.0, 4.0, -1.5)
    car.stays_out = True
    scene.actors = [car]
    decision = planner.step(scene)
    assert not any("cut-in" in note for note in decision.notes)
    assert decision.plan.x[:, 4].min() > -1.0
    assert not decision.candidates[0].problem.movers


def test_a_car_that_stays_out_counts_once_the_car_would_go_into_its_lane():
    """Borrowing the left lane round a blocker: the cars in that lane are kept clear of, and oncoming ones hold it back."""
    planner_options = expertctl.PlannerOptions()
    route = expertctl.Path.from_waypoints([[x, 0.0] for x in range(-15, 200, 5)], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.horizon, options.fine, options.h_fine, options.h = 2.0, 1.0, 0.05, 0.2
    planner = expertctl.Planner(route, params, expertctl.Body(), options, planner_options)
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

    planner_scene = scene_with([])
    assert any(c.label == "borrow_left" for c in planner.step(planner_scene).candidates)  # free to go round

    oncoming = _car(40.0, 3.2, -8.0)
    oncoming.id, oncoming.stays_out, oncoming.psi = 3, True, [math.pi] * len(oncoming.t)
    decision = planner.step(scene_with([oncoming]))
    assert not any(c.label == "borrow_left" for c in decision.candidates)
    assert any("oncoming traffic" in note for note in decision.notes)

    alongside = _car(10.0, 3.2, 5.0)
    alongside.id, alongside.stays_out = 3, True
    decision = planner.step(scene_with([alongside]))
    borrow = next(c for c in decision.candidates if c.label == "borrow_left")
    base = next(c for c in decision.candidates if c.label == "base")
    assert len(borrow.problem.movers) > len(base.problem.movers)


def test_a_parked_car_facing_the_car_is_not_oncoming_traffic():
    """Bench2Drive routes 4 and 6: a row of cars parked on the far side, facing the car, read as oncoming traffic, so
    the way round the blocker was never clear and the car waited for good. They are still kept clear of, as edges of
    the way round."""
    planner_options = expertctl.PlannerOptions()
    route = expertctl.Path.from_waypoints([[x, 0.0] for x in range(-15, 200, 5)], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.horizon, options.fine, options.h_fine, options.h = 2.0, 1.0, 0.05, 0.2
    planner = expertctl.Planner(route, params, expertctl.Body(), options, planner_options)
    scene = expertctl.Scene()
    scene.ego = expertctl.Ego(x=0.3, v=5.0)
    scene.v_max = 8.33
    scene.lanes = [expertctl.LaneSection(lo=-1.6, hi=1.6, left_width=3.2, left_oncoming=True, left_allowed=True,
                                         road_lo=-1.6, road_hi=8.0)]
    blocker = _car(25.0, 0.0, 0.0)
    blocker.id, blocker.parked = 2, True
    parked = _car(30.0, 5.6, 0.0)  # its inner side 0.85 m into the left lane, as on route 4
    parked.id, parked.parked, parked.psi = 3, True, [math.pi] * len(parked.t)
    scene.actors = [blocker, parked]
    decision = planner.step(scene)
    assert not any("oncoming traffic" in note for note in decision.notes)
    borrow = next(c for c in decision.candidates if c.label == "borrow_left")
    # still kept clear of: an edge of the way round, at its inner side less the margin (5.6 - 0.95 - 0.2)
    cr = borrow.problem.corridor
    assert min(hi for s, hi in zip(cr.s, cr.hi) if 43.0 < s < 47.0) == pytest.approx(4.45, abs=0.01)  # x 30: s 45


def _two_way_road_with_a_blocker(v0, oncoming_at=None, oncoming_speed=8.3):
    """A two-way road, a blocker 15 m ahead of a car at rest, SimLingo asking to stop, and maybe an oncoming car."""
    planner_options = expertctl.PlannerOptions()
    route = expertctl.Path.from_waypoints([[x, 0.0] for x in range(-15, 200, 5)], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.horizon, options.fine, options.h_fine, options.h = 2.0, 1.0, 0.05, 0.2
    planner = expertctl.Planner(route, params, expertctl.Body(), options, planner_options)
    scene = expertctl.Scene()
    scene.ego = expertctl.Ego(x=0.3, v=v0)
    scene.v_max = 8.33
    scene.v_ref_t, scene.v_ref = [0.25, 0.75, 1.25, 1.75], [0.3, 0.1, 0.0, 0.0]
    scene.lanes = [expertctl.LaneSection(lo=-1.6, hi=1.6, left_width=3.2, left_oncoming=True, left_allowed=True,
                                         road_lo=-1.6, road_hi=4.8)]
    blocker = _car(15.0, 0.0, 0.0)
    blocker.id, blocker.parked = 2, True
    scene.actors = [blocker]
    if oncoming_at is not None:
        car = _car(oncoming_at, 3.2, -oncoming_speed)
        car.t, car.x, car.y = car.t[:9], car.x[:9], car.y[:9]  # 2 s of track: the pass takes longer
        car.id, car.psi = 3, [math.pi] * len(car.t)
        scene.actors = [blocker, car]
    return planner, scene


def test_the_way_round_fits_between_a_blocker_and_cars_parked_opposite():
    """Bench2Drive route 4: cones on one side, cars parked on the far side, 2.9 m apart: room for the car (1.84 m)
    with 0.5 m a side, but not for the circles that cover it and the parked cars, and the overtake stalled half way.
    Kept to as the edges of the way round, they leave it room."""
    planner, scene = _two_way_road_with_a_blocker(5.0)
    scene.lanes = [expertctl.LaneSection(lo=-1.5, hi=1.5, left_width=3.0, left_oncoming=True, left_allowed=True,
                                         road_lo=-1.5, road_hi=8.0)]
    blocker = scene.actors[0]
    blocker.y = [-0.5] * len(blocker.t)  # its left side at e 0.45
    row = []
    for i, x in enumerate((5.0, 11.0, 17.0, 23.0, 29.0)):  # their inner sides at e 3.35
        car = _car(x, 4.35, 0.0)
        car.id, car.parked, car.psi = 10 + i, True, [math.pi] * len(car.t)
        row.append(car)
    scene.actors = [blocker] + row
    decision = planner.step(scene)
    borrow = next(c for c in decision.candidates if c.label == "borrow_left")
    assert borrow.plan.success
    assert max(borrow.plan.slack_max[:-1]) <= expertctl.PlannerOptions().slack_ok
    assert decision.label == "borrow_left"


def test_a_clear_way_round_a_blocker_is_taken_while_simlingo_waits():
    """Bench2Drive route 4: SimLingo slows to a stop while it waits on the other lane; the overtake goes at the limit
    once the behaviour layer finds it clear, and is taken over waiting behind the blocker."""
    planner, scene = _two_way_road_with_a_blocker(0.0)
    decision = planner.step(scene)
    assert decision.label == "borrow_left"
    borrow = next(c for c in decision.candidates if c.label == "borrow_left")
    assert not borrow.problem.v_ref
    assert decision.plan.x[-1][3] > 1.0


def test_an_oncoming_car_beyond_the_prediction_holds_the_overtake():
    """Bench2Drive route 4: a pass from rest takes about 6 s, longer than the oncoming car's track (2 s here, 4 s in
    CARLA); judged on the track alone the car went, met the next car of the opposite flow half way and both stopped.
    Past its track it is carried on at its speed."""
    planner, scene = _two_way_road_with_a_blocker(0.0, oncoming_at=75.0)
    decision = planner.step(scene)
    assert "blocker: oncoming traffic, waiting" in decision.notes
    assert decision.label == "base"
    planner, scene = _two_way_road_with_a_blocker(0.0, oncoming_at=170.0)  # far enough
    assert planner.step(scene).label == "borrow_left"


def _road_with_a_blocker(lane, others=(), v0=5.0):
    """A blocker 25 m ahead in the car's lane, on a road whose neighbouring lanes are `lane`'s."""
    route = expertctl.Path.from_waypoints([[x, 0.0] for x in range(-15, 200, 5)], False)
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    options = expertctl.UrbanOptions()
    options.horizon, options.fine, options.h_fine, options.h = 2.0, 1.0, 0.05, 0.2
    planner = expertctl.Planner(route, params, expertctl.Body(), options, expertctl.PlannerOptions())
    scene = expertctl.Scene()
    scene.ego = expertctl.Ego(x=0.3, v=v0)
    scene.v_max = 8.33
    scene.lanes = [lane]
    blocker = _car(25.0, 0.0, 0.0)
    blocker.id, blocker.parked = 2, True
    scene.actors = [blocker] + list(others)
    return planner, scene


def test_round_a_blocker_only_where_the_map_has_a_lane():
    """No lane either side: stay behind. A lane only on the right: go round on the right, into that lane."""
    planner, scene = _road_with_a_blocker(expertctl.LaneSection(lo=-1.6, hi=1.6, road_lo=-1.6, road_hi=1.6))
    assert not any(c.label.startswith("borrow") for c in planner.step(scene).candidates)
    right = expertctl.LaneSection(lo=-1.6, hi=1.6, road_lo=-4.8, road_hi=1.6, right_width=3.2, right_oncoming=False,
                                  right_allowed=True)
    planner, scene = _road_with_a_blocker(right)
    decision = planner.step(scene)
    assert not any(c.label == "borrow_left" for c in decision.candidates)
    borrow = next(c for c in decision.candidates if c.label == "borrow_right")
    cr = borrow.problem.corridor
    at_blocker = [i for i, s in enumerate(cr.s) if 38.0 < s < 42.0]  # x 25: s 40
    assert min(cr.lo[i] for i in at_blocker) == pytest.approx(-4.8)
    assert max(cr.hi[i] for i in at_blocker) < -0.9  # right of the blocker
    assert max(cr.ref[i] for i in at_blocker) < -2.0


def test_into_a_lane_our_way_traffic_behind_holds_it_and_traffic_ahead_does_not():
    """Bench2Drive route 1: the lane on the left runs the car's way. A car coming up it from behind would be cut off;
    one ahead in it is followed. Oncoming traffic further over is not in the way."""
    lane = expertctl.LaneSection(lo=-1.6, hi=1.6, left_width=3.2, left_oncoming=False, left_allowed=True,
                                 road_lo=-1.6, road_hi=8.0)
    behind = _car(-8.0, 3.2, 9.0)  # cut off even pulling out at 0.5 g
    behind.id, behind.stays_out = 3, True
    planner, scene = _road_with_a_blocker(lane, [behind])
    decision = planner.step(scene)
    assert not any(c.label == "borrow_left" for c in decision.candidates)
    assert "blocker: traffic in the left lane (our way), waiting" in decision.notes
    ahead = _car(30.0, 3.2, 7.0)
    ahead.id, ahead.stays_out = 4, True
    far_over = _car(40.0, 6.4, -8.0)  # oncoming, two lanes over
    far_over.id, far_over.stays_out, far_over.psi = 5, True, [math.pi] * len(far_over.t)
    planner, scene = _road_with_a_blocker(lane, [ahead, far_over])
    decision = planner.step(scene)
    assert any(c.label == "borrow_left" for c in decision.candidates)


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


def test_a_parked_car_with_a_lane_free_beside_it_in_the_band_is_not_a_blocker():
    """Bench2Drive route 1: changing into the lane on the left, the band takes in both lanes; the car parked in the
    right one leaves the left one free, and is driven past within the band, not waited behind."""
    band = expertctl.LaneSection(lo=-1.6, hi=4.8, left_width=3.2, left_oncoming=True, left_allowed=True,
                                 road_lo=-1.6, road_hi=8.0)
    planner, scene = _road_with_a_blocker(band)
    decision = planner.step(scene)
    assert not any(n.startswith("blocker") for n in decision.notes)
    assert any("room beside it" in n for n in decision.notes)
    base = next(c for c in decision.candidates if c.label == "base")
    assert len(base.problem.movers) == 3  # its circles
    assert decision.plan.x[-1][3] > 3.0  # not stopping


def test_room_beside_a_parked_car_in_the_oncoming_lane_is_a_borrow():
    """A two-way road, SimLingo's path out into the oncoming lane: the band takes that lane in, but the room there is
    the oncoming traffic's. The parked car stays a blocker, and going round it waits for the oncoming car."""
    band = expertctl.LaneSection(lo=-1.6, hi=4.8, left_width=0.0, left_oncoming=True, left_allowed=False,
                                 road_lo=-1.6, road_hi=4.8, way_lo=-1.6, way_hi=1.6)
    oncoming = _car(60.0, 3.2, -8.0)
    oncoming.id, oncoming.psi = 3, [math.pi] * len(oncoming.t)
    planner, scene = _road_with_a_blocker(band, [oncoming])
    decision = planner.step(scene)
    assert not any("room beside it" in n for n in decision.notes)
    assert any(n.startswith("blocker at") for n in decision.notes)
    assert decision.label == "base"
    planner, scene = _road_with_a_blocker(band)  # and without it the room in the band is free to use
    decision = planner.step(scene)
    assert not any("room beside it" in n for n in decision.notes)


def test_the_drive_limit_is_the_vehicles_table_at_the_speed_of_each_solve():
    """0.5 g where the MKZ can give it; above about 10 m/s, what it can (config/vehicles/mkz2020.json, a_drive_max),
    taken at the speed at the start of the solve. A car accelerating harder than that already still gets a plan."""
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    assert params.a_drive(10.0) == pytest.approx(4.6)
    assert params.a_drive(11.25) == pytest.approx(0.5 * (4.6 + 3.831))
    assert params.a_drive(40.0) == pytest.approx(2.102)  # held beyond the table
    route = expertctl.Path.from_waypoints([[0.0, 0.0], [600.0, 0.0]], False)
    options = expertctl.UrbanOptions()
    nmpc = expertctl.UrbanNMPC(route, params, expertctl.Body(), options)

    def plan_from(v, a=0.0):
        nmpc.reset()
        problem = expertctl.Problem()
        problem.ego = expertctl.Ego(x=10.0, v=v, a=a)
        problem.v_max = 30.0
        problem.corridor = expertctl.Corridor([0.0], [-1.75], [1.75], [0.0])
        return nmpc.solve(problem)

    plan = plan_from(3.0)
    assert plan.success and nmpc.a_limit == pytest.approx(0.5 * 9.81)
    assert plan.x[:, 4].max() == pytest.approx(0.5 * 9.81, abs=0.05)  # all of it, to get up to speed
    plan = plan_from(15.0)
    assert plan.success and nmpc.a_limit == pytest.approx(3.431)
    assert plan.x[:, 4].max() <= 3.431 + 1e-6
    assert plan_from(15.0, a=6.0).success  # brought down at the jerk limit, not at once


def test_what_the_car_cannot_see_along_the_oncoming_lane_holds_the_overtake():
    """No oncoming car reported, but the oncoming lane is seen only 40 m ahead: a car may be coming from just beyond.
    Seen far enough, the way round is clear."""
    lane = expertctl.LaneSection(lo=-1.6, hi=1.6, left_width=3.2, left_oncoming=True, left_allowed=True,
                                 road_lo=-1.6, road_hi=4.8)
    planner, scene = _road_with_a_blocker(lane)
    scene.sight_left = 40.0
    decision = planner.step(scene)
    assert not any(c.label == "borrow_left" for c in decision.candidates)
    assert "blocker: left lane seen only 40 m ahead, waiting" in decision.notes
    planner, scene = _road_with_a_blocker(lane)
    scene.sight_left = 150.0
    assert any(c.label == "borrow_left" for c in planner.step(scene).candidates)


def test_an_overtake_goes_up_to_20_kmh_over_the_limit():
    """Out in the other lane the car may be over the limit, to be out of it sooner; the pass is judged at that speed,
    so a gap a little too short at the limit is taken."""
    lane = expertctl.LaneSection(lo=-1.6, hi=1.6, left_width=3.2, left_oncoming=True, left_allowed=True,
                                 road_lo=-1.6, road_hi=4.8)
    planner, scene = _road_with_a_blocker(lane)
    borrow = next(c for c in planner.step(scene).candidates if c.label == "borrow_left")
    assert borrow.problem.v_max == pytest.approx(8.33 + 20.0 / 3.6)

    def clear(over, sight):
        options = expertctl.PlannerOptions()
        options.overtake_over_limit = over
        planner, scene = _road_with_a_blocker(lane, v0=0.0)
        planner = expertctl.Planner(planner.route, expertctl.VehicleParams.from_file(
            os.path.join(REPO, "config", "vehicles", "mkz2020.json")), expertctl.Body(), _short_horizon(), options)
        scene.sight_left = sight
        return any(c.label == "borrow_left" for c in planner.step(scene).candidates)

    # from rest 25 m back: 100 m of sight needed at the limit, 96 m over it (the car is back in its lane further on)
    assert not clear(0.0, 98.0)
    assert clear(20.0 / 3.6, 98.0)


def _short_horizon():
    options = expertctl.UrbanOptions()
    options.horizon, options.fine, options.h_fine, options.h = 2.0, 1.0, 0.05, 0.2
    return options


def test_stopped_to_see_and_seeing_too_little_the_car_backs_up_then_edges_out():
    """Bench2Drive route 4: SimLingo asks to go round a blocker that hides the oncoming lane beyond it. At rest the
    planner asks to back up (the agent drives that), as far as there is room behind; then the plan comes forward
    over to the left of the lane, slowly, to see past the blocker."""
    lane = expertctl.LaneSection(lo=-1.6, hi=1.6, left_width=3.2, left_oncoming=True, left_allowed=True,
                                 road_lo=-1.6, road_hi=4.8)
    planner, scene = _road_with_a_blocker(lane, v0=0.0)
    scene.asks_change, scene.sight_left = 1, 40.0
    decision = planner.step(scene)
    assert decision.reverse == pytest.approx(expertctl.PlannerOptions().peek_back)
    assert decision.keep_lane
    scene.ego = expertctl.Ego(x=0.3 - expertctl.PlannerOptions().peek_back, v=0.0)  # backed up
    decision = planner.step(scene)
    assert decision.reverse == 0.0
    assert "peek: edged out to see down the left lane" in decision.notes
    base = decision.candidates[0]
    assert base.problem.v_max == pytest.approx(expertctl.PlannerOptions().peek_speed)
    cr = base.problem.corridor
    assert max(cr.ref) > 0.6 and max(cr.hi) == pytest.approx(1.9)  # over to the left, a little past the lane's edge
    for _ in range(3):  # from the plan held at rest, a few solves to get going
        decision = planner.step(scene)
    assert max(decision.plan.x[:, 3]) > 0.3  # coming forward
    scene.sight_left = 150.0  # it sees
    assert planner.step(scene).label == "borrow_left"

    behind = _car(-7.0, 0.0, 0.0)  # a car close behind: no room
    behind.id = 3
    planner, scene = _road_with_a_blocker(lane, [behind], v0=0.0)
    scene.asks_change, scene.sight_left = 1, 40.0
    assert planner.step(scene).reverse < 2.0
