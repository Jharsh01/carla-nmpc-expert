"""Route.lanes (middle_layer/perception/sensors/map.py) against a small fake of CARLA's map: a straight road along x with lanes of 3 m."""
import math
import types

import numpy as np
import pytest

from expert import frames
from middle_layer.perception.sensors.map import Route

expertctl = pytest.importorskip("expertctl", reason="build the module first: cmake --build build")

W = 3.0
LaneType = types.SimpleNamespace(Driving="Driving", Bidirectional="Bidirectional", Shoulder="Shoulder",
                                 Sidewalk="Sidewalk")
fake_carla = types.SimpleNamespace(LaneType=LaneType, Location=lambda x, y, z: types.SimpleNamespace(x=x, y=y, z=z))


class _Lane:
    """A lane of the fake road, in CARLA's frame (y to the right): its id, centre y, direction, type."""

    def __init__(self, lane_id, y, yaw, lane_type):
        self.lane_id, self.y, self.yaw, self.lane_type = lane_id, y, yaw, lane_type
        self.left = self.right = None  # CARLA's: relative to the lane's own direction


class _Waypoint:
    def __init__(self, lane, x):
        self._lane, self.road_id, self.section_id, self.lane_id = lane, 1, 0, lane.lane_id
        self.lane_type, self.lane_width, self.is_junction = lane.lane_type, W, False
        self.transform = types.SimpleNamespace(location=types.SimpleNamespace(x=x, y=lane.y, z=0.0),
                                               rotation=types.SimpleNamespace(yaw=lane.yaw))

    def get_left_lane(self):
        return _Waypoint(self._lane.left, self.transform.location.x) if self._lane.left else None

    def get_right_lane(self):
        return _Waypoint(self._lane.right, self.transform.location.x) if self._lane.right else None

    def next(self, d):
        return [_Waypoint(self._lane, self.transform.location.x + d)]


class _Map:
    def __init__(self, lanes):
        self.lanes = lanes

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        self.z = loc.z
        driving = [l for l in self.lanes if l.lane_type == LaneType.Driving]
        return _Waypoint(min(driving, key=lambda l: abs(l.y - loc.y)), loc.x)


def _road(*lanes):
    """Lanes listed from CARLA's left (-y) to right (+y), each (id, yaw, type); linked as CARLA links them."""
    out = [_Lane(i, (k - len(lanes) // 2) * W, yaw, t) for k, (i, yaw, t) in enumerate(lanes)]
    for a, b in zip(out, out[1:]):  # a is on b's left (smaller y)
        if b.yaw == 0.0:
            b.left = a
        else:
            b.right = a
        if a.yaw == 0.0:
            a.right = b
        else:
            a.left = b
    return out


def _route(lanes):
    world = types.SimpleNamespace(get_actors=lambda: types.SimpleNamespace(filter=lambda name: []))
    return Route(expertctl, fake_carla, world, _Map(lanes))


def _path(points):
    return expertctl.Path.from_waypoints([list(p) for p in points], False)


def _at(sections, s):
    return next(l for l in sections if l.s_from <= s < l.s_to)


# from the left: a sidewalk, an oncoming lane, the car's lane (CARLA y = 0), a shoulder
TWO_WAY = [(2, 180.0, LaneType.Sidewalk), (1, 180.0, LaneType.Driving), (-1, 0.0, LaneType.Driving),
           (-2, 0.0, LaneType.Shoulder)]


def test_lane_band_stays_on_the_lane_when_the_path_leaves_it_at_an_angle():
    route = _route(_road(*TWO_WAY))
    frame = frames.Frame(0.0, 0.0, 0.0)
    a = math.radians(-30.0)  # heading off to the right (the library's y is to the left)
    path = _path([(-20.0, 0.0), (0.0, 0.0)] + [(d * math.cos(a), d * math.sin(a)) for d in np.arange(2.0, 30.0, 2.0)])
    sections = route.lanes(path, frame)
    for l in sections:  # never wider than the lane seen at 30 deg (the old union made it 9-15 m)
        assert l.hi - l.lo < W / math.cos(a) + 0.1
    for s in np.arange(0.0, 26.0, 2.0):  # the band's right edge is the lane's, wherever the path is (where measured)
        l, q = _at(sections, s), path.pose(s)
        right = q.y + l.lo * math.cos(q.psi)  # where the normal there meets the lane's right edge (library y)
        assert right == pytest.approx(-0.5 * W, abs=0.15)
        assert l.road_lo == pytest.approx(l.lo, abs=0.05)  # the shoulder is not road
        assert l.left_oncoming and l.left_allowed


def test_road_edges_are_the_outermost_driving_lanes():
    route = _route(_road(*TWO_WAY))
    sections = route.lanes(_path([(-20.0, 0.0), (60.0, 0.0)]), frames.Frame(0.0, 0.0, 0.0))
    l = _at(sections, 30.0)
    assert (l.lo, l.hi) == (pytest.approx(-1.5), pytest.approx(1.5))
    assert (l.road_lo, l.road_hi) == (pytest.approx(-1.5), pytest.approx(4.5))  # the oncoming lane, not the sidewalk


def test_a_lane_change_takes_in_both_lanes_only_where_it_is():
    route = _route(_road((-1, 0.0, LaneType.Driving), (-2, 0.0, LaneType.Driving)))  # two lanes the same way
    # from the left lane (CARLA y -3, library +3) over to the right one (y 0) between x 40 and 60
    pts = [(x, 3.0 if x <= 40 else 0.0 if x >= 60 else 3.0 - 3.0 * (x - 40) / 20) for x in np.arange(-20.0, 121.0, 2.0)]
    sections = route.lanes(_path(pts), frames.Frame(0.0, 0.0, 0.0))
    assert _at(sections, 70.0).hi - _at(sections, 70.0).lo > 1.9 * W  # around the change: both lanes
    for s in (10.0, 125.0):  # well before and after it: one lane
        assert _at(sections, s).hi - _at(sections, s).lo == pytest.approx(W, abs=0.1)


def test_lane_path_keeps_to_the_lane_and_not_the_car():
    """The car 0.4 m left of its lane's centre, a little turned: the path is the lane's centre, in the car's frame."""
    route = _route(_road((-1, 0.0, LaneType.Driving), (1, 180.0, LaneType.Driving)))
    centre = route.map.lanes[0].y
    frame = frames.Frame(10.0, -centre + 0.4, 0.05)
    path = route.lane_path(frame)
    s, e = path.project(0.0, 0.0)
    assert e == pytest.approx(0.4, abs=0.02)
    assert s == pytest.approx(15.0, abs=0.3)
    assert path.length > 150.0
    x, y = frame.to_world(path.pose(s + 30.0).x, path.pose(s + 30.0).y)
    assert y == pytest.approx(-centre, abs=0.02)


def test_lane_path_from_the_oncoming_lane_is_the_lane_to_come_back_to():
    """Bench2Drive route 4: out in the oncoming lane round a blocker, the path is the car's own lane beside it, not
    the oncoming lane along the way the car goes (which lost the blocker and kept the car in the oncoming flow)."""
    route = _route(_road((1, 180.0, LaneType.Driving), (-1, 0.0, LaneType.Driving)))
    oncoming, ours = route.map.lanes
    frame = frames.Frame(10.0, -oncoming.y, 0.0)  # in the oncoming lane, heading our way
    path = route.lane_path(frame)
    s, e = path.project(0.0, 0.0)
    assert e == pytest.approx(W, abs=0.05)  # the car is a lane to the left of the path
    x, y = frame.to_world(path.pose(s + 30.0).x, path.pose(s + 30.0).y)
    assert y == pytest.approx(-ours.y, abs=0.02)
    assert x > 30.0  # along the way the car goes


def test_lanes_are_looked_up_at_the_cars_height():
    """Town12's roads are 300-380 m up: from z = 0 CARLA gives the lane beside the car on 19 % of its road points."""
    route = _route(_road((-1, 0.0, LaneType.Driving), (1, 180.0, LaneType.Driving)))
    route.lane_path(frames.Frame(10.0, -route.map.lanes[0].y, 0.0, 358.0))
    assert route.map.z == 358.0


def _lanes_on(route, path, frame):
    route.lanes(path, frame)
    return lambda x, y: route.ego_lanes(path, x, y)


def test_ego_lanes_are_the_paths_lane_and_both_lanes_of_a_lane_change():
    # from the left: an oncoming lane, a lane our way, the car's lane (CARLA y = 0), a shoulder
    route = _route(_road((1, 180.0, LaneType.Driving), (-2, 0.0, LaneType.Driving), (-1, 0.0, LaneType.Driving),
                         (-3, 0.0, LaneType.Shoulder)))
    frame = frames.Frame(0.0, 0.0, 0.0)
    straight = _path([(x, 0.0) for x in range(-15, 100, 2)])
    used = _lanes_on(route, straight, frame)
    assert {k[2] for k in used(20.0, 0.0)} == {-1}
    assert {k[2] for k in used(20.0, 3.0)} == {-1}  # level with the point: still the car's lane
    change = _path([(x, 0.0) for x in range(-15, 20, 2)] + [(x, 3.0) for x in range(30, 100, 2)])
    used = _lanes_on(route, change, frame)
    assert {k[2] for k in used(25.0, 1.5)} == {-1, -2}
    assert {k[2] for k in used(80.0, 3.0)} == {-2, -1}  # far along it: its lane, and the one the car's body is in now


def test_ego_lanes_are_unknown_in_a_junction():
    route = _route(_road(*TWO_WAY))
    route._ego_lanes = [{(1, 0, -1)}, None, None, {(1, 0, -1)}, {(1, 0, -1)}, {(1, 0, -1)}]
    path = _path([(x, 0.0) for x in range(0, 30, 2)])
    assert route.ego_lanes(path, 2.0, 0.0) is None
    assert route.ego_lanes(path, 9.0, 0.0) == {(1, 0, -1)}


def test_ego_lanes_take_in_any_lane_the_cars_body_is_in():
    """Bench2Drive route 1: stood across the double yellow, nose in the oncoming lane, with the path back in its own: a
    car in that lane was taken to stay out of the car's way, and it hit the car."""
    route = _route(_road(*TWO_WAY))
    frame = frames.Frame(0.0, 1.0, math.radians(45.0))  # library's y: 1 m to the left of the lane's centre, turned left
    path = _path([(x, -x) for x in range(-10, 60, 2)])  # in the car's frame: back along the lane
    used = _lanes_on(route, path, frame)
    assert {k[2] for k in used(20.0, -20.0)} == {-1, 1}


def test_the_lanes_either_side_and_which_way_they_run():
    """From the left: an oncoming lane, the car's lane, a lane the car's way. Read from the map, not assumed."""
    road = [(2, 180.0, LaneType.Driving), (-1, 0.0, LaneType.Driving), (-2, 0.0, LaneType.Driving)]
    route = _route(_road(*road))
    centre = route.map.lanes[1].y
    l = _at(route.lanes(_path([(-20.0, -centre), (60.0, -centre)]), frames.Frame(0.0, 0.0, 0.0)), 30.0)
    assert l.left_allowed and l.left_oncoming and l.left_width == pytest.approx(W)
    assert l.right_allowed and not l.right_oncoming and l.right_width == pytest.approx(W)
    route = _route(_road(*TWO_WAY))  # a shoulder on the right: no lane there
    l = _at(route.lanes(_path([(-20.0, 0.0), (60.0, 0.0)]), frames.Frame(0.0, 0.0, 0.0)), 30.0)
    assert not l.right_allowed and l.right_width == 0.0


def test_where_the_band_takes_in_two_lanes_its_sides_are_beyond_both():
    """Bench2Drive route 1: changing into the lane on the left, which runs the car's way, with an oncoming lane beyond
    it. Around the change the band takes in both lanes our way: what is on its left is the oncoming lane, and on its
    right whatever is right of the lane the car leaves (nothing here)."""
    road = [(2, 180.0, LaneType.Driving), (-1, 0.0, LaneType.Driving), (-2, 0.0, LaneType.Driving)]
    route = _route(_road(*road))
    right_lane, left_lane = -route.map.lanes[2].y, -route.map.lanes[1].y  # library y
    pts = [(x, right_lane if x <= 40 else left_lane if x >= 60 else right_lane + (left_lane - right_lane) * (x - 40) / 20)
           for x in np.arange(-20.0, 121.0, 2.0)]
    sections = route.lanes(_path(pts), frames.Frame(0.0, 0.0, 0.0))
    l = _at(sections, 70.0)
    assert l.hi - l.lo > 1.9 * W
    assert l.left_allowed and l.left_oncoming
    assert not l.right_allowed
    before = _at(sections, 10.0)  # in the right lane: the lane our way on its left
    assert before.left_allowed and not before.left_oncoming


def path_pose(pts, s):
    return _path(pts).pose(s)


def test_a_band_out_into_the_oncoming_lane_says_which_part_runs_our_way():
    route = _route(_road(*TWO_WAY))
    own, other = -route.map.lanes[2].y, -route.map.lanes[1].y  # library y of the car's lane and the oncoming one
    pts = [(x, own if x <= 40 else other if x >= 60 else own + (other - own) * (x - 40) / 20)
           for x in np.arange(-20.0, 121.0, 2.0)]
    sections = route.lanes(_path(pts), frames.Frame(0.0, 0.0, 0.0))
    l = _at(sections, 70.0)  # x 50, half way over
    assert l.hi - l.lo > 1.9 * W  # both lanes
    # all through the change, across the path where it is: our way is the band's right lane only, the left one
    # oncoming (a merge of offsets measured where the path was still in our lane put it on the oncoming lane)
    for s in np.arange(62.0, 80.0, 2.0):
        l, q = _at(sections, s), path_pose(pts, s)
        assert l.way_hi - l.way_lo == pytest.approx(W, abs=0.3)
        assert q.y + l.way_hi * math.cos(q.psi) == pytest.approx(own + 0.5 * W, abs=0.3)  # our lane's left edge
    straight = _at(route.lanes(_path([(-20.0, own), (60.0, own)]), frames.Frame(0.0, 0.0, 0.0)), 30.0)
    assert (straight.way_lo, straight.way_hi) == (pytest.approx(straight.lo), pytest.approx(straight.hi))
