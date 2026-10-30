"""Route.lanes against a small fake of CARLA's map: a straight road along x with lanes of 3 m."""
import math
import types

import numpy as np
import pytest

from expert import frames
from expert.world_model.route import Route

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


class _Map:
    def __init__(self, lanes):
        self.lanes = lanes

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
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
