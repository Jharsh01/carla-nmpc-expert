"""The world model's tracks, on a stand-in for CARLA."""
import types

import numpy as np
import pytest

from expert import frames
from middle_layer.perception.sensors.actors import Actors, Track

expertctl = pytest.importorskip("expertctl", reason="build the module first: cmake --build build")


class _Location:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5


class _Actor:
    """As the agent sees an actor the scenario destroys: `is_alive` stays True and no call raises. Its location is
    the world's origin once it is destroyed (checked on CARLA 0.9.15: the handle that destroys it is another)."""
    type_id = "vehicle.test.car"
    attributes = {"number_of_wheels": 4}
    is_alive = True

    def __init__(self, actor_id, x, y):
        self.id, self.location = actor_id, _Location(x, y)
        self.bounding_box = types.SimpleNamespace(
            location=types.SimpleNamespace(x=0.0, y=0.0), extent=types.SimpleNamespace(x=2.4, y=1.0))

    def get_location(self):
        return self.location

    def get_transform(self):
        return types.SimpleNamespace(location=self.location, rotation=types.SimpleNamespace(yaw=0.0))


class _ActorList(list):
    def filter(self, pattern):
        return [a for a in self if a.type_id.startswith(pattern.rstrip("*"))]


def _actors(listed):
    world = types.SimpleNamespace(get_actors=lambda: _ActorList(listed))  # the list as it is at each call
    hero = _Actor(0, 3000.0, 0.0)  # as on Town12, far from the world's origin
    return Actors(expertctl, types.SimpleNamespace(Location=_Location), world, None, hero, visibility=False)


def test_a_destroyed_actor_is_dropped_at_once():
    """Bench2Drive route 0: kept for `keep` seconds, it was predicted along a road at the world's origin, and stood in
    the lane for 1.5 s as a crossing car, 3715 m of prediction long."""
    car = _Actor(7, 3010.0, 0.0)
    listed = [car]
    actors = _actors(listed)
    actors.detect(0.0)
    assert 7 in actors.tracks
    listed.clear()  # destroyed: gone from the list, and is_alive still True
    car.location = _Location(0.0, 0.0)
    actors.detect(0.2)
    assert 7 not in actors.tracks


def test_an_actor_out_of_sight_is_kept_a_while():
    car = _Actor(7, 3010.0, 0.0)
    actors = _actors([car])
    actors.detect(0.0)
    actors.visibility = True
    actors._visible = lambda actor, eye: False
    actors.detect(1.0)
    assert 7 in actors.tracks
    actors.detect(2.0)
    assert 7 not in actors.tracks


# ---- intent: a straight road along CARLA's x, lanes of 3.2 m, the ego's at y = 0, a lane the same way on its left
# (y = -3.2), an oncoming one beyond it (y = -6.4)

class _Wp:
    is_junction = False
    lane_width = 3.2

    def __init__(self, index, x, junction=False):
        self.index, self.x, self.is_junction = index, x, junction
        self.road_id, self.section_id, self.lane_id = 1, 0, index
        self.transform = types.SimpleNamespace(location=_Location(x, -3.2 * index),
                                               rotation=types.SimpleNamespace(yaw=180.0 if index == 2 else 0.0))

    def _beside(self, index):
        return _Wp(index, self.x) if 0 <= index <= 2 else None

    def get_left_lane(self):
        return self._beside(self.index - 1 if self.index == 2 else self.index + 1)

    def get_right_lane(self):
        return self._beside(self.index + 1 if self.index == 2 else self.index - 1)


class _IntentMap:
    def __init__(self, junction=False):
        self.junction = junction

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        return _Wp(max(0, min(2, round(-loc.y / 3.2))), loc.x, self.junction)


def _intent(y, vy=0.0, vx=8.0, width=1.9, yaw=0.0, parked=False, junction=False, ego=((1, 0, 0),), prediction_y=None):
    """Whether a car at CARLA (20, y), moving (vx, vy) in the library's y (to the left), stays out of the lanes `ego`."""
    carla = types.SimpleNamespace(Location=_Location, LaneType=types.SimpleNamespace(Driving="Driving"))
    actors = Actors(expertctl, carla, None, _IntentMap(junction), None, visibility=False)
    track = Track(1, "vehicle")
    track.width = width
    car = _Actor(1, 20.0, y)
    car.get_transform = lambda: types.SimpleNamespace(location=car.location, rotation=types.SimpleNamespace(yaw=yaw))
    track.actor = car
    for t in np.arange(0.0, 0.5, 0.05):  # library frame: y left
        track.add(t, 20.0 + vx * (t - 0.45), -y + vy * (t - 0.45), 0.0)
    a = expertctl.Actor()
    a.parked = parked
    a.t = [0.25 * i for i in range(21)]
    a.x = [20.0 + vx * t for t in a.t]
    a.y = [-y if prediction_y is None else prediction_y for t in a.t]
    return actors._stays_out(track, a, frames.Frame(0.0, 0.0, 0.0), lambda x, y: set(ego))


def test_a_car_going_along_the_next_lane_stays_out():
    assert _intent(-3.2)


def test_an_oncoming_car_in_its_own_lane_stays_out():
    assert _intent(-6.4, vx=-8.0, yaw=180.0)


def test_a_car_in_the_ego_lane_does_not_stay_out():
    assert not _intent(0.0)


def test_a_car_predicted_in_the_ego_lane_does_not_stay_out():
    assert not _intent(-3.2, prediction_y=0.0)


def test_a_car_moving_into_the_ego_lane_does_not_stay_out():
    assert not _intent(-3.2, vy=-0.8)


def test_a_car_moving_away_from_the_ego_lane_stays_out():
    assert _intent(-3.2, vy=0.8)


def test_a_car_reaching_into_the_ego_lane_does_not_stay_out():
    assert not _intent(-2.1)


def test_small_tracker_noise_is_not_a_lane_change():
    assert _intent(-3.2, vy=-0.1)


def test_the_target_lane_of_a_lane_change_counts():
    assert not _intent(-3.2, ego=((1, 0, 0), (1, 0, 1)))


def test_a_car_in_a_junction_or_crossing_is_not_left_out():
    assert not _intent(-3.2, junction=True)
    assert not _intent(-3.2, yaw=60.0)


def test_a_parked_car_is_left_as_it_is():
    assert not _intent(-3.2, vx=0.0, parked=True)
