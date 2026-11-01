"""The world model's tracks, on a stand-in for CARLA."""
import types

import pytest

from expert.world_model.actors import Actors

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
