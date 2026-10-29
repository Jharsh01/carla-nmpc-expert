"""SimLingo's predictions as the NMPC's reference, in the car's own frame: its path as the route the NMPC plans
along, its speed waypoints as the speed asked for, and the car's motion between plans.

SimLingo predicts in the car's frame at the moment of the prediction, in CARLA's handedness: x forward, y to the
right, from the car's origin. The NMPC plans in that frame too, in the library's handedness (y to the left,
expert/frames.py), so nothing here needs to know where the car is in the world.
"""
import math

import numpy as np

WP_DT = 0.25  # [s] between speed waypoints (config_simlingo.py: 20 fps / (wp_dilation 1 x data_save_freq 5))
BEHIND = 15.0  # [m] the path reaches behind the car, so that actors and stops just behind it project onto it


def path_points(route_ego, min_gap=0.5):
    """The points of the path through SimLingo's route, in the car's frame (y left): from BEHIND metres behind the
    car's origin, through the origin, then the route's points, each at least `min_gap` past the last kept (closer
    ones, and ones going back on the last direction, dropped)."""
    kept = [np.array([-d, 0.0]) for d in np.arange(BEHIND, 0.0, -2.5)] + [np.zeros(2)]
    for p in np.asarray(route_ego, dtype=float).reshape(-1, 2):
        p = np.array([p[0], -p[1]])  # SimLingo's y is to the right
        step = p - kept[-1]
        if np.hypot(*step) < min_gap:
            continue
        if np.dot(step, kept[-1] - kept[-2]) <= 0.0:
            continue
        kept.append(p)
    return np.array(kept)


def speed_reference(speed_wps, age=0.0):
    """(times [s], speeds [m/s]) asked for: the speed over each interval between SimLingo's speed waypoints, at the
    interval's middle. The waypoints may start at the car (t = 0) or one interval on: the first is taken as the
    car's own position when it is much nearer the origin than the next one is to it. For a prediction `age` seconds
    old the times are from now: the intervals already past dropped but for the one now ends in."""
    w = np.asarray(speed_wps, dtype=float).reshape(-1, 2)
    if len(w) == 0:
        return [], []
    first_gap = np.hypot(*(w[1] - w[0])) if len(w) > 1 else 0.0
    if np.hypot(*w[0]) >= 0.5 * first_gap + 0.05:  # the first is one interval on: from the car
        w = np.vstack([np.zeros(2), w])
    v = np.hypot(*np.diff(w, axis=0).T) / WP_DT
    t = (np.arange(len(v)) + 0.5) * WP_DT - age
    keep = max(0, int(np.searchsorted(t, 0.0)) - 1)  # the last interval before now, for the speed now
    return list(map(float, t[keep:])), list(map(float, v[keep:]))


def carry(points_ego, pose):
    """SimLingo's car-frame points (y right) from an earlier prediction, in the car's frame now: `pose` is where the
    car is now in the prediction's frame (y left), as Odometry keeps it."""
    p = np.asarray(points_ego, dtype=float).reshape(-1, 2) * [1.0, -1.0]  # to y left
    x, y, psi = pose
    c, s = math.cos(psi), math.sin(psi)
    d = p - [x, y]
    return np.stack([c * d[:, 0] + s * d[:, 1], s * d[:, 0] - c * d[:, 1]], axis=1)  # rotated by -psi, y right again


class Odometry:
    """The car's motion since the last plan, from the speedometer and the compass, so that the last plan can be
    carried into the new plan's frame (UrbanNMPC::set_route's old_frame)."""

    def __init__(self):
        self.x = self.y = self.psi = 0.0  # where the car is, in the last plan's frame
        self.heading = None  # the compass's last reading [rad, CARLA's: clockwise]

    def update(self, speed, compass, dt):
        """One tick: dt seconds at `speed`, the compass now reading `compass`."""
        turn = 0.0 if self.heading is None else -math.remainder(compass - self.heading, 2.0 * math.pi)
        self.heading = compass
        mid = self.psi + 0.5 * turn
        self.x += speed * dt * math.cos(mid)
        self.y += speed * dt * math.sin(mid)
        self.psi += turn

    def pose(self):
        """Where the car is in the frame it started from: (x, y, psi)."""
        return self.x, self.y, self.psi

    def restart(self):
        """The frame from here on is the car's frame now (the compass's last reading kept)."""
        self.x = self.y = self.psi = 0.0

    def old_frame(self):
        """The last plan's frame as seen from where the car is now, and a fresh start for the next plan."""
        c, s = math.cos(self.psi), math.sin(self.psi)
        out = (-(c * self.x + s * self.y), s * self.x - c * self.y, -self.psi)
        self.x = self.y = self.psi = 0.0
        return out
