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
    """The points of the path along SimLingo's route, in the car's frame (y left): the route where it is, each point at
    least `min_gap` past the last kept (closer ones, and ones going back on the last direction, dropped), carried back
    BEHIND metres along its first few metres. Not through the car: joining the car to a line a little to one side of it
    makes a bend of well under a metre's radius, and past that bend's radius the NMPC's path coordinates break down
    (1 - kappa e < 0: docs/learnings.md). The car's offset from the line is the NMPC's to take out."""
    kept = []
    for p in np.asarray(route_ego, dtype=float).reshape(-1, 2):
        p = np.array([p[0], -p[1]])  # SimLingo's y is to the right
        if kept:
            step = p - kept[-1]
            if np.hypot(*step) < min_gap:
                continue
            if len(kept) > 1 and np.dot(step, kept[-1] - kept[-2]) <= 0.0:
                continue
        kept.append(p)
    first, on = kept[0], kept[min(3, len(kept) - 1)]
    along = on - first
    along = along / np.hypot(*along) if np.hypot(*along) > 0.0 else np.array([1.0, 0.0])
    return np.array([first - d * along for d in np.arange(BEHIND, 0.0, -2.5)] + kept)


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


def asks_to_stop(v_ref_t, v_ref, within=2.0, below=1.0):
    """Whether speeds from speed_reference come down to a stop: under `below` m/s in the next `within` seconds, past
    the first half second (from rest the first interval is short however fast it is asked to go)."""
    return any(0.5 <= t <= within and v < below for t, v in zip(v_ref_t, v_ref))


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
        """One tick: dt seconds at `speed`, the compass now reading `compass`. CARLA's compass reads NaN now and then
        (SimLingo's transfuser_utils guards it as a simulation bug): that tick turns nothing, the last reading kept.
        Unguarded, route 4 carried SimLingo's path to NaN and the map lookup on it crashed the agent."""
        if not math.isfinite(compass):
            compass = self.heading
        turn = 0.0 if self.heading is None or compass is None else -math.remainder(compass - self.heading, 2.0 * math.pi)
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
