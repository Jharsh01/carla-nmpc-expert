"""The plan's acceleration and road-wheel angle as CARLA throttle, brake and steer, for Bench2Drive's ego car
(vehicle.lincoln.mkz_2020, default physics), from the tables scripts/identify_ego.py measured
(config/vehicles/mkz2020_actuation.json).

Longitudinal: the throttle or brake whose measured acceleration at the current speed is the one asked for (the
tables interpolated in speed and level), with a PI loop on the acceleration error for what the tables miss (slope,
gear shifts, the tables' own noise). The tables are cleaned first: missing cells filled from their neighbours,
a median over three speeds, and each speed's column made monotone in the level.

Steering: the angle over the measured road-wheel angle per unit steer at the current speed (CARLA's steering curve
and the tyres together), negated, as CARLA steers right for positive steer.
"""
import json
import math

import numpy as np


def _clean(table, speeds, levels, increasing):
    """Rows by level, columns by speed: holes filled, median over 3 speeds, monotone in the level per speed."""
    m = np.array([[np.nan if v is None else v for v in table[str(lv)]] for lv in levels], dtype=float)
    for i in range(len(levels)):
        row = m[i]
        ok = ~np.isnan(row)
        if ok.any():
            m[i] = np.interp(speeds, np.array(speeds)[ok], row[ok])
    smoothed = m.copy()
    for j in range(len(speeds)):
        lo, hi = max(0, j - 1), min(len(speeds), j + 2)
        smoothed[:, j] = np.median(m[:, lo:hi], axis=1)
    for j in range(len(speeds)):
        col = smoothed[:, j]
        smoothed[:, j] = np.maximum.accumulate(col) if increasing else np.minimum.accumulate(col)
    return smoothed


class MKZActuation:
    def __init__(self, path, dt=0.05, kp=0.15, ki=0.3, i_max=1.5):
        r = json.load(open(path))
        self.speeds = np.array(r["speeds"], dtype=float)
        self.throttles = np.array([0.0] + [float(k) for k in r["throttle"]])
        self.brakes = np.array([0.0] + [float(k) for k in r["brake"]])
        coast = {"0.0": r["coast"]}
        th = _clean({**coast, **r["throttle"]}, self.speeds, self.throttles, increasing=True)
        br = _clean({**coast, **{k: v for k, v in r["brake"].items()}}, self.speeds, self.brakes, increasing=False)
        self.a_throttle, self.a_brake = th, br  # [level, speed] -> acceleration
        self.steer_speeds = np.array(r["steer_speeds"], dtype=float)
        self.steer_gain = np.array(r["steer_gain"], dtype=float)  # road-wheel angle per unit steer [rad], negative
        self.dt, self.kp, self.ki, self.i_max = dt, kp, ki, i_max
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def gain(self, v):
        return float(np.interp(v, self.steer_speeds, self.steer_gain))

    def delta_of(self, steer, v):
        """Road-wheel angle [rad] for a CARLA steer at speed v."""
        return steer * self.gain(v)

    def _column(self, table, v):
        return np.array([np.interp(v, self.speeds, row) for row in table])

    def command(self, a_ref, delta_ref, v, a_meas=None):
        """(throttle, brake, steer) for the acceleration a_ref [m/s^2] and road-wheel angle delta_ref [rad] at speed v."""
        if a_meas is not None:  # PI on the acceleration error, frozen at standstill
            if v > 0.5:
                self.integral = float(np.clip(self.integral + (a_ref - a_meas) * self.dt, -self.i_max, self.i_max))
            a = a_ref + self.kp * (a_ref - a_meas) + self.ki * self.integral
        else:
            a = a_ref
        th_col = self._column(self.a_throttle, v)
        throttle = brake = 0.0
        steer = float(np.clip(delta_ref / self.gain(v), -1.0, 1.0)) if self.gain(v) != 0.0 else 0.0
        if v < 0.3 and a_ref <= 0.0:  # at rest and not asked to go: hold it there
            self.integral = 0.0
            return 0.0, 0.7, steer
        if a >= th_col[0]:  # within what the throttle covers: interpolate the level that gives a
            throttle = float(np.interp(a, th_col, self.throttles)) if th_col[-1] > th_col[0] else 0.0
        else:
            br_col = self._column(self.a_brake, v)
            brake = float(np.interp(-a, -br_col, self.brakes))
        return float(np.clip(throttle, 0.0, 1.0)), float(np.clip(brake, 0.0, 1.0)), steer
