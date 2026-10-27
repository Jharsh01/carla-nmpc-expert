"""The route as the NMPC plans along it: a smooth reference path, the lanes along it and the stop lines on it.

Everything here is in the library's frame (x forward/east, y left, heading counter-clockwise; expert/frames.py),
built once per route from the leaderboard's dense route and CARLA's map, except the traffic-light states, which are
read each step.
"""
import math

import numpy as np

from expert import frames

SPACING = 2.0  # [m] between the reference path's points


def _xy(location):
    return frames.from_carla_xy(location.x, location.y)


def resample(points, spacing):
    """Points at even arc length along the polyline through `points` (N x 2), and their arc lengths."""
    seg = np.hypot(*np.diff(points, axis=0).T)
    keep = np.concatenate([[True], seg > 1e-3])
    points = points[keep]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(points, axis=0).T))])
    s_new = np.arange(0.0, s[-1], spacing)
    if s[-1] - s_new[-1] > 0.25 * spacing:
        s_new = np.append(s_new, s[-1])
    return np.column_stack([np.interp(s_new, s, points[:, 0]), np.interp(s_new, s, points[:, 1])]), s_new


def smooth_lane_changes(points, changes, half_window=12):
    """Where the route jumps from one lane to the next, a gradual change over about 2 x half_window points: the
    points around each change replaced by a Gaussian average of their neighbours (endpoints kept)."""
    out = points.copy()
    n = len(points)
    weights = np.exp(-0.5 * (np.arange(-half_window, half_window + 1) / (0.5 * half_window)) ** 2)
    for i in changes:
        for j in range(max(1, i - half_window), min(n - 1, i + half_window + 1)):
            lo, hi = max(0, j - half_window), min(n, j + half_window + 1)
            w = weights[lo - j + half_window: hi - j + half_window]
            out[j] = (points[lo:hi] * w[:, None]).sum(axis=0) / w.sum()
    return out


def lane_change_indices(points, threshold=1.0):
    """Indices where the route steps sideways by more than `threshold` over one spacing: the lane changes in the
    leaderboard's route, which jumps from one lane centre to the next."""
    d = np.diff(points, axis=0)
    heading = np.arctan2(d[:, 1], d[:, 0])
    out = []
    for i in range(1, len(d) - 1):
        # sideways step relative to the heading before and after
        before = heading[i - 1]
        side = -d[i, 0] * math.sin(before) + d[i, 1] * math.cos(before)
        if abs(side) > threshold:
            out.append(i)
    return out


class Route:
    """The reference path and what lies along it, for one leaderboard route."""

    def __init__(self, expertctl, carla, carla_map, dense_route):
        """`dense_route`: the leaderboard's [(carla.Transform, RoadOption), ...] (the agent's
        org_dense_route_world_coord), about a metre apart."""
        self.carla = carla
        self.map = carla_map
        raw = np.array([_xy(tr.location) for tr, _ in dense_route])
        pts, _ = resample(raw, SPACING)
        pts = smooth_lane_changes(pts, lane_change_indices(pts))
        self.points = pts
        self.path = expertctl.Path.from_waypoints([list(p) for p in pts], False)
        self.length = self.path.length
        self.lanes = self._lane_sections(expertctl)
        self.light_stops = []  # (light actor, s of its stop line); attach_world() fills these
        self.stop_signs = []  # (sign actor, s where the route enters its trigger volume, s where it leaves)

    # ---- lanes

    def _lane_sections(self, expertctl, window=6):
        """The ego lane's band and its left neighbour, every SPACING metres, merged where nothing changes. The band
        at each point is widened to the union over +-window points, so a lane change has both lanes in it."""
        carla = self.carla
        rows = []
        for s in np.arange(0.0, self.length, SPACING):
            pose = self.path.pose(s)
            wp = self.map.get_waypoint(carla.Location(*frames.to_carla_xy(pose.x, pose.y), 0.0),
                                       project_to_road=True, lane_type=carla.LaneType.Driving)
            cx, cy = _xy(wp.transform.location)
            e_c = -(cx - pose.x) * math.sin(pose.psi) + (cy - pose.y) * math.cos(pose.psi)  # lane centre off the route
            half = 0.5 * wp.lane_width
            left = wp.get_left_lane()
            left_ok = left is not None and left.lane_type == carla.LaneType.Driving and not wp.is_junction
            rows.append((s, e_c - half, e_c + half, left.lane_width if left_ok else 0.0,
                         bool(left_ok and left.lane_id * wp.lane_id < 0), bool(left_ok)))
        sections = []
        for i, (s, lo, hi, lw, onc, ok) in enumerate(rows):
            near = rows[max(0, i - window): i + window + 1]
            lo = min(r[1] for r in near)
            hi = max(r[2] for r in near)
            key = (round(lo, 1), round(hi, 1), round(lw, 1), onc, ok)
            if sections and sections[-1][1] == key:
                sections[-1][0][1] = s + SPACING
            else:
                sections.append(([s, s + SPACING], key))
        out = []
        for (s_from, s_to), (lo, hi, lw, onc, ok) in sections:
            out.append(expertctl.LaneSection(s_from=s_from, s_to=s_to, lo=lo, hi=hi, left_width=lw, left_oncoming=onc,
                                             left_allowed=ok))
        out[-1].s_to = float("inf")
        return out

    # ---- stop lines

    def _on_route(self, location, yaw_deg=None, max_offset=1.75, max_angle=45.0):
        """Route position s of a CARLA location within max_offset of the route (and heading the route's way if
        yaw_deg is given), else None."""
        x, y = _xy(location)
        s, e = self.path.project(x, y)
        if abs(e) > max_offset or s <= 0.0 or s >= self.length:
            return None
        if yaw_deg is not None:
            d = (math.degrees(frames.from_carla_yaw(yaw_deg) - self.path.pose(s).psi) + 180.0) % 360.0 - 180.0
            if abs(d) > max_angle:
                return None
        return s

    def attach_world(self, world):
        """The traffic lights and stop signs on the route (they are actors, so need the world)."""
        lights = []
        for light in world.get_actors().filter("traffic.traffic_light"):
            for wp in light.get_stop_waypoints():
                s = self._on_route(wp.transform.location, wp.transform.rotation.yaw)
                if s is not None:
                    lights.append((light, s))
        self.light_stops = sorted(lights, key=lambda ls: ls[1])
        signs = []
        for sign in world.get_actors().filter("traffic.stop"):
            # the route's stretch inside the sign's trigger volume: the car must come to rest in it
            tv = sign.trigger_volume
            tr = sign.get_transform()
            centre = tr.transform(tv.location)
            box = self.carla.BoundingBox(centre, self.carla.Vector3D(tv.extent.x + 0.5, tv.extent.y + 0.5, 3.0))
            box.rotation = tr.rotation
            x, y = _xy(centre)
            s_c, e_c = self.path.project(x, y)
            if abs(e_c) > 15.0:
                continue
            inside = [s for s in np.arange(max(0.0, s_c - 20.0), min(self.length, s_c + 20.0), 0.5)
                      if box.contains(self.carla.Location(*frames.to_carla_xy(*self._xy_at(s)), centre.z),
                                      self.carla.Transform())]
            if inside:
                signs.append((sign, min(inside), max(inside)))
        self.stop_signs = sorted(signs, key=lambda s: s[1])

    def _xy_at(self, s):
        p = self.path.pose(s)
        return p.x, p.y

    def stops(self, expertctl, s_ego, look_ahead=80.0):
        """The stop lines ahead of s_ego: red and amber lights (their current state only), stop signs, the route's end."""
        carla = self.carla
        out = []
        for light, s in self.light_stops:
            if s < s_ego - 5.0 or s > s_ego + look_ahead:
                continue
            state = light.get_state()
            if state == carla.TrafficLightState.Red:
                out.append(expertctl.StopLine(light.id, expertctl.StopLine.Kind.red_light, s))
            elif state == carla.TrafficLightState.Yellow:
                out.append(expertctl.StopLine(light.id, expertctl.StopLine.Kind.amber_light, s))
        for sign, s_in, s_out in self.stop_signs:
            if s_ego - 5.0 < s_out < s_ego + look_ahead:
                # the front may go to the far end of the trigger volume, less a margin; the hold rule needs the car
                # at rest within 4 m of the line
                out.append(expertctl.StopLine(100000 + sign.id, expertctl.StopLine.Kind.stop_sign, max(s_in, s_out - 1.0)))
        if self.length - s_ego < look_ahead + 20.0:
            out.append(expertctl.StopLine(-1, expertctl.StopLine.Kind.route_end, self.length - 1.0))
        return out
