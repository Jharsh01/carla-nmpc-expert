"""The road along the route the NMPC plans on, which is SimLingo's predicted path, rebuilt every plan: the path
continued along the map's lane past SimLingo's last point, the lanes along it and the stop lines on it, all in the
car's own frame, as the NMPC plans in it. The map and the lights are known in the world, so they are read there
through the car's true pose (frames.Frame), as a sensor on the car (or an HD map it is localised in) would see them.

SimLingo's path reaches about 20 m. The behaviour layer looks for actors and stops up to 160 m ahead, and the
NMPC's horizon reaches 70 m at speed, so past SimLingo's last point the path goes on along the lane that point is
in (the map, a stand-in for an HD map or online mapping, as for the rest of the world model), joining its centre
over 10 m, taking at each junction the branch nearest the way it is heading. Where the map gives out, it goes on
straight. Only SimLingo's part is asked to be followed (the Scene's line); the rest is where the constraints are
measured along.

The library's handedness throughout (y left, heading counter-clockwise; expert/frames.py). The traffic-light
states are read at each plan, as a light detector would see them.
"""
import math

import numpy as np

from expert import frames

SPACING = 2.0  # [m] between the lane continuation's points, and between lane samples
CONTINUE = 160.0  # [m] of lane continuation past SimLingo's last point
JOIN = 10.0  # [m] over which the continuation joins its lane's centre
LOOK = 200.0  # [m] stop lines and stop signs further from the car than this are not looked at


def _xy(location):
    return frames.from_carla_xy(location.x, location.y)


def _angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _lane_key(wp):
    return wp.road_id, wp.section_id, wp.lane_id


class _Across:
    """The path's normal at a point, in the world (x, y, heading psi): where map lanes cross it."""

    def __init__(self, x, y, psi):
        self.x, self.y, self.psi = x, y, psi
        self.n = (-math.sin(psi), math.cos(psi))  # to the left

    def against(self, wp):
        """The lane runs against the path."""
        return math.cos(_angle(frames.from_carla_yaw(wp.transform.rotation.yaw) - self.psi)) < 0.0

    def neighbour(self, wp, left):
        """The lane beside wp on the path's left (or right); CARLA's left and right are the lane's own."""
        return wp.get_right_lane() if left == self.against(wp) else wp.get_left_lane()

    def cut(self, wp, min_cos=0.5):
        """(lo, hi): where the normal crosses the lane's right and left edges (the path's right and left), or None if
        the lane runs more than 60 deg across the path (then it is not the path's lane: a crossing road)."""
        yaw = frames.from_carla_yaw(wp.transform.rotation.yaw)
        nl = (-math.sin(yaw), math.cos(yaw))  # the lane's normal
        cos = self.n[0] * nl[0] + self.n[1] * nl[1]
        if abs(cos) < min_cos:
            return None
        cx, cy = _xy(wp.transform.location)
        centre = ((cx - self.x) * nl[0] + (cy - self.y) * nl[1]) / cos
        half = 0.5 * wp.lane_width / abs(cos)
        return centre - half, centre + half


class Route:
    """The road along each new path. Built once per route (it indexes the lights and stop signs); `path()` makes
    the path for a plan, `lanes()` and `stops()` read the road along it, each given the car's frame then."""

    def __init__(self, expertctl, carla, world, carla_map, route_end_xy=None):
        """`route_end_xy`: where the route ends, in the library's world frame (the last point of the leaderboard's
        sparse plan, which every agent is given), or None."""
        self.ex, self.carla, self.map = expertctl, carla, carla_map
        self.route_end = route_end_xy
        self.lights = []  # (light, x, y, psi) of each stop waypoint
        for light in world.get_actors().filter("traffic.traffic_light"):
            for wp in light.get_stop_waypoints():
                x, y = _xy(wp.transform.location)
                self.lights.append((light, x, y, frames.from_carla_yaw(wp.transform.rotation.yaw)))
        self.signs = []  # (sign, x, y, its trigger volume as a box, the box centre's z)
        for sign in world.get_actors().filter("traffic.stop"):
            tv, tr = sign.trigger_volume, sign.get_transform()
            centre = tr.transform(tv.location)
            box = carla.BoundingBox(centre, carla.Vector3D(tv.extent.x + 0.5, tv.extent.y + 0.5, 3.0))
            box.rotation = tr.rotation
            self.signs.append((sign, *_xy(centre), box, centre.z))

    # ---- the path

    def _waypoint(self, x, y):
        return self.map.get_waypoint(self.carla.Location(*frames.to_carla_xy(x, y), 0.0), project_to_road=True,
                                     lane_type=self.carla.LaneType.Driving)

    def continuation(self, frame, x, y, psi):
        """Points past the car-frame point (x, y), heading psi: along the lane there, its offset from the lane centre
        fading over JOIN, for CONTINUE metres; straight on where the map gives out. In the car's frame."""
        x, y = frame.to_world(x, y)
        psi += frame.psi
        out = []
        wp = self._waypoint(x, y)
        if wp is not None:
            lane_psi = frames.from_carla_yaw(wp.transform.rotation.yaw)
            forward = math.cos(_angle(lane_psi - psi)) >= 0.0  # the lane runs the way the car goes (not oncoming)
            cx, cy = _xy(wp.transform.location)
            off = np.array([x - cx, y - cy])
            cur, heading, d = wp, psi, 0.0
            while d < CONTINUE:
                nxt = cur.next(SPACING) if forward else cur.previous(SPACING)
                if not nxt:
                    break

                def turn(w):
                    yaw = frames.from_carla_yaw(w.transform.rotation.yaw) + (0.0 if forward else math.pi)
                    return abs(_angle(yaw - heading))
                cur = min(nxt, key=turn)
                heading = frames.from_carla_yaw(cur.transform.rotation.yaw) + (0.0 if forward else math.pi)
                d += SPACING
                fade = 0.5 * (1.0 + math.cos(math.pi * d / JOIN)) if d < JOIN else 0.0
                px, py = _xy(cur.transform.location)
                out.append((px + fade * off[0], py + fade * off[1]))
        last = np.array(out[-1] if out else (x, y))
        heading = math.atan2(out[-1][1] - out[-2][1], out[-1][0] - out[-2][0]) if len(out) > 1 else psi
        done = SPACING * len(out)
        while done < CONTINUE:  # where the map gives out, straight on
            done += 5.0
            last = last + 5.0 * np.array([math.cos(heading), math.sin(heading)])
            out.append(tuple(last))
        return [frame.to_car(*p) for p in out]

    def path(self, points, frame):
        """The path through `points` (SimLingo's, in the car's frame: simlingo_ref.path_points), continued along the
        lane; and the arc length at which SimLingo's part ends."""
        pts = [tuple(p) for p in np.asarray(points, dtype=float)]
        psi = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
        path = self.ex.Path.from_waypoints([list(p) for p in pts + self.continuation(frame, *pts[-1], psi)], False)
        s_end = path.project(*pts[-1])[0]
        return path, s_end

    # ---- the road along it

    def lanes(self, path, frame, window=6):
        """The lane under the path, its left neighbour and the road's edges, every SPACING metres, merged where nothing
        changes; all as offsets across the path (y left). Each edge is taken where the path's normal there crosses it,
        so the lane stays where it is when the path runs at an angle to it (SimLingo's path drifting off the lane, the
        NMPC swerving). Only where the path changes lane (the lane at one point a neighbour of the lane at another,
        within +-window points) does the band take in both lanes. The road's edges are the outer edges of the
        outermost driving lanes either side, oncoming ones included: beyond them a shoulder, parking, a kerb, a
        sidewalk, which the corridor never goes into. In a junction, and where the nearest lane crosses the path, the
        band is a lane's width on the path, with no road edges."""
        rows = []
        for s in np.arange(0.0, path.length, SPACING):
            q = path.pose(s)
            across = _Across(*frame.to_world(q.x, q.y), q.psi + frame.psi)  # in the world, where the map is
            wp = self._waypoint(across.x, across.y)
            band = None if wp is None else across.cut(wp)
            if band is None or wp.is_junction:
                half = 0.5 * (wp.lane_width if wp is not None else 3.5)
                rows.append({"lo": -half, "hi": half, "left": 0.0, "oncoming": True, "allowed": False,
                             "road": (-math.inf, math.inf), "lane": None, "neighbours": ()})
                continue
            left, right = across.neighbour(wp, left=True), across.neighbour(wp, left=False)
            left_ok = self._drivable(left)
            road = [self._road_edge(across, wp, left=False), self._road_edge(across, wp, left=True)]
            rows.append({"lo": band[0], "hi": band[1], "left": left.lane_width if left_ok else 0.0,
                         "oncoming": bool(left_ok and across.against(left)),
                         "allowed": bool(left_ok), "road": tuple(road), "lane": _lane_key(wp),
                         "neighbours": tuple(_lane_key(n) for n in (left, right) if self._drivable(n))})
        sections = []
        for i, row in enumerate(rows):
            lo, hi = row["lo"], row["hi"]
            for other in rows[max(0, i - window): i + window + 1]:  # a lane change: both lanes
                if other["lane"] is not None and other["lane"] in row["neighbours"]:
                    lo, hi = min(lo, other["lo"]), max(hi, other["hi"])
            road_lo, road_hi = min(row["road"][0], lo), max(row["road"][1], hi)
            key = (round(lo, 1), round(hi, 1), round(row["left"], 1), row["oncoming"], row["allowed"],
                   round(road_lo, 1), round(road_hi, 1))
            if sections and sections[-1][1] == key:
                sections[-1][0][1] = (i + 1) * SPACING
            else:
                sections.append(([i * SPACING, (i + 1) * SPACING], key))
        out = [self.ex.LaneSection(s_from=a, s_to=b, lo=lo, hi=hi, left_width=lw, left_oncoming=onc, left_allowed=ok,
                                   road_lo=rlo, road_hi=rhi)
               for (a, b), (lo, hi, lw, onc, ok, rlo, rhi) in sections]
        out[-1].s_to = float("inf")
        return out

    def _drivable(self, wp):
        return wp is not None and wp.lane_type in (self.carla.LaneType.Driving, self.carla.LaneType.Bidirectional)

    def _road_edge(self, across, wp, left):
        """The offset across the path of the road's edge on one side: the outer edge of the outermost driving lane
        reached from wp through neighbouring driving lanes (oncoming ones too)."""
        edge = across.cut(wp)[1 if left else 0]
        seen = {_lane_key(wp)}
        cur = wp
        for _ in range(8):
            cur = across.neighbour(cur, left)
            if not self._drivable(cur) or _lane_key(cur) in seen:
                break
            seen.add(_lane_key(cur))
            band = across.cut(cur)
            if band is None:
                break
            edge = max(edge, band[1]) if left else min(edge, band[0])
        return edge

    def _on_path(self, path, x, y, psi=None, max_offset=1.75, max_angle=45.0):
        """Arc length of (x, y) on the path if within max_offset of it (and heading its way if psi is given)."""
        s, e = path.project(x, y)
        if abs(e) > max_offset or s <= 0.0 or s >= path.length:
            return None
        if psi is not None and abs(math.degrees(_angle(psi - path.pose(s).psi))) > max_angle:
            return None
        return s

    def stops(self, path, s_ego, frame, look_ahead=80.0):
        """The stop lines ahead of s_ego on the path: red and amber lights (their state now), stop signs, the route's
        end."""
        ex, carla = self.ex, self.carla
        out = []
        for light, x, y, psi in self.lights:
            if math.hypot(x - frame.x, y - frame.y) > LOOK:
                continue
            s = self._on_path(path, *frame.to_car(x, y), psi - frame.psi)
            if s is None or s < s_ego - 5.0 or s > s_ego + look_ahead:
                continue
            state = light.get_state()
            if state == carla.TrafficLightState.Red:
                out.append(ex.StopLine(light.id, ex.StopLine.Kind.red_light, s))
            elif state == carla.TrafficLightState.Yellow:
                out.append(ex.StopLine(light.id, ex.StopLine.Kind.amber_light, s))
        for sign, x, y, box, z in self.signs:
            if math.hypot(x - frame.x, y - frame.y) > LOOK:
                continue
            s_c, e_c = path.project(*frame.to_car(x, y))
            if abs(e_c) > 15.0:
                continue
            # the path's stretch inside the sign's trigger volume: the car must come to rest in it

            def in_box(s):
                q = path.pose(s)
                return box.contains(carla.Location(*frames.to_carla_xy(*frame.to_world(q.x, q.y)), z), carla.Transform())
            inside = [s for s in np.arange(max(0.0, s_c - 20.0), min(path.length, s_c + 20.0), 0.5) if in_box(s)]
            if inside and s_ego - 5.0 < max(inside) < s_ego + look_ahead:
                # the front may go to the far end of the trigger volume, less a margin; the hold rule needs the car
                # at rest within 4 m of the line
                out.append(ex.StopLine(100000 + sign.id, ex.StopLine.Kind.stop_sign, max(min(inside), max(inside) - 1.0)))
        if self.route_end is not None:
            s = self._on_path(path, *frame.to_car(*self.route_end), max_offset=3.5)
            if s is not None and s_ego < s < s_ego + look_ahead + 20.0:
                out.append(ex.StopLine(-1, ex.StopLine.Kind.route_end, s - 1.0))
        return out
