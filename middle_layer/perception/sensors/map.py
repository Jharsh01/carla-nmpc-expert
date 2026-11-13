"""The road along the route the NMPC plans on, rebuilt every plan: the lane the car is in, or, while SimLingo's path
is a manoeuvre (expertctl.PathIntent), that path continued along the map's lane past SimLingo's last point; the
lanes along it and the stop lines on it, all in the car's own frame, as the NMPC plans in it. The map and the lights are known in the world, so they are read there
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
BEHIND = 15.0  # [m] the lane's path reaches behind the car, so that actors and stops just behind it project onto it
LOOK = 200.0  # [m] stop lines and stop signs further from the car than this are not looked at
BODY = (2.6, 1.2)  # [m] half the car's length and width, with a little over: the corners that say which lanes it is in


def _xy(location):
    return frames.from_carla_xy(location.x, location.y)


def _angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def lane_key(wp):
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

    def __init__(self, expertctl, carla, world, carla_map, route_end_xy=None, route_xy=None):
        """`route_end_xy`: where the route ends, in the library's world frame (the last point of the leaderboard's
        sparse plan, which every agent is given), or None. `route_xy`: all that plan's points, in the same frame, to
        take the route's branch at a junction (`continuation`), or None (the straightest branch)."""
        self.ex, self.carla, self.map = expertctl, carla, carla_map
        self.route_end = route_end_xy
        self.route_xy = None if route_xy is None or len(route_xy) == 0 else np.asarray(route_xy, dtype=float).reshape(-1, 2)
        self._ego_lanes, self._body_lanes = [], set()  # from the last lanes()
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

    def _waypoint(self, x, y, z):
        """The driving lane at (x, y). `z` matters: from far above or below a road the nearest lane is often not the
        one under the point (Town12's roads are 300-380 m up), so it is the car's height, close enough along a path."""
        return self.map.get_waypoint(self.carla.Location(*frames.to_carla_xy(x, y), z), project_to_road=True,
                                     lane_type=self.carla.LaneType.Driving)

    def continuation(self, frame, x, y, psi):
        """Points past the car-frame point (x, y), heading psi: along the lane there, its offset from the lane centre
        fading over JOIN, for CONTINUE metres; straight on where the map gives out. In the car's frame."""
        x, y = frame.to_world(x, y)
        psi += frame.psi
        out = []
        wp = self._waypoint(x, y, frame.z)
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
                cur = self._route_branch(nxt) if forward and len(nxt) > 1 else None
                cur = cur if cur is not None else min(nxt, key=turn)
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

    def _route_branch(self, branches, near=5.0):
        """Of the lanes a junction forks into, the one the route takes: whose end (followed through the junction)
        lies nearest one of the route's points, if within `near`; else None. Straight on, the lane path ran through
        route 8's junction, where the route turns right, until SimLingo's own path turned some 19 m before it: the
        cars coming down the lane the route turns into were seen as merging only 3.5 s before they came in."""
        if self.route_xy is None:
            return None
        best, best_d = None, near
        for b in branches:
            w = b
            for _ in range(40):
                if not w.is_junction:
                    break
                nxt = [n for n in w.next(2.0) if n.road_id == w.road_id and n.lane_id == w.lane_id] or w.next(2.0)
                if not nxt:
                    break
                w = nxt[0]
            d = float(np.min(np.hypot(*(self.route_xy - np.array(_xy(w.transform.location))).T)))
            if d < best_d:
                best, best_d = b, d
        return best

    def path(self, points, frame):
        """The path through `points` (SimLingo's, in the car's frame: simlingo_ref.path_points), continued along the
        lane; and the arc length at which SimLingo's part ends."""
        pts = [tuple(p) for p in np.asarray(points, dtype=float)]
        psi = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
        path = self.ex.Path.from_waypoints([list(p) for p in pts + self.continuation(frame, *pts[-1], psi)], False)
        s_end = path.project(*pts[-1])[0]
        return path, s_end

    def lane_path(self, frame):
        """The centre of the lane the car is in, from BEHIND metres behind it, along the lane past it. It does not go
        through the car: the car's offset from it stays the NMPC's to take out. None where the map has no lane."""
        wp = self._waypoint(frame.x, frame.y, frame.z)
        if wp is None:
            return None
        psi = frames.from_carla_yaw(wp.transform.rotation.yaw)
        if math.cos(_angle(psi - frame.psi)) < 0.0:
            # In the oncoming lane: the lane beside it that goes the car's way, the one to come back to. Kept to the
            # oncoming lane, as SimLingo's path stopped leaving it, the car lost the blocker it was going round and
            # stood in the oncoming flow beside it (Bench2Drive route 4).
            back = wp.get_left_lane()  # CARLA's left, of the oncoming lane: across the centre line
            if self._drivable(back) and math.cos(_angle(frames.from_carla_yaw(back.transform.rotation.yaw) - frame.psi)) > 0.0:
                wp = back
                psi = frames.from_carla_yaw(wp.transform.rotation.yaw)
            else:  # no lane our way: along the oncoming one, the way the car goes
                psi += math.pi
        x, y = _xy(wp.transform.location)
        ahead = self.continuation(frame, *frame.to_car(x, y), psi - frame.psi)
        c, s = math.cos(psi), math.sin(psi)
        behind = [frame.to_car(x - d * c, y - d * s) for d in np.arange(BEHIND, -0.1, -2.5)]
        return self.ex.Path.from_waypoints([list(p) for p in behind + ahead], False)

    def turn_path(self, points, frame, tolerance=2.0, look=40.0):
        """The map's own line for a turn SimLingo's path takes: the lane the car is in up to the next junction, the
        junction lane (from the car's lane) that SimLingo's `points` (car frame) follow most closely, and the lane it
        leaves by; from BEHIND metres behind the car. None where there is no junction within `look`, or none of its
        lanes lies within `tolerance` of the points on average. SimLingo's own line through a turn is a little
        different each time and tighter than the lane (Bench2Drive route 8: 8-10 m radius where the lane's is 12.5 m),
        and it ends some 18 m ahead; the lane's is the same from plan to plan and is the junction's geometry."""
        carla = self.carla
        wp = self._waypoint(frame.x, frame.y, frame.z)
        if wp is None or math.cos(_angle(frames.from_carla_yaw(wp.transform.rotation.yaw) - frame.psi)) < 0.0:
            return None
        before, cur, d = [wp], wp, 0.0  # the car's lane up to the junction
        while not cur.is_junction and d < look:
            nxt = cur.next(2.0)
            if not nxt:
                return None
            cur = nxt[0] if not nxt[0].is_junction or len(nxt) == 1 else nxt[0]
            before.append(cur)
            d += 2.0
        if not cur.is_junction:
            return None
        junction = cur.get_junction()
        inside = wp.is_junction
        entry = None if inside else before[-2]
        here = np.array(_xy(wp.transform.location))
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) > 3.0]
        pts = pts[pts[:, 0] > 0.0]
        if len(pts) < 3:
            return None
        best, best_score = None, tolerance
        for start, end in junction.get_waypoints(carla.LaneType.Driving):
            line, w = [start], start
            for _ in range(80):
                nxt = [n for n in w.next(1.0) if n.road_id == w.road_id and n.lane_id == w.lane_id]
                if not nxt:
                    break
                w = nxt[0]
                line.append(w)
            if inside:  # one of the lanes the car is on
                if min(np.hypot(*(np.array(_xy(l.transform.location)) - here)) for l in line) > 1.5:
                    continue
            else:
                prev = start.previous(1.0)
                if not prev or lane_key(prev[0]) != lane_key(entry):
                    continue
            after, w, heading = [], end, end.transform.rotation.yaw
            for _ in range(int(CONTINUE / 2.0)):  # the lane it leaves by, straightest on
                nxt = w.next(2.0)
                if not nxt:
                    break
                w = min(nxt, key=lambda n: abs((n.transform.rotation.yaw - heading + 180.0) % 360.0 - 180.0))
                heading = w.transform.rotation.yaw
                after.append(w)
            world = [_xy(l.transform.location) for l in line + after]
            car = np.array([frame.to_car(*q) for q in world])
            score = float(np.mean([np.min(np.hypot(car[:, 0] - px, car[:, 1] - py)) for px, py in pts]))
            if score < best_score:
                best, best_score = (line, after), score
        if best is None:
            return None
        line, after = best
        lead = [] if inside else before[:-1]
        back, w, d = [], (lead[0] if lead else line[0]), 0.0
        while d < BEHIND + 30.0:  # behind the car, along the lane it came by
            prev = w.previous(2.5)
            if not prev:
                break
            w = prev[0]
            back.insert(0, w)
            d += 2.5
        world = [_xy(l.transform.location) for l in back + lead + line + after]
        car = [frame.to_car(*q) for q in world]
        # from BEHIND metres behind the car, no point closer than 0.5 m to the last
        kept = []
        for q in car:  # 2 m apart: closer, the lane's own small steps show as bends
            if kept and math.hypot(q[0] - kept[-1][0], q[1] - kept[-1][1]) < 1.9:
                continue
            kept.append(q)
        nearest = min(range(len(kept)), key=lambda i: math.hypot(*kept[i]))
        first = nearest
        behind = 0.0
        while first > 0 and behind < BEHIND + 2.0:
            behind += math.hypot(kept[first][0] - kept[first - 1][0], kept[first][1] - kept[first - 1][1])
            first -= 1
        return self.ex.Path.from_waypoints([list(q) for q in kept[first:]], False)

    # ---- the road along it

    def lanes(self, path, frame, window=6):
        """The lane under the path, its neighbours either side (whether there is one, and which way it runs) and the road's edges, every SPACING metres, merged where nothing
        changes; all as offsets across the path (y left). Each edge is taken where the path's normal there crosses it,
        so the lane stays where it is when the path runs at an angle to it (SimLingo's path drifting off the lane, the
        NMPC swerving). Only where the path changes lane (the lane at one point a neighbour of the lane at another,
        within +-window points) does the band take in both lanes. The road's edges are the outer edges of the
        outermost driving lanes either side, oncoming ones included: beyond them a shoulder, parking, a kerb, a
        sidewalk, which the corridor never goes into. In a junction, and where the nearest lane crosses the path, the
        band is a lane's width on the path, with no road edges (the NMPC's corridor takes a little more there:
        ConstraintOptions::junction_slack)."""
        rows = []
        for s in np.arange(0.0, path.length, SPACING):
            q = path.pose(s)
            across = _Across(*frame.to_world(q.x, q.y), q.psi + frame.psi)  # in the world, where the map is
            wp = self._waypoint(across.x, across.y, frame.z)
            band = None if wp is None else across.cut(wp)
            if band is None or wp.is_junction:
                half = 0.5 * (wp.lane_width if wp is not None else 3.5)
                rows.append({"lo": -half, "hi": half, "against": False, "cuts": [], "left": 0.0, "oncoming": True, "allowed": False,
                             "right": 0.0, "right_oncoming": False, "right_allowed": False,
                             "road": (-math.inf, math.inf), "lane": None, "neighbours": ()})
                continue
            left, right = across.neighbour(wp, left=True), across.neighbour(wp, left=False)
            left_ok, right_ok = self._drivable(left), self._drivable(right)
            road = [self._road_edge(across, wp, left=False), self._road_edge(across, wp, left=True)]
            rows.append({"lo": band[0], "hi": band[1], "against": across.against(wp), "cuts": self._cuts(across, wp),
                         "left": left.lane_width if left_ok else 0.0,
                         "oncoming": bool(left_ok and across.against(left)),
                         "allowed": bool(left_ok), "right": right.lane_width if right_ok else 0.0,
                         "right_oncoming": bool(right_ok and across.against(right)), "right_allowed": bool(right_ok),
                         "road": tuple(road), "lane": lane_key(wp),
                         "neighbours": tuple(lane_key(n) for n in (left, right) if self._drivable(n))})
        sections = []
        self._ego_lanes = []
        corners = [self._waypoint(*frame.to_world(a * BODY[0], b * BODY[1]), frame.z) for a in (-1, 1) for b in (-1, 1)]
        self._body_lanes = {lane_key(wp) for wp in corners if wp is not None and not wp.is_junction}
        for i, row in enumerate(rows):
            lo, hi = row["lo"], row["hi"]
            # what lies either side is the outermost lane's neighbour on that side: where the band takes in two
            # lanes, beyond the left one is not what is beside the path's lane (route 1: the lane our way, inside
            # the band, was taken for the one beyond it, which is oncoming)
            left_of, right_of = row, row
            used = None if row["lane"] is None else {row["lane"]}
            for other in rows[max(0, i - window): i + window + 1]:  # a lane change: both lanes
                if other["lane"] is not None and other["lane"] in row["neighbours"]:
                    if other["hi"] > hi:
                        left_of = other
                    if other["lo"] < lo:
                        right_of = other
                    lo, hi = min(lo, other["lo"]), max(hi, other["hi"])
                    used.add(other["lane"])
            self._ego_lanes.append(used)
            # Which part of the band runs the car's way (a lane change into the oncoming lane takes that lane in too):
            # the lanes it takes in, cut here, across the path here. Not the other rows' lo and hi: each is across
            # the path where that row is, and partway into the oncoming lane our lane's [lo, hi] from a row before
            # the change lands on the oncoming lane here (route 4, planner_r14).
            way = [(a, b) for a, b, against, key in row["cuts"] if not against and key in (used or ())]
            if row["lane"] is None:
                way = [(lo, hi)]
            road_lo, road_hi = min(row["road"][0], lo), max(row["road"][1], hi)
            way_lo, way_hi = (min(a for a, _ in way), max(b for _, b in way)) if way else (math.inf, -math.inf)
            key = (round(lo, 1), round(hi, 1), round(left_of["left"], 1), left_of["oncoming"], left_of["allowed"],
                   round(road_lo, 1), round(road_hi, 1), round(right_of["right"], 1), right_of["right_oncoming"],
                   right_of["right_allowed"], round(way_lo, 1), round(way_hi, 1))
            if sections and sections[-1][1] == key:
                sections[-1][0][1] = (i + 1) * SPACING
            else:
                sections.append(([i * SPACING, (i + 1) * SPACING], key))
        out = [self.ex.LaneSection(s_from=a, s_to=b, lo=lo, hi=hi, left_width=lw, left_oncoming=onc, left_allowed=ok,
                                   road_lo=rlo, road_hi=rhi, right_width=rw, right_oncoming=ronc, right_allowed=rok,
                                   way_lo=wlo, way_hi=whi)
               for (a, b), (lo, hi, lw, onc, ok, rlo, rhi, rw, ronc, rok, wlo, whi) in sections]
        out[-1].s_to = float("inf")
        return out

    def ego_lanes(self, path, x, y):
        """The map lanes the car is using level with the car-frame point (x, y), as of the last `lanes()`: the path's
        lane there, both lanes where the path changes lane, one row either side as a lane's ids change along it; and
        whatever lanes the car's body is in now, wherever the path goes. None where that cannot be said (a junction, or
        the path's lane crossing it)."""
        i = int(path.project(x, y)[0] / SPACING)
        rows = self._ego_lanes[max(0, i - 1): i + 2]
        if not rows or any(row is None for row in rows):
            return None
        return set().union(*rows) | self._body_lanes

    def _drivable(self, wp):
        return wp is not None and wp.lane_type in (self.carla.LaneType.Driving, self.carla.LaneType.Bidirectional)

    def _cuts(self, across, wp):
        """(lo, hi, runs against the path, lane key) of wp's lane and the driving lanes beside it, across the path."""
        out = [(*across.cut(wp), across.against(wp), lane_key(wp))]
        for left in (True, False):
            cur, seen = wp, {lane_key(wp)}
            for _ in range(3):
                cur = across.neighbour(cur, left)
                if not self._drivable(cur) or lane_key(cur) in seen:
                    break
                seen.add(lane_key(cur))
                band = across.cut(cur)
                if band is None:
                    break
                out.append((*band, across.against(cur), lane_key(cur)))
        return out

    def _road_edge(self, across, wp, left):
        """The offset across the path of the road's edge on one side: the outer edge of the outermost driving lane
        reached from wp through neighbouring driving lanes (oncoming ones too)."""
        edge = across.cut(wp)[1 if left else 0]
        seen = {lane_key(wp)}
        cur = wp
        for _ in range(8):
            cur = across.neighbour(cur, left)
            if not self._drivable(cur) or lane_key(cur) in seen:
                break
            seen.add(lane_key(cur))
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

    # ---- the junction ahead

    def conflicts(self, path, s_ego, frame, world=None, eye=None, look_ahead=60.0, reach=80.0):
        """The lanes of the next junction on the path that cross it or merge with it: for each, a dict with `kind`
        ("merge": it ends in the lane the path leaves the junction by; "cross": it does not), where along the path the
        two overlap (`s_lo`, `s_hi`), `lead_in` (how far along that lane its traffic comes from the junction's edge to
        `s_lo`) and `sight` (how far up the road before the junction that lane is seen from `eye`, by ray casts in
        `world`, up to `reach`; `reach` without them). A lane from the path's own entry is none (it is the path's, or
        parts from it). Nothing where the path meets no junction within `look_ahead`."""
        carla = self.carla
        own = []  # the path's waypoints from the car on: (s, wp)
        for s in np.arange(max(0.0, s_ego), min(path.length, s_ego + look_ahead), 2.0):
            q = path.pose(float(s))
            wp = self._waypoint(*frame.to_world(q.x, q.y), frame.z)
            if wp is not None:
                own.append((float(s), wp))
        inside = [i for i, (_, wp) in enumerate(own) if wp.is_junction]
        if not inside:
            return []
        first = inside[0]
        junction = own[first][1].get_junction()
        last = first
        while last + 1 < len(own) and own[last + 1][1].is_junction and own[last + 1][1].get_junction().id == junction.id:
            last += 1
        entry = own[first - 1][1] if first > 0 else None  # the lane the path comes in by, and leaves by
        leave = own[last + 1][1] if last + 1 < len(own) else None
        out = []
        for start, end in junction.get_waypoints(carla.LaneType.Driving):
            before = start.previous(1.0)
            if entry is not None and before and lane_key(before[0]) == lane_key(entry):
                continue
            # the lane through the junction, every metre, in path coordinates
            cur, points = start, []
            for _ in range(80):
                x, y = frames.from_carla_xy(cur.transform.location.x, cur.transform.location.y)
                points.append((path.project(*frame.to_car(x, y)), cur.lane_width))
                nxt = [w for w in cur.next(1.0) if w.road_id == cur.road_id and w.lane_id == cur.lane_id]
                if not nxt:
                    break
                cur = nxt[0]
            hit = [i for i, ((s, e), width) in enumerate(points)
                   if s_ego < s < s_ego + look_ahead and abs(e) < 0.5 * width + 1.0]
            if not hit:
                continue
            after = end.next(1.0)
            merges = leave is not None and bool(after) and lane_key(after[0]) == lane_key(leave)
            sight = reach
            if world is not None and eye is not None:  # up the road it comes in by, 1 m above it
                cur, sight = start, 0.0
                while sight < reach:
                    prev = cur.previous(2.0)
                    if not prev:
                        break
                    cur = prev[0]
                    loc = cur.transform.location
                    target = carla.Location(loc.x, loc.y, loc.z + 1.0)
                    hits = world.cast_ray(eye, target)
                    if hits and hits[0].location.distance(eye) < target.distance(eye) - 1.5:
                        break
                    sight += 2.0
            out.append({"kind": "merge" if merges else "cross", "s_lo": min(points[i][0][0] for i in hit),
                        "s_hi": max(points[i][0][0] for i in hit), "lead_in": float(hit[0]), "sight": float(sight)})
        return out

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
