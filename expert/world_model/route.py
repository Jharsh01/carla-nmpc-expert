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


class _Pose:
    def __init__(self, x, y, psi):
        self.x, self.y, self.psi = x, y, psi


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
        """The lane under the path and its left neighbour, every SPACING metres, merged where nothing changes. The
        band at each point is widened to the union over +-window points, so a lane change has both lanes in it."""
        carla = self.carla
        rows = []
        for s in np.arange(0.0, path.length, SPACING):
            q = path.pose(s)
            px, py = frame.to_world(q.x, q.y)
            pose = _Pose(px, py, q.psi + frame.psi)  # in the world, where the map is
            wp = self._waypoint(pose.x, pose.y)
            if wp is None:
                rows.append((s, -1.75, 1.75, 0.0, True, False))
                continue
            cx, cy = _xy(wp.transform.location)
            e_c = -(cx - pose.x) * math.sin(pose.psi) + (cy - pose.y) * math.cos(pose.psi)  # lane centre off the path
            half = 0.5 * wp.lane_width
            # on a lane the path runs against (SimLingo overtaking in the oncoming lane), the car's left is the lane's
            # right, and a neighbour running against that lane runs the car's way
            against = math.cos(_angle(frames.from_carla_yaw(wp.transform.rotation.yaw) - pose.psi)) < 0.0
            left = wp.get_right_lane() if against else wp.get_left_lane()
            left_ok = left is not None and left.lane_type == carla.LaneType.Driving and not wp.is_junction
            oncoming = left_ok and (left.lane_id * wp.lane_id < 0) != against
            rows.append((s, e_c - half, e_c + half, left.lane_width if left_ok else 0.0, bool(oncoming), bool(left_ok)))
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
        out = [self.ex.LaneSection(s_from=a, s_to=b, lo=lo, hi=hi, left_width=lw, left_oncoming=onc, left_allowed=ok)
               for (a, b), (lo, hi, lw, onc, ok) in sections]
        out[-1].s_to = float("inf")
        return out

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
