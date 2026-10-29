"""Other road users as a perception stack would report them: detected only when visible, tracked over time, and
predicted from what was seen.

What is read from CARLA, and what a perception stack would have in its place:

    actor boxes (pose, size, class)       3D detection: only for actors within range and in line of sight from the
                                          lidar's position (ray casts to the box's centre and corners)
    velocities, accelerations             not read: estimated by the tracker from the boxes over the last half second
    vehicle lights (hazards)              a camera's view of the indicators
    the map                               an HD map (or online mapping), for lane-following prediction

Not read at all: other actors' controls, their routes or Traffic Manager targets, scenario triggers, actors out of
sight (an actor seen recently is kept, coasting on its last estimate, for `keep` seconds, as a tracker would).
"""
import math

import numpy as np

from expert import frames

HORIZON, STEP = 5.0, 0.25  # [s] of each predicted trajectory
# Bench2Drive spawns the parked cars along the route as static.prop.mesh actors with car meshes
# (leaderboard/scenarios/route_scenario.py, spawn_parked_vehicles), whose bounding boxes CARLA gets wrong (a 5 m
# Charger reports 3.0 x 3.9 m; on Town12 they came out 2.0 x 4.3 m, a car turned sideways): taken as parked cars of
# this size, along the actor's heading, as a detector would report them
PARKED_CAR = (4.8, 2.0)  # length, width [m]


class Track:
    def __init__(self, actor_id, kind):
        self.id = actor_id
        self.kind = kind  # "vehicle", "bike", "walker", "object"
        self.history = []  # (t, x, y, psi) in the library's frame
        self.last_seen = -math.inf
        self.still_since = None  # when it last came to rest
        self.length = self.width = 0.0
        self.mesh_car = False  # a parked car spawned as a static mesh (PARKED_CAR)

    def add(self, t, x, y, psi):
        self.history.append((t, x, y, psi))
        self.history = [h for h in self.history if h[0] >= t - 1.0]
        v = self.speed()
        if v < 0.3:
            self.still_since = t if self.still_since is None else self.still_since
        else:
            self.still_since = None

    def velocity(self):
        """(vx, vy) from a straight-line fit to the last half second of positions; zero until there are three."""
        h = [p for p in self.history if p[0] >= self.history[-1][0] - 0.5]
        if len(h) < 3:
            return 0.0, 0.0
        t = np.array([p[0] for p in h])
        t = t - t.mean()
        if np.allclose(t, 0.0):
            return 0.0, 0.0
        vx = float(np.dot(t, [p[1] for p in h]) / np.dot(t, t))
        vy = float(np.dot(t, [p[2] for p in h]) / np.dot(t, t))
        return vx, vy

    def speed(self):
        return math.hypot(*self.velocity())

    def accel(self):
        """Along its heading, from the speeds of the first and second halves of the last second; 0 if too short."""
        h = self.history
        if len(h) < 8:
            return 0.0
        mid = len(h) // 2

        def fit_speed(part):
            t = np.array([p[0] for p in part])
            t = t - t.mean()
            if np.dot(t, t) == 0.0:
                return 0.0
            return math.hypot(np.dot(t, [p[1] for p in part]) / np.dot(t, t), np.dot(t, [p[2] for p in part]) / np.dot(t, t))
        dt = 0.5 * (h[-1][0] - h[0][0])
        return (fit_speed(h[mid:]) - fit_speed(h[:mid])) / dt if dt > 0 else 0.0


def kind_of(actor):
    tid = actor.type_id
    if tid.startswith("walker."):
        return "walker"
    if tid.startswith("vehicle."):
        wheels = int(actor.attributes.get("number_of_wheels", 4))
        return "bike" if wheels == 2 else "vehicle"
    if tid == "static.prop.mesh":  # Bench2Drive's parked cars (PARKED_CAR)
        return "vehicle"
    if tid.startswith("static.prop."):
        return "object"
    return None


class Actors:
    def __init__(self, expertctl, carla, world, carla_map, hero, look_range=60.0, walker_range=40.0, keep=1.5,
                 visibility=True):
        self.ex = expertctl
        self.carla = carla
        self.world = world
        self.map = carla_map
        self.hero = hero
        self.look_range, self.walker_range, self.keep = look_range, walker_range, keep
        self.visibility = visibility
        self.tracks = {}

    # ---- detection

    def _visible(self, actor, eye):
        """Line of sight from `eye` (the lidar's position) to the box's centre or one of its upper corners: the first
        thing each ray hits must be inside the box (grown by 0.3 m)."""
        bb = actor.bounding_box
        tr = actor.get_transform()
        grown = self.carla.BoundingBox(bb.location, self.carla.Vector3D(bb.extent.x + 0.3, bb.extent.y + 0.3, bb.extent.z + 0.3))
        targets = [tr.transform(bb.location)] + [v for v in bb.get_world_vertices(tr) if v.z > tr.location.z + 0.3 * bb.extent.z][:4]
        for target in targets:
            hits = self.world.cast_ray(eye, target)
            if not hits or grown.contains(hits[0].location, tr):
                return True
        return False

    def detect(self, t):
        """Update the tracks with what is visible now."""
        ego_tr = self.hero.get_transform()
        eye = self.carla.Location(ego_tr.location.x, ego_tr.location.y, ego_tr.location.z + 2.5)
        candidates = list(self.world.get_actors().filter("vehicle.*")) + list(self.world.get_actors().filter("walker.*")) + \
            list(self.world.get_actors().filter("static.prop.*"))
        for actor in candidates:
            if actor.id == self.hero.id:
                continue
            kind = kind_of(actor)
            if kind is None:
                continue
            d = actor.get_location().distance(ego_tr.location)
            if d > (self.walker_range if kind == "walker" else self.look_range):
                continue
            if kind == "object" and max(actor.bounding_box.extent.x, actor.bounding_box.extent.y) < 0.15:
                continue  # litter, not obstacles
            if self.visibility and not self._visible(actor, eye):
                continue
            track = self.tracks.get(actor.id) or Track(actor.id, kind)
            self.tracks[actor.id] = track
            tr = actor.get_transform()
            x, y = frames.from_carla_xy(tr.location.x, tr.location.y)
            bb = actor.bounding_box
            cx, cy = frames.from_carla_xy(*_rotate(bb.location.x, bb.location.y, tr.rotation.yaw))
            track.add(t, x + cx, y + cy, frames.from_carla_yaw(tr.rotation.yaw))
            track.length, track.width = 2.0 * bb.extent.x, 2.0 * bb.extent.y
            if actor.type_id == "static.prop.mesh":
                track.mesh_car = True
                track.length, track.width = PARKED_CAR
            track.last_seen = t
            track.actor = actor
        # tracks lapse `keep` seconds after last seen; an actor the scenario has removed is gone at once (CARLA
        # answers nothing about it any more)
        for aid in [a for a, tr in self.tracks.items() if t - tr.last_seen > self.keep or not tr.actor.is_alive]:
            del self.tracks[aid]

    # ---- prediction

    def _parked(self, track, t):
        """At rest and not waiting in traffic: hazard lights on, off the driving lanes or well off a lane's centre,
        or at rest for 4 s with nothing ahead holding it up (no vehicle within 10 m ahead in its lane, no light)."""
        if track.mesh_car:
            return True
        if track.kind != "vehicle" or track.still_since is None:
            return False
        actor = track.actor
        lights = actor.get_light_state()
        hazard = self.carla.VehicleLightState
        if (lights & hazard.LeftBlinker) and (lights & hazard.RightBlinker):
            return True
        loc = actor.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=False, lane_type=self.carla.LaneType.Any)
        if wp is None or wp.lane_type != self.carla.LaneType.Driving:
            return True
        if wp.transform.location.distance(loc) > 0.6 * wp.lane_width:
            return True
        if t - track.still_since < 4.0 or actor.is_at_traffic_light():
            return False
        x0, y0, psi0 = track.history[-1][1:]
        for other in self.tracks.values():
            if other is track or other.kind not in ("vehicle", "bike"):
                continue
            dx, dy = other.history[-1][1] - x0, other.history[-1][2] - y0
            ahead = dx * math.cos(psi0) + dy * math.sin(psi0)
            side = -dx * math.sin(psi0) + dy * math.cos(psi0)
            if 0.0 < ahead < 10.0 + 0.5 * (track.length + other.length) and abs(side) < 1.5:
                return False
        return True

    def _follow_lane(self, track, speed, accel):
        """Centre positions along its lane at each step, at its speed (braking to rest if it is braking), or None if
        it is not going along a driving lane. At a junction, the branch nearest its heading."""
        carla = self.carla
        actor = track.actor
        wp = self.map.get_waypoint(actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            return None
        yaw = actor.get_transform().rotation.yaw
        if abs((wp.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0) > 35.0:
            return None
        times = np.arange(0.0, HORIZON + 1e-9, STEP)
        if accel < -1.0:  # braking: to rest
            dist = np.minimum(speed * times + 0.5 * accel * times ** 2, speed * speed / (-2.0 * accel))
        else:
            dist = speed * times
        points, travelled, cur = [wp], [0.0], wp
        while travelled[-1] < dist[-1] + 2.0:
            nxt = cur.next(2.0)
            if not nxt:
                break
            cur = min(nxt, key=lambda w: abs((w.transform.rotation.yaw - cur.transform.rotation.yaw + 180.0) % 360.0 - 180.0))
            points.append(cur)
            travelled.append(travelled[-1] + 2.0)
        if len(points) < 2:
            return None
        xs = np.array([frames.from_carla_xy(p.transform.location.x, p.transform.location.y) for p in points])
        psis = np.unwrap([frames.from_carla_yaw(p.transform.rotation.yaw) for p in points])
        tx, ty = track.history[-1][1], track.history[-1][2]
        # start from where it is, its offset from the lane centre fading over two seconds
        off = np.array([tx, ty]) - xs[0]
        fade = np.clip(1.0 - times / 2.0, 0.0, 1.0)
        x = np.interp(dist, travelled, xs[:, 0]) + fade * off[0]
        y = np.interp(dist, travelled, xs[:, 1]) + fade * off[1]
        psi = np.interp(dist, travelled, psis)
        return times, x, y, psi

    def scene_actors(self, t, frame):
        """expertctl.Actor for every track, with its predicted trajectory, in the car's frame (`frame`: the car's
        true pose, as a detector on the car reports what it sees relative to the car). The tracks themselves are kept
        in the world, where they stand still when the car moves."""
        ex = self.ex
        kinds = {"vehicle": ex.Actor.Kind.vehicle, "bike": ex.Actor.Kind.bike, "walker": ex.Actor.Kind.walker,
                 "object": ex.Actor.Kind.object}
        out = []
        gone = []
        for track in list(self.tracks.values()):
            _, x0, y0, psi0 = track.history[-1]
            vx, vy = track.velocity()
            speed = math.hypot(vx, vy)
            a = ex.Actor()
            a.id = track.id
            a.kind = kinds[track.kind]
            a.length, a.width = max(track.length, 0.5), max(track.width, 0.5)
            try:  # both read the actor (its lights, its lane): one the scenario has just removed answers nothing
                a.parked = self._parked(track, t)
                predicted = None
                if track.kind == "vehicle" and speed >= 0.5:
                    predicted = self._follow_lane(track, speed, track.accel())
            except RuntimeError:
                gone.append(track.id)
                continue
            if predicted is None:  # constant velocity (at rest for a parked car or an object)
                times = np.arange(0.0, HORIZON + 1e-9, STEP)
                moving = track.kind != "object" and not a.parked and speed >= 0.2
                psi = math.atan2(vy, vx) if moving and track.kind == "walker" else psi0
                predicted = (times, x0 + (vx * times if moving else 0.0 * times), y0 + (vy * times if moving else 0.0 * times),
                             np.full_like(times, psi))
            times, x, y, psi = predicted
            cx, cy = frame.to_car(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
            a.t, a.x, a.y, a.psi = [list(map(float, v)) for v in (times, cx, cy, np.asarray(psi) - frame.psi)]
            out.append(a)
        for aid in gone:
            del self.tracks[aid]
        return out


def _rotate(x, y, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return c * x - s * y, s * x + c * y
