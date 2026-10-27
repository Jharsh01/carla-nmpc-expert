"""The expert: the world model, the behaviour layer and the NMPC, and the MKZ's actuation, tick by tick.

Shared by the leaderboard agent (expert/agents/expert_agent.py) and scripts/drive_expert.py. Each tick (20 Hz):
the ego's state; every `plan_every` ticks the visible actors, the Scene and a behaviour step; then the plan sampled
a little ahead (by the actuation's lags) for the acceleration and road-wheel angle, and those as throttle, brake and
steer.
"""
import math
import os

import numpy as np

from expert import frames
from expert.mkz_actuation import MKZActuation
from expert.world_model.actors import Actors
from expert.world_model.route import Route

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COM_AHEAD = 0.30  # the MKZ's centre of mass ahead of its actor origin [m] (config/vehicles/mkz2020.json)


class ExpertDriver:
    def __init__(self, expertctl, carla, world, hero, dense_route, dt=0.05, plan_every=2, visibility=True,
                 cache_dir=None, log=None):
        self.ex, self.carla, self.world, self.hero = expertctl, carla, world, hero
        self.map = world.get_map()
        self.dt, self.plan_every = dt, plan_every
        self.route = Route(expertctl, carla, self.map, dense_route)
        self.route.attach_world(world)
        self.actors = Actors(expertctl, carla, world, self.map, hero, visibility=visibility)
        self.params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
        self.body = expertctl.Body(length=4.89, width=1.84 + 0.2, cg_to_center=0.31)  # mirrors on the width
        nmpc = expertctl.UrbanOptions()
        nmpc.a_min = -7.0
        nmpc.jit = True
        nmpc.cache_dir = cache_dir or os.path.join(REPO, "build", "urban_cache")
        os.makedirs(nmpc.cache_dir, exist_ok=True)
        self.behaviour = expertctl.Behaviour(self.route.path, self.params, self.body, nmpc, expertctl.BehaviourOptions())
        self.actuation = MKZActuation(os.path.join(REPO, "config", "vehicles", "mkz2020_actuation.json"), dt=dt)
        self.tick = 0
        self.plan = None
        self.t_plan = 0.0
        self.decision = None
        self.steer = 0.0
        self.a_filtered = 0.0
        self._s = None  # the ego's position along the route, last tick
        self.log = log  # a callable taking a dict per tick, or None

    def ego(self):
        """The ego at its centre of mass in the library's frame, with its speed, acceleration (from the IMU's
        reading, low-passed) and the road-wheel angle the last steer command gives."""
        tr = self.hero.get_transform()
        psi = frames.from_carla_yaw(tr.rotation.yaw)
        x0, y0 = frames.from_carla_xy(tr.location.x, tr.location.y)
        vel, acc = self.hero.get_velocity(), self.hero.get_acceleration()
        yaw = math.radians(tr.rotation.yaw)
        v = vel.x * math.cos(yaw) + vel.y * math.sin(yaw)
        a = acc.x * math.cos(yaw) + acc.y * math.sin(yaw)
        self.a_filtered += 0.3 * (a - self.a_filtered)
        delta = self.actuation.delta_of(self.steer, max(v, 0.0))
        return self.ex.Ego(x=x0 + COM_AHEAD * math.cos(psi), y=y0 + COM_AHEAD * math.sin(psi), psi=psi, v=max(v, 0.0),
                           a=self.a_filtered, delta=delta)

    def scene(self, t, ego):
        sc = self.ex.Scene()
        sc.t = t
        sc.ego = ego
        sc.v_max = max(1.0, self.hero.get_speed_limit() / 3.6)
        sc.lanes = self.route.lanes
        sc.actors = self.actors.scene_actors(t)
        sc.stops = self.route.stops(self.ex, self._s)
        return sc

    def locate(self, ego):
        """(s, e_y) of the ego on the route, searched near where it was last."""
        path = self.route.path
        s, e = path.project(ego.x, ego.y) if self._s is None else path.project(ego.x, ego.y, self._s - 30.0, self._s + 60.0)
        self._s = s
        return s, e

    def _describe(self, scene, s_now):
        """The scene as the log keeps it: each actor's route position now, and the lane band where the car is."""
        path = self.route.path
        actors = []
        for a in scene.actors:
            s, e = path.project(a.x[0], a.y[0], s_now - 50.0, s_now + 160.0)
            actors.append([a.id, str(a.kind).split(".")[-1], bool(a.parked), round(s, 1), round(e, 2),
                           round(a.length, 1), round(a.width, 1), round(math.hypot(a.x[-1] - a.x[0], a.y[-1] - a.y[0]), 1)])
        lane = next((l for l in scene.lanes if l.s_from <= s_now < l.s_to), scene.lanes[-1])
        return {"actors": actors, "lane": [round(lane.lo, 2), round(lane.hi, 2), round(lane.left_width, 2)],
                "stops": [[str(st.kind).split(".")[-1], round(st.s, 1)] for st in scene.stops]}

    def _plan_at(self, tau, i):
        times = self.plan.times
        col = self.plan.x[:, i]
        return float(np.interp(tau, times, col))

    def step(self, t):
        """CARLA control for this tick, at simulation time t."""
        ego = self.ego()
        s_now, e_now = self.locate(ego)
        planned = self.plan is None or self.tick % self.plan_every == 0
        if planned:
            self.actors.detect(t)
            scene = self.scene(t, ego)
            self.decision = self.behaviour.step(scene)
            self.plan, self.t_plan = self.decision.plan, t
        self.tick += 1
        since = t - self.t_plan
        a_ref = self._plan_at(since + 0.25, 4)  # the acceleration a lag ahead: the drivetrain takes that long
        delta_ref = self._plan_at(since + 0.15, 5)  # the steering's lag (scripts/identify_ego.py)
        if self._plan_at(since, 3) < 0.2 and self._plan_at(since + 1.0, 3) < 0.2:
            a_ref = min(a_ref, -1.0)  # planned at rest: hold the brake
        throttle, brake, steer = self.actuation.command(a_ref, delta_ref, ego.v, self.a_filtered)
        self.steer = steer
        if self.log is not None:
            d = self.decision
            self.log({"t": round(t, 3), "s": s_now, "e_y": e_now, "x": ego.x, "y": ego.y, "v": ego.v, "a": self.a_filtered,
                      "v_max": self.hero.get_speed_limit() / 3.6, "a_ref": a_ref, "delta_ref": delta_ref,
                      "throttle": throttle, "brake": brake, "steer": steer, "label": d.label,
                      "step_ms": 1e3 * d.wall_time if since == 0.0 else 0.0, "success": d.plan.success,
                      "n_actors": len(self.actors.tracks), "notes": list(d.notes) if since == 0.0 else [],
                      "scene": self._describe(scene, s_now) if planned else None})
        return self.carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
