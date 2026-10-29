"""SimLingo driving through the urban NMPC, as a leaderboard agent: SimLingo's predicted path is the route the NMPC
plans along and its speed waypoints the speed asked for, in place of its PID (LingoAgent.control_pid). The world
model (expert/world_model) adds, as constraints, what the car must keep clear of and the rules it must keep: the
visible actors, the lanes, the lights and signs.

    scripts/run_bench2drive.sh --agent simlingo_nmpc --routes 0-9 [--no-cot] [--fast]

Everything the NMPC sees is in the car's own frame, as SimLingo predicts in it: the car at the origin, its path,
and the world model's actors, lanes and stops as a sensor on the car would report them. The car's state is what it
measures: the speedometer, the IMU's forward acceleration, the steering it commanded. Between plans the last plan
is carried into the new frame by odometry (speed and compass). No global position is used for driving; the world
model reads its ground truth through the car's true pose, as the stand-in for perception it is.

Each tick (20 Hz) LingoAgent.run_step runs, but SimLingo itself predicts only every SIMLINGO_EVERY ticks (1 Hz); in
between its last prediction is held, its path carried into the car's frame now by odometry and its speeds taken from
the prediction's age on. Every PLAN_EVERY ticks a new path, the scene and a behaviour step (core/urban); then the plan sampled a little ahead (by the actuation's lags) for the acceleration and
road-wheel angle, and those as throttle, brake and steer (expert/mkz_actuation.py). SimLingo's restart creep
(forced throttle after a long stop) is off: the NMPC decides when to move. With SAVE_PATH set (run_bench2drive.sh
sets it), each route's ticks are logged to <SAVE_PATH>/<route>/nmpc.jsonl. With NMPC_LIVE_PORT set
(run_bench2drive.sh --live), each plan is also shown live in a browser (expert/live_view.py).
"""
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [p for p in (os.path.join(REPO, "build", "python"), REPO) if p not in sys.path]

import carla  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from agent_simlingo import LingoAgent  # noqa: E402  (SimLingo's team_code, on PYTHONPATH)
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider  # noqa: E402

import expertctl  # noqa: E402
from expert import frames, live_view, simlingo_ref  # noqa: E402
from expert.mkz_actuation import MKZActuation  # noqa: E402
from expert.world_model.actors import Actors  # noqa: E402
from expert.world_model.route import Route  # noqa: E402

DT = 0.05  # [s] a tick
PLAN_EVERY = 2  # ticks between plans (10 Hz)
SIMLINGO_EVERY = 20  # ticks between SimLingo's predictions (1 Hz)
COM_AHEAD = 0.30  # the MKZ's centre of mass ahead of its origin [m] (docs/vehicle_parameters.md)


def get_entry_point():
    return "SimLingoNMPCAgent"


class _HeldModel:
    """SimLingo's model, run every SIMLINGO_EVERY ticks; in between its last prediction, carried into the car's frame
    now (the path and speed waypoints as LingoAgent draws them; the speeds the NMPC is asked for come from the
    prediction as made, by its age: SimLingoNMPCAgent.control_pid)."""

    def __init__(self, model, agent):
        self.__dict__.update(_model=model, _agent=agent)

    def __getattr__(self, name):
        return getattr(self._model, name)

    def __call__(self, model_input):
        a = self._agent
        if a.held is None or a.held_ticks >= SIMLINGO_EVERY:
            speed_wps, route, language = self._model(model_input)
            a.held = (None if speed_wps is None else speed_wps.float()[0].detach().cpu().numpy(),
                      route.float()[0].detach().cpu().numpy(), language)
            a.held_ticks = 0
            a.held_odometry.restart()
            return speed_wps, route, language
        speed_wps, route, language = a.held
        pose = a.held_odometry.pose()
        as_tensor = lambda p: torch.as_tensor(simlingo_ref.carry(p, pose), dtype=torch.float32).unsqueeze(0)  # noqa: E731
        return (None if speed_wps is None else as_tensor(speed_wps)), as_tensor(route), language


class SimLingoNMPCAgent(LingoAgent):
    def setup(self, path_to_conf_file, route_index=None):
        super().setup(path_to_conf_file, route_index)
        self.config.stuck_threshold = float("inf")  # no forced creep past the NMPC
        self.started = False
        self.log_file = None
        self.live = None
        self._t = 0.0
        self._imu = None
        self.held, self.held_ticks = None, 0  # SimLingo's last prediction (speed waypoints, route, words), its age
        self.held_odometry = simlingo_ref.Odometry()  # the car's motion since that prediction
        self.model = _HeldModel(self.model, self)

    def tick(self, input_data):
        self._imu = input_data["imu"][1]  # accelerometer x, y, z, gyroscope x, y, z, compass
        result = super().tick(input_data)
        self.held_odometry.update(max(float(result["speed"].flatten()[0]), 0.0), float(self._imu[-1]), DT)
        self.held_ticks += 1
        return result

    def run_step(self, input_data, timestamp, sensors=None):
        self._t = timestamp
        return super().run_step(input_data, timestamp, sensors)

    def _start(self):
        world = CarlaDataProvider.get_world()
        self.hero = CarlaDataProvider.get_hero_actor()  # for the world model only: where its stand-in sensors are
        carla_map = world.get_map()
        end = self._global_plan_world_coord[-1][0].location
        self.route = Route(expertctl, carla, world, carla_map, frames.from_carla_xy(end.x, end.y))
        self.actors = Actors(expertctl, carla, world, carla_map, self.hero)
        self.params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
        self.body = expertctl.Body(length=4.89, width=1.84 + 0.2, cg_to_center=0.31)  # mirrors on the width
        self.nmpc_options = expertctl.UrbanOptions()
        self.nmpc_options.a_min = -7.0
        self.nmpc_options.jit = True
        self.nmpc_options.cache_dir = os.path.join(REPO, "build", "urban_cache")
        os.makedirs(self.nmpc_options.cache_dir, exist_ok=True)
        self.behaviour = None  # made with the first path
        self.actuation = MKZActuation(os.path.join(REPO, "config", "vehicles", "mkz2020_actuation.json"), dt=DT)
        self.odometry = simlingo_ref.Odometry()
        self.ticks = 0
        self.plan = self.decision = None
        self.t_plan = 0.0
        self.steer = self.a_filtered = 0.0
        save = os.environ.get("SAVE_PATH")
        if save:
            route = os.path.basename(os.environ.get("ROUTES", "route")).replace(".xml", "")
            os.makedirs(os.path.join(save, route), exist_ok=True)
            self.log_file = open(os.path.join(save, route, "nmpc.jsonl"), "w")
        port = os.environ.get("NMPC_LIVE_PORT")
        if port:
            try:
                self.live = live_view.LiveView(int(port))
                print(f"NMPC live view: http://localhost:{port} (from another machine: ssh -L {port}:localhost:{port})",
                      flush=True)
            except OSError as e:
                print(f"NMPC live view not started: {e}", flush=True)
        self.started = True

    def _plan_at(self, tau, i):
        return float(np.interp(tau, self.plan.times, self.plan.x[:, i]))

    def control_pid(self, route_waypoints, velocity, speed_waypoints):
        """In place of SimLingo's PID: (steer, throttle, brake) from the NMPC on SimLingo's path and speed."""
        if not self.started:
            self._start()
        route = route_waypoints[0].detach().cpu().numpy()
        speed = max(float(velocity.flatten()[0]), 0.0)
        t = self._t
        self.odometry.update(speed, float(self._imu[-1]), DT)
        self.a_filtered += 0.3 * (float(self._imu[0]) - self.a_filtered)
        delta = self.actuation.delta_of(self.steer, speed)
        ego = expertctl.Ego(x=COM_AHEAD, y=0.0, psi=0.0, v=speed, a=self.a_filtered, delta=delta)  # in its own frame

        planned = self.plan is None or self.ticks % PLAN_EVERY == 0
        if planned:
            frame = frames.Frame.of(self.hero.get_transform())  # the world model's, to report in the car's frame
            path, s_simlingo = self.route.path(simlingo_ref.path_points(route), frame)
            if self.behaviour is None:
                self.behaviour = expertctl.Behaviour(path, self.params, self.body, self.nmpc_options,
                                                     expertctl.BehaviourOptions())
                self.odometry.old_frame()
            else:
                self.behaviour.set_route(path, expertctl.Pose(*self.odometry.old_frame()))
            s_now = path.project(ego.x, ego.y)[0]
            self.actors.detect(t)
            scene = expertctl.Scene()
            scene.t = t
            scene.ego = ego
            scene.v_max = max(1.0, self.hero.get_speed_limit() / 3.6)  # a stand-in for reading the signs
            scene.v_ref_t, scene.v_ref = simlingo_ref.speed_reference(self.held[0], age=self.held_ticks * DT)
            scene.line_s, scene.line_e = [0.0, path.length], [0.0, 0.0]  # SimLingo's path itself
            scene.lanes = self.route.lanes(path, frame)
            scene.actors = self.actors.scene_actors(t, frame)
            scene.stops = self.route.stops(path, s_now, frame)
            self.decision = self.behaviour.step(scene)
            self.plan, self.t_plan = self.decision.plan, t
            if self.live is not None:
                try:  # the view must never stop the driving
                    self.live.publish(live_view.snapshot(t, path, s_now, s_simlingo, scene, self.decision))
                except Exception as e:  # noqa: BLE001
                    print(f"NMPC live view stopped: {e!r}", flush=True)
                    self.live = None
        self.ticks += 1

        since = t - self.t_plan
        a_ref = self._plan_at(since + 0.25, 4)  # the acceleration a lag ahead: the drivetrain takes that long
        delta_ref = self._plan_at(since + 0.15, 5)  # the steering's lag (scripts/identify_ego.py)
        if self._plan_at(since, 3) < 0.2 and self._plan_at(since + 1.0, 3) < 0.2:
            a_ref = min(a_ref, -1.0)  # planned at rest: hold the brake
        throttle, brake, steer = self.actuation.command(a_ref, delta_ref, speed, self.a_filtered)
        self.steer = steer
        if self.log_file is not None:
            row = {"t": round(t, 3), "v": speed, "a": self.a_filtered, "a_ref": a_ref, "delta_ref": delta_ref,
                   "throttle": throttle, "brake": brake, "steer": steer, "label": self.decision.label,
                   "step_ms": 1e3 * self.decision.wall_time if planned else 0.0, "success": self.decision.plan.success,
                   "n_actors": len(self.actors.tracks), "notes": list(self.decision.notes) if planned else []}
            if planned:
                row.update(self._describe(scene, path, s_now, s_simlingo))
            self.log_file.write(json.dumps(row) + "\n")
        return steer, throttle, brake

    def _describe(self, scene, path, s_now, s_simlingo):
        """The plan's scene as the log keeps it: each actor's position along the path, the lane band at the car, the
        stops, how far SimLingo's part of the path reaches; and how far the plan departs from what SimLingo asked
        for (the largest offset from its path over 2 s, the speed less the one asked for at 1 s)."""
        actors = []
        for a in scene.actors:
            s, e = path.project(a.x[0], a.y[0])
            actors.append([a.id, str(a.kind).split(".")[-1], bool(a.parked), round(s - s_now, 1), round(e, 2),
                           round(a.length, 1), round(a.width, 1), round(math.hypot(a.x[-1] - a.x[0], a.y[-1] - a.y[0]), 1)])
        lane = next((l for l in scene.lanes if l.s_from <= s_now < l.s_to), scene.lanes[-1])
        early = np.asarray(self.plan.times) <= 2.0
        dv = self._plan_at(1.0, 3) - float(np.interp(1.0, scene.v_ref_t, scene.v_ref)) if scene.v_ref else 0.0
        return {"scene": {"actors": actors, "lane": [round(lane.lo, 2), round(lane.hi, 2), round(lane.left_width, 2)],
                          "stops": [[str(st.kind).split(".")[-1], round(st.s - s_now, 1)] for st in scene.stops],
                          "simlingo_reach": round(s_simlingo - s_now, 1)},
                "override": [round(float(np.max(np.abs(self.plan.x[early, 1]))), 3), round(dv, 3)]}

    def destroy(self, results=None):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        if self.live is not None:
            self.live.close()
            self.live = None
        self.started = False
        self.model = self.model._model  # LingoAgent.destroy deletes it
        try:
            super().destroy(results)
        except AttributeError:  # LingoAgent.destroy reads cfg.data_module.encoder, which its config lacks
            pass
