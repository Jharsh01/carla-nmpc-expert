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

Each tick (20 Hz) SimLingo's bookkeeping runs (GNSS, its filter, its route planner: LingoAgent.tick without the
camera). SimLingo's model runs in its own process on its own CPUs (expert/simlingo_worker.py, expert/cpus.py): every
SIMLINGO_EVERY ticks (1 Hz) that tick's camera is sent to it, and its prediction is taken SIMLINGO_LATENCY ticks
later, as a car would get it, so the simulation goes on while it thinks; in between, the last prediction is held,
its path carried into the car's frame now by odometry and its speeds taken from the prediction's age on. Every
PLAN_EVERY ticks a new path, the scene and a behaviour step (core/urban), the step solved on the NMPC's own CPUs and
its plan taken NMPC_LATENCY ticks later. SimLingo's path is planned along only while it is a manoeuvre, a lane change
or a turn (it leaves the lane the car is in: expertctl.PathIntent); otherwise the NMPC keeps to the lane's centre, and
SimLingo's speeds count only when they ask to stop. A plan that fails to solve is never driven (expert/plan_hold.py). Then the plan sampled a little ahead (by the actuation's lags) for the
acceleration and road-wheel angle, and those as throttle, brake and steer (expert/mkz_actuation.py). Both latencies
are counted in ticks, not wall-clock time, so a run does not depend on how fast the machine is: if a result is not
ready when due, the tick waits for it. SimLingo's restart creep (forced throttle after a long stop) is off: the NMPC
decides when to move. With SAVE_PATH set (run_bench2drive.sh sets it), each route's ticks are logged to
<SAVE_PATH>/<route>/nmpc.jsonl. With NMPC_LIVE_PORT set (run_bench2drive.sh --live), each plan is also shown live in
a browser (expert/live_view.py).
"""
import concurrent.futures
import contextlib
import json
import math
import os
import socket
import subprocess
import sys
from multiprocessing.connection import Connection

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [p for p in (os.path.join(REPO, "build", "python"), REPO) if p not in sys.path]

import carla  # noqa: E402
import hydra  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import agent_simlingo  # noqa: E402  (SimLingo's team_code, on PYTHONPATH)
from agent_simlingo import LingoAgent  # noqa: E402
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider  # noqa: E402

import expertctl  # noqa: E402
from expert import cpus, frames, live_view, simlingo_ref, timing  # noqa: E402
from expert.mkz_actuation import MKZActuation, rate_limited  # noqa: E402
from expert.plan_hold import PlanHold  # noqa: E402
from expert.world_model.actors import Actors  # noqa: E402
from expert.world_model.route import Route  # noqa: E402

DT = 0.05  # [s] a tick
PLAN_EVERY = 4  # ticks between plans (5 Hz)
NMPC_LATENCY = 1  # ticks from a plan's scene to its use: the NMPC has 50 ms, as it would in a car
SIMLINGO_EVERY = 10  # ticks between SimLingo's predictions (2 Hz)
SIMLINGO_LATENCY = 6  # ticks from SimLingo's camera frame to its prediction's use: on its own core of an L4
                      # machine it takes 290 ms (60 preprocessing, 230 the model), so 300 ms would hold in real time
METRIC_EVERY = 200  # ticks between writes of SimLingo's metric_info.json (and at the end)
COM_AHEAD = 0.30  # the MKZ's centre of mass ahead of its origin [m] (docs/vehicle_parameters.md)
WORKER_START_TIMEOUT = 600.0  # [s] for the worker to load SimLingo's model


def get_entry_point():
    return "SimLingoNMPCAgent"


class _NoModel:
    """Stands in for SimLingo's model in this process, where LingoAgent.setup would load it (it runs in the worker):
    every attribute and every call is itself."""

    def __getattr__(self, name):
        return self

    def __call__(self, *args, **kwargs):
        return self


@contextlib.contextmanager
def _without_model():
    instantiate, load = hydra.utils.instantiate, torch.load
    hydra.utils.instantiate, torch.load = _NoModel(), (lambda *a, **k: {})
    try:
        yield
    finally:
        hydra.utils.instantiate, torch.load = instantiate, load


class _Worker:
    """SimLingo's model in its own process (expert/simlingo_worker.py): one request at a time."""

    def __init__(self, setup_args):
        mine, theirs = socket.socketpair()
        env = dict(os.environ, SIMLINGO_WORKER_FD=str(theirs.fileno()))
        self.proc = subprocess.Popen([sys.executable, "-u", os.path.join(REPO, "expert", "simlingo_worker.py")],
                                     env=env, pass_fds=(theirs.fileno(),))
        theirs.close()
        self.conn = Connection(mine.detach())
        self.conn.send(setup_args)
        if self.recv(WORKER_START_TIMEOUT) != "ready":
            raise RuntimeError("SimLingo's worker did not start")

    def send(self, request):
        self.conn.send(request)

    def recv(self, timeout=None):
        waited = 0.0
        while not self.conn.poll(1.0):
            waited += 1.0
            if self.proc.poll() is not None:
                raise RuntimeError(f"SimLingo's worker exited ({self.proc.returncode})")
            if timeout is not None and waited >= timeout:
                raise RuntimeError(f"SimLingo's worker did not answer in {timeout:g} s")
        try:
            return self.conn.recv()
        except (EOFError, OSError) as e:
            raise RuntimeError(f"SimLingo's worker exited (see its traceback above): {e!r}") from None

    def close(self):
        try:
            self.conn.send(None)
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            self.proc.kill()
        self.conn.close()


class _Job:
    """A behaviour step being solved on the NMPC's thread, with the scene it was made from (for the log)."""

    def __init__(self, future, tick, t, scene, path, s_now, s_simlingo, frame, route, intent):
        self.future, self.tick, self.t = future, tick, t
        self.scene, self.path, self.s_now, self.s_simlingo = scene, path, s_now, s_simlingo
        self.frame, self.route = frame, route  # the car's true pose then; SimLingo's path carried to it (y right)
        self.intent = intent  # for the log: [a manoeuvre, SimLingo's offset from the lane, it asks to stop]


class SimLingoNMPCAgent(LingoAgent):
    def setup(self, path_to_conf_file, route_index=None):
        assert agent_simlingo.USE_UKF, "the bookkeeping here follows LingoAgent.tick with its filter on"
        with _without_model():
            super().setup(path_to_conf_file, route_index)
        self.config.stuck_threshold = float("inf")  # no forced creep past the NMPC
        self.started = False
        self.log_file = None
        self.live = None
        self.held = None  # SimLingo's last prediction: speed waypoints, route, words
        self.held_step = 0  # the tick of its camera frame
        self.held_odometry = None  # the car's motion since then
        self.request = None  # (tick, odometry since then) of the prediction being made
        self.worker = _Worker((path_to_conf_file, route_index))
        self.nmpc_thread = concurrent.futures.ThreadPoolExecutor(1, initializer=cpus.pin, initargs=("nmpc",))

    @timing.timed("agent.run_step")
    def run_step(self, input_data, timestamp, sensors=None):
        """LingoAgent.run_step with SimLingo's model in its worker and the NMPC in place of its PID."""
        self.step += 1
        if not self.initialized:
            self._init()
            self.control = carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
            with timing.stage("run_step.measure"):
                self._measure(input_data)
            return self.control
        with timing.stage("run_step.measure"):
            speed, measured = self._measure(input_data)
            compass = float(input_data["imu"][1][-1])
            for odometry in (self.held_odometry, self.request and self.request[1]):
                if odometry is not None:
                    odometry.update(speed, compass, DT)
        self._simlingo(input_data, speed, measured, compass)
        steer, throttle, brake = self.control_pid(input_data["imu"][1], speed, timestamp)
        control = carla.VehicleControl(steer=float(steer), throttle=float(throttle), brake=float(brake))
        # CARLA will not let the car drive in the initial frames; braking keeps SimLingo's filter right
        self.control = carla.VehicleControl(0.0, 0.0, 1.0) if self.step < self.config.inital_frames_delay else control
        with timing.stage("run_step.metric"):
            self.metric_info[self.step] = self.get_metric_info()
            if self.step % METRIC_EVERY == 0:
                self._write_metric_info()
        return self.control

    def _measure(self, input_data):
        """SimLingo's bookkeeping for this tick, as LingoAgent.tick does it (GNSS, its filter, its route planner, the
        commands), without the camera and the prompt: the speed, and what the worker needs to build the model's input
        as LingoAgent.tick would (the bookkeeping's results and its state before them)."""
        state = {"commands": list(self.commands), "last_command": self.last_command,
                 "last_command_tmp": self.last_command_tmp, "target_point_prev": self.target_point_prev}
        gps_pos = self._route_planner.convert_gps_to_carla(input_data["gps"][1])
        compass = agent_simlingo.t_u.preprocess_compass(input_data["imu"][1][-1])
        speed = input_data["speed"][1]["speed"]
        z = np.array([gps_pos[0], gps_pos[1], agent_simlingo.t_u.normalize_angle(compass), speed])
        if not self.filter_initialized:
            self.ukf.x = z
            self.filter_initialized = True
        self.ukf.predict(steer=self.control.steer, throttle=self.control.throttle, brake=self.control.brake)
        self.ukf.update(z)
        filtered = np.array(self.ukf.x)
        self.state_log.append(filtered)
        route = list(self._route_planner.run_step(np.append(filtered[0:2], gps_pos[2])))
        target_point, far_command = route[1] if len(route) > 1 else route[0]
        if self.last_command_tmp != far_command:
            self.last_command = self.last_command_tmp
        self.last_command_tmp = far_command
        if (target_point != self.target_point_prev).all():
            self.target_point_prev = target_point
            self.commands.append(far_command.value)
        return max(float(speed), 0.0), dict(state, gps_pos=gps_pos, filtered=filtered, route=route)

    def _simlingo(self, input_data, speed, measured, compass):
        """Send this tick's camera to SimLingo's worker every SIMLINGO_EVERY ticks; take its prediction SIMLINGO_LATENCY
        ticks after (at once for the first, when there is nothing to hold)."""
        if self.request is None and (self.held is None or self.step - self.held_step >= SIMLINGO_EVERY):
            with timing.stage("run_step.simlingo_send"):
                frame = input_data["imu"][0]
                self.worker.send(dict(measured, input_data={
                    "rgb_0": input_data["rgb_0"], "imu": input_data["imu"], "speed": input_data["speed"],
                    "gps": (frame, None)}))
                odometry = simlingo_ref.Odometry()
                odometry.heading = compass  # the car's frame now
                self.request = (self.step, odometry)
            timing.flag("simlingo_sent", True)
        if self.request is not None and (self.held is None or self.step - self.request[0] >= SIMLINGO_LATENCY):
            with timing.stage("run_step.simlingo_wait"):
                reply = self.worker.recv()
            self.held = (reply["speed_wps"], reply["route"], reply["language"])
            self.held_step, self.held_odometry = self.request
            self.request = None
            timing.flag("simlingo_received", True)
            timing.flag("simlingo.preprocess_ms", round(reply["preprocess_ms"], 2))
            timing.flag("simlingo.model_ms", round(reply["model_ms"], 2))

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
        # 2 s ahead in 25 steps: 20 of 50 ms over the first second (a tick each), 5 of 200 ms over the second
        self.nmpc_options.horizon, self.nmpc_options.fine = 2.0, 1.0
        self.nmpc_options.h_fine, self.nmpc_options.h = 0.05, 0.2
        self.nmpc_options.jit = True
        self.nmpc_options.cache_dir = os.path.join(REPO, "build", "urban_cache")
        os.makedirs(self.nmpc_options.cache_dir, exist_ok=True)
        self.behaviour = None  # made with the first path
        self.actuation = MKZActuation(os.path.join(REPO, "config", "vehicles", "mkz2020_actuation.json"), dt=DT)
        self.odometry = simlingo_ref.Odometry()
        self.ticks = 0
        self.plan = self.decision = self.job = None
        self.hold = PlanHold()
        self.t_plan = 0.0
        self.steer = self.a_filtered = self.delta_command = 0.0
        self.logged_step = None  # the last prediction written to the log
        self.intent = expertctl.PathIntent()
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

    def _path(self, route, frame, ego):
        """The path for this plan and where SimLingo's part of it ends: the lane the car is in, unless SimLingo's path
        leaves it. `route`: SimLingo's path in the car's frame now (y right)."""
        lane = self.route.lane_path(frame)
        if lane is not None:
            s_now = lane.project(ego.x, ego.y)[0]
            points = [lane.project(float(x), float(-y)) for x, y in route]
            self.intent.update([p[0] - s_now for p in points], [p[1] for p in points])
            if not self.intent.manoeuvre:
                return lane, points[-1][0]
        return self.route.path(simlingo_ref.path_points(route), frame)

    def _plan_at(self, tau, i):
        return float(np.interp(tau, self.plan.times, self.plan.x[:, i]))

    @timing.timed("run_step.nmpc")
    def control_pid(self, imu, speed, t):
        """In place of SimLingo's PID: (steer, throttle, brake) from the NMPC on SimLingo's held path and speed, carried
        into the car's frame now. imu: accelerometer x, y, z, gyroscope x, y, z, compass."""
        if not self.started:
            self._start()
        route = simlingo_ref.carry(self.held[1], self.held_odometry.pose())
        self.odometry.update(speed, float(imu[-1]), DT)
        self.a_filtered += 0.3 * (float(imu[0]) - self.a_filtered)
        delta = self.actuation.delta_of(self.steer, speed)
        ego = expertctl.Ego(x=COM_AHEAD, y=0.0, psi=0.0, v=speed, a=self.a_filtered, delta=delta)  # in its own frame

        submitted = self.job is None and (self.plan is None or self.ticks % PLAN_EVERY == 0)
        timing.flag("planned", submitted)
        if submitted:
            with timing.stage("nmpc.path"):
                frame = frames.Frame.of(self.hero.get_transform())  # the world model's, to report in the car's frame
                path, s_simlingo = self._path(route, frame, ego)
            with timing.stage("nmpc.set_route"):  # the first: the solver's set-up (and compile, on a fresh cache)
                if self.behaviour is None:
                    self.behaviour = expertctl.Behaviour(path, self.params, self.body, self.nmpc_options,
                                                         expertctl.BehaviourOptions())
                    self.odometry.old_frame()
                else:
                    self.behaviour.set_route(path, expertctl.Pose(*self.odometry.old_frame()))
            s_now = path.project(ego.x, ego.y)[0]
            with timing.stage("nmpc.detect"):
                self.actors.detect(t)
            scene = expertctl.Scene()
            scene.t = t
            scene.ego = ego
            scene.v_max = max(1.0, self.hero.get_speed_limit() / 3.6)  # a stand-in for reading the signs
            scene.v_ref_t, scene.v_ref = simlingo_ref.speed_reference(self.held[0], age=(self.step - self.held_step) * DT)
            stop = simlingo_ref.asks_to_stop(scene.v_ref_t, scene.v_ref)
            if not self.intent.manoeuvre and not stop:  # the limit, for the actors and the rules to bring down
                scene.v_ref_t, scene.v_ref = [], []
            scene.line_s, scene.line_e = [0.0, path.length], [0.0, 0.0]  # SimLingo's path itself
            with timing.stage("nmpc.lanes"):
                scene.lanes = self.route.lanes(path, frame)
            with timing.stage("nmpc.scene_actors"):
                scene.actors = self.actors.scene_actors(t, frame, lambda x, y: self.route.ego_lanes(path, x, y))
            with timing.stage("nmpc.stops"):
                scene.stops = self.route.stops(path, s_now, frame)
            self.job = _Job(self.nmpc_thread.submit(self.behaviour.step, scene), self.ticks, t, scene, path, s_now,
                            s_simlingo, frame, route, [self.intent.manoeuvre, round(self.intent.offset, 3), stop])
        job = None
        if self.job is not None and (self.plan is None or self.ticks - self.job.tick >= NMPC_LATENCY):
            job, self.job = self.job, None
            with timing.stage("nmpc.wait"):
                self.decision = job.future.result()
            timing.flag("solve_ms", round(1e3 * self.decision.wall_time, 2))
            self.hold.update(self.decision.plan, job.t)
            self.plan, self.t_plan = self.hold.plan, self.hold.t
            if self.live is not None:
                try:  # the view must never stop the driving
                    with timing.stage("nmpc.live"):
                        self.live.publish(live_view.snapshot(job.t, job.path, job.s_now, job.s_simlingo, job.scene,
                                                             self.decision))
                except Exception as e:  # noqa: BLE001
                    print(f"NMPC live view stopped: {e!r}", flush=True)
                    self.live = None
        self.ticks += 1

        since = t - self.t_plan
        a_ref = self._plan_at(since + 0.25, 4)  # the acceleration a lag ahead: the drivetrain takes that long
        delta_ref = self._plan_at(since + 0.15, 5)  # the steering's lag (scripts/identify_ego.py)
        delta_ref = self.delta_command = rate_limited(delta_ref, self.delta_command, self.params.delta_rate_max, DT)
        if self._plan_at(since, 3) < 0.2 and self._plan_at(since + 1.0, 3) < 0.2:
            a_ref = min(a_ref, -1.0)  # planned at rest: hold the brake
        a_ref = min(a_ref, self.hold.a_limit(t))
        throttle, brake, steer = self.actuation.command(a_ref, delta_ref, speed, self.a_filtered)
        self.steer = steer
        if self.log_file is not None:
            row = {"t": round(t, 3), "v": speed, "a": self.a_filtered, "a_ref": a_ref, "delta_ref": delta_ref,
                   "throttle": throttle, "brake": brake, "steer": steer, "label": self.decision.label,
                   "step_ms": 1e3 * self.decision.wall_time if job else 0.0, "success": self.decision.plan.success,
                   "holding": self.hold.failed_since is not None,
                   "n_actors": len(self.actors.tracks), "notes": list(self.decision.notes) if job else []}
            true = frames.Frame.of(self.hero.get_transform())
            row["pose"] = [round(true.x, 3), round(true.y, 3), round(true.psi, 5)]  # for the video and analysis only
            row["held"] = [self.held_step, *(round(v, 3) for v in self.held_odometry.pose())]
            if job:  # a new plan: the scene it was made from, at its time (t_plan)
                row.update(self._describe(job.scene, job.path, job.s_now, job.s_simlingo))
                row.update(self._geometry(job))
            self.log_file.write(json.dumps(row) + "\n")
        return steer, throttle, brake

    @timing.timed("nmpc.describe")
    def _describe(self, scene, path, s_now, s_simlingo):
        """The plan's scene as the log keeps it: each actor's position along the path, the lane band at the car, the
        stops, how far SimLingo's part of the path reaches; and how far the plan departs from what SimLingo asked
        for (the largest offset from its path over 2 s, the speed less the one asked for at 1 s)."""
        actors = []
        for a in scene.actors:
            s, e = path.project(a.x[0], a.y[0])
            actors.append([a.id, str(a.kind).split(".")[-1], bool(a.parked), round(s - s_now, 1), round(e, 2),
                           round(a.length, 1), round(a.width, 1), round(math.hypot(a.x[-1] - a.x[0], a.y[-1] - a.y[0]), 1),
                           bool(a.stays_out)])
        lane = next((l for l in scene.lanes if l.s_from <= s_now < l.s_to), scene.lanes[-1])
        plan = self.decision.plan  # the one just solved, on this path, though it may not be the one driven
        early = np.asarray(plan.times) <= 2.0
        dv = float(np.interp(1.0, plan.times, plan.x[:, 3])) - float(np.interp(1.0, scene.v_ref_t, scene.v_ref)) if scene.v_ref else 0.0
        return {"scene": {"actors": actors, "lane": [round(lane.lo, 2), round(lane.hi, 2), round(lane.left_width, 2)],
                          "stops": [[str(st.kind).split(".")[-1], round(st.s - s_now, 1)] for st in scene.stops],
                          "simlingo_reach": round(s_simlingo - s_now, 1)},
                "override": [round(float(np.max(np.abs(plan.x[early, 1]))), 3), round(dv, 3)]}

    def _geometry(self, job):
        """The plan's lines for the video and analysis, in the car's frame (y left) at the true pose `frame`: SimLingo's
        path carried there, the path the NMPC got (from 15 m behind, every metre, 40 m ahead), its plan; and each
        prediction as SimLingo made it (y right), once."""
        pose = [job.frame.x, job.frame.y, job.frame.psi]
        q = [job.path.pose(float(s)) for s in np.arange(max(0.0, job.s_now - 15.0), min(job.path.length, job.s_now + 40.0), 1.0)]
        plan = [job.path.pose(float(s), float(e)) for s, e in self.decision.plan.x[:, :2]]
        row = {"frame": [round(float(v), 3) for v in pose[:2]] + [round(float(pose[2]), 5)],
               "simlingo": np.round(np.asarray(job.route) * [1.0, -1.0], 3).tolist(),
               "path": [[round(p.x, 3), round(p.y, 3)] for p in q],
               "plan_xy": [[round(p.x, 3), round(p.y, 3)] for p in plan], "s_now": round(job.s_now, 3),
               "intent": job.intent}
        if self.held_step != self.logged_step:
            self.logged_step = self.held_step
            row["prediction"] = {"step": self.held_step, "route": np.round(self.held[1], 3).tolist(),
                                 "speed_wps": None if self.held[0] is None else np.round(self.held[0], 3).tolist()}
        return row

    def _write_metric_info(self):
        with open(f"{self.save_path_metric}/metric_info.json", "w") as f:
            json.dump(self.metric_info, f, indent=4)

    def destroy(self, results=None):
        if getattr(self, "metric_info", None):
            self._write_metric_info()
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        if self.live is not None:
            self.live.close()
            self.live = None
        self.started = False
        self.nmpc_thread.shutdown(wait=True)
        self.worker.close()
        try:
            super().destroy(results)
        except AttributeError:  # LingoAgent.destroy reads cfg.data_module.encoder, which its config lacks
            pass
