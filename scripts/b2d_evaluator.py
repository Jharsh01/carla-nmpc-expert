"""Bench2Drive's leaderboard_evaluator.py with three changes: a route watchdog, and two so that several can run side
by side on one GPU (each run owns the 100 ports from its --port up: give job k --port=20000 + 100 k).

- Watchdog: a route is stopped, and recorded as crashed ("Failed - Simulation crashed (watchdog: <why>)"), once the
  ego has been at rest (under 0.1 m/s) for B2D_MAX_STOPPED seconds, or the route has run for B2D_MAX_TIME seconds,
  both in simulation time (0 or unset: off). The leaderboard's score for what was driven until then still counts:
  Bench2Drive's merge_route_json.py leaves out only routes whose agent crashed. It sits in the leaderboard's tick
  loop (ScenarioManager._tick_scenario), outside the agent, and reads the ego's speed from CarlaDataProvider.

- Sync check: at a route's start, that the world is in synchronous mode with a fixed step (0.05 s, the
  leaderboard's 20 Hz); then that each tick advances simulation time by exactly that step (a skipped or repeated
  frame is reported); and every 10 s of simulation time, how fast it runs against real time (simulation seconds
  per wall-clock second: 1.0 is real time). All printed as "Sync: ..." lines.

- Ports: the evaluator takes the first port from --port up that is free, but CARLA also binds the next two
  (streaming and secondary). With either of those still held (a server shutting down, a stray one) the new server
  dies at start ("bind: Address already in use") and the evaluator waits for it for up to 20 x --timeout. Here a
  port is taken only with the two after it free too, within the run's range.
- Cleanup after a crash: the evaluator kills every process started with `-graphicsadapter=<gpu rank>`, which with
  one GPU is every job's server. Here it kills the servers in this run's port range only.

    python scripts/b2d_evaluator.py <the evaluator's arguments>
"""
import faulthandler
import importlib.util
import os
import socket
import subprocess
import sys
import time

faulthandler.enable()  # a crash in CARLA's client library prints the Python stack to the log
base = int(next(a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--port=")))
MAX_STOPPED = float(os.environ.get("B2D_MAX_STOPPED") or 0.0)  # [s] at rest before the route is stopped; 0: off
MAX_TIME = float(os.environ.get("B2D_MAX_TIME") or 0.0)  # [s] of simulation time before the route is stopped; 0: off
STOPPED_SPEED = 0.1  # [m/s] under which the ego counts as at rest
FIXED_DT = 0.05  # [s] the leaderboard's step
SYNC_REPORT = 10.0  # [s] of simulation time between real-time reports


def kill_servers_cmd(lo, hi=None):
    """A shell command killing the CARLA servers whose RPC port is in [lo, hi] (the run's range by default)."""
    hi = lo + 99 if hi is None else hi
    return ("ps -eo pid,args | awk -v lo=%d -v hi=%d 'match($0, /-carla-rpc-port=[0-9]+/) "
            "{ p = substr($0, RSTART + 16, RLENGTH - 16) + 0; if (p >= lo && p <= hi) print $1 }' "
            "| xargs -r kill -9" % (lo, hi))


def _free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("", port))
            return True
        except OSError:
            return False


def find_free_port(starting_port):
    """The first port from starting_port up with it and the two after it free, the RPC port within the run's range
    (the traffic manager's port, from its own base, needs only itself)."""
    rpc = starting_port == base
    for port in range(starting_port, starting_port + (97 if rpc else 100)):
        if all(_free(p) for p in ((port, port + 1, port + 2) if rpc else (port,))):
            return port
    raise RuntimeError(f"no free port from {starting_port}")


_Popen = subprocess.Popen


def _popen(cmd, *args, **kwargs):
    if isinstance(cmd, str) and "-graphicsadapter=" in cmd and "kill -9" in cmd:
        cmd = kill_servers_cmd(base)
    return _Popen(cmd, *args, **kwargs)


class WatchdogStop(Exception):
    """Raised in the tick loop to stop the route: the evaluator records it as a simulation crash."""


def install_watchdog(evaluator):
    """Wrap the leaderboard's tick with the watchdog's checks, and add its reason to the crash it records."""
    from leaderboard.scenarios.scenario_manager import ScenarioManager
    from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
    from srunner.scenariomanager.timer import GameTime

    fired = {"reason": None}
    tick = ScenarioManager._tick_scenario

    def watched_tick(self):
        tick(self)
        if not self._running:
            return
        now = GameTime.get_time()
        if getattr(self, "_watch_start", None) != self.start_game_time:  # a new route
            self._watch_start, self._stopped_since = self.start_game_time, None
            settings = CarlaDataProvider.get_world().get_settings()
            ok = settings.synchronous_mode and settings.fixed_delta_seconds is not None \
                and abs(settings.fixed_delta_seconds - FIXED_DT) < 1e-6
            print(f"\033[{'92' if ok else '91'}mSync: synchronous_mode={settings.synchronous_mode}, "
                  f"fixed_delta_seconds={settings.fixed_delta_seconds} ({'ok' if ok else f'expected {FIXED_DT}'})"
                  "\033[0m", flush=True)
            self._sync = {"last": now, "bad": 0, "t0": now, "w0": time.monotonic(), "next": now + SYNC_REPORT}
        sync = self._sync
        if now != sync["last"] and abs(now - sync["last"] - FIXED_DT) > 1e-3:  # no new frame: not a step
            sync["bad"] += 1
            print(f"\033[91mSync: a tick advanced {now - sync['last']:.3f} s, not {FIXED_DT} s, at "
                  f"{now - self.start_game_time:.2f} s\033[0m", flush=True)
        sync["last"] = now
        if now >= sync["next"]:
            wall = time.monotonic() - sync["w0"]
            print(f"Sync: {now - sync['t0']:.1f} s simulated in {wall:.1f} s wall clock, "
                  f"{(now - sync['t0']) / wall:.3f}x real time, {sync['bad']} irregular ticks", flush=True)
            sync["next"] += SYNC_REPORT
        if CarlaDataProvider.get_velocity(self.ego_vehicles[0]) > STOPPED_SPEED:
            self._stopped_since = None
        elif self._stopped_since is None:
            self._stopped_since = now
        reason = None
        if MAX_STOPPED > 0 and self._stopped_since is not None and now - self._stopped_since >= MAX_STOPPED:
            reason = f"ego at rest for {MAX_STOPPED:g} s"
        elif MAX_TIME > 0 and now - self.start_game_time >= MAX_TIME:
            reason = f"route over {MAX_TIME:g} s of simulation time"
        if reason:
            fired["reason"] = reason
            print(f"\n\033[91mWatchdog: {reason}, at {now - self.start_game_time:.1f} s into the route: stopping it, "
                  "recorded as crashed\033[0m", flush=True)
            raise WatchdogStop(reason)

    register = evaluator.LeaderboardEvaluator._register_statistics

    def watched_register(self, route_index, entry_status, crash_message=""):
        if fired["reason"]:
            crash_message = f"{crash_message} (watchdog: {fired['reason']})"
            fired["reason"] = None
        return register(self, route_index, entry_status, crash_message)

    ScenarioManager._tick_scenario = watched_tick
    evaluator.LeaderboardEvaluator._register_statistics = watched_register


subprocess.Popen = _popen
path = os.path.join(os.environ["SCENARIO_RUNNER_ROOT"], "..", "leaderboard", "leaderboard", "leaderboard_evaluator.py")
spec = importlib.util.spec_from_file_location("leaderboard_evaluator", path)
evaluator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluator)
evaluator.find_free_port = find_free_port
install_watchdog(evaluator)  # the sync check always; the watchdog's limits only when set
sys.argv[0] = path
evaluator.main()
