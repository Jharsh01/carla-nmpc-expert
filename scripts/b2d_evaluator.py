"""Bench2Drive's leaderboard_evaluator.py with two changes, so that several can run side by side on one GPU.
Each run owns the 100 ports from its --port up (give job k --port=20000 + 100 k).

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

faulthandler.enable()  # a crash in CARLA's client library prints the Python stack to the log
base = int(next(a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--port=")))


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


subprocess.Popen = _popen
path = os.path.join(os.environ["SCENARIO_RUNNER_ROOT"], "..", "leaderboard", "leaderboard", "leaderboard_evaluator.py")
spec = importlib.util.spec_from_file_location("leaderboard_evaluator", path)
evaluator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluator)
evaluator.find_free_port = find_free_port
sys.argv[0] = path
evaluator.main()
