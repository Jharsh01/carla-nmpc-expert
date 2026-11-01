"""SimLingo's model in its own process, on its own CPUs (expert/cpus.py), so that its preprocessing and its forward
pass never hold up the leaderboard's loop: started by SimLingoNMPCAgent, one per route.

The agent keeps SimLingo's per-tick bookkeeping (GNSS, the filter, the route planner, the commands) and, once a
second, sends a request: that tick's camera, IMU and speed, with the bookkeeping's results and its state before
them. Here LingoAgent.tick runs on it unchanged, its filter and route planner replaying the agent's results, so the
model sees exactly the input SimLingo's own agent would have built; then the model. The reply: the speed waypoints,
the route and the words, as numpy, and how long the two took.

    started by the agent: python expert/simlingo_worker.py, SIMLINGO_WORKER_FD the socket to it
"""
import os
import sys
import tempfile
import time
from collections import deque
from multiprocessing.connection import Connection

# the repository in place of this script's folder, expert/, whose agents/ would hide CARLA's agents package
sys.path[0] = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from expert import cpus  # noqa: E402

PINNED = cpus.pin("simlingo")  # before torch starts its threads

import numpy as np  # noqa: E402
import torch  # noqa: E402
import agent_simlingo  # noqa: E402  (SimLingo's team_code, on PYTHONPATH)


class _ReplayFilter:
    """The agent's filter: its state after this tick's update, which predict and update leave as it is."""

    def __init__(self, x):
        self.x = x

    def predict(self, **kwargs):
        pass

    def update(self, z):
        pass


class _ReplayPlanner:
    """The agent's route planner: this tick's GNSS position and route, as the agent's planner gave them."""

    def __init__(self, gps_pos, route):
        self.gps_pos, self.route = gps_pos, route

    def convert_gps_to_carla(self, gps):
        return self.gps_pos

    def run_step(self, gps):
        return self.route


def main():
    if PINNED:
        torch.set_num_threads(len(PINNED))
    conn = Connection(int(os.environ["SIMLINGO_WORKER_FD"]))
    setup_args = conn.recv()
    os.environ["SAVE_PATH"] = tempfile.mkdtemp(prefix="simlingo_worker_")  # LingoAgent.setup's folders, unused
    agent = agent_simlingo.LingoAgent.__new__(agent_simlingo.LingoAgent)  # no leaderboard agent: no CARLA client
    agent.setup(*setup_args)
    agent.control = agent_simlingo.carla.VehicleControl()  # read by the filter's predict, which replays
    conn.send("ready")
    while True:
        request = conn.recv()
        if request is None:
            break
        t0 = time.perf_counter()
        agent.ukf = _ReplayFilter(request["filtered"])
        agent.filter_initialized = True
        agent._route_planner = _ReplayPlanner(request["gps_pos"], request["route"])
        agent.commands = deque(request["commands"], maxlen=2)
        agent.last_command, agent.last_command_tmp = request["last_command"], request["last_command_tmp"]
        agent.target_point_prev = request["target_point_prev"]
        agent.tick(request["input_data"])
        t1 = time.perf_counter()
        with torch.inference_mode():
            speed_wps, route, language = agent.model(agent_simlingo.DrivingInput(**agent.DrivingInput))
            speed_wps = None if speed_wps is None else speed_wps.float()[0].cpu().numpy()
            route = route.float()[0].cpu().numpy()  # to the CPU: waits for the GPU
        t2 = time.perf_counter()
        conn.send({"speed_wps": speed_wps, "route": np.asarray(route), "language": language,
                   "preprocess_ms": 1e3 * (t1 - t0), "model_ms": 1e3 * (t2 - t1)})


if __name__ == "__main__":
    main()
