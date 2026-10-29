"""The world model and the NMPC as a leaderboard agent, on the map route (SimLingo not in the loop): privileged, but
only with what a perception stack could estimate (expert/world_model), planning with the behaviour layer and the
urban NMPC (core/urban), driving the MKZ through its measured actuation (expert/mkz_actuation.py).

    scripts/run_bench2drive.sh --agent expert --routes 0-9

It declares a speedometer only: the leaderboard's sensors are not what it drives on. The world model reads the
world through CarlaDataProvider, within the limits set out in expert/world_model/actors.py.
With SAVE_PATH set (run_bench2drive.sh sets it), each route's ticks are logged to <SAVE_PATH>/<route>/expert.jsonl.
"""
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [p for p in (os.path.join(REPO, "build", "python"), REPO) if p not in sys.path]

import carla  # noqa: E402
from leaderboard.autoagents import autonomous_agent  # noqa: E402
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider  # noqa: E402

import expertctl  # noqa: E402
from expert.driver import ExpertDriver  # noqa: E402


def get_entry_point():
    return "ExpertAgent"


class ExpertAgent(autonomous_agent.AutonomousAgent):
    def setup(self, path_to_conf_file, *args, **kwargs):
        self.track = autonomous_agent.Track.SENSORS
        self.config = json.load(open(path_to_conf_file)) if path_to_conf_file and os.path.isfile(path_to_conf_file) else {}
        self.driver = None
        self.log_file = None

    def sensors(self):
        return [{"type": "sensor.speedometer", "reading_frequency": 20, "id": "speed"}]

    def _start(self):
        world = CarlaDataProvider.get_world()
        hero = CarlaDataProvider.get_hero_actor()
        save = os.environ.get("SAVE_PATH")
        log = None
        if save:
            route = os.path.basename(os.environ.get("ROUTES", "route")).replace(".xml", "")
            os.makedirs(os.path.join(save, route), exist_ok=True)
            self.log_file = open(os.path.join(save, route, "expert.jsonl"), "w")
            log = lambda row: self.log_file.write(json.dumps(row) + "\n")  # noqa: E731
        self.driver = ExpertDriver(expertctl, carla, world, hero, self.org_dense_route_world_coord,
                                   visibility=self.config.get("visibility", True), log=log)

    def run_step(self, input_data, timestamp):
        if self.driver is None:
            self._start()
        return self.driver.step(timestamp)

    def destroy(self, results=None):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        self.driver = None
