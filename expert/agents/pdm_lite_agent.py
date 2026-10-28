"""PDM-Lite (SimLingo's team_code/autopilot.py) as a Bench2Drive leaderboard agent: the baseline the expert is
compared with. The autopilot is unchanged; this wrapper only fits it to Bench2Drive's leaderboard, turns its
dataset writing off and records what it did.

autopilot.py subclasses carla_garage's `autonomous_agent_local`, which Bench2Drive's leaderboard does not have.
Its differences (the constructor, `run_step`'s sensors argument, the route downsampling) don't matter to the
autopilot, so the module is aliased to Bench2Drive's `autonomous_agent`, which also gives the `get_metric_info`
the evaluator expects.

Environment:
  PDM_OUT    a folder for steps.csv (each tick), route.json (the planned route and the map's lanes around it) and,
             with PDM_VIDEO=1, frames/ (a chase camera, every 5th tick, with what the autopilot is doing written on)
"""
import csv
import json
import os
import sys

import leaderboard.autoagents
from leaderboard.autoagents import autonomous_agent

sys.modules["leaderboard.autoagents.autonomous_agent_local"] = autonomous_agent
leaderboard.autoagents.autonomous_agent_local = autonomous_agent

import autopilot  # noqa: E402  (team_code/, on the leaderboard's PYTHONPATH)

FRAME_EVERY = 5  # 4 frames/s at 20 Hz
STEP_FIELDS = ["step", "t", "x", "y", "yaw", "speed", "target_speed", "speed_limit", "throttle", "steer", "brake",
               "junction", "vehicle_hazard", "light_hazard", "walker_hazard", "stop_sign_hazard", "changed_route",
               "reduced_by", "reduced_by_distance"]


def get_entry_point():
    return "PDMLiteAgent"


class PDMLiteAgent(autopilot.AutoPilot):
    def setup(self, path_to_conf_file, route_index=None, traffic_manager=None):
        # SAVE_PATH makes the autopilot write a dataset (and needs TOWN and REPETITION); the leaderboard reads it too
        save_path = os.environ.pop("SAVE_PATH", None)
        try:
            super().setup(path_to_conf_file, route_index, traffic_manager)
        finally:
            if save_path is not None:
                os.environ["SAVE_PATH"] = save_path
        self.out = os.environ.get("PDM_OUT")
        self.video = int(os.environ.get("PDM_VIDEO", 0)) == 1
        self._csv = None
        if self.out:
            os.makedirs(self.out, exist_ok=True)
            if self.video:
                os.makedirs(os.path.join(self.out, "frames"), exist_ok=True)
            self._csv_file = open(os.path.join(self.out, "steps.csv"), "w", newline="")
            self._csv = csv.DictWriter(self._csv_file, STEP_FIELDS)
            self._csv.writeheader()

    def sensors(self):
        specs = super().sensors()
        if self.video:  # behind and above the car (the leaderboard allows it when SAVE_PATH is set)
            specs.append({"type": "sensor.camera.rgb", "x": -7.0, "y": 0.0, "z": 3.5, "roll": 0.0, "pitch": -15.0,
                          "yaw": 0.0, "width": 960, "height": 540, "fov": 90, "id": "chase"})
        return specs

    def _init(self, hd_map):
        super()._init(hd_map)
        if self.out:
            self._dump_route()

    def _dump_route(self):
        """The planned route and the lane centre lines within 100 m of it, for the route map."""
        route = [[tr.location.x, tr.location.y] for tr, _ in self.org_dense_route_world_coord]
        xs, ys = [p[0] for p in route], [p[1] for p in route]
        x0, x1, y0, y1 = min(xs) - 100, max(xs) + 100, min(ys) - 100, max(ys) + 100

        def inside(loc):
            return x0 < loc.x < x1 and y0 < loc.y < y1

        lanes = []
        for a, b in self.world_map.get_topology():
            if not (inside(a.transform.location) or inside(b.transform.location)):
                continue
            pts = self._walk_lane(a, b)
            lanes.append({"junction": a.is_junction,
                          "xy": [[round(w.transform.location.x, 2), round(w.transform.location.y, 2)] for w in pts]})
        with open(os.path.join(self.out, "route.json"), "w") as f:
            json.dump({"town": self.world_map.name.split("/")[-1], "route": route, "lanes": lanes}, f)

    @staticmethod
    def _walk_lane(a, b, step=2.0, max_steps=500):
        """The waypoints from a topology segment's start a to its end b, `step` apart. CARLA's own
        next_until_lane_end crashes the client on some of Town12's lanes, so this walks with next()."""
        pts, w = [a], a
        end = b.transform.location
        for _ in range(max_steps):
            if w.transform.location.distance(end) <= step:
                break
            ahead = [n for n in w.next(step) if n.road_id == w.road_id and n.lane_id == w.lane_id]
            if not ahead:
                break
            w = ahead[0]
            pts.append(w)
        return pts + [b]

    def run_step(self, input_data, timestamp, sensors=None, plant=False):
        self.step += 1
        if not self.initialized:
            self._init(autopilot.CarlaDataProvider.get_client().get_world().get_map())
        control, data = self._get_control(input_data, plant)
        if self._csv is not None:
            self._record(input_data, timestamp, control, data)
        return control

    def _record(self, input_data, timestamp, control, d):
        tr = self._vehicle.get_transform()
        reduced = d["speed_reduced_by_obj_type"]
        row = {"step": self.step, "t": round(timestamp, 2), "x": round(tr.location.x, 2), "y": round(tr.location.y, 2),
               "yaw": round(tr.rotation.yaw, 1), "speed": round(d["speed"], 2), "target_speed": round(d["target_speed"], 2),
               "speed_limit": round(d["speed_limit"], 2), "throttle": round(control.throttle, 3),
               "steer": round(control.steer, 3), "brake": round(control.brake, 3), "junction": int(d["junction"]),
               "vehicle_hazard": int(d["vehicle_hazard"]), "light_hazard": int(d["light_hazard"]),
               "walker_hazard": int(d["walker_hazard"]), "stop_sign_hazard": int(d["stop_sign_hazard"]),
               "changed_route": int(d["changed_route"]), "reduced_by": reduced or "",
               "reduced_by_distance": "" if d["speed_reduced_by_obj_distance"] is None
               else round(d["speed_reduced_by_obj_distance"], 1)}
        self._csv.writerow(row)
        if self.video and self.step % FRAME_EVERY == 0 and "chase" in input_data:
            self._frame(input_data["chase"][1], row)

    def _frame(self, bgra, row):
        import cv2
        img = bgra[:, :, :3].copy()
        hazards = [k.replace("_hazard", "") for k in ("vehicle_hazard", "light_hazard", "walker_hazard",
                                                       "stop_sign_hazard") if row[k]]
        lines = [f"t {row['t']:6.1f} s   v {3.6 * row['speed']:5.1f} km/h   target {3.6 * row['target_speed']:5.1f} km/h",
                 f"throttle {row['throttle']:.2f}  brake {row['brake']:.2f}  steer {row['steer']:+.2f}",
                 ("hazard: " + ", ".join(hazards) if hazards else "no hazard")
                 + (f"   slowing for {row['reduced_by']} at {row['reduced_by_distance']} m" if row["reduced_by"] else "")
                 + ("   route shifted (overtaking)" if row["changed_route"] else "")]
        cv2.rectangle(img, (0, 0), (img.shape[1], 18 + 26 * len(lines)), (0, 0, 0), -1)
        for i, text in enumerate(lines):
            cv2.putText(img, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(self.out, "frames", f"{self.step:06d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])

    def destroy(self, results=None):
        if self._csv is not None:
            self._csv_file.close()
            self._csv = None
        super().destroy(results)
