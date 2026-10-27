"""Measure how Bench2Drive's ego car (vehicle.lincoln.mkz_2020, CARLA 0.9.15, default physics) responds to throttle,
brake and steer, for the expert's actuation (expert/actuation.py).

    scripts/start_carla.sh && external/simlingo_venv/bin/python scripts/identify_ego.py [--out results/identify]
    external/simlingo_venv/bin/python scripts/identify_ego.py --fit-only        # refit from the recorded runs

The car runs in synchronous mode at 20 Hz, as in the leaderboard, on the longest straight of Town06. Each test
starts with the car teleported to the start of the straight at rest, driven up to the test's speed and held there
by a PI loop on the throttle for a second (teleporting it at speed would not do: the body moves but the wheels do
not turn, and the tyres skid until they do), and then:

    throttle   a throttle step held 1.5 s: the acceleration averaged from 0.3 s on
    coast      no throttle and no brake for 1.5 s, the same way
    brake      a brake step held 0.6 s, averaged from 0.2 s on (a stop is quick)
    steer      a steer step held 1.5 s: the yaw rate it settles to gives the road-wheel angle a kinematic bicycle
               would need for it, and its rise the steering lag

The raw ticks go to <out>/runs.csv; the fit to config/vehicles/mkz2020_actuation.json:

    accel[throttle][speed], accel[brake][speed], coast[speed]   tables of the longitudinal acceleration [m/s^2]
    steer_gain[speed]                                           road-wheel angle per unit steer [rad]
    steer_tau                                                   first-order lag from steer to yaw rate [s]
"""
import argparse
import csv
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DT = 0.05
SPEEDS = [0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0]
THROTTLES = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
BRAKES = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
BRAKE_STARTS = [5.0, 10.0, 15.0, 20.0, 25.0]
STEER_SPEEDS = [3.0, 6.0, 10.0, 15.0, 20.0]
STEERS = [0.02, 0.05, 0.1, 0.2, 0.3, 0.5]
A_LAT_MAX = 6.0  # steer tests that would turn harder than this [m/s^2] are skipped
WHEELBASE = 2.86  # [m], from the wheel positions


def longest_straight(world_map):
    """Start transform and length of the longest stretch of lane that keeps its heading within 1 degree."""
    best = (None, 0.0)
    for wp in world_map.generate_waypoints(50.0):
        if wp.is_junction:
            continue
        start, length, yaw0, cur = wp, 0.0, wp.transform.rotation.yaw, wp
        while length < 1500.0:
            nxt = cur.next(5.0)
            if len(nxt) != 1:
                break
            cur = nxt[0]
            if abs((cur.transform.rotation.yaw - yaw0 + 180.0) % 360.0 - 180.0) > 1.0:
                break
            length += 5.0
        if length > best[1]:
            best = (start.transform, length)
    return best


class Rig:
    def __init__(self, carla, client, town):
        self.carla = carla
        self.world = client.load_world(town) if town not in client.get_world().get_map().name else client.get_world()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        self.world.apply_settings(settings)
        self.start, self.length = longest_straight(self.world.get_map())
        print(f"{town}: straight of {self.length:.0f} m", file=sys.stderr)
        self.start.location.z += 0.3
        bp = self.world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
        self.car = self.world.spawn_actor(bp, self.start)
        self.world.tick()
        self.rows = []

    def close(self):
        self.car.destroy()
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)

    def reset(self, speed):
        """At the start of the straight, at rest (speed must be 0: see the module's notes)."""
        self.car.apply_control(self.carla.VehicleControl(throttle=0.0, brake=1.0))
        self.car.set_target_velocity(self.carla.Vector3D())
        self.car.set_target_angular_velocity(self.carla.Vector3D())
        self.car.set_transform(self.start)
        for _ in range(10):
            self.world.tick()
        self.car.apply_control(self.carla.VehicleControl(throttle=0.0, brake=0.0))
        self.world.tick()

    def state(self):
        tr = self.car.get_transform()
        v = self.car.get_velocity()
        yaw = math.radians(tr.rotation.yaw)
        ux = v.x * math.cos(yaw) + v.y * math.sin(yaw)  # along the car
        along = (tr.location.x - self.start.location.x) * math.cos(math.radians(self.start.rotation.yaw)) + \
                (tr.location.y - self.start.location.y) * math.sin(math.radians(self.start.rotation.yaw))
        # CARLA's yaw is clockwise (left-handed frame): the library's yaw rate is minus it
        return ux, -math.radians(self.car.get_angular_velocity().z), along

    def run(self, test, level, speed, control, until, max_t, step_start=lambda: None):
        """Tick with control(t, ux) -> (throttle, brake, steer) until until(t, ux, along) or max_t; record each tick,
        with its time since the step (step_start(), None before it is known)."""
        self.reset(0.0)
        t = 0.0
        while t < max_t:
            ux, r, along = self.state()
            th, br, st = control(t, ux)
            self.car.apply_control(self.carla.VehicleControl(throttle=th, brake=br, steer=st))
            at = step_start()
            since = "" if at is None or t < at else round(t - at, 3)
            self.rows.append({"test": test, "level": level, "start": speed, "t": round(t, 3), "since": since, "ux": ux, "r": r,
                              "throttle": th, "brake": br, "steer": st, "gear": self.car.get_control().gear})
            if until(t, ux, along):
                break
            self.world.tick()
            t += DT


def held(v):
    """Throttle and brake that drive the car up to speed v and hold it there (a PI loop on the speed)."""
    integral = [0.0]

    def control(u):
        e = v - u
        integral[0] = max(-2.0, min(2.0, integral[0] + e * DT))
        th = max(0.0, min(1.0, 0.4 * e + 0.2 * integral[0] + 0.02 * v))
        br = max(0.0, min(1.0, -0.2 * e)) if e < -1.0 else 0.0
        return th, br
    return control


def step_test(rig, test, level, v, step, t_step):
    """Up to speed v, held for a second there, then the step (throttle, brake, steer) for t_step seconds."""
    hold = held(v)
    phase = {"at": None}

    def control(t, u):
        if phase["at"] is None and abs(u - v) < 0.3 and t > 1.0:
            phase["at"] = t + 1.0  # a second steady before the step
        if phase["at"] is not None and t >= phase["at"]:
            return step(t - phase["at"])
        return hold(u) + (0.0,)

    rig.run(test, level, v, control,
            lambda t, u, s: (phase["at"] is not None and t >= phase["at"] + t_step) or s > rig.length - 40.0, 60.0,
            step_start=lambda: phase["at"])


def record(rig):
    for v in [2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0]:
        step_test(rig, "coast", 0.0, v, lambda t: (0.0, 0.0, 0.0), 1.5)
        for th in THROTTLES:
            step_test(rig, "throttle", th, v, lambda t, th=th: (th, 0.0, 0.0), 1.5)
        for br in BRAKES:
            step_test(rig, "brake", br, v, lambda t, br=br: (0.0, br, 0.0), 0.6)
    for th in THROTTLES:  # from standstill
        rig.run("throttle", th, 0.0, lambda t, u, th=th: (th, 0.0, 0.0), lambda t, u, s: t >= 1.5, 1.5,
                step_start=lambda: 0.0)
    for v in STEER_SPEEDS:
        for st in STEERS:
            if v * v * math.tan(st * math.radians(70.0) * 0.9) / WHEELBASE > A_LAT_MAX:
                continue
            hold = held(v)
            step_test(rig, "steer", st, v, lambda t, st=st, hold=hold: (0.3, 0.0, st), 1.5)


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    return None if n == 0 else (xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2]))


def fit(rows):
    """Tables from the recorded ticks: each step test gives one (speed at the step, mean acceleration) sample."""
    runs = {}
    for r in rows:
        runs.setdefault((r["test"], float(r["level"]), float(r["start"])), []).append(r)
    window = {"throttle": (0.3, 1.5), "coast": (0.3, 1.5), "brake": (0.2, 0.6)}

    def step_accel(test, ticks):
        lo, hi = window[test]
        inside = [t for t in ticks if t["since"] != "" and lo <= float(t["since"]) <= hi]
        if len(inside) < 3:
            return None
        dt = float(inside[-1]["t"]) - float(inside[0]["t"])
        speed_at_step = float(next(t for t in ticks if t["since"] != "")["ux"])
        return speed_at_step, (float(inside[-1]["ux"]) - float(inside[0]["ux"])) / dt

    def table(test, levels):
        out = {}
        for lv in levels:
            row = []
            for v in SPEEDS:
                samples = [step_accel(test, ticks) for (tt, l, s), ticks in runs.items()
                           if tt == test and abs(l - lv) < 1e-9 and abs(s - v) < 1e-9]
                samples = [a for a in samples if a is not None]
                row.append(round(samples[0][1], 3) if samples else None)
            out[str(lv)] = row
        return out

    result = {"speeds": SPEEDS, "throttle": table("throttle", THROTTLES), "brake": table("brake", BRAKES)}
    result["coast"] = table("coast", [0.0])["0.0"]

    # steering: settled yaw rate over the last 0.5 s of each step -> kinematic road-wheel angle
    gain, rises = {}, []
    for (tt, st, v), ticks in runs.items():
        if tt != "steer":
            continue
        stepped = [t for t in ticks if t["since"] != ""]
        late = [t for t in stepped if float(t["since"]) >= 1.0]
        r = median([float(t["r"]) for t in late])
        u = median([float(t["ux"]) for t in late])
        if not r or not u or u < 1.0:
            continue
        gain.setdefault(v, []).append(math.atan(WHEELBASE * r / u) / st)
        for t in stepped:  # the lag: time for the yaw rate to reach 63% of where it settles
            if abs(float(t["r"])) >= 0.63 * abs(r):
                rises.append(float(t["since"]))
                break
    result["steer_speeds"] = sorted(gain)
    result["steer_gain"] = [round(median(gain[v]), 4) for v in sorted(gain)]
    result["steer_gain_by_level"] = {str(v): [round(g, 4) for g in gain[v]] for v in sorted(gain)}
    result["steer_tau"] = round(median(rises), 3) if rises else None
    result["wheelbase"] = WHEELBASE
    result["dt"] = DT
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=os.path.join(REPO, "results", "identify"))
    ap.add_argument("--town", default="Town06")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--fit-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    raw = os.path.join(a.out, "runs.csv")
    if not a.fit_only:
        import carla
        client = carla.Client("localhost", a.port)
        client.set_timeout(120.0)
        rig = Rig(carla, client, a.town)
        try:
            record(rig)
        finally:
            rig.close()
        with open(raw, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rig.rows[0]))
            w.writeheader()
            w.writerows(rig.rows)
        print(f"{len(rig.rows)} ticks -> {raw}", file=sys.stderr)
    with open(raw) as f:
        rows = list(csv.DictReader(f))
    result = fit(rows)
    path = os.path.join(REPO, "config", "vehicles", "mkz2020_actuation.json")
    with open(path, "w") as f:
        json.dump(result, f, indent=1)
    print(path)


if __name__ == "__main__":
    main()
