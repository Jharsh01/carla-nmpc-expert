"""Measure how much lateral acceleration Bench2Drive's ego car (vehicle.lincoln.mkz_2020, CARLA 0.9.15, default
physics) can hold before it slides: the grip the NMPC's friction circle (VehicleParams::mu) may use.

    scripts/start_carla.sh && external/simlingo_venv/bin/python scripts/identify_grip.py [--out results/identify]
    external/simlingo_venv/bin/python scripts/identify_grip.py --fit-only        # summarise the recorded runs

On Town06's longest straight (scripts/identify_ego.py's rig), for each speed the car is brought up to it at rest
first (see identify_ego.py on teleporting), held a second, and then the steer is stepped to a level and held
T_STEP s with the speed held by a PI loop on the throttle (the speed still drops in hard turns: what the car keeps
is recorded). Each run's settled values, over its last SETTLE s: speed, lateral acceleration (from the car's
acceleration, across its heading; and speed x yaw rate) and sideslip at the centre of mass. The grip is where the
lateral acceleration stops rising with the steer while the sideslip grows.

The raw ticks go to <out>/grip_runs.csv, the summary to <out>/grip.json; also the tyre friction CARLA has for the
car's wheels.
"""
import argparse
import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from identify_ego import DT, REPO, Rig, held  # noqa: E402

SPEEDS = [6.0, 8.0, 10.0, 12.0, 14.0]
STEERS = [0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0]
T_STEP, SETTLE = 1.0, 0.4  # [s] (held 2 s, the car left the road and struck the barriers at every speed over 6 m/s)
G = 9.81


def state(rig):
    tr = rig.car.get_transform()
    yaw = math.radians(tr.rotation.yaw)
    v, acc = rig.car.get_velocity(), rig.car.get_acceleration()
    c, s = math.cos(yaw), math.sin(yaw)
    ux, uy = v.x * c + v.y * s, -v.x * s + v.y * c  # along, across (CARLA's frame: y to the right)
    ax_lat = -acc.x * s + acc.y * c
    r = math.radians(rig.car.get_angular_velocity().z)  # CARLA's yaw rate (clockwise positive, as y to the right)
    return ux, uy, ax_lat, r


def wide_side(rig):
    """+1 to steer right, -1 left: towards the side with more lanes the car's way and beyond."""
    wp = rig.world.get_map().get_waypoint(rig.start.location)
    def count(step):
        n, w = 0, step(wp)
        while w is not None and n < 8 and w.lane_type == rig.carla.LaneType.Driving:
            n, w = n + 1, step(w)
        return n
    right, left = count(lambda w: w.get_right_lane()), count(lambda w: w.get_left_lane())
    print(f"lanes right {right} left {left}", file=sys.stderr)
    return 1.0 if right >= left else -1.0


def run(rig, v, steer, rows, side, hits):
    hold = held(v)
    rig.reset(0.0)
    hits.clear()
    t, at = 0.0, None
    while t < 40.0:
        ux, uy, a_lat, r = state(rig)
        if at is None and abs(ux - v) < 0.3 and t > 1.0:
            at = t + 1.0
        th, br = hold(ux)
        st = side * steer if at is not None and t >= at else 0.0
        rig.car.apply_control(rig.carla.VehicleControl(throttle=th, brake=br, steer=st))
        rows.append({"v": v, "steer": steer, "t": round(t, 3), "since": "" if at is None or t < at else round(t - at, 3),
                     "ux": ux, "uy": uy, "a_lat": a_lat, "r": r, "throttle": th, "brake": br, "hit": int(bool(hits))})
        if at is not None and t >= at + T_STEP:
            break
        rig.world.tick()
        t += DT


def summarise(rows):
    runs = {}
    for r in rows:
        if r["since"] != "" and float(r["since"]) >= T_STEP - SETTLE:
            runs.setdefault((float(r["v"]), float(r["steer"])), []).append(r)
    out = []
    for (v, st), ticks in sorted(runs.items()):
        if any(int(t.get("hit", 0)) for t in ticks):
            continue  # struck something before it settled
        m = lambda k: sum(float(t[k]) for t in ticks) / len(ticks)  # noqa: E731
        ux, uy = m("ux"), m("uy")
        out.append({"v_target": v, "steer": st, "speed": round(math.hypot(ux, uy), 2),
                    "a_lat": round(abs(m("a_lat")), 2), "v_r": round(abs(math.hypot(ux, uy) * m("r")), 2),
                    "beta_deg": round(math.degrees(math.atan2(abs(uy), max(ux, 0.1))), 1)})
    peak = max(out, key=lambda o: o["a_lat"]) if out else None
    return {"runs": out, "peak": peak, "mu_peak": round(peak["a_lat"] / G, 3) if peak else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=os.path.join(REPO, "results", "identify"))
    ap.add_argument("--town", default="Town06")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--fit-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    raw = os.path.join(a.out, "grip_runs.csv")
    wheels = None
    if not a.fit_only:
        import carla
        client = carla.Client("localhost", a.port)
        client.set_timeout(120.0)
        rig = Rig(carla, client, a.town)
        rows = []
        try:
            pc = rig.car.get_physics_control()
            wheels = [{"tire_friction": w.tire_friction, "max_steer_angle": w.max_steer_angle,
                       "lat_stiff_value": getattr(w, "lat_stiff_value", None)} for w in pc.wheels]
            print("wheels:", wheels, "mass", pc.mass, file=sys.stderr)
            side, hits = wide_side(rig), []
            sensor = rig.world.spawn_actor(rig.world.get_blueprint_library().find("sensor.other.collision"),
                                           carla.Transform(), attach_to=rig.car)
            sensor.listen(lambda e: hits.append(e.frame))
            for v in SPEEDS:
                for st in STEERS:
                    run(rig, v, st, rows, side, hits)
                    print(f"v {v} steer {st} done", file=sys.stderr)
        finally:
            if "sensor" in locals():
                sensor.destroy()
            rig.close()
        with open(raw, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    with open(raw) as f:
        result = summarise(list(csv.DictReader(f)))
    if wheels is not None:
        result["wheels"] = wheels
    with open(os.path.join(a.out, "grip.json"), "w") as f:
        json.dump(result, f, indent=1)
    for o in result["runs"]:
        print(o)
    print("peak", result["peak"], "mu", result["mu_peak"])


if __name__ == "__main__":
    main()
