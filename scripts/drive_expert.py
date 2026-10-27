"""Drive the expert on a CARLA route outside the leaderboard: the M2 checks (lane keeping, stops at lights, no
contact), with or without traffic.

    scripts/start_carla.sh
    external/simlingo_venv/bin/python scripts/drive_expert.py --town Town10HD --length 600 [--traffic 30] [--seconds 120]

The route runs from a spawn point (--start, an index into the map's spawn points) to the spawn point whose route
from it is nearest --length metres long, planned by CARLA's GlobalRoutePlanner at 1 m, as the leaderboard's routes are.
The world runs synchronously at 20 Hz. The run ends at the route's end, at a collision, or after --seconds.
Results: results/drive/<town>_<start>[_traffic<N>].csv (each tick) and .json (the summary).
"""
import argparse
import csv
import json
import math
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(REPO, "build", "python"), REPO]
CARLA_ROOT = os.environ.get("CARLA_ROOT", os.path.join(REPO, "external", "carla"))
sys.path.append(os.path.join(CARLA_ROOT, "PythonAPI", "carla"))

import carla  # noqa: E402
import expertctl  # noqa: E402
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402

from expert.driver import ExpertDriver  # noqa: E402


def route_of_length(grp, carla_map, start, length):
    best = None
    for sp in carla_map.get_spawn_points():
        if not 0.15 * length < sp.location.distance(start.location) < 1.2 * length:  # routes wind in compact towns
            continue
        r = grp.trace_route(start.location, sp.location)
        d = sum(r[i][0].transform.location.distance(r[i + 1][0].transform.location) for i in range(len(r) - 1))
        if best is None or abs(d - length) < abs(best[1] - length):
            best = (r, d)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--town", default="Town10HD")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--length", type=float, default=600.0)
    ap.add_argument("--traffic", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=150.0)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--no-visibility", action="store_true", help="every actor in range known, seen or not")
    a = ap.parse_args()

    client = carla.Client("localhost", a.port)
    client.set_timeout(120.0)
    world = client.get_world()
    if a.town not in world.get_map().name:
        world = client.load_world(a.town)
    settings = world.get_settings()
    settings.synchronous_mode, settings.fixed_delta_seconds = True, 0.05
    world.apply_settings(settings)
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(1)
    carla_map = world.get_map()
    spawns = carla_map.get_spawn_points()
    start = spawns[a.start]
    grp = GlobalRoutePlanner(carla_map, 1.0)
    found = route_of_length(grp, carla_map, start, a.length)
    if found is None:
        sys.exit("no route of about that length from that start")
    route, length = found
    print(f"route: {len(route)} points, {length:.0f} m", file=sys.stderr)

    actors = []
    bp = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
    bp.set_attribute("role_name", "hero")
    hero = world.spawn_actor(bp, start)
    actors.append(hero)
    others = [b for b in world.get_blueprint_library().filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
    for i, sp in enumerate(spawns):
        if len(actors) > a.traffic or i == a.start or sp.location.distance(start.location) < 15.0:
            continue
        v = world.try_spawn_actor(others[i % len(others)], sp)
        if v is not None:
            v.set_autopilot(True, 8000)
            actors.append(v)
    hits = []
    sensor = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"), carla.Transform(), attach_to=hero)
    def on_hit(e):  # on CARLA's thread: nothing here may raise
        try:
            hits.append((e.other_actor.type_id, round(e.normal_impulse.length(), 1)))
        except Exception:
            hits.append(("unknown", 0.0))
    sensor.listen(on_hit)
    for _ in range(20):  # settle
        world.tick()

    name = f"{a.town}_{a.start}" + (f"_traffic{a.traffic}" if a.traffic else "")
    out_dir = os.path.join(REPO, "results", "drive")
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    t0 = time.time()
    driver = ExpertDriver(expertctl, carla, world, hero, [(wp.transform, opt) for wp, opt in route],
                          visibility=not a.no_visibility, log=rows.append)
    print(f"driver ready in {time.time() - t0:.1f} s", file=sys.stderr)
    t_start = world.get_snapshot().timestamp.elapsed_seconds
    end = "time"
    wall = time.time()
    try:
        while True:
            t = world.get_snapshot().timestamp.elapsed_seconds - t_start
            hero.apply_control(driver.step(t))
            world.tick()
            if hits:
                end = "collision"
                break
            if rows and rows[-1]["s"] > driver.route.length - 6.0 and rows[-1]["v"] < 0.2:
                end = "route end"
                break
            if t > a.seconds:
                break
    finally:
        # the Traffic Manager runs in this process and drives its cars until told not to: off first, then destroy
        sensor.stop()
        for x in actors[1:]:
            x.set_autopilot(False, 8000)
        world.tick()
        client.apply_batch_sync([carla.command.DestroyActor(x) for x in [sensor] + actors], True)
        tm.set_synchronous_mode(False)
        settings.synchronous_mode = False
        world.apply_settings(settings)

    with open(os.path.join(out_dir, name + ".csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[k for k in rows[0] if k != "notes"], extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    steps = sorted(r["step_ms"] for r in rows if r["step_ms"] > 0)
    moving = [r for r in rows if r["v"] > 1.0]
    summary = {
        "route_m": round(length, 1), "end": end, "collision": hits[:1], "sim_s": round(rows[-1]["t"], 1),
        "wall_s": round(time.time() - wall, 1), "progress_m": round(rows[-1]["s"], 1),
        "max_abs_e_y": round(max(abs(r["e_y"]) for r in moving), 2) if moving else None,
        "p95_abs_e_y": round(sorted(abs(r["e_y"]) for r in moving)[int(0.95 * (len(moving) - 1))], 2) if moving else None,
        "mean_speed": round(sum(r["v"] for r in rows) / len(rows), 2),
        "step_ms_p50": round(steps[len(steps) // 2], 1), "step_ms_p95": round(steps[int(0.95 * (len(steps) - 1))], 1),
        "step_ms_max": round(steps[-1], 1),
        "failed_solves": sum(1 for r in rows if r["step_ms"] > 0 and not r["success"]),
        "labels": sorted({r["label"] for r in rows}),
        "notes": sorted({n for r in rows for n in r["notes"] if not n.startswith(("blocker at", "crosser", "cut-in"))})[:20],
    }
    json.dump(summary, open(os.path.join(out_dir, name + ".json"), "w"), indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
