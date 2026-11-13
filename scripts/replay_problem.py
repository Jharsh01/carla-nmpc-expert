#!/usr/bin/env python3
"""Solve a plan's problem again offline, from the agent's problems.jsonl (expert/agents/simlingo_nmpc_agent.py), and
again without each kind of row (movers, leads, gates, chasers, the speed asked for), to see which one holds the plan.

    scripts/replay_problem.py results/bench2drive/<run>/viz/bench2drive_08/problems.jsonl 10.5 11.5
    scripts/replay_problem.py <problems.jsonl> 11.15 --only        the plan nearest 11.15 s only

For each plan: its speed every 0.4 s as driven (logged), as solved here, and as solved without each kind. The NMPC is
set up as the agent sets it up (2 s, 20 x 50 ms then 5 x 200 ms, braking at 4.6 m/s^2), cold: the agent's solver is
warm, so a replay can differ a little where the problem has more than one answer.
"""
import argparse
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "build", "python"))
import expertctl  # noqa: E402


def options():
    o = expertctl.UrbanOptions()
    o.a_min = -4.6
    o.horizon, o.fine = 2.0, 1.0
    o.h_fine, o.h = 0.05, 0.2
    o.jit = True
    o.cache_dir = os.path.join(REPO, "build", "urban_cache")
    return o


def problem(r, drop=()):
    pr = expertctl.Problem()
    pr.t = r["pt"]
    pr.ego = expertctl.Ego(*r["ego"])
    pr.corridor = expertctl.Corridor(*r["corridor"])
    pr.v_max = r["v_max"]
    if "v_ref" not in drop:
        pr.v_ref_t, pr.v_ref = r["v_ref_t"], r["v_ref"]
    movers = []
    for t, s, e, radius, growth, margin in r["movers"]:
        m = expertctl.Mover(t, s, e, radius, growth)
        m.margin = math.nan if margin is None else margin
        movers.append(m)
    pr.movers = [] if "movers" in drop else movers
    pr.leads = [] if "leads" in drop else [expertctl.Lead(t, s) for t, s in r["leads"]]
    gates = []
    for s, t_from, t_to, before, route_end, taper in r["gates"]:
        g = expertctl.Gate(s, t_from, math.inf if t_to is None else t_to, before)
        g.route_end, g.taper = route_end, taper
        gates.append(g)
    pr.gates = [] if "gates" in drop else gates
    pr.chasers = [] if "chasers" in drop else [expertctl.Chaser(t, s, t_from) for t, s, t_from in r["chasers"]]
    for name in ("a_lat_ref", "follow_gap", "follow_time_gap", "a_max", "w_accel", "w_lat"):
        v = r.get(name)  # (w_accel, w_lat: logged from 13 Nov, 08:00)
        setattr(pr, name, math.nan if v is None else v)
    return pr


def speeds(times, x, every=0.4):
    out, nxt = [], 0.0
    for t, row in zip(times, x):
        if t >= nxt - 1e-6:
            out.append(row[3])
            nxt += every
    return " ".join(f"{v:4.1f}" for v in out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("t0", type=float)
    ap.add_argument("t1", type=float, nargs="?")
    ap.add_argument("--only", action="store_true", help="the plan nearest t0 only")
    a = ap.parse_args()
    rows = [json.loads(l) for l in open(a.log)]
    if a.only or a.t1 is None:
        rows = [min(rows, key=lambda r: abs(r["t"] - a.t0))]
    else:
        rows = [r for r in rows if a.t0 <= r["t"] <= a.t1]
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    body = expertctl.Body(length=4.89, width=1.84 + 0.2, cg_to_center=0.31)
    for r in rows:
        path = expertctl.Path.from_waypoints(r["path"], False)
        print(f"t {r['t']:.2f} {r['label']}  v {r['ego'][3]:.2f}  movers {len(r['movers'])} leads {len(r['leads'])} "
              f"gates {len(r['gates'])} chasers {len(r['chasers'])}  (speed every 0.4 s)")
        print(f"  {'driven':10s} {speeds(r['plan']['times'], r['plan']['x'])}  ok {r['plan']['success']}")
        for drop in ((), ("movers",), ("leads",), ("gates",), ("chasers",), ("v_ref",)):
            nmpc = expertctl.UrbanNMPC(path, params, body, options())
            plan = nmpc.solve(problem(r, drop))
            name = "replay" if not drop else "no " + drop[0]
            print(f"  {name:10s} {speeds(plan.times, plan.x)}  ok {plan.success}")


if __name__ == "__main__":
    main()
