#!/usr/bin/env python3
"""Run the planner again offline on the scenes a route logged (scenes.jsonl, expert/agents/simlingo_nmpc_agent.py),
with other planner options, to see what it would have chosen.

    scripts/replay_scene.py results/bench2drive/<run>/viz/bench2drive_10/scenes.jsonl 18 24
    scripts/replay_scene.py <scenes.jsonl> 18 24 gate_buffer=0.3 gate_margin=0.5

One planner steps through the scenes in order (its last choice carries on, as in the car); each plan prints the label
chosen, the candidates (label, solved, metres in 2 s) and the plan's speed every 0.4 s. The car does not move as the
replayed plans say: each scene is where the real car was. So a replay shows what would be chosen from there, not
where it would lead.
"""
import argparse
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "build", "python"))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import expertctl  # noqa: E402
from replay_problem import options, speeds  # noqa: E402


def scene(r):
    sc = expertctl.Scene()
    sc.t = r["st"]
    sc.ego = expertctl.Ego(*r["ego"])
    sc.v_max = r["v_max"]
    sc.v_ref_t, sc.v_ref = r["v_ref_t"], r["v_ref"]
    sc.line_s, sc.line_e = r["line_s"], r["line_e"]
    sc.lanes = [expertctl.LaneSection(*l) for l in r["lanes"]]
    actors = []
    for aid, kind, length, width, parked, stays_out, t, x, y, psi in r["actors"]:
        a = expertctl.Actor()
        a.id, a.kind = aid, getattr(expertctl.Actor.Kind, kind)
        a.length, a.width, a.parked, a.stays_out = length, width, parked, stays_out
        a.t, a.x, a.y, a.psi = t, x, y, psi
        actors.append(a)
    sc.actors = actors
    sc.stops = [expertctl.StopLine(i, getattr(expertctl.StopLine.Kind, k), s) for i, k, s in r["stops"]]
    sc.sight_left, sc.sight_right = r["sight"]
    sc.asks_change = r["asks_change"]
    return sc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("t0", type=float)
    ap.add_argument("t1", type=float)
    ap.add_argument("set", nargs="*", help="planner options, name=value")
    ap.add_argument("--notes", action="store_true", help="print the planner's notes too")
    ap.add_argument("--drop", default="", help="actor ids to leave out of every scene, comma separated")
    ap.add_argument("--unslack", type=float, default=0.0,
                    help="narrow the band by this much either side where it has no road edges (scenes logged on 13 "
                         "Nov before 06:00 had the junction slack in the band itself: 0.4)")
    a = ap.parse_args()
    rows = [json.loads(l) for l in open(a.log)]
    rows = [r for r in rows if a.t0 <= r["t"] <= a.t1]
    params = expertctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "mkz2020.json"))
    body = expertctl.Body(length=4.89, width=1.84 + 0.2, cg_to_center=0.31)
    opt = expertctl.PlannerOptions()
    opt.oncoming_gap = 5.0  # as the agent sets it
    for kv in a.set:
        k, v = kv.split("=")
        old = getattr(opt, k)
        setattr(opt, k, type(old)(float(v)) if not isinstance(old, bool) else v in ("1", "true", "True"))
    planner = None
    for r in rows:
        path = expertctl.Path.from_waypoints(r["path"], False)
        if planner is None:
            planner = expertctl.Planner(path, params, body, options(), opt)
        else:
            planner.set_route(path)
        if a.drop:
            gone = {int(i) for i in a.drop.split(",")}
            r = dict(r, actors=[x for x in r["actors"] if x[0] not in gone])
        if a.unslack:
            lanes = []
            for l in r["lanes"]:
                l = list(l)
                if not math.isfinite(l[7]) and not math.isfinite(l[8]):
                    l[2], l[3] = l[2] + a.unslack, l[3] - a.unslack
                lanes.append(l)
            r = dict(r, lanes=lanes)
        d = planner.step(scene(r))
        cands = " ".join(f"{c.label}{'' if c.plan.success else '!'}:{c.plan.x[-1][0] - c.plan.x[0][0]:.0f}m"
                         for c in d.candidates)
        print(f"{r['t']:6.2f} v {r['ego'][3]:4.2f} {d.label:14s} {speeds(d.plan.times, d.plan.x)}   [{cands}]")
        if a.notes:
            for n in d.notes:
                print("        ", n)


if __name__ == "__main__":
    main()
