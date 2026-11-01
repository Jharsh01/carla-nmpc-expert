"""Where a route's wall-clock time goes, per tick, from the evaluator's timing.jsonl (expert/timing.py).

    python scripts/timing_report.py results/bench2drive/<mode>/seed1/viz/bench2drive_00/timing.jsonl [--csv out.csv]

Prints each stage's time per tick (mean, p50, p95, max, and its share of the tick), as a tree: a stage's time
includes its children's, "(other)" is what is left of it; SimLingo's model and the NMPC's solve run on their own
CPUs, so the tick sees only the waits for them (run_step.simlingo_wait, nmpc.wait), and their own times are listed
after. Then the same over the ticks of each kind (a SimLingo prediction taken or not; the NMPC planned or not), and
the slowest ticks. The first tick that plans sets up the solver and is left
out of the statistics (it is listed on its own). --csv writes the tick-by-tick table.
"""
import argparse
import json

import numpy as np

TREE = {
    "total": ["carla.world_tick", "carla.data_provider", "agent", "scenario_tree"],
    "agent": ["agent.sensors", "agent.run_step"],
    "agent.run_step": ["run_step.measure", "run_step.simlingo_send", "run_step.simlingo_wait", "run_step.nmpc",
                       "run_step.metric"],
    "run_step.nmpc": ["nmpc.path", "nmpc.set_route", "nmpc.detect", "nmpc.lanes", "nmpc.scene_actors", "nmpc.stops",
                      "nmpc.wait", "nmpc.live", "nmpc.describe"],
}


def order():
    """The stages depth first, with their depth, and an "(other)" after each parent's children."""
    out = []

    def visit(name, depth):
        out.append((name, depth))
        if name in TREE:
            for child in TREE[name]:
                visit(child, depth + 1)
            out.append((name + " (other)", depth + 1))

    visit("total", 0)
    return out


def columns(rows):
    """Each stage's time per tick (0 where it did not run), and the "(other)" remainders."""
    cols = {}
    for name, _ in order():
        if not name.endswith(" (other)"):
            cols[name] = np.array([r.get(name, 0.0) for r in rows])
    for parent, children in TREE.items():
        cols[parent + " (other)"] = cols[parent] - sum(cols[c] for c in children)
    return cols


def table(rows, title):
    if not rows:
        return
    cols = columns(rows)
    total = cols["total"].sum()
    print(f"\n{title}: {len(rows)} ticks, {total / 1e3:.1f} s wall clock, {total / len(rows):.0f} ms a tick"
          f" ({0.05 * len(rows) / (total / 1e3):.3f}x real time)")
    print(f"{'stage [ms per tick]':<34}{'mean':>8}{'p50':>8}{'p95':>8}{'max':>8}{'share':>8}")
    for name, depth in order():
        c = cols[name]
        if not c.any():
            continue
        print(f"{'  ' * depth + name:<34}{c.mean():8.1f}{np.median(c):8.1f}{np.percentile(c, 95):8.1f}{c.max():8.1f}"
              f"{100 * c.sum() / total:7.1f}%")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log")
    parser.add_argument("--csv")
    parser.add_argument("--slowest", type=int, default=10)
    args = parser.parse_args()
    rows = [json.loads(line) for line in open(args.log)]
    rows = [r for r in rows if "agent" in r]  # ticks the agent ran in
    setup = next((i for i, r in enumerate(rows) if r.get("planned")), None)
    if setup is not None:
        r = rows[setup]
        print(f"first plan (solver set-up), tick {r['tick']}: {r['total']:.0f} ms, of which nmpc.set_route "
              f"{r.get('nmpc.set_route', 0):.0f} ms")
    steady = [r for i, r in enumerate(rows) if i != setup]
    table(steady, "all ticks")
    off = {"SimLingo's preprocessing (its worker)": "simlingo.preprocess_ms", "SimLingo's model (its worker)":
           "simlingo.model_ms", "the NMPC's behaviour step (its thread)": "solve_ms"}
    for label, key in off.items():
        c = np.array([r[key] for r in steady if key in r])
        if len(c):
            print(f"{label:<40} p50 {np.median(c):6.1f}  p95 {np.percentile(c, 95):6.1f}  max {c.max():6.1f} ms "
                  f"({len(c)} times)")
    table([r for r in steady if r.get("simlingo_received")], "ticks a SimLingo prediction was taken in")
    table([r for r in steady if not r.get("simlingo_received") and r.get("planned")], "no prediction, the NMPC planned")
    table([r for r in steady if not r.get("simlingo_received") and not r.get("planned")], "no prediction, no plan")
    leaves = [n for n, _ in order() if n not in TREE and n != "total"]
    print(f"\nslowest {args.slowest} ticks:")
    cols = columns(steady)
    for i in np.argsort(-cols["total"])[:args.slowest]:
        r = steady[i]
        top = sorted(((cols[n][i], n) for n in leaves), reverse=True)[:3]
        print(f"  tick {r['tick']:5d} (t {r['t']:6.2f} s): {r['total']:7.1f} ms; "
              + ", ".join(f"{n} {v:.0f}" for v, n in top))
    if args.csv:
        cols = columns(rows)
        names = list(cols)
        with open(args.csv, "w") as f:
            f.write("tick,t,simlingo_received,planned,solve_ms," + ",".join(names) + "\n")
            for i, r in enumerate(rows):
                f.write(f"{r['tick']},{r['t']},{int(bool(r.get('simlingo_received')))},{int(bool(r.get('planned')))},"
                        f"{r.get('solve_ms', '')}," + ",".join(f"{cols[n][i]:.2f}" for n in names) + "\n")
        print(f"\ntick by tick: {args.csv}")


if __name__ == "__main__":
    main()
