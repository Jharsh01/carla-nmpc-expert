"""Score a folder of Bench2Drive routes (one sub-folder per route with the leaderboard's result.json, as
scripts/run_pdm_lite_route.sh writes them).

    python scripts/summarize_b2d.py <out dir>

Writes <out>/res/ (the results, linked under Bench2Drive's names) with merged.json from Bench2Drive's own
tools/merge_route_json.py (driving score and success rate as published), <out>/routes.csv (one row per route) and
<out>/summary.md (the totals, each scenario type, the infractions and the routes that went wrong).
"""
import collections
import csv
import glob
import json
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MERGE = os.path.join(REPO, "external", "simlingo", "Bench2Drive", "tools", "merge_route_json.py")
BROKEN = ("Simulation crashed", "Agent crashed", "Agent couldn't be set up")  # not a result of the driving


def main():
    out = os.path.abspath(sys.argv[1])
    rows, missing = [], []  # missing: routes run without a result (the simulator or the agent crashed)
    for path in sorted(glob.glob(os.path.join(out, "[0-9][0-9]*_*"))):
        folder = os.path.basename(path)
        index, scenario = folder.split("_", 1)
        try:
            rec = json.load(open(os.path.join(path, "result.json")))["_checkpoint"]["records"][0]
        except (OSError, ValueError, KeyError, IndexError):
            missing.append((int(index), scenario, "no result"))
            continue
        if any(b in rec["status"] for b in BROKEN):
            missing.append((int(index), scenario, rec["status"]))
            continue
        bad = {k: len(v) for k, v in rec["infractions"].items() if v and k != "min_speed_infractions"}
        rows.append({"route": int(index), "scenario": scenario, "town": rec.get("town_name", ""), "status": rec["status"],
                     "driving_score": round(rec["scores"]["score_composed"], 2),
                     "route_completion": round(rec["scores"]["score_route"], 2),
                     "penalty": round(rec["scores"]["score_penalty"], 3),
                     "success": int(rec["status"] in ("Completed", "Perfect") and not bad),
                     "infractions": "; ".join(f"{k} x{n}" if n > 1 else k for k, n in bad.items()),
                     "sim_s": rec["meta"].get("duration_game", 0.0), "wall_s": rec["meta"].get("duration_system", 0.0),
                     "folder": folder})
    if not rows:
        sys.exit(f"no results under {out}")
    rows.sort(key=lambda r: r["route"])

    with open(os.path.join(out, "routes.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    res_dir = os.path.join(out, "res")
    os.makedirs(res_dir, exist_ok=True)
    for old in glob.glob(os.path.join(res_dir, "bench2drive_*.json")):
        os.remove(old)
    for r in rows:
        link = os.path.join(res_dir, f"bench2drive_{r['route']:02d}.json")
        os.symlink(os.path.join("..", r["folder"], "result.json"), link)
    subprocess.run([sys.executable, MERGE, "-f", res_dir], check=True, stdout=subprocess.DEVNULL)
    merged = json.load(open(os.path.join(res_dir, "merged.json")))

    n = len(rows)
    lines = ["# PDM-Lite on Bench2Drive", "",
             f"- routes with a result: {n} of 220" + ("" if n == 220 else " (Bench2Drive's numbers are over all 220)")
             + (f"; {len(missing)} run without one (listed at the end)" if missing else ""),
             f"- **driving score {merged['driving score']:.2f}**, **success rate {100 * merged['success rate']:.1f} %** "
             "(Bench2Drive's merge_route_json.py)",
             f"- mean route completion {sum(r['route_completion'] for r in rows) / n:.1f} %, "
             f"mean penalty {sum(r['penalty'] for r in rows) / n:.3f}",
             f"- simulated {sum(r['sim_s'] for r in rows) / 60:.0f} min in {sum(r['wall_s'] for r in rows) / 3600:.1f} h "
             "of leaderboard time (server start-up not included)", ""]

    lines += ["## Each scenario type", "", "| scenario | routes | driving score | success | completion |",
              "|---|---:|---:|---:|---:|"]
    by = collections.defaultdict(list)
    for r in rows:
        by[r["scenario"]].append(r)
    for sc, rs in sorted(by.items(), key=lambda kv: sum(r["driving_score"] for r in kv[1]) / len(kv[1])):
        k = len(rs)
        lines.append(f"| {sc} | {k} | {sum(r['driving_score'] for r in rs) / k:.1f} | "
                     f"{sum(r['success'] for r in rs)}/{k} | {sum(r['route_completion'] for r in rs) / k:.0f} % |")

    counts = collections.Counter()
    for r in rows:
        for part in filter(None, r["infractions"].split("; ")):
            kind, _, times = part.partition(" x")
            counts[kind] += int(times or 1)
    lines += ["", "## Infractions (all routes)", ""]
    lines += [f"- {k}: {v}" for k, v in counts.most_common()] or ["- none"]

    failed = [r for r in rows if not r["success"]]
    lines += ["", f"## Routes that were not a success ({len(failed)})", "",
              "| route | scenario | score | completion | status | infractions |", "|---:|---|---:|---:|---|---|"]
    for r in sorted(failed, key=lambda r: r["driving_score"]):
        lines.append(f"| [{r['route']:02d}]({r['folder']}/summary.md) | {r['scenario']} | {r['driving_score']:.1f} | "
                     f"{r['route_completion']:.0f} % | {r['status']} | {r['infractions']} |")
    if missing:
        lines += ["", f"## Routes run without a result ({len(missing)}): not in the scores above", "",
                  "| route | scenario | why |", "|---:|---|---|"]
        lines += [f"| {i:02d} | {sc} | {why} |" for i, sc, why in sorted(missing)]
    lines += ["", "Each route's folder has its summary.md, map.png and video.mp4; routes.csv has a row per route."]
    open(os.path.join(out, "summary.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines[:6]))


if __name__ == "__main__":
    main()
