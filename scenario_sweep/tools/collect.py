"""Collect every run's result into runs.json for the report."""
import glob, json, os, re
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
order = [l.split() for l in open(f"{root}/routes.txt")]
out = []
for i, (r, t) in enumerate(order, 1):
    name = f"{int(r):03d}_{t}"
    d = {"n": i, "route": int(r), "type": t, "name": name}
    f = glob.glob(f"{root}/runs/{name}/res/bench2drive_*.json")
    recs = json.load(open(f[0]))["_checkpoint"]["records"] if f else []
    if recs:
        rec = recs[0]
        d.update(status=rec["status"], score=round(rec["scores"]["score_composed"], 1),
                 route_pct=round(rec["scores"]["score_route"], 1), penalty=round(rec["scores"]["score_penalty"], 2),
                 game_s=rec["meta"].get("duration_game"),
                 infractions={k: v for k, v in rec["infractions"].items() if v})
    log = f"{root}/runs/{name}.log"
    if os.path.exists(log):
        m = re.search(r"Sync: ([\d.]+) s simulated in ([\d.]+) s wall", open(log, errors="ignore").read())
        if m: d["wall_s"] = float(m[2])
    d["video"] = os.path.exists(f"{root}/videos/{name}.mp4")
    d["frame"] = os.path.exists(f"{root}/frames/{name}.jpg")
    out.append(d)
json.dump(out, open(f"{root}/runs.json", "w"), indent=1)
for d in out:
    print(d["n"], d["route"], d["type"], d.get("score"), d.get("route_pct"), d.get("status", "")[:40], list(d.get("infractions", {}).keys()))
