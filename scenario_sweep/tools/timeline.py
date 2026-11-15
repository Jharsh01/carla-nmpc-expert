"""Timeline of a run around its first infraction (or the stop): python timeline.py runs/<name>"""
import glob, json, re, sys, math
run = sys.argv[1]
rec = json.load(open(glob.glob(f"{run}/res/bench2drive_*.json")[0]))["_checkpoint"]["records"][0]
rows = [json.loads(l) for l in open(glob.glob(f"{run}/viz/bench2drive_*/nmpc.jsonl")[0])]
ev = []
for k, v in rec["infractions"].items():
    for m in v:
        g = re.search(r"x=(-?[\d.]+), y=(-?[\d.]+)", m)
        idm = re.search(r"id=(\d+)", m)
        if g: ev.append((k, float(g[1]), float(g[2]), int(idm[1]) if idm else None, m))
def near(x, y):
    return min(rows, key=lambda r: math.hypot(r["pose"][0] - x, r["pose"][1] + y))  # pose y is flipped
print("status:", rec["status"], "score", round(rec["scores"]["score_composed"], 1), "route", rec["scores"]["score_route"])
moved = [r for r in rows if r["v"] > 0.5]
print("first moving t:", moved[0]["t"] if moved else None, " last moving t:", moved[-1]["t"] if moved else None,
      " end t:", rows[-1]["t"], " failed plans:", sum(not r["success"] for r in rows), "/", len(rows))
for k, x, y, aid, m in ev:
    r = near(x, y)
    print(f"\n## {k} near t={r['t']}: {m[:150]}")
    t0 = r["t"]
    for q in rows:
        if t0 - 2.5 <= q["t"] <= t0 + 0.5 and round(q["t"] * 20) % 5 == 0:
            a = next((s for s in q.get("scene", {}).get("actors", []) if s[0] == aid), None) if aid else None
            print(f"  t={q['t']:6.2f} v={q['v']:5.2f} a={q['a']:6.2f} {q['label'][:28]:28s} ok={int(q['success'])} hold={int(q['holding'])}"
                  f" notes={','.join(map(str, q['notes']))[:70]}" + (f" | id{aid} rel=({a[3]:.1f},{a[4]:.1f})" if a else ""))
if not ev and rows:
    print("\nno located infraction; last 3 s:")
    for q in rows[-60::4]:
        print(f"  t={q['t']:6.2f} v={q['v']:5.2f} a_ref={q['a_ref']:5.2f} {q['label'][:28]:28s} ok={int(q['success'])} hold={int(q['holding'])} notes={','.join(map(str, q['notes']))[:90]} gates={(q.get('asked') or {}).get('gates')}")
