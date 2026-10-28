"""After a Bench2Drive route: the route map, the video and a summary, from what the agent and the leaderboard wrote.

    python scripts/route_report.py <run dir> <route xml>

<run dir> holds steps.csv, route.json and frames/ (expert/agents/pdm_lite_agent.py) and result.json (the
leaderboard's checkpoint). Writes map.png (the map's lanes, the planned route, the path driven coloured by speed,
the scenario's trigger and the infractions), video.mp4 (the frames at real time) and summary.md.
"""
import csv
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

FPS = 4  # the agent's frames: every 5th tick at 20 Hz


def infraction_points(record):
    """(kind, text, x, y) of each infraction whose text carries a location."""
    out = []
    for kind, texts in record["infractions"].items():
        for text in texts:
            m = re.search(r"x=(-?[\d.]+), y=(-?[\d.]+)", text)
            if m:
                out.append((kind, text, float(m.group(1)), float(m.group(2))))
    return out


def route_map(run, steps, record, scenario, path):
    rj = json.load(open(os.path.join(run, "route.json")))
    route = np.array(rj["route"])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in steps])
    v = 3.6 * np.array([float(r["speed"]) for r in steps])
    pad = 25
    lo = np.minimum(route.min(axis=0), xy.min(axis=0)) - pad
    hi = np.maximum(route.max(axis=0), xy.max(axis=0)) + pad
    if (hi - lo)[1] < 0.35 * (hi - lo)[0]:  # a long straight route: keep some road either side of it
        c = 0.5 * (lo[1] + hi[1])
        lo[1], hi[1] = c - 0.175 * (hi - lo)[0], c + 0.175 * (hi - lo)[0]
    aspect = (hi - lo)[1] / (hi - lo)[0]
    fig, ax = plt.subplots(figsize=(11, max(4.0, min(11.0, 9.5 * aspect + 1.5))))
    for lane in rj["lanes"]:
        p = np.array(lane["xy"])
        ax.plot(p[:, 0], p[:, 1], color="#c8c8c8" if lane["junction"] else "#a0a0a0", lw=0.6, zorder=1)
    ax.plot(route[:, 0], route[:, 1], "--", color="#3060c0", lw=2.0, alpha=0.6, label="planned route", zorder=2)
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=v, cmap="viridis", s=6, zorder=3, label="driven (colour: speed)")
    fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.02, label="speed [km/h]")
    ax.plot(*xy[0], "o", color="#20a040", ms=10, zorder=4, label="start")
    ax.plot(*xy[-1], "s", color="#202020", ms=9, zorder=4, label="end")
    if scenario["trigger"] is not None:
        ax.plot(*scenario["trigger"], "*", color="#e09000", ms=18, zorder=4, label=f"{scenario['type']} trigger")
    for i, (kind, _, x, y) in enumerate(infraction_points(record)):
        ax.plot(x, y, "X", color="#d02020", ms=13, zorder=5, label="infraction" if i == 0 else None)
        ax.annotate(kind, (x, y), textcoords="offset points", xytext=(8, 8), color="#d02020", fontsize=9)
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(hi[1], lo[1])  # CARLA's y points right of x: flipped, the map reads as seen from above
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    s = record["scores"]
    ax.set_title(f"{rj['town']}  {scenario['type']}   driving score {s['score_composed']:.1f}, "
                 f"route {s['score_route']:.0f} %, penalty {s['score_penalty']:.2f}")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def video(run, path):
    frames = sorted(f for f in os.listdir(os.path.join(run, "frames")) if f.endswith(".jpg"))
    if not frames:
        return False
    import imageio_ffmpeg
    lst = os.path.join(run, "frames", "list.txt")
    with open(lst, "w") as f:
        f.writelines(f"file '{os.path.join(run, 'frames', n)}'\n" for n in frames)
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-f", "concat", "-safe", "0",
                    "-r", str(FPS), "-i", lst, "-vf", f"fps={FPS}", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "23", path], check=True)
    return True


def summary(steps, record, scenario, has_video, path):
    s = record["scores"]
    v = np.array([float(r["speed"]) for r in steps])
    lines = [f"# Bench2Drive route {scenario['index']} (id {scenario['route']}): {scenario['type']} ({scenario['town']})", "",
             f"- status: {record['status']}",
             f"- driving score {s['score_composed']:.2f}, route completion {s['score_route']:.1f} %, "
             f"infraction penalty {s['score_penalty']:.2f}",
             f"- simulated {record['meta']['duration_game']:.1f} s in {record['meta']['duration_system']:.0f} s wall "
             f"time, route length {record['meta']['route_length']:.0f} m",
             f"- speed: mean {3.6 * v.mean():.1f} km/h, max {3.6 * v.max():.1f} km/h", ""]
    hits = {k: t for k, t in record["infractions"].items() if t and k != "min_speed_infractions"}
    slow = record["infractions"].get("min_speed_infractions", [])
    lines += ["## Infractions", ""] + ([f"- **{k}**: {t}" for k, ts in hits.items() for t in ts] or ["- none"])
    if slow:  # one per section of the route, no location: their count and the speeds
        ratios = "; ".join(re.findall(r"[\d.]+%", " ".join(slow)))
        lines.append(f"- min_speed_infractions: {len(slow)} checks ({ratios} of the surrounding traffic's average speed)")
    lines.append("")
    lines += ["## What the autopilot reacted to", ""]
    for key, name in [("vehicle_hazard", "a vehicle"), ("light_hazard", "a red light"), ("walker_hazard", "a pedestrian"),
                      ("stop_sign_hazard", "a stop sign"), ("changed_route", "route shifted to pass an obstacle")]:
        n = sum(int(r[key]) for r in steps)
        if n:
            t0 = next(float(r["t"]) for r in steps if int(r[key]))
            lines.append(f"- {name}: {n / 20:.1f} s in all, first at t = {t0:.1f} s")
    lines += ["", "## Files", "", "- `map.png`: the route map", "- `steps.csv`: each tick (20 Hz)",
              "- `result.json`: the leaderboard's result", "- `leaderboard.log`: the leaderboard's output"]
    if has_video:
        lines.insert(-3, "- `video.mp4`: chase camera, 4 frames/s at real time")
    open(path, "w").write("\n".join(lines) + "\n")


def main():
    run, route_xml = sys.argv[1], sys.argv[2]
    record = json.load(open(os.path.join(run, "result.json")))["_checkpoint"]["records"][0]
    steps = list(csv.DictReader(open(os.path.join(run, "steps.csv"))))
    root = ET.parse(route_xml).getroot()
    r = root.find("route")
    sc = r.find("scenarios/scenario")
    tp = sc.find("trigger_point") if sc is not None else None
    scenario = {"index": os.path.basename(run).split("_")[0], "route": r.get("id"), "town": r.get("town"),
                "type": sc.get("type") if sc is not None else "none",
                "trigger": (float(tp.get("x")), float(tp.get("y"))) if tp is not None else None}
    route_map(run, steps, record, scenario, os.path.join(run, "map.png"))
    if os.path.isdir(os.path.join(run, "frames")) and video(run, os.path.join(run, "video.mp4")):
        subprocess.run(["rm", "-rf", os.path.join(run, "frames")], check=True)
    has_video = os.path.exists(os.path.join(run, "video.mp4"))
    summary(steps, record, scenario, has_video, os.path.join(run, "summary.md"))
    print(open(os.path.join(run, "summary.md")).read())


if __name__ == "__main__":
    main()
