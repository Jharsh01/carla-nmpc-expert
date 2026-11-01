"""A route's video from the frames scripts/b2d_evaluator.py saves with B2D_VIDEO set: the chase view and the view from
above side by side, at real time, with a bar under them from the agent's nmpc.jsonl (time, speed, the candidate
chosen, the lane band at the car, the solve), and, where the log has them, SimLingo's path, the path the NMPC got and
its plan drawn on the road in both views. Run with SimLingo's Python (OpenCV, imageio-ffmpeg).

    python scripts/make_video.py <SAVE_PATH>/<route> out.mp4 [--title "route 0"]
"""
import argparse
import bisect
import glob
import json
import os
import re
import subprocess

import cv2
import imageio_ffmpeg
import numpy as np

FPS = 10  # the evaluator saves every other 50 ms tick
BAR = 64
VIEWS = {"chase": (-9.0, 4.5, -18.0, 90.0), "top": (6.0, 40.0, -90.0, 70.0)}  # x, z, pitch, fov: as b2d_evaluator.py
W, H = 960, 540
SIMLINGO, ROAD, PLAN, FAILED = (70, 230, 90), (215, 215, 215), (255, 70, 230), (60, 60, 255)  # BGR


def frames(folder):
    """{timestamp: {view: path}} for the timestamps with every view."""
    out = {}
    for p in glob.glob(os.path.join(folder, "*.jpg")):
        m = re.match(r"(\w+)_([\d.]+)\.jpg$", os.path.basename(p))
        out.setdefault(float(m.group(2)), {})[m.group(1)] = p
    return {t: v for t, v in sorted(out.items()) if {"chase", "top"} <= v.keys()}


def log(path):
    """The NMPC's rows, each with the latest scene before it (scenes are logged only on plan ticks)."""
    rows, scene, lines = [], {}, None
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            scene = r.get("scene", scene)
            lines = r if "plan_xy" in r else lines
            rows.append({**r, "scene": scene, "lines": lines})
    return rows


def between(points, old, new):
    """Points in the car's frame at the pose `old`, in its frame at the pose `new` (x, y, psi; y left)."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = np.cos(old[2]), np.sin(old[2])
    x, y = old[0] + c * p[:, 0] - s * p[:, 1], old[1] + s * p[:, 0] + c * p[:, 1]
    c, s = np.cos(new[2]), np.sin(new[2])
    dx, dy = x - new[0], y - new[1]
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def on_image(points, view):
    """Points on the ground in the car's frame as pixels of a view, with a mask of those in front of the camera."""
    cx, cz, pitch, fov = VIEWS[view]
    f = 0.5 * W / np.tan(np.radians(fov) / 2.0)
    c, s = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    dx, dz = points[:, 0] - cx, -cz
    depth, up = dx * c + dz * s, -dx * s + dz * c
    front = depth > 1.0
    d = np.where(front, depth, 1.0)
    return np.stack([W / 2 - f * points[:, 1] / d, H / 2 - f * up / d], axis=1), front


def draw_line(panel, points, view, colour, thick, dots=False):
    pixels, front = on_image(points, view)
    pixels = np.clip(pixels[front], -4000, 4000).astype(np.int32)
    if len(pixels) > 1:
        cv2.polylines(panel, [pixels], False, colour, thick, cv2.LINE_AA)
    if dots:
        for p in pixels:
            cv2.circle(panel, tuple(int(v) for v in p), thick + 1, colour, -1, cv2.LINE_AA)


def draw_lines(panel, view, row, plan_row):
    """The latest plan's lines, carried by the true pose to this frame's."""
    old, new = plan_row["frame"], row["pose"]
    draw_line(panel, between(plan_row["path"], old, new), view, ROAD, 1)
    draw_line(panel, between(plan_row["simlingo"], old, new), view, SIMLINGO, 2, dots=True)
    draw_line(panel, between(plan_row["plan_xy"], old, new), view, PLAN if plan_row["success"] else FAILED, 2)
    for i, (name, colour) in enumerate((("SimLingo path", SIMLINGO), ("path the NMPC got", ROAD),
                                        ("NMPC plan (red: failed)", PLAN))):
        text(panel, name, W - 260, 28 + 24 * i, 0.55, colour)


def text(img, s, x, y, scale=0.62, colour=(235, 235, 235), bold=1):
    cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, colour, bold, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route_dir")
    ap.add_argument("out")
    ap.add_argument("--title", default="")
    a = ap.parse_args()
    folder = os.path.join(a.route_dir, "video")
    start = float(open(os.path.join(folder, "start.txt")).read())
    shots = frames(folder)
    rows = log(os.path.join(a.route_dir, "nmpc.jsonl"))
    times = [r["t"] for r in rows]
    w, h = 1920, 540 + BAR
    ff = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-f", "rawvideo",
                           "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", "-crf", "23", "-movflags", "+faststart", a.out], stdin=subprocess.PIPE)
    for ts, views in shots.items():
        t = ts - start
        img = np.full((h, w, 3), 24, np.uint8)
        chase, top = cv2.imread(views["chase"]), cv2.imread(views["top"])
        i = bisect.bisect_right(times, t) - 1
        if i >= 0 and rows[i]["lines"] is not None and "pose" in rows[i]:
            draw_lines(chase, "chase", rows[i], rows[i]["lines"])
            draw_lines(top, "top", rows[i], rows[i]["lines"])
        img[:540, :960], img[:540, 960:] = chase, top
        text(img, "chase", 14, 30)
        text(img, "from above (car points up)", 974, 30)
        line = f"t {t:5.1f} s"
        if i >= 0:
            r = rows[i]
            lane = r["scene"].get("lane")
            line += f"   speed {3.6 * r['v']:4.1f} km/h   plan: {r['label']}"
            intent = r["lines"]["intent"] if r["lines"] and "intent" in r["lines"] else None
            if intent:
                line += f"   {'FOLLOW SimLingo' if intent[0] else 'keep lane'} ({intent[1]:.1f} m)"
                line += "  stop asked" if intent[2] else ""
            if lane and not intent:
                line += f"   lane band at the car [{lane[0]:+.1f}, {lane[1]:+.1f}] m"
            if not r["success"]:
                line += "   solve FAILED"
            if r["notes"]:
                line += "   " + ", ".join(map(str, r["notes"]))[:60]
        text(img, line, 14, 540 + 40, 0.7)
        if a.title:
            (tw, _), _ = cv2.getTextSize(a.title, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            text(img, a.title, w - tw - 14, 540 + 40, 0.7, (120, 200, 255), 2)
        ff.stdin.write(img.tobytes())
    ff.stdin.close()
    ff.wait()
    print(f"{len(shots)} frames, {len(shots) / FPS:.1f} s: {a.out}")


if __name__ == "__main__":
    main()
