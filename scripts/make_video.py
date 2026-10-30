"""A route's video from the frames scripts/b2d_evaluator.py saves with B2D_VIDEO set: the chase view and the view from
above side by side, at real time, with a bar under them from the agent's nmpc.jsonl (time, speed, the candidate
chosen, the lane band at the car, the solve). Run with SimLingo's Python (OpenCV, imageio-ffmpeg).

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


def frames(folder):
    """{timestamp: {view: path}} for the timestamps with every view."""
    out = {}
    for p in glob.glob(os.path.join(folder, "*.jpg")):
        m = re.match(r"(\w+)_([\d.]+)\.jpg$", os.path.basename(p))
        out.setdefault(float(m.group(2)), {})[m.group(1)] = p
    return {t: v for t, v in sorted(out.items()) if {"chase", "top"} <= v.keys()}


def log(path):
    """The NMPC's rows, each with the latest scene before it (scenes are logged only on plan ticks)."""
    rows, scene = [], {}
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            scene = r.get("scene", scene)
            rows.append({**r, "scene": scene})
    return rows


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
        img[:540, :960] = cv2.imread(views["chase"])
        img[:540, 960:] = cv2.imread(views["top"])
        text(img, "chase", 14, 30)
        text(img, "from above (car points up)", 974, 30)
        i = bisect.bisect_right(times, t) - 1
        line = f"t {t:5.1f} s"
        if i >= 0:
            r = rows[i]
            lane = r["scene"].get("lane")
            line += f"   speed {3.6 * r['v']:4.1f} km/h   plan: {r['label']}"
            if lane:
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
