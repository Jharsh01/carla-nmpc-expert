"""Build docs/architecture.pdf: how the stack works, with diagrams and every parameter, read from the source.

    python docs/report/make_architecture.py [--out docs/architecture.pdf]

Needs reportlab and matplotlib (not in the project's environment: any Python 3 with both will do). The parameter
tables are parsed from the code itself (the C++ option structs, config/vehicles/mkz2020.json, the Python
constants), so they show what the code holds when the document is built.
"""
import argparse
import io
import json
import os
import re
import subprocess

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle  # noqa: E402
from reportlab.lib import colors  # noqa: E402
from reportlab.lib.enums import TA_LEFT  # noqa: E402
from reportlab.lib.pagesizes import A4  # noqa: E402
from reportlab.lib.styles import ParagraphStyle  # noqa: E402
from reportlab.lib.units import mm  # noqa: E402
from reportlab.pdfbase import pdfmetrics  # noqa: E402
from reportlab.pdfbase.ttfonts import TTFont  # noqa: E402
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table,  # noqa: E402
                                TableStyle)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---- fonts and styles, as docs/report/make_m1_report.py

TTF = os.path.join(os.path.dirname(matplotlib.__file__), "mpl-data", "fonts", "ttf")
for name, file in [("Sans", "DejaVuSans.ttf"), ("Sans-Bold", "DejaVuSans-Bold.ttf"),
                   ("Sans-Oblique", "DejaVuSans-Oblique.ttf"), ("Mono", "DejaVuSansMono.ttf")]:
    pdfmetrics.registerFont(TTFont(name, os.path.join(TTF, file)))
pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-Bold", italic="Sans-Oblique", boldItalic="Sans-Bold")

INK, INK2, MUTED, RULE = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, AQUA, VIOLET, GREY = "#2a78d6", "#eb6834", "#1baf7a", "#7a5bd6", "#9aa3ad"
FILL = {"carla": "#eef1f5", "simlingo": "#e6f0fb", "world": "#e7f6ef", "nmpc": "#fdeee6", "act": "#f1ecfb",
        "tool": "#f4f4f2"}

body = ParagraphStyle("body", fontName="Sans", fontSize=9.2, leading=13.2, textColor=INK, alignment=TA_LEFT, spaceAfter=5)
small = ParagraphStyle("small", parent=body, fontSize=8, leading=10.6, textColor=INK2)
cell = ParagraphStyle("cell", parent=body, fontSize=7.4, leading=9.4, spaceAfter=0)
cell_b = ParagraphStyle("cell_b", parent=cell, fontName="Sans-Bold")
cell_m = ParagraphStyle("cell_m", parent=cell, fontName="Mono", fontSize=7.0)
h1 = ParagraphStyle("h1", parent=body, fontName="Sans-Bold", fontSize=13.5, leading=17, spaceBefore=10, spaceAfter=6)
h2 = ParagraphStyle("h2", parent=body, fontName="Sans-Bold", fontSize=10.5, leading=14, spaceBefore=8, spaceAfter=4)
title = ParagraphStyle("title", parent=body, fontName="Sans-Bold", fontSize=19, leading=24, spaceAfter=4)
sub = ParagraphStyle("sub", parent=body, fontSize=10, leading=14, textColor=INK2, spaceAfter=12)
eq = ParagraphStyle("eq", parent=body, fontName="Mono", fontSize=8.0, leading=11.2, leftIndent=10, spaceAfter=6)
caption = ParagraphStyle("caption", parent=small, spaceBefore=2, spaceAfter=10)


def P(text, style=body):
    return Paragraph(text, style)


def bullets(items, style=body):
    return [Paragraph(f"•&nbsp;&nbsp;{t}", ParagraphStyle("b", parent=style, leftIndent=10, firstLineIndent=-8,
                                                           spaceAfter=2)) for t in items]


def table(rows, widths, mono_cols=(0,)):
    data = []
    for i, r in enumerate(rows):
        data.append([P(_esc(c), cell_b if i == 0 else (cell_m if j in mono_cols else cell)) for j, c in enumerate(r)])
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1)
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"),
                           ("LINEBELOW", (0, 0), (-1, 0), 0.8, colors.HexColor(INK2)),
                           ("LINEBELOW", (0, 1), (-1, -1), 0.3, colors.HexColor(RULE)),
                           ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
                           ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3)]))
    return t


def _esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def figure(fig, width_mm=172):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    w, h = fig.get_size_inches()
    img = Image(buf, width=width_mm * mm, height=width_mm * mm * h / w)
    return img


# ---- parameters, read from the source

MEANINGS = {  # for fields whose comment in the code is only a unit, or none
    "horizon": "how far ahead the plan reaches", "fine": "how long the fine steps last",
    "time_gap": "time gap kept behind a lead, in the speed asked for", "a_max": "strongest acceleration",
    "jerk_max": "largest |jerk|", "tol": "FATROP's convergence tolerance",
    "acceptable_iter": "iterations within acceptable_tol before stopping", "max_iter": "FATROP's iteration limit",
    "cache_dir": "where the compiled solver is kept (the agent: build/urban_cache)",
}


def struct_fields(path, opening):
    """(name, value, comment) of each field of the struct starting at the line `opening` in a C++ header: the
    trailing comment, or else the first sentence of the comment block above."""
    lines = open(os.path.join(REPO, path)).read().splitlines()
    i = next(k for k, l in enumerate(lines) if l.strip() == opening)
    out, above = [], []
    depth = 0
    for line in lines[i + 1:]:
        s = line.strip()
        if s.startswith("};") and depth == 0:
            break
        if s.startswith("//"):
            above.append(s[2:].strip())
            continue
        m = re.match(r"(?:double|int|bool|std::string)\s+(\w+)(?:\s*=\s*([^;]+))?;\s*(?://\s*(.*))?$", s)
        if m:
            name, value, comment = m.group(1), (m.group(2) or "").strip(), (m.group(3) or "").strip()
            block = re.split(r"(?<=\.)\s", " ".join(above))[0] if above else ""
            if block and (not comment or (len(comment) < 14 and comment.startswith("["))):
                comment = (block + " " + comment).strip()
            if name.startswith("w_"):
                comment = f"weight of the {name[2:]} cost term" + (f" ({comment})" if comment else "")
            if (not comment or re.fullmatch(r"\[[^\]]*\]", comment)) and name in MEANINGS:
                comment = MEANINGS[name] + (" " + comment if comment else "")
            out.append((name, value or "–", comment))
        above = [] if not s.startswith("//") else above
    return out


def py_constants(path, names=None):
    out = []
    for line in open(os.path.join(REPO, path)):
        m = re.match(r"^([A-Z_][A-Z0-9_]*) = (.+?)\s*(?:#\s*(.*))?$", line.rstrip())
        if m and (names is None or m.group(1) in names):
            out.append((m.group(1), m.group(2), m.group(3) or ""))
    return out


def py_defaults(path, func_line_start):
    """Keyword defaults of the function whose definition starts with `func_line_start`."""
    text = open(os.path.join(REPO, path)).read()
    i = text.index(func_line_start)
    sig = text[i:text.index(":", text.index(")", i))]
    return re.findall(r"(\w+)=([-\d.]+)", sig)


def vehicle_rows():
    params = json.load(open(os.path.join(REPO, "config", "vehicles", "mkz2020.json")))["vehicle"]["parameters"]
    return [[k, f"{v['value']:g} {v.get('unit', '')}".strip(), v.get("source", ""),
             (v.get("description", "") + (": " + v["note"] if v.get("note") else ""))] for k, v in params.items()]


# ---- diagrams

def _box(ax, x, y, w, h, text, fill, size=7.0, edge=INK2):
    """A rounded box: the first line of `text` as a bold title at its top, the rest below it."""
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.004,rounding_size=0.012", fc=fill, ec=edge, lw=0.9))
    head, *rest = text.split("\n")
    ax.text(x + w / 2, y + h - 0.012, head, ha="center", va="top", fontsize=size + 0.5, fontweight="bold", color=INK,
            family="DejaVu Sans")
    if rest:
        ax.text(x + w / 2, y + h - 0.012 - (size + 3.5) / 72 / 8.6 * 1.25, "\n".join(rest), ha="center", va="top",
                fontsize=size, color=INK2, linespacing=1.3, family="DejaVu Sans")


def _arrow(ax, a, b, label=None, color=INK2, rad=0.0, size=6.4, lw=1.0, ls="-", off=(0, 0.012)):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=8, lw=lw, color=color, ls=ls,
                                 connectionstyle=f"arc3,rad={rad}", shrinkA=2, shrinkB=2))
    if label:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], label, ha="center", va="bottom", fontsize=size,
                color=color, family="DejaVu Sans", bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.9))


def _canvas(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, h / w)
    ax.axis("off")
    return fig, ax


def fig_system():
    W, Hin = 8.6, 8.0
    fig, ax = _canvas(W, Hin)
    H = Hin / W
    row = lambda k: H - 0.02 - k * 0.165  # the top of row k
    bh = 0.125
    _box(ax, 0.02, row(0) - 0.075, 0.96, 0.075, "CARLA 0.9.15 + Bench2Drive leaderboard (20 Hz, synchronous)\n"
         "town, route, scenarios, Traffic Manager; the ego: vehicle.lincoln.mkz_2020 (the hero)", FILL["carla"])
    _box(ax, 0.02, row(1) - bh, 0.29, bh, "SimLingo (LingoAgent)\ncameras → InternVL2 + LoRA\n"
         "pred_route: 20 points, ~1 m apart\npred_speed_wps: 11, 0.25 s apart\n(car frame: x forward, y right)",
         FILL["simlingo"])
    _box(ax, 0.355, row(1) - bh, 0.29, bh, "simlingo_ref.py\npath_points: 15 m behind + SimLingo's\n"
         "speed_reference: v_ref(t)\nOdometry: speedometer + compass\n→ the old frame for set_route", FILL["simlingo"])
    _box(ax, 0.69, row(1) - bh, 0.29, bh, "World model (privileged)\nactors.py: visible, tracked, predicted\n"
         "route.py: lane continuation (160 m),\nlanes, stop lines, route end\nreported in the car frame",
         FILL["world"])
    _box(ax, 0.17, row(2) - 0.09, 0.66, 0.09, "Scene (car frame: the car at the origin, x forward, y left)\n"
         "ego (v, a, δ) · path · line · v_ref(t) · lanes · actors (t, x, y, ψ) · stops · v_max", FILL["tool"])
    _box(ax, 0.02, row(3) - bh - 0.02, 0.43, bh + 0.02, "Behaviour layer (core/urban/behaviour)\n"
         "classify: lead · blocker · crosser · mover\ncandidates: base · pass:<id> · borrow_left\n"
         "rules: stop-sign hold, red-light dilemma\nscore, last choice × 0.9 → the chosen plan", FILL["nmpc"])
    _box(ax, 0.55, row(3) - bh - 0.02, 0.43, bh + 0.02, "UrbanNMPC × 4 (one thread each)\n"
         "kinematic bicycle in path coordinates\nx = [s, e_y, e_ψ, v, a, δ],  u = [jerk, δ_rate]\n"
         "N = 30 steps over 5 s, CasADi + FATROP\nwarm start, compiled once (disk cache)", FILL["nmpc"])
    _box(ax, 0.17, row(4) - 0.105, 0.66, 0.105, "Actuation (mkz_actuation.py, 20 Hz)\n"
         "the plan sampled at t + 0.25 s (a) and t + 0.15 s (δ)\n"
         "measured tables inverted + PI on a → throttle, brake;  δ / gain(v) → steer", FILL["act"])
    _box(ax, 0.02, 0.012, 0.36, 0.075, "run_bench2drive.sh → b2d_evaluator.py\nwatchdog: 30 s at rest, 75 s route",
         FILL["tool"], size=6.6)
    _box(ax, 0.62, 0.012, 0.36, 0.075, "live_view.py → browser (SSE)\nnmpc.jsonl: a row per tick", FILL["tool"], size=6.6)
    y1 = row(0) - 0.075
    _arrow(ax, (0.165, y1), (0.165, row(1)), "cameras, GNSS, IMU, speed", off=(0.0, -0.012))
    _arrow(ax, (0.835, y1), (0.835, row(1)), "ground truth", off=(0.0, -0.012))
    _arrow(ax, (0.31, row(1) - bh / 2), (0.355, row(1) - bh / 2))
    _arrow(ax, (0.5, row(1) - bh), (0.5, row(2)), "path, v_ref")
    _arrow(ax, (0.835, row(1) - bh), (0.72, row(2)), "actors, lanes, stops")
    _arrow(ax, (0.3, row(2) - 0.09), (0.235, row(3)), "scene")
    yb = row(3) - (bh + 0.02) / 2
    _arrow(ax, (0.45, yb + 0.02), (0.55, yb + 0.02), "problems")
    _arrow(ax, (0.55, yb - 0.025), (0.45, yb - 0.025), "plans")
    _arrow(ax, (0.235, row(3) - bh - 0.02), (0.32, row(4)), "chosen plan")
    yc = row(4) - 0.0525
    ax.add_patch(FancyArrowPatch((0.83, yc), (0.995, yc), arrowstyle="-", lw=1.0, color=INK2))
    ax.add_patch(FancyArrowPatch((0.995, yc), (0.995, y1 + 0.04), arrowstyle="-|>", mutation_scale=8, lw=1.0,
                                 color=INK2, shrinkA=0, shrinkB=0))
    ax.add_patch(FancyArrowPatch((0.995, y1 + 0.04), (0.98, y1 + 0.04), arrowstyle="-|>", mutation_scale=8, lw=1.0,
                                 color=INK2, shrinkA=0, shrinkB=0))
    ax.text(0.988, (yc + y1) / 2, "throttle, brake, steer", rotation=90, fontsize=6.4, color=INK2, va="center",
            ha="right", family="DejaVu Sans")
    _arrow(ax, (0.93, row(3) - bh - 0.02), (0.90, 0.087), "each plan", ls="--", color=MUTED, off=(-0.05, 0.0))
    return fig


def fig_tick():
    fig, ax = plt.subplots(figsize=(8.6, 2.5))
    ticks = list(range(9))
    for k in ticks:
        t = k * 0.05
        ax.add_patch(Rectangle((t, 2.0), 0.044, 0.55, fc=FILL["simlingo"], ec=BLUE, lw=0.8))
        ax.add_patch(Rectangle((t, 1.1), 0.044, 0.55, fc=FILL["act"], ec=VIOLET, lw=0.8))
        if k % 2 == 0:
            ax.add_patch(Rectangle((t, 0.2), 0.044 * 1.6, 0.55, fc=FILL["nmpc"], ec=ORANGE, lw=0.8))
    for y, lab in [(2.27, "SimLingo predicts\n(every tick)"), (1.37, "actuation from the plan\n(every tick, 20 Hz)"),
                   (0.47, "new path + scene + behaviour\nstep (every 2nd tick, 10 Hz)")]:
        ax.text(-0.012, y, lab, ha="right", va="center", fontsize=7, color=INK2)
    ax.annotate("", xy=(0.10 + 0.25, 1.05), xytext=(0.10, 1.05), arrowprops=dict(arrowstyle="->", color=MUTED, lw=0.8))
    ax.text(0.10 + 0.125, 0.93, "a sampled 0.25 s ahead, δ 0.15 s ahead (the car's lags)", ha="center", fontsize=6.4,
            color=MUTED)
    ax.set_xlim(-0.005, 0.45)
    ax.set_ylim(0.0, 2.75)
    ax.set_yticks([])
    ax.set_xticks([k * 0.05 for k in ticks])
    ax.set_xticklabels([f"{k * 50}" for k in ticks], fontsize=7)
    ax.set_xlabel("simulation time [ms]; the leaderboard waits for the agent (with --fast, about 0.4 s of wall time a tick)",
                  fontsize=7, color=INK2)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    return fig


def fig_frame():
    import numpy as np
    fig, ax = plt.subplots(figsize=(8.6, 4.6))
    # a road straight for 25 m past the car, then bending left (radius 45 m); drawn with forward up, left to the left
    ds = 0.2
    s = np.arange(-15.0, 80.0, ds)
    psi = np.cumsum(np.where(s < 25.0, 0.0, 1.0 / 45.0)) * ds
    fx = np.cumsum(np.cos(psi)) * ds - 15.0  # forward
    ly = np.cumsum(np.sin(psi)) * ds  # left
    nf, nl = -np.sin(psi), np.cos(psi)  # the left normal

    def pt(i, off=0.0):  # screen (x right, y up) of the path point i, offset `off` to its left
        return -(ly[i] + off * nl[i]), fx[i] + off * nf[i]

    def curve(mask, off=0.0):
        return [-(ly[mask] + off * nl[mask]), fx[mask] + off * nf[mask]]
    allm = np.ones_like(s, dtype=bool)
    lo, hi = -1.6, 1.6
    band = np.concatenate([np.c_[curve(allm, hi)[0], curve(allm, hi)[1]],
                           np.c_[curve(allm, lo)[0], curve(allm, lo)[1]][::-1]])
    ax.add_patch(Polygon(band, fc="#e6f0fb", ec="none"))
    for off, c in [(hi, GREY), (lo, GREY), (hi + 3.2, "#cdd3da")]:
        ax.plot(*curve(allm, off), color=c, lw=0.9)
    ax.plot(*curve(s <= 0.0), color=BLUE, lw=1.4, ls=":")
    ax.plot(*curve((s >= 0.0) & (s <= 20.0)), color=BLUE, lw=2.6)
    ax.plot(*curve(s >= 20.0), color=GREY, lw=1.6, ls="--")
    ax.add_patch(Rectangle((-0.92, -2.15), 1.84, 4.89, fc="white", ec=INK, lw=1.0))  # the car, origin 0.30 m behind its centre
    ax.annotate("", xy=(0, 7), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", lw=1.1, color=INK))
    ax.annotate("", xy=(-7, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", lw=1.1, color=INK))
    ax.text(0.6, 6.6, "x", fontsize=8)
    ax.text(-7.2, 0.6, "y", fontsize=8)
    i_park = np.searchsorted(s, 33.0)
    cx, cy = pt(i_park, 2.9)
    ax.add_patch(Rectangle((cx - 1.0, cy - 2.4), 2.0, 4.8, fc="#c5ccd4", ec=INK2, lw=0.7,
                           transform=matplotlib.transforms.Affine2D().rotate_deg_around(cx, cy, np.degrees(psi[i_park])) + ax.transData))
    i_stop = np.searchsorted(s, 62.0)
    ax.plot([pt(i_stop, lo)[0], pt(i_stop, hi)[0]], [pt(i_stop, lo)[1], pt(i_stop, hi)[1]], color=ORANGE, lw=3)

    def note(xy, text, at, color=INK2):
        ax.annotate(text, xy=xy, xytext=at, fontsize=6.8, color=color, va="center",
                    arrowprops=dict(arrowstyle="-", lw=0.6, color=MUTED))
    note(pt(np.searchsorted(s, -8.0)), "15 m behind the car (s = 0 at its start),\nso actors and stops just behind project",
         (8, -9))
    note(pt(np.searchsorted(s, 12.0)), "SimLingo's path (~20 m): the line asked for, e_y = 0", (8, 11), BLUE)
    note(pt(np.searchsorted(s, 45.0)), "the lane past SimLingo's last point (map), 160 m:\nwhere the constraints are measured",
         (8, 40))
    note(pt(np.searchsorted(s, 15.0), hi), "corridor: the lane band [lo, hi](s)\n(the left lane added by borrow_left)",
         (-62, 18), BLUE)
    note((cx, cy), "parked car beside the lane: a mover", (-62, 34))
    note(pt(i_stop, lo), "stop line at s (red light)", (8, 60), ORANGE)
    note((0.0, -2.0), "the car: origin at (0, 0), state = what it measures", (8, -2))
    ax.set_aspect("equal")
    ax.set_xlim(-63, 40)
    ax.set_ylim(-16, 66)
    ax.axis("off")
    return fig


def fig_nmpc():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(8.6, 3.0), gridspec_kw={"width_ratios": [1.25, 1]})
    import numpy as np
    # left: the steps over the horizon
    t = [0.0]
    while t[-1] < 1.0 - 1e-9:
        t.append(round(t[-1] + 0.1, 3))
    while t[-1] < 5.0 - 1e-9:
        t.append(round(t[-1] + 0.2, 3))
    a1.vlines(t, 0, 1, color=ORANGE, lw=0.8)
    a1.set_xlim(-0.1, 5.1)
    a1.set_ylim(0, 1.4)
    a1.set_yticks([])
    a1.set_xlabel("time from the measurement [s]", fontsize=7)
    a1.tick_params(labelsize=7)
    a1.text(0.5, 1.12, "h_fine = 0.1 s for 1 s", ha="center", fontsize=7)
    a1.text(3.0, 1.12, f"h = 0.2 s to 5 s:  N = {len(t) - 1} steps", ha="center", fontsize=7)
    for s_ in ("top", "right", "left"):
        a1.spines[s_].set_visible(False)
    a1.set_title("the horizon", fontsize=8)
    # right: body circles and a mover circle
    L, W, cg = 4.89, 2.04, 0.31
    r_ego = float(np.hypot(L / 6, W / 2))
    a2.add_patch(Rectangle((-L / 2, -W / 2), L, W, fc="#ffffff", ec=INK, lw=0.9))
    for c in (-1, 0, 1):
        a2.add_patch(Circle((c * L / 3, 0), r_ego, fc="none", ec=ORANGE, lw=0.8))
    Lm, Wm = 4.8, 2.0
    rm = float(np.hypot(Lm / 6, Wm / 2))
    ox, oy = 1.5, 3.3
    a2.add_patch(Rectangle((ox - Lm / 2, oy - Wm / 2), Lm, Wm, fc="#c5ccd4", ec=INK2, lw=0.8))
    for c in (-1, 0, 1):
        a2.add_patch(Circle((ox + c * Lm / 3, oy), rm, fc="none", ec=GREY, lw=0.8))
    a2.add_patch(Circle((ox, oy), r_ego + rm + 0.2, fc="none", ec=BLUE, lw=0.8, ls="--"))
    a2.text(-L / 2, -2.5, f"ego: 3 circles, r = hypot(L/6, W/2) = {r_ego:.2f} m (W with mirrors 2.04 m)", fontsize=6.3)
    a2.text(ox - 4.3, oy + 3.9, f"parked car: 3 circles, r = {rm:.2f} m; kept apart by r_ego + r + margin\n"
            "(margin 0.5 m + 0.1 m/s growth; parked/objects 0.2 m, no growth)", fontsize=6.3)
    a2.set_aspect("equal")
    a2.set_xlim(-4.5, 6.5)
    a2.set_ylim(-3.2, 8.2)
    a2.axis("off")
    a2.set_title("movers: circles on circles", fontsize=8)
    return fig


def fig_behaviour():
    fig, ax = _canvas(8.6, 3.4)
    H = 3.4 / 8.6
    steps = [("1 Project", "actors' predicted trajectories\nonto the path: s, e_y, extents", FILL["world"]),
             ("2 Classify", "blocker: parked/object 0.3 m into lane\nlead · crosser · mover", FILL["nmpc"]),
             ("3 Rules", "stop lines → gates; stop sign 1 s;\nred-light dilemma above 5 m/s²", FILL["nmpc"]),
             ("4 Candidates", "base (yield to all) · pass:<id>\nper crosser · borrow_left if clear", FILL["nmpc"]),
             ("5 Solve", "in parallel on the pool, each\non its own NMPC (warm start kept)", FILL["nmpc"]),
             ("6 Choose", "lowest score with slack ≤ 0.05;\nlast choice × 0.9; penalties", FILL["nmpc"])]
    w, h, gx = 0.29, 0.13, 0.055
    for i, (head, text, fill) in enumerate(steps):
        r, c = divmod(i, 3)
        x0, y0 = 0.01 + c * (w + gx), H - 0.02 - h - r * (h + 0.09)
        _box(ax, x0, y0, w, h, head + "\n" + text, fill, size=6.8)
        if c:
            _arrow(ax, (x0 - gx, y0 + h / 2), (x0, y0 + h / 2))
    _arrow(ax, (0.01 + 2 * (w + gx) + w / 2, H - 0.02 - h), (0.01 + w / 2, H - 0.02 - h - 0.09), rad=-0.08)
    return fig


def fig_launch():
    fig, ax = _canvas(8.6, 2.6)
    H = 2.6 / 8.6
    _box(ax, 0.01, H * 0.12, 0.2, H * 0.76, "run_bench2drive.sh\n--agent simlingo_nmpc\n--no-cot --fast --live\n"
         "--max-stopped 30\n--max-time 75", FILL["tool"], size=6.4)
    _box(ax, 0.26, H * 0.12, 0.2, H * 0.76, "b2d_evaluator.py\nwatchdog, sync check\nin the tick loop; ports,\ncleanup (a process\nper route)", FILL["tool"], size=6.4)
    _box(ax, 0.51, H * 0.12, 0.2, H * 0.76, "leaderboard_evaluator\nCARLA server (port 20000)\nroute XML, scenarios\n"
         "→ the agent each tick", FILL["carla"], size=6.4)
    _box(ax, 0.76, H * 0.12, 0.23, H * 0.76, "results\n<tag>/seed1/res/<route>.json\nres/merged.json\nout/<route>.log\nviz/<route>/nmpc.jsonl", FILL["tool"], size=6.4)
    _arrow(ax, (0.21, H * 0.5), (0.26, H * 0.5))
    _arrow(ax, (0.46, H * 0.5), (0.51, H * 0.5))
    _arrow(ax, (0.71, H * 0.5), (0.76, H * 0.5))
    return fig


# ---- the document

def git_rev():
    try:
        rev = subprocess.check_output(["git", "-C", REPO, "rev-parse", "--short", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(["git", "-C", REPO, "status", "--porcelain"], text=True).strip()
        return rev + (" + uncommitted changes" if dirty else "")
    except Exception:  # noqa: BLE001
        return "unknown"


def build(out):
    s = []
    s.append(P("carla-expert-nmpc: architecture", title))
    s.append(P(f"SimLingo through an urban NMPC on Bench2Drive: how the parts fit, what each one computes, and every "
               f"parameter. Built from commit {git_rev()}; the parameter tables are read from the code.", sub))

    s.append(P("1. The stack", h1))
    s.append(P("One agent drives Bench2Drive's ego. <b>SimLingo</b> (a vision-language driving model) looks at the "
               "cameras and predicts a path and timed speed waypoints; its own PID used to follow them. Here the "
               "<b>urban NMPC</b> follows them instead, inside the constraints a <b>world model</b> gives: the "
               "actors it must keep clear of, the lane, the lights and signs. The world model reads CARLA's ground "
               "truth for now, but only what a perception stack on Bench2Drive's sensors could see, and reports it "
               "relative to the car, so a learned perception model can take its place behind the same Scene."))
    s.append(figure(fig_system()))
    s.append(P("Figure 1. Data flow through one plan. Blue: SimLingo and its reference; green: the world model "
               "(privileged); orange: the behaviour layer and the NMPC (C++, core/urban); violet: actuation; grey: "
               "the launcher, logging and the live view, which do not take part in driving.", caption))
    s.append(table([
        ["Part", "Where", "What it does"],
        ["SimLingo", "external/simlingo/team_code/agent_simlingo.py", "LingoAgent: predicts pred_route and pred_speed_wps each tick"],
        ["Agent", "expert/agents/simlingo_nmpc_agent.py", "LingoAgent with control_pid replaced: the per-tick loop, scene, actuation, log"],
        ["Reference", "expert/simlingo_ref.py", "SimLingo's outputs as path points and v_ref(t); odometry between plans"],
        ["World model", "expert/world_model/actors.py, route.py", "visible actors with predictions; lanes, stop lines, route end, lane continuation"],
        ["Frames", "expert/frames.py", "CARLA (left-handed, degrees) to the library's frame; Frame: world to car frame"],
        ["Behaviour", "core/urban/{include,src}/behaviour", "classification, traffic rules, candidates, parallel solves, choice"],
        ["NMPC", "core/urban/{include,src}/urban_nmpc", "the optimal control problem, CasADi + FATROP, warm start, set_route"],
        ["Path", "core/urban/.../path", "biarc path through waypoints: projection, pose, curvature"],
        ["Bindings", "bindings/expertctl.cpp", "the Python module expertctl (pybind11, Python 3.8)"],
        ["Actuation", "expert/mkz_actuation.py", "acceleration and road-wheel angle to throttle, brake, steer (measured tables)"],
        ["Launcher", "scripts/run_bench2drive.sh, b2d_evaluator.py", "one leaderboard process per route; watchdog; results"],
        ["Live view", "expert/live_view.py, .html", "each plan pushed to a browser page (SSE)"],
        ["Offline tests", "sim/urban/scenarios.cpp, tests/", "22 closed-loop scenarios without CARLA (11 × 2), pytest"],
    ], [24, 62, 86], mono_cols=(1,)))

    s.append(P("2. One tick", h1))
    s.append(P("The leaderboard ticks CARLA at 20 Hz in synchronous mode and waits for the agent each tick. SimLingo "
               "predicts every tick. Every second tick the agent builds a new path from the latest prediction, reads "
               "the world model, and runs a behaviour step (10 Hz). Every tick it samples the current plan a little "
               "ahead, for the car's lags, and turns it into throttle, brake and steer."))
    s.append(figure(fig_tick(), 160))
    s.append(P("Figure 2. Timing within a tick and across ticks.", caption))
    s.append(P("Per plan, in order (SimLingoNMPCAgent.control_pid):", body))
    s += bullets([
        "Odometry: the speedometer and the compass's change since the last tick move the car in the last plan's frame.",
        "Path: SimLingo's 20 points, y flipped to the left, 15 m of straight line prepended behind the car, then "
        "160 m along the map's lane past the last point (route.py). Built as a biarc Path.",
        "set_route(path, old frame): the last plan carried into the new path's coordinates through the old frame's "
        "pose in the new one, so the solver starts warm.",
        "World model: actors detected and tracked (in the world), predicted 5 s ahead, reported in the car's frame; "
        "lanes and stop lines along the path; v_max from the speed limit.",
        "Scene → Behaviour.step → Decision (the chosen plan, all candidates, notes), then the log row and the live view.",
    ])

    s.append(P("3. Frames: why nothing needs a global position", h1))
    s.append(P("SimLingo predicts in the car's frame at the moment of the prediction, so the NMPC plans in that frame: "
               "the car at the origin, x forward, y left. The car's state is what it measures: speed (speedometer), "
               "acceleration (IMU, low-passed), road-wheel angle (from the last steer command). Between two plans the "
               "frame moves with the car; odometry (speed × dt, compass change) gives the old frame's pose in the new "
               "one for the warm start. Only the world model touches the world: its ground truth is placed in the car's "
               "frame through the car's true pose (frames.Frame), which is what a sensor on the car would report."))
    s.append(figure(fig_frame(), 150))
    s.append(P("Figure 3. The path and what lies along it, in the car's frame. Path coordinates: s along the path "
               "from its start 15 m behind the car (so s ≈ 15 m at the car), e_y to the left of it.", caption))
    s.append(P("An earlier version placed SimLingo's path in the world through SimLingo's own GNSS/UKF pose. On "
               "Town12 that frame was 497.6 m off CARLA's, because SimLingo estimates its GNSS reference itself, so "
               "every actor and lane came out 497 m away. Planning in the car's frame removes that class of error.",
               small))

    s.append(P("4. The NMPC", h1))
    s.append(P("A kinematic bicycle in the path's coordinates, solved as one nonlinear program per candidate:", body))
    s.append(P("state x = [s, e_y, e_ψ, v, a, δ];  input u = [jerk, δ_rate]<br/>"
               "β = atan(b tan δ / L)  (the sideslip at the centre of gravity)<br/>"
               "s′ = v cos(e_ψ + β) / (1 − κ(s) e_y)<br/>"
               "e_y′ = v sin(e_ψ + β)<br/>"
               "e_ψ′ = v cos β tan δ / L − κ(s) s′<br/>"
               "v′ = a;  a′ = jerk;  δ′ = δ_rate", eq))
    s.append(P("κ(s) enters as parameters: the path's mean curvature over each step of the shifted previous plan, so "
               "the compiled solver does not depend on the route and one compiled solver serves every path. RK4 "
               "integration per step; the steps are 0.1 s for the first second and 0.2 s after."))
    s.append(figure(fig_nmpc(), 165))
    s.append(P("Figure 4. Left: the horizon's steps. Right: how bodies are kept apart.", caption))
    s.append(KeepTogether([P("Cost per step, each weighted by its step's length (w_* in Table 4):", h2), table([
        ["Term", "Expression", "Why"],
        ["speed", "(v − v_ref)²", "v_ref: SimLingo's speed, capped by the limit, tapered to stops, leads and bends"],
        ["lane", "(e_y − e_ref)²", "e_ref: SimLingo's path (0), kept inside the lane band; the left lane centre when borrowing"],
        ["heading", "e_ψ²", "aligned with the path"],
        ["accel, jerk", "a², jerk²", "comfort"],
        ["rate", "δ_rate²", "smooth steering"],
        ["lat", "(v² tan δ / L)²", "lateral acceleration"],
        ["slack", "Σ(s + s²) over the hard groups", "every constraint is soft, its slack priced at 1e5"],
        ["headway", "s_h + s_h²", "room to stop behind a lead at the horizon's end, softer (1e2)"],
    ], [20, 52, 100], mono_cols=(0, 1))]))
    s.append(P("Constraint groups (one slack each per step):", h2))
    s.append(table([
        ["Group", "Constraint"],
        ["corridor", "the body's front and rear corners within [lo, hi](s) less edge_margin"],
        ["movers", "each of the ego's 3 circles outside each mover's circle: r_ego + r + margin (+ growth · t)"],
        ["ahead", "the front gap_min behind a lead's rear; behind a stop gate, with the distance to stop at a_stop at the end"],
        ["behind", "the rear past a passing gate (going ahead of a crossing actor) by its time"],
        ["grip", "a² + a_lat² ≤ (μ g)²;  v ≤ v_max"],
        ["headway", "at the last step: front + gap + (v² − v_lead²) / (2 a_stop) ≤ the lead's rear"],
    ], [22, 150], mono_cols=(0,)))
    s.append(P("Bounds: v ≥ −1 m/s (not 0: an active v ≥ 0 over many stages at standstill stalls FATROP), a in "
               "[a_min, a_max], |jerk| ≤ jerk_max, |δ| ≤ delta_max, |δ_rate| ≤ delta_rate_max (Table 6).", small))

    s.append(P("5. The behaviour layer", h1))
    s.append(P("The NMPC is local: given a gate it will stop before it or go ahead of it, but it cannot choose which, "
               "and through a circle it has no gradient to go round. The behaviour layer makes those discrete choices "
               "and lets the NMPC solve each one."))
    s.append(figure(fig_behaviour(), 172))
    s.append(P("Figure 5. One behaviour step.", caption))
    s += bullets([
        "<b>Blocker</b>: a parked car or an object in the lane (reaching blocker_overlap into it), kept until passed: "
        "stopped behind, or gone round in the left lane (borrow_left) when it is within blocker_range and no visible "
        "oncoming traffic comes within oncoming_gap over the manoeuvre. Once out in the other lane the overtake is "
        "committed.",
        "<b>Lead</b>: a vehicle in the lane ahead (or cutting in, from when it does): a hard gap and a time gap in v_ref.",
        "<b>Crosser</b>: a walker or a vehicle whose prediction crosses the lane: a yield gate (front stays before it "
        "over its window ± gate_buffer) in every candidate but one pass:&lt;id&gt; per crosser.",
        "<b>Mover</b>: everything else near the path: circles the NMPC keeps clear of.",
        "<b>Rules</b>: a red light is a stop gate unless stopping needs more than dilemma_decel; amber, more than 1.5 "
        "a_ref; a stop sign holds stop_hold at rest within 4 m of the line, then releases; the route's end is a gate.",
        "<b>Choice</b>: the candidate with the lowest score among those with slack ≤ slack_ok (any, if none is): its "
        "cost, plus w_out_of_lane times the squared distance from the lane centre, plus pass_penalty per crosser "
        "gone ahead of; the last choice's score × hysteresis.",
    ])

    s.append(P("6. The world model", h1))
    s.append(table([
        ["Field", "Read from CARLA", "Stands in for", "Limits"],
        ["actors", "world.get_actors(): vehicles, walkers, static props", "3D detection + tracking",
         "within 60 m (walkers 40 m), line of sight by ray casts from 2.5 m above the car; velocity and acceleration "
         "from the tracked boxes, not CARLA's; kept 1.5 s after last seen"],
        ["parked", "hazard lights, lane, 4 s at rest with nothing ahead", "a classifier",
         "Bench2Drive's static-mesh parked cars: a 4.8 × 2.0 m box (CARLA's boxes for them are wrong)"],
        ["predictions", "the map's lanes", "a motion predictor", "vehicles along their lane, the rest at constant velocity, 5 s at 0.25 s"],
        ["lanes", "map waypoints along the path", "HD map / online mapping", "ego lane band, left lane (width, oncoming, borrowable), every 2 m"],
        ["stops", "light stop waypoints and state, stop-sign volumes", "light / sign detection", "the light's state now, never its phase"],
        ["route end", "the leaderboard's sparse plan", "given to every agent", "–"],
        ["v_max", "hero.get_speed_limit()", "sign reading", "–"],
        ["car's pose", "hero.get_transform()", "sensors on the car", "only to report the rest in the car's frame"],
    ], [18, 46, 32, 76], mono_cols=(0,)))
    s.append(P("Never read: other actors' controls, routes or Traffic Manager targets; scenario triggers; actors out "
               "of sight; future light phases.", small))

    s.append(P("7. Running it", h1))
    s.append(figure(fig_launch(), 172))
    s.append(P("Figure 6. One route.", caption))
    s.append(P("scripts/run_bench2drive.sh --agent simlingo_nmpc --routes 0-19 --no-cot --fast --live<br/>"
               "ssh -L 8765:localhost:8765 &lt;machine&gt;, then http://localhost:8765", eq))
    s += bullets([
        "One leaderboard process per route; a route with a finished result is skipped, so a stopped run carries on.",
        "Watchdog (b2d_evaluator.py, in the leaderboard's tick loop, outside the agent): the route is stopped and "
        "recorded as crashed once the ego has been at rest for --max-stopped seconds or after --max-time seconds of "
        "simulation time (0: off). Its score so far still counts.",
        "Sync check (b2d_evaluator.py): at a route's start, synchronous mode with a 0.05 s step; then every tick "
        "advancing exactly that step (a skipped or repeated frame is reported); every 10 s of simulation time, its "
        "speed against real time. Printed as \"Sync: ...\" lines in the route's log.",
        "The first NMPC in a fresh build/urban_cache compiles the solver (about a minute); later ones load it.",
    ])

    s.append(PageBreak())
    s.append(P("8. Parameters", h1))
    s.append(P("Read from the code when this document was built.", small))
    s.append(P("Table 4. UrbanNMPC::Options (core/urban/include/urban/urban_nmpc.hpp)", h2))
    s.append(table([["Name", "Default", "Meaning"]] +
                   [[n, v, c] for n, v, c in struct_fields("core/urban/include/urban/urban_nmpc.hpp", "struct Options {")],
                   [30, 18, 124]))
    s.append(P("The agent sets a_min = −7.0, jit = true, cache_dir = build/urban_cache.", small))
    s.append(P("Table 5. Behaviour::Options (core/urban/include/urban/behaviour.hpp)", h2))
    s.append(table([["Name", "Default", "Meaning"]] +
                   [[n, v, c] for n, v, c in struct_fields("core/urban/include/urban/behaviour.hpp", "struct Options {")],
                   [30, 18, 124]))
    s.append(P("Table 6. The vehicle: config/vehicles/mkz2020.json (derivations: docs/vehicle_parameters.md)", h2))
    s.append(table([["Name", "Value", "Source", "Meaning"]] + vehicle_rows(), [24, 22, 16, 110]))
    s.append(P("Body (the agent's): length 4.89 m, width 1.84 + 0.2 m (mirrors), centre of gravity 0.31 m ahead of "
               "the body's centre, 0.30 m ahead of the car's origin.", small))
    s.append(P("Table 7. The agent, the reference and the world model (Python constants)", h2))
    rows = [["Name", "Value", "Meaning", "File"]]
    for path in ["expert/agents/simlingo_nmpc_agent.py", "expert/simlingo_ref.py", "expert/world_model/route.py",
                 "expert/world_model/actors.py", "scripts/b2d_evaluator.py"]:
        for n, v, c in py_constants(path):
            if n in ("REPO", "PAGE"):
                continue
            rows.append([n, v.replace("float(os.environ.get(", "env ").replace(") or 0.0)", ""), c, path.split("/")[-1]])
    for n, v in py_defaults("expert/world_model/actors.py", "def __init__(self, expertctl, carla, world, carla_map, hero"):
        rows.append([n, v, {"look_range": "detection range [m]", "walker_range": "detection range for walkers [m]",
                            "keep": "a track is kept this long after last seen [s]"}.get(n, ""), "actors.py"])
    for n, v in py_defaults("expert/mkz_actuation.py", "def __init__(self, path, dt"):
        rows.append([n, v, {"dt": "control period [s]", "kp": "PI on the acceleration error: gain",
                            "ki": "PI: integral gain", "i_max": "PI: integral limit"}.get(n, ""), "mkz_actuation.py"])
    s.append(table(rows, [28, 30, 82, 32], mono_cols=(0, 1, 3)))
    s.append(P("Launcher defaults (run_bench2drive.sh): --seed 1, --tries 2, --max-stopped 30, --max-time 75, live "
               "view port 8765 (NMPC_LIVE_PORT). Offline scenario plant (sim/urban/scenarios.cpp): kinematic bicycle "
               "at 5 ms, acceleration lag 0.25 s, steering lag 0.05 s rate-limited to delta_rate_max; solves at 10 Hz, "
               "plays the plan at 20 Hz.", small))

    s.append(P("9. Where it stands", h1))
    s += bullets([
        "Offline: 22 scenarios (11 on the scenario's road, 11 on SimLingo-like local paths rebuilt each plan) drive "
        "correctly; parked_car_overtake (both) misses the 50 ms p95 solve-time check at about 52–59 ms.",
        "Bench2Drive route 0 (Town12, ParkingCutIn), latest run: 27.1 % route completion, no collisions, red lights, "
        "stop-sign or lane infractions; stopped by the watchdog after 30 s at rest behind a false blocker (the lane "
        "band came out 5.4 m wide, likely from the ±6-sample union in route.py's lanes). Solve time p50 32 ms, p95 "
        "105 ms, 36 of 423 plans failed.",
        "Not yet: SimLingo's own overtakes into the oncoming lane are held to the lane unless borrow_left is chosen; "
        "pedestrians at constant velocity only; no junction-branch hypotheses; no emergency vehicles.",
    ])

    doc = SimpleDocTemplate(out, pagesize=A4, leftMargin=19 * mm, rightMargin=19 * mm, topMargin=17 * mm,
                            bottomMargin=17 * mm, title="carla-expert-nmpc: architecture", author="carla-expert-nmpc")

    def footer(canvas, d):
        canvas.saveState()
        canvas.setFont("Sans", 7.5)
        canvas.setFillColor(colors.HexColor(MUTED))
        canvas.drawString(19 * mm, 10 * mm, "carla-expert-nmpc: architecture")
        canvas.drawRightString(A4[0] - 19 * mm, 10 * mm, str(d.page))
        canvas.restoreState()
    doc.build(s, onFirstPage=footer, onLaterPages=footer)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(REPO, "docs", "architecture.pdf"))
    a = ap.parse_args()
    build(a.out)
    print(a.out)
