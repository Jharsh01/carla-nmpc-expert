"""Build docs/m0_m1_report.pdf: what milestones M0 and M1 did, with the offline scenario results.

    python docs/report/make_m1_report.py [--traces results/urban] [--out docs/m0_m1_report.pdf]

Needs reportlab and matplotlib (not in the project's environment: any Python 3 with both will do), and the scenario
traces: build/urban_scenarios --out results/urban > results/urban/summary.txt
"""
import argparse
import csv
import subprocess
import io
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
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

# ---- fonts: matplotlib's DejaVu, which has the Greek letters the formulas need

TTF = os.path.join(os.path.dirname(matplotlib.__file__), "mpl-data", "fonts", "ttf")
for name, file in [("Sans", "DejaVuSans.ttf"), ("Sans-Bold", "DejaVuSans-Bold.ttf"),
                   ("Sans-Oblique", "DejaVuSans-Oblique.ttf"), ("Mono", "DejaVuSansMono.ttf")]:
    pdfmetrics.registerFont(TTFont(name, os.path.join(TTF, file)))
pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-Bold", italic="Sans-Oblique", boldItalic="Sans-Bold")

INK, INK2, MUTED, RULE = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"  # categorical slots 1-3

body = ParagraphStyle("body", fontName="Sans", fontSize=9.2, leading=13.2, textColor=INK, alignment=TA_LEFT, spaceAfter=5)
small = ParagraphStyle("small", parent=body, fontSize=8, leading=10.6, textColor=INK2)
cell = ParagraphStyle("cell", parent=body, fontSize=7.8, leading=10, spaceAfter=0)
cell_b = ParagraphStyle("cell_b", parent=cell, fontName="Sans-Bold")
h1 = ParagraphStyle("h1", parent=body, fontName="Sans-Bold", fontSize=13.5, leading=17, spaceBefore=10, spaceAfter=6)
h2 = ParagraphStyle("h2", parent=body, fontName="Sans-Bold", fontSize=10.5, leading=14, spaceBefore=8, spaceAfter=4)
title = ParagraphStyle("title", parent=body, fontName="Sans-Bold", fontSize=19, leading=24, spaceAfter=4)
sub = ParagraphStyle("sub", parent=body, fontSize=10, leading=14, textColor=INK2, spaceAfter=12)
eq = ParagraphStyle("eq", parent=body, fontName="Mono", fontSize=8.2, leading=11.5, leftIndent=10, spaceAfter=6,
                    textColor=INK)
caption = ParagraphStyle("caption", parent=small, spaceBefore=2, spaceAfter=10)


def P(text, style=body):
    return Paragraph(text, style)


def bullets(items, style=body):
    return [Paragraph(f"•&nbsp;&nbsp;{t}", ParagraphStyle("b", parent=style, leftIndent=10, firstLineIndent=-8,
                                                           spaceAfter=2)) for t in items]


def table(rows, widths, header=True, mono_cols=()):
    data = []
    for i, r in enumerate(rows):
        data.append([P(str(c), cell_b if (header and i == 0) else cell) for c in r])
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if header else 0)
    style = [("VALIGN", (0, 0), (-1, -1), "TOP"),
             ("LINEBELOW", (0, 0), (-1, 0), 0.8, colors.HexColor(INK2)),
             ("LINEBELOW", (0, 1), (-1, -1), 0.3, colors.HexColor(RULE)),
             ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
             ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4)]
    t.setStyle(TableStyle(style))
    return t


# ---- the traces and the figures

def load(traces, name):
    with open(os.path.join(traces, name + ".csv")) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def style_axes(ax, xlabel=None, ylabel=None):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
        ax.spines[side].set_linewidth(0.6)
    ax.tick_params(colors=INK2, labelsize=7, width=0.6, length=3)
    ax.grid(True, color=RULE, linewidth=0.5)
    ax.set_axisbelow(True)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=7.5, color=INK2)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=7.5, color=INK2)


def figure_image(fig, width_mm):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    buf.seek(0)
    w, h = fig.get_size_inches()
    return Image(buf, width=width_mm * mm, height=width_mm * mm * h / w)


def label_end(ax, x, y, text, color_ink=INK2, dx=0.15):
    ax.annotate(text, (x, y), xytext=(4, 0), textcoords="offset points", fontsize=7, color=color_ink, va="center")


def fig_stops(traces, front):
    """Speed over time where the car must stop: the light, the stop sign, the lead braking, the crossing car."""
    cases = [("red_light", "Red light at s = 60 m, green at 9 s", 9.0),
             ("stop_sign", "Stop sign at s = 50 m", None),
             ("lead_brake", "Lead brakes at 6 m/s² from 3 s", 3.0),
             ("crossing_yield", "Car crossing at s = 45 m", None)]
    fig, axes = plt.subplots(1, 4, figsize=(10.5, 2.3), sharey=True)
    for ax, (name, ttl, mark) in zip(axes, cases):
        d = load(traces, name)
        ax.plot(d["t"], d["v"], color=BLUE, lw=1.6)
        if mark is not None:
            ax.axvline(mark, color=MUTED, lw=0.8, ls=(0, (3, 2)))
        ax.set_title(ttl, fontsize=7.5, color=INK, loc="left")
        style_axes(ax, "time [s]", "speed [m/s]" if ax is axes[0] else None)
    fig.tight_layout(w_pad=1.0)
    return fig


def fig_overtake(traces, body_len, body_w):
    """The ego's path round the parked car, clear road against waiting for oncoming traffic."""
    fig, ax = plt.subplots(figsize=(10.5, 2.4))
    lane = 3.5
    for e in (-0.5 * lane, 0.5 * lane, 1.5 * lane):
        ax.axhline(e, color=MUTED, lw=0.7, ls="-" if e != 0.5 * lane else (0, (6, 4)))
    ax.add_patch(plt.Rectangle((55 - 2.4, -1.0), 4.8, 2.0, color="#b9b8b2", lw=0))
    ax.text(55, 0, "parked", fontsize=6.5, color=INK, ha="center", va="center")
    for name, col, lbl in [("parked_car_overtake", BLUE, "road clear: overtakes at once"),
                           ("parked_car_wait", ORANGE, "oncoming car: waits, then overtakes")]:
        d = load(traces, name)
        ax.plot(d["s"], d["e_y"], color=col, lw=1.6, label=lbl)
    ax.set_xlim(10, 120)
    ax.set_ylim(-2.2, 6.2)
    ax.text(118, 0.5 * lane + 1.75, "oncoming lane", fontsize=6.5, color=INK2, ha="right", va="center")
    ax.text(118, -1.1, "ego lane", fontsize=6.5, color=INK2, ha="right", va="center")
    style_axes(ax, "along the route s [m]", "lateral e_y [m]")
    ax.legend(fontsize=7, frameon=False, loc="upper left")
    fig.tight_layout()
    return fig


def fig_cut_ped(traces):
    """Speed and lateral position for the cut-in and the pedestrian: braking, not swerving."""
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 2.3))
    for name, col, lbl in [("cut_in", BLUE, "cut-in"), ("pedestrian_behind_van", ORANGE, "pedestrian from behind a van")]:
        d = load(traces, name)
        axes[0].plot(d["t"], d["v"], color=col, lw=1.6)
        axes[1].plot(d["t"], d["e_y"], color=col, lw=1.6)
        label_end(axes[0], d["t"][-1], d["v"][-1], lbl)
    axes[0].set_title("Speed", fontsize=7.5, color=INK, loc="left")
    axes[1].set_title("Lateral position in the lane (the body's edge reaches the lane edge at ±0.67 m)",
                      fontsize=7.5, color=INK, loc="left")
    for e in (-0.67, 0.67):
        axes[1].axhline(e, color=MUTED, lw=0.7, ls=(0, (3, 2)))
    style_axes(axes[0], "time [s]", "speed [m/s]")
    style_axes(axes[1], "time [s]", "e_y [m]")
    axes[0].set_xlim(0, 15.5)
    fig.tight_layout(w_pad=2.0)
    return fig


def fig_step_times(traces, names):
    """Wall time of each behaviour step (all candidates, in parallel), per scenario."""
    data = []
    for n in names:
        d = load(traces, n)
        data.append([v for v in d["step_ms"] if v > 0])
    fig, ax = plt.subplots(figsize=(10.5, 2.5))
    pos = list(range(len(names)))
    bp = ax.boxplot(data, positions=pos, widths=0.5, whis=(5, 95), showfliers=True, patch_artist=True,
                    medianprops=dict(color=INK, lw=1.2), boxprops=dict(facecolor="#d6e6f8", edgecolor=BLUE, lw=0.8),
                    whiskerprops=dict(color=BLUE, lw=0.8), capprops=dict(color=BLUE, lw=0.8),
                    flierprops=dict(marker="o", markersize=2.5, markerfacecolor=MUTED, markeredgecolor="none"))
    ax.axhline(50, color=ORANGE, lw=1.0, ls=(0, (4, 2)))
    ax.text(len(names) - 0.5, 51.5, "50 ms budget", fontsize=7, color=INK2, ha="right", va="bottom")
    ax.set_xticks(pos)
    ax.set_xticklabels([n.replace("_", " ") for n in names], rotation=25, ha="right", fontsize=7)
    style_axes(ax, None, "step time [ms]")
    fig.tight_layout()
    return fig


# ---- the document

def commit():
    try:
        return subprocess.check_output(["git", "-C", REPO, "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        return "?"


def build(traces, out):
    body_len, body_w, cg = 4.79, 2.16, 0.45
    front = 0.5 * body_len - cg
    summary = {}
    path = os.path.join(traces, "summary.txt")
    if os.path.exists(path):
        for line in open(path):
            parts = line.split()
            if len(parts) >= 8 and parts[1] in ("ok", "FAIL"):
                summary[parts[0]] = parts
    names = ["curve", "lead_brake", "cut_in", "pedestrian_behind_van", "red_light", "red_light_dilemma", "stop_sign",
             "parked_car_overtake", "parked_car_wait", "crossing_yield", "crossing_pass"]

    s = []
    s.append(P("Expert NMPC for Bench2Drive", title))
    s.append(P("Milestones M0 and M1: the new repository, the urban NMPC and the behaviour layer, tested offline · "
               f"carla-expert-nmpc @ {commit()} · 27 October 2026", sub))

    s.append(P("Summary", h1))
    s.extend(bullets([
        "<b>M0, done.</b> A new repository, <font face='Mono'>~/carla-expert-nmpc</font>, holding the NMPC core and the "
        "SimLingo / Bench2Drive scripts, copied from carla-track-control at <font face='Mono'>8495ae9</font> (your "
        "uncommitted work there was committed first, so the copy has an exact source). It builds against SimLingo's "
        "Python 3.8 environment, in which the leaderboard and both agents will run.",
        "<b>M1, done offline.</b> A new NMPC for urban driving (<font face='Mono'>core/urban</font>) and, brought "
        "forward from M3, the behaviour layer that decides what it is asked to do. All 11 closed-loop scenarios pass: no "
        "contact (nearest 1.08 m), the body on the road, every scenario's own goal met.",
        "<b>Speed.</b> One NMPC solve takes about 10 ms once compiled (JIT, cached on disk, loaded in 0.2 s). A "
        "behaviour step is 13–19 ms at p95 with one candidate, and up to 49 ms with three solved in parallel.",
        "<b>Not yet done.</b> Nothing has run in CARLA. The offline plant is a kinematic bicycle with actuator lags, "
        "not CARLA's Lincoln MKZ; its parameters are still the race-track Model 3's. M2 (the world model, the MKZ's "
        "actuation, the expert on an empty route) is next and has started.",
    ]))

    s.append(P("1&nbsp;&nbsp;The new repository (M0)", h1))
    s.append(P("The repository was started fresh, copying what the expert needs and leaving the race track behind. "
               "<font face='Mono'>PROVENANCE.md</font> lists where each part came from."))
    s.append(table([
        ["Here", "From carla-track-control", "Notes"],
        ["core/track_control/", "track_control/", "Unchanged: race-track NMPC, pure pursuit, paths, bicycle model, directives. Its 3 tests pass."],
        ["core/urban/", "new", "The urban NMPC and the behaviour layer (section 2)."],
        ["sim/plant/, sim/urban/", "sim/plant/ (bicycle only)", "The offline scenarios are new."],
        ["bindings/expertctl.cpp", "bindings/trackctl.cpp", "Python module: paths, vehicle parameters, the urban NMPC."],
        ["expert/", "carla_bridge/frames.py, actuation.py", "CARLA ↔ library frames; the world model and agents go here."],
        ["scripts/", "scripts/", "CARLA, SimLingo, Bench2Drive; env.sh now uses SimLingo's venv."],
        ["external/", "symlink", "Points to the old repository's: CARLA, SimLingo, checkpoints (about 25 GB) shared."],
    ], [38, 46, 86]))
    s.append(Spacer(1, 4))
    s.append(P("Changes outside the new repository: the snapshot commit <font face='Mono'>8495ae9</font> in "
               "carla-track-control (on the working branch, not main), and CasADi 3.7.2, pybind11, imageio-ffmpeg and "
               "pytest installed into <font face='Mono'>external/simlingo_venv</font>. The JIT needs a C compiler at "
               "run time (gcc, already present).", small))

    s.append(P("2&nbsp;&nbsp;The urban NMPC", h1))
    s.append(P("The race-track NMPC models tyre slip, which is undefined at standstill, and handles two cars as "
               "ellipses. Bench2Drive needs stop-and-go traffic, traffic rules and interaction, so the new NMPC uses a "
               "kinematic bicycle in the route's path coordinates, with grip as a friction circle on the planned "
               "accelerations:"))
    s.append(P("state  x = [s, e_y, e_psi, v, a, δ]        input  u = [jerk, δ_rate]        (′ = d/dt)<br/>"
               "β      = atan(b·tan δ / L)          b: centre of gravity to rear axle, L: wheelbase<br/>"
               "s′     = v·cos(e_psi + β) / (1 − κ·e_y)<br/>"
               "e_y′   = v·sin(e_psi + β)<br/>"
               "e_psi′ = v·cos β·tan δ / L − κ·s′<br/>"
               "v′ = a        a′ = jerk        δ′ = δ_rate", eq))
    s.append(P("Horizon 5 s: steps of 0.1 s for the first second, 0.2 s after (30 steps), integrated by RK4. It is "
               "solved by FATROP (an interior-point method with a Riccati recursion over the stages) and "
               "warm-started from the previous plan, shifted by the time since. Every constraint is soft, with one "
               "slack per group per step. A plan therefore always exists, the least-violating one when a violation "
               "can no longer be avoided."))
    s.append(table([
        ["Group", "Keeps", "Weight"],
        ["corridor", "the body's front and rear corners inside the drivable band [lo(s), hi(s)], 0.15 m from its edges", "10⁵"],
        ["movers", "the body's 3 circles outside each mover's clearance circle (radius + growth·t + 0.5 m), 8 slots", "10⁵"],
        ["ahead", "front ≥ 0.5 m behind a lead's rear; front behind stop gates (with room to stop at the last step)", "10⁵"],
        ["behind", "rear past pass gates (going before a crossing car)", "10⁵"],
        ["grip", "a² + a<sub>lat</sub>² ≤ (μg)²; v ≤ speed limit", "10⁵"],
        ["headway", "at the last step: room to stop behind the lead were it to brake", "10²"],
    ], [22, 128, 20]))
    s.append(Spacer(1, 4))
    s.append(P("The cost tracks the speed asked for and the line asked for, with small weights on heading, "
               "acceleration, jerk, steering rate and lateral acceleration. The speed asked for is tapered, step by "
               "step along the shifted previous plan: to standstill at each stop gate along √(2·a<sub>ref</sub>·d), to "
               "the lead's speed and to the time gap (d / 1 s), and to √(a<sub>lat,ref</sub>/|κ|) in bends "
               "(a<sub>ref</sub> = 2, a<sub>lat,ref</sub> = 3 m/s²). The hard bounds are a ∈ [−7, 3] m/s², "
               "|jerk| ≤ 15 m/s³, the steering angle and rate of the vehicle parameters, and v ≥ −1 m/s."))

    s.append(P("3&nbsp;&nbsp;The behaviour layer", h1))
    s.append(P("A local NMPC cannot choose between waiting for a pedestrian and going before them, or between staying "
               "behind a parked car and overtaking it: a plan through the middle of an obstacle has no gradient towards "
               "either side, so the solver stays there. The scenarios hit exactly this, so the behaviour layer (planned "
               "for M3) was built now. At each step it takes a <font face='Mono'>Scene</font>: the ego, the lanes "
               "along the route, the actors with predicted trajectories, and the stop lines. Then:"))
    s.extend(bullets([
        "<b>Classify</b> each actor against the ego lane. A <i>lead</i> is in the lane ahead going along it (a cut-in "
        "counts from when it enters). A <i>blocker</i> is a parked car or object in the lane, kept until passed. A "
        "<i>crosser</i> is a walker, or a vehicle at an angle, entering the lane, with its time window and the span "
        "of route it takes. A <i>mover</i> is anything else nearby.",
        "<b>Build candidates.</b> <font face='Mono'>base</font> yields to every crosser (a gate keeping the front "
        "behind it over its window ± 1 s) and waits behind blockers. <font face='Mono'>pass:&lt;id&gt;</font> goes "
        "before one crosser, only if the car can clear its span in time. <font face='Mono'>borrow_left</font> widens "
        "the corridor into the left lane round the blockers, only if that lane may be used and oncoming traffic leaves "
        "a gap; once the car is out in the other lane it finishes the overtake.",
        "<b>Solve</b> the candidates in parallel, each on its own NMPC, warm-started from its own last plan.",
        "<b>Choose</b> on what each plan does, not on its NMPC cost (the tapered speeds make waiting look free). The "
        "score is speed against the untapered speed asked for, comfort, time out of the ego lane, slack, and +300 for "
        "a pass. Plans within the hard constraints win; the last choice's score is ×0.9.",
        "<b>Rules.</b> A stop sign is released after 1 s at rest within 4 m of the line. A light turning red (amber) "
        "that needs more than 5 (3) m/s² to stop for is driven through. The end of the route is a stop gate.",
    ]))

    s.append(PageBreak())
    s.append(P("4&nbsp;&nbsp;Offline results", h1))
    s.append(P("<font face='Mono'>build/urban_scenarios</font> (also a ctest) drives each scenario in closed loop: "
               "the plant is a kinematic bicycle with a 0.25 s lag on the acceleration and a rate-limited steering lag. "
               "The behaviour layer steps every 0.1 s and its plan is played at 20 Hz, as in the leaderboard. Actors "
               "follow scripts, and are predicted at constant velocity (to standstill when braking) from when they "
               "become visible. The pedestrian appears 1.6 s in, 18 m ahead, from behind the van. Contact is an exact "
               "oriented-box test; clearance is the true distance between the bodies."))
    rows = [["Scenario", "Result", "Nearest body [m]", "Peak a<sub>lat</sub> [m/s²]", "Step p95 [ms]", "Step max [ms]",
             "Candidates chosen"]]
    for n in names:
        r = summary.get(n)
        if r:
            worst = max(v for v in load(traces, n)["step_ms"] if v > 0)
            rows.append([n.replace("_", " "), r[1], r[2] if r[2] != "inf" else "–", r[4], r[6], f"{worst:.0f}",
                         r[8] if len(r) > 8 else ""])
    s.append(table(rows, [40, 13, 22, 22, 18, 18, 41]))
    s.append(P("All 11 pass. The candidate sequence shows the behaviour layer at work: the overtakes go base → "
               "borrow_left → base, and the crossing car that is far enough off in time is passed (base → pass:1 → "
               "base). The step time is highest when three candidates are solved at once, and its worst cases (up to about 105 ms) are the first solves after the scene changes abruptly: a new candidate starting cold, or the pedestrian appearing 18 m ahead.", caption))

    s.append(KeepTogether([figure_image(fig_stops(traces, front), 175),
                           P("<b>Figure 1.</b> Speed where the car must stop. It drives up and stops, rather than creeping "
                             "(the speed asked for tapers to the stop). At the light it stops, waits and goes on green. At "
                             "the stop sign it holds 1 s. Behind the braking lead it stops at the standstill gap. For the "
                             "crossing car it slows, lets it through, and goes on.", caption)]))
    s.append(KeepTogether([figure_image(fig_overtake(traces, body_len, body_w), 175),
                           P("<b>Figure 2.</b> Path round the parked car (grey). With the road clear it moves out and "
                             "back. With a car coming the other way it waits behind the parked car until the gap is "
                             "clear, then overtakes. It merges back over about 25 m, not in a jump.", caption)]))
    s.append(KeepTogether([figure_image(fig_cut_ped(traces), 175),
                           P("<b>Figure 3.</b> The cut-in and the pedestrian. The car brakes for both and stays in its "
                             "lane. For the pedestrian it stops, edging to the left side of its lane to keep clear of the "
                             "van, then goes on once the pedestrian has crossed. Before the time gap was moved into the "
                             "speed asked for, the cut-in made it swerve out of its lane (section 5).", caption)]))
    s.append(KeepTogether([figure_image(fig_step_times(traces, names), 175),
                           P("<b>Figure 4.</b> Wall time per behaviour step (box: quartiles; whiskers: 5th–95th "
                             "percentile; dots beyond). One candidate: 10–20 ms. With two or three candidates in "
                             "parallel the p95 reaches 40–50 ms, and single steps reach about 105 ms right after an abrupt "
                             "change (a new candidate starting cold, the pedestrian appearing).", caption)]))

    s.append(P("5&nbsp;&nbsp;What the scenarios taught (design decisions)", h1))
    s.append(P("The first version failed 7 of 9 scenarios. Each row below is a failure found that way, and the change "
               "that fixed it. These are the reasons the formulation looks as it does."))
    s.append(table([
        ["Symptom", "Cause", "Change"],
        ["Solves at the 100-iteration limit (≈200 ms) whenever a stop was planned",
         "Dual infeasibility crawls after the plan has converged; degenerate multipliers at standstill",
         "Stop on FATROP's acceptable level (0.1, 3 iterations); v ≥ −1 instead of v ≥ 0"],
        ["Never stops at a red light; creeps towards it",
         "No terminal condition; a convex speed cost spreads a fixed distance over the horizon (Jensen)",
         "Room to stop at the last step; speed asked for tapered to stops, leads and bends"],
        ["Drives into the pedestrian's path", "Slack weight 10³: a full violation was cheaper than braking",
         "Slack weight 10⁵"],
        ["Plan passes straight through obstacles; stuck behind the parked car",
         "No gradient along the route through a circle's centre (a local minimum)",
         "Behaviour layer: yield/pass gates and lane borrowing as discrete candidates"],
        ["Overtake never chosen", "Waiting costs nearly nothing under tapered speeds",
         "Score candidates on outcomes against the untapered speed"],
        ["Hits the parked car mid-overtake", "Blocker dropped once alongside it; the borrow candidate vanished",
         "Blockers kept until passed; overtake committed once in the other lane"],
        ["Swerves back at 7 m/s² after an overtake", "The corridor jumped back to the lane",
         "The corridor narrows back from where the car is over ≥ 25 m"],
        ["Swerves during the cut-in",
         "A high-priced position constraint pays the plan to weave (steering shortens progress along s)",
         "Time gap in the speed asked for; only the hard 0.5 m gap as a constraint"],
        ["Body a few cm past the lane edge", "Plans hug the edge; the plant's lag overshoots",
         "0.15 m margin to the corridor edge"],
        ["27 ms per solve", "Function evaluations are 75% of the time",
         "JIT to native code, cached on disk (66 s once, then 0.2 s): about 10 ms"],
    ], [52, 62, 56]))

    s.append(P("6&nbsp;&nbsp;Limitations and open points", h1))
    s.extend(bullets([
        "<b>Offline plant.</b> It is a kinematic bicycle and its parameters are the race-track Model 3's, so these "
        "results show the logic works, not that it drives CARLA's MKZ well. M2 identifies the MKZ.",
        "<b>Prediction.</b> The offline actors are predicted perfectly at constant velocity. CARLA's traffic will "
        "not be: junction turns, braking, pedestrians stopping.",
        "<b>Geometry.</b> Only straight roads and one bend. No junction geometry, lane changes along the route, "
        "right-side nudging, or several blockers in a row. These come with real maps.",
        "<b>Step-time margin.</b> It is thin with three candidates (49 ms at p95), and the first solve after an abrupt change "
        "(a new candidate starting cold, an actor appearing close) reaches about 105 ms. The NMPC runs at 10 Hz in a synchronous simulator, so this costs wall time, not "
        "safety, but the margin should grow. Options: start a new candidate from the chosen plan rather than cold, "
        "fewer mover slots, fewer steps.",
        "<b>Circles are conservative laterally.</b> The ego is covered by 3 circles of radius 1.34 m against a half "
        "width of 1.08 m, so tight gaps (a narrow lane past a parked van) are lost.",
        "<b>Mode B is not started.</b> SimLingo's path and speed waypoints have an entry point in the Scene "
        "(line_s/line_e, v_ref), but nothing feeds it yet.",
    ]))

    s.append(P("7&nbsp;&nbsp;Next: M2", h1))
    s.extend(bullets([
        "<b>World model</b> (<font face='Mono'>expert/world_model</font>). CARLA to Scene, limited to what a sensor "
        "stack could estimate: actors in range and visible (a lidar-point or ray test), velocities from tracked boxes "
        "rather than the simulator's, the current state of lights only, lanes from the map. Map-based prediction for "
        "vehicles.",
        "<b>Actuation for the Lincoln MKZ.</b> Scripted runs to measure throttle and brake against acceleration over "
        "speed, and steer against road-wheel angle. Then an inverse map with a PI loop on the acceleration.",
        "<b>The expert agent</b> (a leaderboard AutonomousAgent). First on an empty route: lane keeping within "
        "0.3 m, smooth stops at lights.",
    ]))
    s.append(P("Rebuild this report: <font face='Mono'>build/urban_scenarios --out results/urban &gt; "
               "results/urban/summary.txt; python docs/report/make_m1_report.py</font>", small))

    def footer(canvas, doc):
        canvas.saveState()
        canvas.setFont("Sans", 7)
        canvas.setFillColor(colors.HexColor(MUTED))
        canvas.drawString(18 * mm, 10 * mm, "carla-expert-nmpc · M0 and M1")
        canvas.drawRightString(A4[0] - 18 * mm, 10 * mm, str(doc.page))
        canvas.restoreState()

    doc = SimpleDocTemplate(out, pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm, topMargin=16 * mm,
                            bottomMargin=16 * mm, title="Expert NMPC for Bench2Drive: M0 and M1",
                            author="carla-expert-nmpc")
    doc.build(s, onFirstPage=footer, onLaterPages=footer)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--traces", default=os.path.join(REPO, "results", "urban"))
    ap.add_argument("--out", default=os.path.join(REPO, "docs", "m0_m1_report.pdf"))
    a = ap.parse_args()
    build(a.traces, a.out)
    print(a.out)
