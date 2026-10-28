"""Build a PDF of a PDM-Lite run on Bench2Drive (scripts/run_pdm_lite_all.sh): the scores, each scenario type, what
went wrong on the routes that were not a success, the routes without a result and how the run was made.

    uv run --no-project --with reportlab --with matplotlib python docs/report/make_pdm_lite_report.py \
        [--run ~/b2d_results/pdm_lite] [--out <run>/pdm_lite_bench2drive_report.pdf]

Reads <run>/routes.csv, res/merged.json and summary.md (scripts/summarize_b2d.py) and each route's result.json,
leaderboard.log and map.png.
"""
import argparse
import collections
import csv
import datetime
import glob
import io
import json
import os
import re
import subprocess

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
from reportlab.platypus import (Image, KeepTogether, Paragraph, SimpleDocTemplate, Spacer, Table,  # noqa: E402
                                TableStyle)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---- fonts and styles, as docs/report/make_m1_report.py

TTF = os.path.join(os.path.dirname(matplotlib.__file__), "mpl-data", "fonts", "ttf")
for name, file in [("Sans", "DejaVuSans.ttf"), ("Sans-Bold", "DejaVuSans-Bold.ttf"),
                   ("Sans-Oblique", "DejaVuSans-Oblique.ttf"), ("Mono", "DejaVuSansMono.ttf")]:
    pdfmetrics.registerFont(TTFont(name, os.path.join(TTF, file)))
pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-Bold", italic="Sans-Oblique", boldItalic="Sans-Bold")

INK, INK2, MUTED, RULE = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"  # categorical slots 1-3

body = ParagraphStyle("body", fontName="Sans", fontSize=9.2, leading=13.2, textColor=INK, alignment=TA_LEFT, spaceAfter=5)
small = ParagraphStyle("small", parent=body, fontSize=8, leading=10.6, textColor=INK2)
cell = ParagraphStyle("cell", parent=body, fontSize=7.6, leading=9.6, spaceAfter=0)
cell_b = ParagraphStyle("cell_b", parent=cell, fontName="Sans-Bold")
h1 = ParagraphStyle("h1", parent=body, fontName="Sans-Bold", fontSize=13.5, leading=17, spaceBefore=10, spaceAfter=6)
h2 = ParagraphStyle("h2", parent=body, fontName="Sans-Bold", fontSize=10.5, leading=14, spaceBefore=8, spaceAfter=4)
title = ParagraphStyle("title", parent=body, fontName="Sans-Bold", fontSize=19, leading=24, spaceAfter=4)
sub = ParagraphStyle("sub", parent=body, fontSize=10, leading=14, textColor=INK2, spaceAfter=12)
caption = ParagraphStyle("caption", parent=small, spaceBefore=2, spaceAfter=10)
stat_n = ParagraphStyle("stat_n", parent=body, fontName="Sans-Bold", fontSize=20, leading=24, spaceAfter=0)
stat_l = ParagraphStyle("stat_l", parent=small, fontSize=7.8, leading=10, spaceAfter=0)


def P(text, style=body):
    return Paragraph(text, style)


def mono(text):
    return f"<font face='Mono'>{text}</font>"


def bullets(items, style=body):
    return [Paragraph(f"•&nbsp;&nbsp;{t}", ParagraphStyle("b", parent=style, leftIndent=10, firstLineIndent=-8,
                                                           spaceAfter=2)) for t in items]


def table(rows, widths, header=True, align_right=()):
    data = []
    for i, r in enumerate(rows):
        data.append([P(str(c), cell_b if (header and i == 0) else cell) for c in r])
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if header else 0)
    style = [("VALIGN", (0, 0), (-1, -1), "TOP"),
             ("LINEBELOW", (0, 0), (-1, 0), 0.8, colors.HexColor(INK2)),
             ("LINEBELOW", (0, 1), (-1, -1), 0.3, colors.HexColor(RULE)),
             ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
             ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4)]
    t.setStyle(TableStyle(style))
    return t


def stats(items):
    """A row of big numbers, each with a label under it."""
    cells = [[P(n, stat_n) for n, _ in items], [P(l, stat_l) for _, l in items]]
    t = Table(cells, colWidths=[174 / len(items) * mm] * len(items))
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 0),
                           ("TOPPADDING", (0, 0), (-1, -1), 0), ("BOTTOMPADDING", (0, 0), (-1, -1), 1),
                           ("LINEABOVE", (0, 0), (-1, 0), 0.8, colors.HexColor(INK2))]))
    return t


def style_axes(ax, xlabel=None, ylabel=None):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
        ax.spines[side].set_linewidth(0.6)
    ax.tick_params(colors=INK2, labelsize=7, width=0.6, length=3)
    ax.grid(True, axis="x", color=RULE, linewidth=0.5)
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


def file_image(path, width_mm):
    from reportlab.lib.utils import ImageReader
    iw, ih = ImageReader(path).getSize()
    return Image(path, width=width_mm * mm, height=width_mm * mm * ih / iw)


# ---- the run

INFRACTION = {  # the leaderboard's infraction kinds, said plainly
    "collisions_vehicle": "hit a vehicle", "collisions_pedestrian": "hit a pedestrian",
    "collisions_layout": "hit a static object", "red_light": "ran a red light", "stop_infraction": "ran a stop sign",
    "outside_route_lanes": "left the route's lanes", "route_dev": "deviated from the route",
    "route_timeout": "ran out of time on the route", "vehicle_blocked": "got stuck",
    "scenario_timeouts": "waited past the scenario's time limit", "yield_emergency_vehicle_infractions":
    "did not yield to an emergency vehicle"}

NO_RESULT = {  # why a route has no result, from its leaderboard.log
    "dump": "Crashed in the wrapper's map dump (CARLA's next_until_lane_end); fixed during the run",
    "carla_setup": "The CARLA server crashed while the route was set up",
    "carla_drive": "The CARLA server crashed during the drive",
    "carla_start": "The CARLA server crashed on start-up",
}


def load(run):
    rows = list(csv.DictReader(open(os.path.join(run, "routes.csv"))))
    for r in rows:
        r["route"] = int(r["route"])
        for k in ("driving_score", "route_completion", "penalty", "sim_s", "wall_s"):
            r[k] = float(r[k])
        r["success"] = r["success"] == "1"
        rec = json.load(open(os.path.join(run, r["folder"], "result.json")))["_checkpoint"]["records"][0]
        r["detail"] = {k: v for k, v in rec["infractions"].items() if v and k != "min_speed_infractions"}
    merged = json.load(open(os.path.join(run, "res", "merged.json")))
    have = {r["folder"] for r in rows}
    missing = []
    for path in sorted(glob.glob(os.path.join(run, "[0-9]*_*")), key=lambda p: int(os.path.basename(p).split("_")[0])):
        folder = os.path.basename(path)
        if folder in have:
            continue
        log = open(os.path.join(path, "leaderboard.log"), errors="replace").read()
        drove = os.path.exists(os.path.join(path, "steps.csv")) and os.path.getsize(os.path.join(path, "steps.csv")) > 400
        if "Signal 11 caught" in log:
            why = "carla_drive" if drove else ("carla_setup" if "load_world success" in log else "carla_start")
        else:
            why = "dump"  # the agent's first tick, before the fix (route 52's log has the traceback)
        index, scenario = folder.split("_", 1)
        missing.append({"route": int(index), "scenario": scenario, "why": why})
    log = open(os.path.join(run, "run.log")).read().splitlines()
    start = next(l for l in log if " routes, " in l).split("from ")[1]
    end = [l for l in log if l.startswith("finished")][-1].split("finished ")[1]
    return rows, merged, missing, start, end


def what_happened(r):
    parts = []
    for kind, texts in r["detail"].items():
        what = INFRACTION.get(kind, kind.replace("_", " "))
        hit = [m.group(1) for t in texts for m in [re.search(r"type=([\w.]+)", t)] if m]
        if hit:
            names = sorted({" ".join(h.split(".")[1:]).replace("_", " ") for h in hit})  # vehicle.audi.tt: audi tt
            what += f" ({', '.join(names)})"
        if len(texts) > 1:
            what += f" ×{len(texts)}"
        parts.append(what)
    return "; ".join(parts)


def fig_scenarios(by):
    """Driving score of each scenario type below 100, lowest first."""
    imperfect = sorted(((sc, rs) for sc, rs in by.items() if mean(rs, "driving_score") < 100),
                       key=lambda kv: mean(kv[1], "driving_score"), reverse=True)
    fig, ax = plt.subplots(figsize=(7.0, 0.27 * len(imperfect) + 0.6))
    names = [sc for sc, _ in imperfect]
    ds = [mean(rs, "driving_score") for _, rs in imperfect]
    ax.barh(range(len(names)), ds, color=BLUE, height=0.62)
    for i, (v, (_, rs)) in enumerate(zip(ds, imperfect)):
        ok = sum(r["success"] for r in rs)
        ax.text(v + 1, i, f"{v:.1f}   ({ok}/{len(rs)} clean)", va="center", fontsize=6.8, color=INK2)
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names, fontsize=7, color=INK)
    ax.set_xlim(0, 118)
    ax.set_xticks([0, 20, 40, 60, 80, 100])
    style_axes(ax, xlabel="mean driving score of the scenario type")
    return figure_image(fig, 168)


def mean(rs, key):
    return sum(r[key] for r in rs) / len(rs)


def commit():
    try:
        return subprocess.check_output(["git", "-C", REPO, "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        return "?"


def build(run, out):
    rows, merged, missing, start, end = load(run)
    n = len(rows)
    by = collections.defaultdict(list)
    for r in rows:
        by[r["scenario"]].append(r)
    failed = sorted((r for r in rows if not r["success"]), key=lambda r: r["driving_score"])
    counts = collections.Counter()
    for r in rows:
        for k, v in r["detail"].items():
            counts[k] += len(v)
    perfect_types = sum(1 for rs in by.values() if mean(rs, "driving_score") >= 100)
    t0 = datetime.datetime.strptime(start, "%Y-%m-%d %H:%M:%S")
    t1 = datetime.datetime.strptime(end, "%Y-%m-%d %H:%M:%S")

    s = []
    s.append(P("PDM-Lite on Bench2Drive", title))
    s.append(P(f"The baseline for the expert NMPC: PDM-Lite, unmodified, on Bench2Drive's 220 routes · "
               f"carla-expert-nmpc @ {commit()} · run {t0:%d %B %Y}", sub))

    s.append(stats([(f"{merged['driving score']:.2f}", "driving score"),
                    (f"{100 * merged['success rate']:.1f} %", "success rate"),
                    (f"{mean(rows, 'route_completion'):.1f} %", "mean route completion"),
                    (f"{n} / 220", "routes with a result")]))
    s.append(Spacer(1, 8))

    s.append(P("Summary", h1))
    s.extend(bullets([
        f"<b>PDM-Lite drives Bench2Drive almost cleanly.</b> Driving score {merged['driving score']:.2f} and success "
        f"rate {100 * merged['success rate']:.1f} %, from Bench2Drive's own {mono('merge_route_json.py')}. "
        f"{n - len(failed)} of its {n} routes were clean; {perfect_types} of the {len(by)} scenario types scored 100 "
        "on every route.",
        f"<b>Where it loses points.</b> {len(failed)} routes were not a success: it got stuck twice after hitting a "
        f"static object, hit a vehicle {counts['collisions_vehicle']} times, ran {counts['red_light']} red lights, and "
        f"waited past a scenario's time limit {counts['scenario_timeouts']} times behind obstacles it would not pass.",
        f"<b>The numbers cover {n} routes, not 220.</b> {len(missing)} routes ran without a result: the CARLA server "
        "crashed on four, and a bug in this run's wrapper (since fixed) crashed five. They are left out of every "
        "number here, so the scores are not exactly Bench2Drive's 220-route figures: had those routes been driven, "
        "the scores could move either way.",
        "<b>PDM-Lite itself was not changed.</b> Its autopilot and controllers are SimLingo's "
        f"{mono('team_code/autopilot.py')} as published. A thin wrapper only fits it to Bench2Drive's leaderboard and "
        "records what it does (see How the run was made).",
    ]))

    s.append(P("Each scenario type", h1))
    s.append(P(f"Bench2Drive's 220 routes each hold one of {len(by)} scenario types. {perfect_types} types scored 100 on "
               f"every route. The {len(by) - perfect_types} below did not, lowest first:"))
    s.append(fig_scenarios(by))
    s.append(P("The mean driving score of each type's routes with a result, and how many of them were clean "
               "(completed with no infraction other than a minimum-speed one).", caption))

    s.append(P("Infractions", h2))
    s.append(table([["infraction", "count", "what it means"]]
                   + [[k, v, INFRACTION.get(k, "")] for k, v in counts.most_common()],
                   [42, 16, 116]))
    s.append(P("Minimum-speed checks (driving slower than the surrounding traffic) are left out, as Bench2Drive's "
               "success rate leaves them out.", caption))

    s.append(P(f"The {len(failed)} routes that were not a success", h1))
    s.append(P("Lowest score first. A collision with a vehicle costs a factor 0.6, a red light 0.7, a scenario timeout "
               "0.7 and a static object 0.65; a route the car stops on is scored by how far it got."))
    s.append(table([["route", "scenario", "score", "done", "what happened"]]
                   + [[f"{r['route']:03d}", r["scenario"], f"{r['driving_score']:.1f}", f"{r['route_completion']:.0f} %",
                       what_happened(r)] for r in failed],
                   [12, 52, 13, 13, 84]))
    s.append(Spacer(1, 8))
    blocked = [r for r in failed if "vehicle_blocked" in r["detail"]][:2]
    if blocked:
        imgs = [file_image(os.path.join(run, r["folder"], "map.png"), 84) for r in blocked]
        caps = [P(f"Route {r['route']:03d}, {r['scenario']}: {what_happened(r)}. Score {r['driving_score']:.1f}, "
                  f"{r['route_completion']:.0f} % of the route.", caption) for r in blocked]
        grid = Table([imgs, caps], colWidths=[87 * mm] * len(blocked))
        grid.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 0)]))
        s.append(KeepTogether([P("The two routes it got stuck on", h2), grid,
                               P("Dashed: the route planned; dots: the path driven, coloured by speed; ✕: an "
                                 "infraction. Each route's folder has the same map and a video.", caption)]))

    s.append(KeepTogether([
        P(f"The {len(missing)} routes without a result", h1),
        P("These ran but produced no score: the simulator or the wrapper crashed, not the driving. As asked, they "
          "were not run again; they are out of every number above."),
        table([["route", "scenario", "why"]]
              + [[f"{m['route']:03d}", m["scenario"], NO_RESULT[m["why"]]] for m in missing], [12, 56, 106])]))
    s.append(Spacer(1, 4))
    s.append(P(f"Running {mono('scripts/run_pdm_lite_all.sh --out ' + run)} again runs only these routes "
               "(about 15 minutes) and rewrites the summary with all of them.", small))

    s.append(P("How the run was made", h1))
    s.extend(bullets([
        "<b>Agent.</b> PDM-Lite, the rule-based privileged expert of carla_garage and SimLingo "
        f"({mono('external/simlingo/team_code/autopilot.py')}, unmodified), on the MAP track, as it was written for.",
        f"<b>Wrapper</b> ({mono('expert/agents/pdm_lite_agent.py')}). PDM-Lite subclasses carla_garage's "
        f"{mono('autonomous_agent_local')}, which Bench2Drive's leaderboard does not have; the wrapper maps it to "
        "Bench2Drive's base class. The only difference that reaches the agent is the sparse route's downsampling "
        "(every 50 points rather than 200), which feeds the command planner used for saved data, not the control. It "
        "also turns the dataset writing off, adds a chase camera and logs every tick. None of it changes the control.",
        f"<b>Evaluator.</b> Bench2Drive's {mono('leaderboard_evaluator.py')} through {mono('scripts/b2d_evaluator.py')}, "
        "which changes two things so three servers can share one GPU: a port is taken only with the two after it free "
        "(CARLA binds three), and the cleanup after a crash stops only that run's server.",
        f"<b>Setup.</b> CARLA 0.9.15, one NVIDIA L4, three routes at a time, traffic manager seed 1, one attempt per "
        f"route. The run went from {t0:%H:%M} to {t1:%H:%M} on {t0:%d %B}; the first hours were lost to the two "
        "infrastructure faults above, fixed during the run.",
        "<b>Repeatability.</b> PDM-Lite adds a small random steering noise (σ = 0.001) of its own, unseeded, so a "
        "second run will not be identical route by route.",
    ]))

    s.append(P("Files", h2))
    s.append(table([["", ""],
                    [mono("summary.md"), "this report's numbers as text, with links to each route"],
                    [mono("routes.csv"), "a row per route: score, completion, penalty, status, infractions"],
                    [mono("res/merged.json"), "Bench2Drive's merged result"],
                    [mono("&lt;NN&gt;_&lt;scenario&gt;/"), "each route: summary.md, map.png, video.mp4 (a chase camera with "
                                               "what the autopilot is doing), steps.csv (each tick), result.json, "
                                               "leaderboard.log"]], [40, 134], header=False))
    s.append(Spacer(1, 6))
    s.append(P(f"Results: {mono(run)}. Rebuild this report: "
               f"{mono('uv run --no-project --with reportlab --with matplotlib python docs/report/make_pdm_lite_report.py')}",
               small))

    def footer(canvas, doc):
        canvas.saveState()
        canvas.setFont("Sans", 7)
        canvas.setFillColor(colors.HexColor(MUTED))
        canvas.drawString(18 * mm, 10 * mm, "carla-expert-nmpc · PDM-Lite on Bench2Drive")
        canvas.drawRightString(A4[0] - 18 * mm, 10 * mm, str(doc.page))
        canvas.restoreState()

    doc = SimpleDocTemplate(out, pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm, topMargin=16 * mm,
                            bottomMargin=16 * mm, title="PDM-Lite on Bench2Drive", author="carla-expert-nmpc")
    doc.build(s, onFirstPage=footer, onLaterPages=footer)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=os.path.expanduser("~/b2d_results/pdm_lite"))
    ap.add_argument("--out")
    a = ap.parse_args()
    out = a.out or os.path.join(a.run, "pdm_lite_bench2drive_report.pdf")
    build(os.path.abspath(a.run), out)
    print(out)
