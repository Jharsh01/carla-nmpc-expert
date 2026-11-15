"""Write report.md from runs.json and the hand-written failure analyses below."""
import json, os

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
runs = json.load(open(f"{root}/runs.json"))
by = {r["name"]: r for r in runs}

GROUPS = [
    ("Obstacles in our lane: go round them", ["ParkedObstacle", "ParkedObstacleTwoWays", "ConstructionObstacle",
      "ConstructionObstacleTwoWays", "Accident", "AccidentTwoWays", "HazardAtSideLane", "HazardAtSideLaneTwoWays",
      "VehicleOpensDoorTwoWays", "BlockedIntersection", "InvadingTurn"]),
    ("Cars cutting in or stopping ahead", ["ParkingCutIn", "StaticCutIn", "HighwayCutIn", "HardBreakRoute", "ParkingExit"]),
    ("Junctions: turning across or into traffic", ["SignalizedJunctionLeftTurn", "NonSignalizedJunctionLeftTurn",
      "SignalizedJunctionLeftTurnEnterFlow", "NonSignalizedJunctionLeftTurnEnterFlow", "SignalizedJunctionRightTurn",
      "NonSignalizedJunctionRightTurn", "InterurbanActorFlow", "InterurbanAdvancedActorFlow", "EnterActorFlow",
      "OppositeVehicleRunningRedLight", "OppositeVehicleTakingPriority", "CrossingBicycleFlow"]),
    ("Pedestrians and cyclists crossing", ["DynamicObjectCrossing", "ParkingCrossingPedestrian", "PedestrianCrossing",
      "VehicleTurningRoute", "VehicleTurningRoutePedestrian"]),
    ("Traffic rules only (no scenario actor)", ["VanillaSignalizedTurnEncounterRedLight",
      "VanillaSignalizedTurnEncounterGreenLight", "VanillaNonSignalizedTurn",
      "VanillaNonSignalizedTurnEncounterStopsign", "T_Junction", "ControlLoss"]),
]

# Hand-written from the NMPC logs (runs/*/viz/*/nmpc.jsonl, timelines in analysis/) and the video frames.
FAIL = {
"039_StaticCutIn": dict(kind="Collision", title="Too fast past a stopped queue; the cut-in car hit us side-on",
 what="""The scenario parks a queue of cars in the lane to our left; one of them cuts into our lane. Our car accelerated
at about 4 m/s² to **17.4 m/s (63 km/h)** on the base plan, beside the stopped queue, with no note about it in the
planner. The cut-in car (id 3708) sat 3.4 m to our left. From t ≈ 8.4 s it was coming over and the planner braked
hard (down to −7 m/s²), but at t ≈ 9.9 s the car was 1.4 m to our side and the two collided; a second car of the
queue (id 3698) was hit at t ≈ 10.6 s.""",
 why="""The stopped queue next to us is not treated as a risk: nothing caps speed beside stopped traffic in the next
lane, and the cut-in is only predicted once the car is already moving across. PDM-Lite drives past such queues
slowly.""",
 fix="Cap speed beside a stopped queue in the adjacent lane (StaticCutIn), and predict a cut-in from a stationary car whose nose turns toward our lane."),
"073_VehicleOpensDoorTwoWays": dict(kind="Collision", title="Drove past the open door without moving over",
 what="""After waiting for oncoming traffic (notes: "blocker: oncoming traffic, waiting"), the car moved off at about
2 m/s. At t ≈ 21.5 s it briefly chose `borrow_left` and then went back to `base` (keep lane, 0.3 m offset) and
drove past the parked SUV whose door was open into our lane. It struck the SUV (id 4708, 2 m ahead and 2.4 m to the
right) at t ≈ 22 s at about 3 m/s (frame below).""",
 why="""The open door was not part of the obstacle shape: the SUV is a parked car whose box does not include the
door, so the base plan thought it had room. The borrow candidate was dropped after one plan.""",
 fix="Add the open door to the parked car's footprint (or grow the blocker on its traffic side), so the base plan has to move over or borrow."),
"103_SignalizedJunctionLeftTurn": dict(kind="Collision → stuck", title="Turned left in front of oncoming traffic",
 what="""Went at green, accelerating hard (up to 6.6 m/s²) to 9.4 m/s on `cross:3875`, judging it could go in
front of car 3875. An oncoming car (id 3873) was in the predicted crossing area the whole time
("crosser in the lane from 0.0 s…"). It hit us at t ≈ 14.8 s in the middle of the junction. Then the car stood
there until the 30 s watchdog. Same failure as on 15 Nov (26.9 % then, 28.6 % now).""",
 why="""The crossing check only looked at the one car it chose to go ahead of (3875) and not at 3873, which
was already in the crossing area. Committing at full acceleration left no room to stop.""",
 fix="Check every car that is predicted to be in the crossing area, not only the chosen one; don't start the turn while any car is already in the conflict zone."),
"210_SignalizedJunctionLeftTurnEnterFlow": dict(kind="Collision → stuck", title="Stop-and-go in the junction, hit by the flow",
 what="""Turning left at a light into a road that oncoming cars also turn into. The plan switched between
`cross:172`, `base` and `merge:172` every half second, crawling at 0–3 m/s inside the junction. It stopped at
t ≈ 14.75 s, started again on `merge:172`, and car 172 hit it at t ≈ 15.6 s. Then it stood until the
watchdog. SimLingo was also asking to stop ("stop asked" on the video bar).""",
 why="""The plan keeps switching between candidates, so the car neither commits nor waits. Car 172 was both the one it
crossed ahead of and the one it merged behind.""",
 fix="Make the candidate choice stick for a while (hysteresis) inside a junction, and don't restart from rest in front of a car that is 3 m away."),
"136_InterurbanActorFlow": dict(kind="Off road, out of time", title="Left the road while waiting to turn; ran out of time",
 what="""Took 59 s to reach the turn (it waited a long time for a gap in the oncoming flow). At t ≈ 59 s it
moved off and drove onto the verge (`static.terrain`, 1 m outside its lanes), stopped, and ran out of the 70 s
route time at 88.6 %. 72 of 1399 plans failed. The first try of this route **hung**: the simulation stopped
advancing at 19.4 s and the 10 min limit stopped it; the second try is the one scored here.""",
 why="""Long waits for a gap in a fast interurban flow, then a turn that clips the verge. The hang is unexplained
(the same as the route 8 hang in todo.md).""",
 fix="Look at why the gap search waits so long on interurban roads; keep the turning path on the lane; find the hang."),
"019_EnterActorFlow": dict(kind="Stuck", title="Never found a gap to join the flow",
 what="""Crept up to the junction and stopped at t ≈ 11 s, 26.7 % of the way along. It never moved again;
the 30 s watchdog stopped it. The candidate was `cross:<id>` with the target changing from car to car
(3825, 3836, 3837, … 3848), and SimLingo was asking to stop ("stop asked"). No infraction.""",
 why="""Every gap was judged too small. The flow is continuous, and our merge needs more room than it ever gets;
PDM-Lite accepts smaller gaps (see learnings for route 8: "Route 8's gaps fit only without margin").""",
 fix="Use PDM-Lite's gap acceptance for EnterActorFlow; don't let SimLingo's stop request hold the car when the gap check says go."),
"014_OppositeVehicleTakingPriority": dict(kind="Collision → off route", title="Hit by the fire truck that took priority",
 what="""At the unsignalised junction the planner judged it could cross ahead of car 3717 and merge ahead of 3696,
the fire truck that does not yield (`merge:3696`, "merging in at 1.6 s"). It accelerated to 9.8 m/s, then
braked very hard (−16.9 m/s² on one step; several plans failed and were held) and stopped at t ≈ 19 s, but the
fire truck clipped it at t ≈ 19.25 s. Afterwards it hit a fence (t ≈ 27 s), went 1.5 m outside its lanes, and
left the route at t ≈ 32 s, which ended the run.
**This route scored 100 % on 15 Nov**, so this type is not reliably passed.""",
 why="""It went for a gap ahead of a vehicle that the scenario makes take priority, and after the near miss it
lost the path.""",
 fix="Yield to any vehicle on a crossing course that is not slowing down; after a collision, rejoin the lane rather than steering anywhere."),
"063_ParkingCrossingPedestrian": dict(kind="Pedestrian hit", title="Hit the pedestrian who stepped out between parked cars",
 what="""At 7.5–9.9 m/s the planner saw the pedestrian (id 4710) at t ≈ 8.75 s, 11 m ahead and 1.9 m to the
right, and noted "crosser in the lane from 0.2 s to 2.0 s … **cross: no stopping**", i.e. it decided it could not
stop and would pass first. It only braked at about −2 to −4.5 m/s² and hit the pedestrian at t ≈ 9.7 s at about
8 m/s.""",
 why="""Once the planner decides it cannot stop for a crosser it keeps going rather than braking as hard as it can.
It also drove at 8–10 m/s past a row of parked cars that hide pedestrians.""",
 fix="For pedestrians, always brake at the limit, even when a full stop is not possible; slow down past parked cars on streets where people can step out."),
"018_VehicleTurningRoutePedestrian": dict(kind="Rule", title="Ran a stop sign",
 what="""No collision; the pedestrian after the turn was handled. But at t ≈ 17.4 s it went through stop sign 320
at about 5 m/s without coming to a full stop: the plan was `cross`, and the notes say "cross: clear".""",
 why="""The crossing check decided the junction was clear, and nothing forced a full stop at the sign first.""",
 fix="Make the stop-sign stop a hard gate that must be met (v ≈ 0 at the line) before any cross or merge candidate can go."),
"114_VanillaSignalizedTurnEncounterGreenLight": dict(kind="Stuck at start", title="Never moved: a stop gate 5.9 m ahead never cleared",
 what="""The car never moved (0 % of the route). From the very first plan there was a stop gate 5.9 m ahead
(`gates: [[5.9, 0.0, …]]`) and another at 56.4 m. The light was green (the scenario's whole point), but the
gate stayed for 30 s and the watchdog ended the run. The planner noted two blockers 76–86 m ahead.""",
 why="""The 5.9 m gate is probably a stop line or light that is not ours (or a stale red). The log does not say
which source made the gate, so this needs a look.""",
 fix="Log the source of every gate; check which light or line is at 5.9 m from route 114's start."),
"080_ControlLoss": dict(kind="Stuck", title="Treated the debris as blockers and waited to go round",
 what="""The scenario puts debris on the road to drive over. The planner treated the pieces as **blockers** (at 25,
58 and 84 m) and wanted to go round through the oncoming lane: "blocker: oncoming traffic, waiting". It slowed from
3.4 m/s to a stop at t ≈ 9.75 s, 24.5 m before the first piece, and waited until the watchdog. 14.4 % of the route.""",
 why="""Debris is a small static object that should be driven over, not avoided.""",
 fix="Ignore objects lower than the car's ground clearance (debris), or classify ControlLoss props as drive-over."),
}


def row(r):
    s = r.get("score")
    ok = s is not None and s >= 100
    inf = [k for k in r.get("infractions", {}) if k != "min_speed_infractions"]
    return (f"| {r['n']} | {r['type']} | {r['route']} | {'✅' if ok else '❌'} {s:.1f} | {r.get('route_pct', 0):.1f} % "
            f"| {', '.join(inf) or '—'} | [video](videos/{r['name']}.mp4) |")


L = []
w = L.append
passed = [r for r in runs if r.get("score", 0) >= 100]
mean = sum(r.get("score", 0) for r in runs) / len(runs)
w("# Scenario sweep: simlingo_nmpc on one route of each Bench2Drive type\n")
w("Run 15 Nov 2026, 05:53–09:08, on commit `b012cdd` (branch `lane-band-worker-timing`). One route per type, "
  "39 types (the 44 minus the five highway and yielding ones: YieldToEmergencyVehicle, MergerIntoSlowTraffic, "
  "MergerIntoSlowTrafficV2, HighwayExit, SequentialLaneChange). Each route is the one tried before where there was "
  "one, otherwise the first of its five.\n")
w("```\nscripts/run_bench2drive.sh --agent simlingo_nmpc --routes <r> --tries 1 --no-cot --fast --video\n```\n")
w("## Summary\n")
w(f"- **{len(passed)} of {len(runs)} passed** (100 %, no infractions); {len(runs) - len(passed)} failed.")
w(f"- Mean driving score over the 39: **{mean:.1f}**.")
w("- Failures by kind: 6 collisions (5 with vehicles, 1 with a pedestrian), 3 stuck and never finished "
  "(EnterActorFlow, GreenLight, ControlLoss), 1 off road and out of time (InterurbanActorFlow), 1 rule "
  "(ran a stop sign).")
w("- **Junctions are the weak spot**: 5 of the 11 failures are junction gap decisions (left turns across "
  "traffic, joining a flow, a vehicle taking priority). Going round obstacles is strong: 10 of 11 pass.")
w("- Every run also logs `min_speed_infractions`; Bench2Drive does not penalise them (penalty stays 1.0) and "
  "they are left out below.\n")
w("| Group | Passed |\n|---|---|")
for g, types in GROUPS:
    rs = [r for r in runs if r["type"] in types]
    w(f"| {g} | {sum(r.get('score', 0) >= 100 for r in rs)} / {len(rs)} |")
w("")
for g, types in GROUPS:
    w(f"### {g}\n")
    w("| # | Type | Route | Score | Route done | Infractions | Video |\n|---|---|---|---|---|---|---|")
    for t in types:
        w(row(next(r for r in runs if r["type"] == t)))
    w("")

w("## What went wrong, one failure at a time\n")
w("Times are simulation seconds. Each analysis comes from the planner's log (`runs/<name>/viz/bench2drive_<r>/nmpc.jsonl`; "
  "the timeline around each infraction is in `analysis/<name>.txt`, made by `tools/timeline.py`). Each frame is "
  "from the video at the moment it went wrong (left: chase view, right: from above; green = SimLingo's path, "
  "magenta = NMPC plan).\n")
for name in sorted(FAIL, key=lambda n: by[n]["score"]):
    f, r = FAIL[name], by[name]
    w(f"### {r['type']} (route {r['route']}): {f['title']}\n")
    w(f"**{f['kind']}**. Score {r['score']:.1f}, route {r['route_pct']:.1f} % done. `{r['status']}`. "
      f"[Video](videos/{name}.mp4)\n")
    if os.path.exists(f"{root}/frames/{name}.jpg"):
        w(f"![{r['type']}](frames/{name}.jpg)\n")
    w(f"**What happened.** {f['what']}\n")
    w(f"**Why.** {f['why']}\n")
    w(f"**What to try.** {f['fix']}\n")

w("## What to fix first\n")
w("1. **Junction gap decisions** (routes 103, 210, 14, 19, 136): judge every car in the conflict zone, not one; "
  "stick with a choice inside the junction; accept PDM-Lite's gap sizes.")
w("2. **Pedestrians** (route 63): brake at the limit even when a full stop is not possible; slow past parked cars.")
w("3. **Obstacle shapes** (routes 73, 80): the open door belongs to the parked car's footprint; debris is driven over.")
w("4. **Stopped traffic beside us** (route 39): cap speed past a stopped queue in the next lane.")
w("5. **Gates** (routes 114, 18): log where each gate comes from; a stop sign needs a full stop before a cross candidate can go.")
w("6. **Hang** (route 136, first try): the simulation stopped advancing with no error, as on route 8 before.\n")

w("## Caveats\n")
w("- One run per type, and runs are not repeatable: route 14 passed on 15 Nov and failed here; route 103 failed both "
  "times in the same way. A pass here is not proof the type is solved.")
w("- `--no-cot --fast` (SimLingo without commentary, LoRA merged), as in the runs before. The watchdog ends a route "
  "after 30 s at rest or 70 s in all.")
w("- Route 136's first try hung and was stopped at 10 min; it is kept in `runs/136_InterurbanActorFlow.hung1/`.\n")
w("## Files\n")
w("- `videos/<route>_<Type>.mp4`: one per run (chase and top view, planner bar)")
w("- `frames/`: the failure frames above; `analysis/`: failure timelines")
w("- `runs/<route>_<Type>/`: leaderboard result (`res/`), logs (`out/`), planner logs (`viz/`); `runs/<name>.log`: the runner's output")
w("- `runs.json`: all results; `routes.txt`, `run_one.sh`, `tools/`: how to rerun and rebuild this report")
open(f"{root}/report.md", "w").write("\n".join(L) + "\n")
print("wrote report.md", len(L), "lines")
