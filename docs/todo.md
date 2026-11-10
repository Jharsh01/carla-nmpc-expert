# To do

Open work, most pressing first. When a task is done, delete it here and, if it taught something, add that to
[learnings.md](learnings.md). Last updated 9 November 2026, on lane-band-worker-timing. Latest report:
[reports/2026-11-01-lane-keeping-and-traffic](reports/2026-11-01-lane-keeping-and-traffic/README.md).

## Now: routes 0 and 1 drive; the car does not get through traffic on routes 8 and 10

0. **Re-run routes 4, 6, 8, 10 with the lanes either side** (10 Nov): borrow_right, the lane-our-way check and
   "room beside it" are new and only routes 0 and 1 have run with them. Watch for a parked car beside a two-lane band
   being driven past too close, and for borrow_right on the wrong side of the road where the map's right lane is
   oncoming (left-hand traffic).
0b. **The band's lo/hi mix frames during a lane change** (`Route.lanes`, found 10 Nov): the merge takes min/max of
   offsets measured across the path at other rows, where the path is elsewhere. Half way into the next lane the band's
   far edge is clipped (a straight two-way test: our lane at [-3.9, -0.9] across the path, the band [-3.0, 2.2]).
   `way_lo/way_hi` are now cut at the row itself (`Route._cuts`); do the same for lo/hi.

00. **Route 4: the overtake is planned along SimLingo's kinked path** (results/bench2drive/r4_left_near, 9 Nov, 28.8 %;
    learnings.md). From rest SimLingo's path jumps 2 m left within 3 m (heading 30-39 deg) onto its line through the gap,
    so the car, pointing down the road, is 30-39 deg off the path and its ends swing about 1.2 m across: in a 2.9 m gap
    `borrow_left` gets 0-3 m per plan, then breaks its limits. Agreed next step (9 Nov): plan on the lane path always.
    Drafted, not kept (untested):
    a. `_path` in `simlingo_nmpc_agent.py` returns the lane path whenever the map has one (SimLingo's own path only
       without a lane), and while `intent.manoeuvre` passes SimLingo's points projected on the lane as
       `scene.line_s`/`line_e` (s increasing, plus a point 15 m past the last at e 0, back to the lane); else
       `[0, L]`, `[0, 0]` as now.
    b. In `constraints.cpp`'s `lane_corridor`, where the asked line leaves the lane the band widens to take it in
       (`lo = max(road_lo, min(lo, asked - margin))`, `hi` likewise), and `ref` is the line clamped to that band, so
       SimLingo's own moves out of the lane (route 1's dodge, route 2's side-lane hazard) still go through.
    Then re-run 4, and check 0, 1 and 2 for regressions. Then route 6, not yet run with any of the 9 Nov changes (with
    the parked fix alone: 36.9 %, r6_parked_fix, the same deadlock as route 4). The agent now logs each candidate
    (`candidates` in nmpc.jsonl: label, solved, slack, metres, score), and a scene can be rebuilt offline from the log
    on a straight path (a short script: actors at their logged s, e, sizes, the logged lane band).
    Also: route 1 has not been run since `lane_path` changed (out in an oncoming lane the path is now the lane to come
    back to), which changes its return from the pass; check it.
01. **A left turn across oncoming traffic holds it as cut-in leads** (route 9): opposed cars predicted into a turning
    path should be crossers with gates. Same root as item 0 (`crossing_kind` at t = 0). Also replay route 9 at 29-31 s:
    plans with no note still hold the car.
02. **No lane commitment through a lane change**: route 1's end weave follows SimLingo's flipping line (learnings.md).

0. **The car waits for good in steady traffic: routes 8 and 10 end on the watchdog** (43 %, 35 %;
   results/bench2drive/intent_z, 31 Oct, no collisions). Routes 8 and 10 face a stream of cars at 13 and 10 m/s about 2 s
   apart (route 8's is `flow_speed` 13 m/s with 24-26 m spacing, at constant speed, ignoring other vehicles until any of
   them collides: ActorFlow). Every plan there has a crosser due within 3 s, and each crosser's gate is widened by `gate_buffer` (1 s) either
   side, so the base candidate never gets a gap, and `pass:` needs one too. SimLingo asks to stop on only 17 % and 28 % of
   those plans, so it is the behaviour layer holding the car. On route 8 every plan also has a window "to inf": the stream comes from the left and runs down the exit lane (rebuilt
   from the log: x 2537, e 0 on the path after the merge), so it is traffic to merge with, treated as crossers and not
   leads (`crossing_kind` is read at t = 0 only). Route 1 is no longer in this group: it finishes (results/bench2drive/nokink_hold, learnings.md). The old path gave the same waits on
   route 10. Plain SimLingo enters route 8's stream (100 %, but 11.5 s for 78.6 m looks over-counted). Design for
   this: conflicts, a registry with a lifecycle, a phase, and one space-time tube per plan compiled into the NMPC's fixed
   slots (https://claude.ai/artifact/5mgXZSJodoQ7Sk3Cf27chh; not implemented). First step: stream cars as merges and cars
   behind as none in `sensor.cpp`, tested on routes 0 and 8. Before building the merge, decide the minimum margin and
   how long to wait: the slot leaves about 1 m at each end.
1. **Route 8 hung once:** results/bench2drive/intent_a stopped ticking at 38.75 s with 47 of 192 plans failed, and the run
   sat until the script's 3600 s timeout. It was on the wrong lane then (learnings.md), but nothing says what hung. The
   script's limit per route is now 600 s (9 Nov: no run is to take more than 10 minutes).
2. **Route 0 could still stall for good** (analysed 1 Nov, learnings.md). The car behind as a cut-in and the phantom are fixed
   (two runs of route 0: 100 % in about 22.6 s, at most one stop, for the real cut-in), which also takes away the stop beside
   parked cars at 18.7 s. Left:
   a. **Seen on route 1 (8 Nov, learnings.md): the stop beside the parked car is what makes route 1 weave at the end.**
      `stay_behind` against a blocker the car is level with braked it to rest in the passing lane; SimLingo then kept
      that lane and came back late, in a weave (no lane commitment in the agent). Fix this first, then check route 1 again.
      **Stuck beside parked cars.** The way past is 8 cm wide (e -0.23 to -0.15). A stop there, say for a real cut-in, could still
      be for good. A clearance already broken at t = 0 should not hold the car: open it for what the car occupies now, as line 457
      does for leads, or use boxes for parked cars. Then test from rest at e = 0 beside a row. Check too that a blocker level
      with the car cannot pin it the same way (`stay_behind`).
   b. **The brake under 6.5 m/s** is 26 m/s² in first gear. Model it or avoid it (the table's median hides it).
   c. **A crossing-kind actor behind the car** still goes through `inside`, against where the car's rear is now, so it can set a
      gate as the car behind did; not seen in a run. Actor 3705 on route 1 (alive, 58 m ahead, 491 m predicted over 2 plans, in
      older runs too) is another bad prediction, not yet looked at.
   d. Repeat route 0 with the fixes: repeat runs are not independent (learnings.md), and this is one run.
   video_route0_oct31's stall is still unexplained: its log has no path, and the full lock there was not reproduced. The new logs
   have the path, so the next one can be replayed.
3a. **A lane change has no merge decision.** A faster car behind in the target lane is a mover, so the plan swerves round it (learnings.md). Route 1 now gets by without one, on one run. Choose: go ahead
   of it at SimLingo's speed if the gap holds, or slot in behind it.
3. **Still open from the plans:** steering jumps at each new plan are capped but not explained, and the yaw rate is at 0.77 of `v tan(delta) / 2.87`. A failed plan is now held, then braked on (`expert/plan_hold.py`): `HOLD` 0.6 s and
   `BRAKE` -2 are first guesses, and the brake under 6.5 m/s is still item 2b's. `1 - kappa e` is not guarded: a sharp bend in SimLingo's own line could still break it.
4. **A car cutting in close ahead is a mover (swerve), not a lead (brake).** On route 0 the cut-in car (3697)
   pulled into the lane 5 m ahead with no "cut-in" note; the NMPC steered round it to full lock. The road edge now
   stops that ending off the road, but braking is the right answer. Check how `sensor.cpp` classifies it.
5. **The cut-in car stands in the lane 35 m ahead** (route 0, actor 3702, parked False, at rest). Check how it is
   classified and whether the car would wait behind it for good.
6. **Solve time in CARLA:** p50 32 ms but p95 105 ms, and 36 of 423 plans failed on route 0. Find which candidates
   and situations (the live view and `nmpc.jsonl` have per-candidate times). Idea from before: start a new candidate
   from the chosen plan, not cold.
7. **MinSpeedTest failed on route 0** (value 126.98 %). Find out what it measured.
8. **The route runs at 0.33x real time** (150 ms a tick on route 0, with SimLingo and the NMPC on their own cores).
   What is left on the loop: the scenario tree (48 ms), CARLA's step (42 ms), the sensors (19 ms), `Actors.detect`
   (36 ms a plan). Real time needs more cores (CARLA and the loop on their own) and a faster `Actors.detect`; check
   too whether `--debug` (SimLingo's frames, which the agent no longer draws) is still wanted. See learnings.md.

## Found reading the code (1 November, at f14a0aa; learnings.md)

19. **Choosing a candidate ignores `Plan.success`** (`Planner::step`, the choice), so a failed plan can win. Reaches the agent; goes
    with item 13.
20. **Reset a reclaimed solver slot's `s_last_`** (`Planner::step`, where slots are handed out). Fixed routes only: `set_route` clears it in the agent.
21. **A candidate that cannot be met holds the step for its full solve:** look at bounding it, for items 6 and 11.

## Next

9. **Run routes 0–21** with `--agent simlingo_nmpc --no-cot --fast` and compare with SimLingo + PID on the same
   routes (`--agent simlingo`). Then pick one route per scenario type as the dev set.
10. **SimLingo's own overtakes** (its path into the oncoming lane) are held back to the lane unless the behaviour
   layer chooses `borrow_left`. Decide: let the corridor follow SimLingo into a legal adjacent lane when the world
   model sees it clear, or keep the behaviour layer in charge.
11. **The offline solve-time check:** both `parked_car_overtake` variants are at 52–59 ms p95 against 50 ms. Tune, or
   decide the limit.
12. **Steering-rate limit 0.6 rad/s in CARLA:** check tight, slow junction turns on a route (1.5 was used before).
13. **A fallback for a late solve:** a failed one is done (`expert/plan_hold.py`); one that is late is not.

## Housekeeping

14. **Look at the live view in a browser** (`--live`, `ssh -L 8765:localhost:8765`): its drawing code has never run.
15. **`docs/STATUS.md` is stale** (it predates the SimLingo agent). Fold what it still holds into this file and
    learnings.md, or update it.

## Later

16. World model: pedestrians beyond constant velocity; junction-branch hypotheses for vehicles; emergency vehicles
    (YieldToEmergencyVehicle); right-side nudging.
17. With CoT on, let SimLingo's commentary steer the behaviour layer (which candidate to prefer).
18. Perception ablations: the visibility filter off, then noise, latency, drops (plan, section 5).
