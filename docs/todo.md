# To do

Open work, most pressing first. When a task is done, delete it here and, if it taught something, add that to
[learnings.md](learnings.md). Last updated 1 November 2026, on lane-band-worker-timing. Latest report:
[reports/2026-11-01-lane-keeping-and-traffic](reports/2026-11-01-lane-keeping-and-traffic/README.md).

## Now: routes 0 and 1 drive; the car does not get through traffic on routes 8 and 10

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
   behind as none in `behaviour.cpp`, tested on routes 0 and 8. Before building the merge, decide the minimum margin and
   how long to wait: the slot leaves about 1 m at each end.
1. **Route 8 hung once:** results/bench2drive/intent_a stopped ticking at 38.75 s with 47 of 192 plans failed, and the run
   sat until the script's 3600 s timeout. It was on the wrong lane then (learnings.md), but nothing says what hung. The
   route's `timeout 3600` also costs an hour each time; consider less.
2. **Route 0 could still stall for good** (analysed 1 Nov, learnings.md). The car behind as a cut-in and the phantom are fixed
   (two runs of route 0: 100 % in about 22.6 s, at most one stop, for the real cut-in), which also takes away the stop beside
   parked cars at 18.7 s. Left:
   a. **Stuck beside parked cars.** The way past is 8 cm wide (e -0.23 to -0.15). A stop there, say for a real cut-in, could still
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
   stops that ending off the road, but braking is the right answer. Check how `behaviour.cpp` classifies it.
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

19. **Choosing a candidate ignores `Plan.success`** (`behaviour.cpp:431-438`), so a failed plan can win. Reaches the agent; goes
    with item 13.
20. **Reset a reclaimed solver slot's `s_last_`** (`behaviour.cpp:391-393`). Fixed routes only: `set_route` clears it in the agent.
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
