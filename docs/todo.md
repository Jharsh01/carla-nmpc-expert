# To do

Open work, most pressing first. When a task is done, delete it here and, if it taught something, add that to
[learnings.md](learnings.md). Last updated 29 October 2026, at commit b341365.

## Now: get route 0 driving

1. **The lane band is too wide.** On route 0 it came out 5.4 m ([−1.6, 3.8]) where the car stopped, so the parked
   cars on the left counted as blockers and the car waited until the watchdog stopped it (27.1 % completion).
   Likely the ±6-sample union in `Route.lanes` (meant for lane changes). Try the band without the union, or the union
   only near a lane change; check the `scene.lane` column in `nmpc.jsonl`; re-run route 0.
2. **The cut-in car stands in the lane 35 m ahead** (route 0, actor 3702, parked False, at rest). Check how it is
   classified and whether the car would wait behind it for good.
3. **Solve time in CARLA:** p50 32 ms but p95 105 ms, and 36 of 423 plans failed on route 0. Find which candidates
   and situations (the live view and `nmpc.jsonl` have per-candidate times). Idea from before: start a new candidate
   from the chosen plan, not cold.
4. **MinSpeedTest failed on route 0** (value 126.98 %). Find out what it measured.

## Next

5. **Run routes 0–21** with `--agent simlingo_nmpc --no-cot --fast` and compare with SimLingo + PID on the same
   routes (`--agent simlingo`). Then pick one route per scenario type as the dev set.
6. **SimLingo's own overtakes** (its path into the oncoming lane) are held back to the lane unless the behaviour
   layer chooses `borrow_left`. Decide: let the corridor follow SimLingo into a legal adjacent lane when the world
   model sees it clear, or keep the behaviour layer in charge.
7. **The offline solve-time check:** both `parked_car_overtake` variants are at 52–59 ms p95 against 50 ms. Tune, or
   decide the limit.
8. **Steering-rate limit 0.6 rad/s in CARLA:** check tight, slow junction turns on a route (1.5 was used before).
9. **Odometry's sign conventions** (compass change, speed) are checked only by a unit test. Check against a run:
   the warm start's iterations after `set_route` should stay low.
10. **A fallback for a failed or late solve:** follow the last plan, then brake at the friction limit (plan, section 2).

## Housekeeping

11. **Look at the live view in a browser** (`--live`, `ssh -L 8765:localhost:8765`): its drawing code has never run.
12. **`docs/STATUS.md` is stale** (it predates the SimLingo agent). Fold what it still holds into this file and
    learnings.md, or update it.
13. **Merge `narrow-to-simlingo-nmpc` into `main`** when ready (all work since d320b04 is on it).

## Later

14. World model: pedestrians beyond constant velocity; junction-branch hypotheses for vehicles; emergency vehicles
    (YieldToEmergencyVehicle); right-side nudging.
15. With CoT on, let SimLingo's commentary steer the behaviour layer (which candidate to prefer).
16. Perception ablations: the visibility filter off, then noise, latency, drops (plan, section 5).
