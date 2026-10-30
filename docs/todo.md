# To do

Open work, most pressing first. When a task is done, delete it here and, if it taught something, add that to
[learnings.md](learnings.md). Last updated 29 October 2026, at commit a53284d on main.

## Now: route 0 drives (100 %, 30 Oct); next the dev set

0. **Route 0 does not always finish.** A second run with the same code (`--video`, results/bench2drive/video_route0)
   stopped 9.8 m before the route's end at 91 %, at rest from t 36 s until the watchdog, with SimLingo still asking
   for 18.7 m. Logged then: a stop sign 49.8 m ahead, the route end 9.8 m ahead, "blocker at s 59 m". Find what
   held it (the route-end stop line's margin?), and how much runs vary.
1. **A car cutting in close ahead is a mover (swerve), not a lead (brake).** On route 0 the cut-in car (3697)
   pulled into the lane 5 m ahead with no "cut-in" note; the NMPC steered round it to full lock. The road edge now
   stops that ending off the road, but braking is the right answer. Check how `behaviour.cpp` classifies it.
2. **The cut-in car stands in the lane 35 m ahead** (route 0, actor 3702, parked False, at rest). Check how it is
   classified and whether the car would wait behind it for good.
3. **Solve time in CARLA:** p50 32 ms but p95 105 ms, and 36 of 423 plans failed on route 0. Find which candidates
   and situations (the live view and `nmpc.jsonl` have per-candidate times). Idea from before: start a new candidate
   from the chosen plan, not cold.
4. **MinSpeedTest failed on route 0** (value 126.98 %). Find out what it measured.
5. **The route runs at 0.33x real time** (150 ms a tick on route 0, with SimLingo and the NMPC on their own cores).
   What is left on the loop: the scenario tree (48 ms), CARLA's step (42 ms), the sensors (19 ms), `Actors.detect`
   (36 ms a plan). Real time needs more cores (CARLA and the loop on their own) and a faster `Actors.detect`; check
   too whether `--debug` (SimLingo's frames, which the agent no longer draws) is still wanted. See learnings.md.

## Next

6. **Run routes 0–21** with `--agent simlingo_nmpc --no-cot --fast` and compare with SimLingo + PID on the same
   routes (`--agent simlingo`). Then pick one route per scenario type as the dev set.
7. **SimLingo's own overtakes** (its path into the oncoming lane) are held back to the lane unless the behaviour
   layer chooses `borrow_left`. Decide: let the corridor follow SimLingo into a legal adjacent lane when the world
   model sees it clear, or keep the behaviour layer in charge.
8. **The offline solve-time check:** both `parked_car_overtake` variants are at 52–59 ms p95 against 50 ms. Tune, or
   decide the limit.
9. **Steering-rate limit 0.6 rad/s in CARLA:** check tight, slow junction turns on a route (1.5 was used before).
10. **Odometry's sign conventions** (compass change, speed) are checked only by a unit test. Check against a run:
   the warm start's iterations after `set_route` should stay low.
11. **A fallback for a failed or late solve:** follow the last plan, then brake at the friction limit (plan, section 2).

## Housekeeping

12. **Look at the live view in a browser** (`--live`, `ssh -L 8765:localhost:8765`): its drawing code has never run.
13. **`docs/STATUS.md` is stale** (it predates the SimLingo agent). Fold what it still holds into this file and
    learnings.md, or update it.

## Later

14. World model: pedestrians beyond constant velocity; junction-branch hypotheses for vehicles; emergency vehicles
    (YieldToEmergencyVehicle); right-side nudging.
15. With CoT on, let SimLingo's commentary steer the behaviour layer (which candidate to prefer).
16. Perception ablations: the visibility filter off, then noise, latency, drops (plan, section 5).
