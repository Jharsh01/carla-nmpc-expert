# Overtakes past a hidden lane, and merging at a junction, 12 November 2026

Summary of the work on lane-band-worker-timing, with `--agent simlingo_nmpc --no-cot --fast` on Town12, one run per
row. The detail behind each finding is in [learnings.md](../../learnings.md); open work is in [todo.md](../../todo.md).

## Result

Routes 4 and 6 now finish. Route 8 still waits at its junction. Route 10 was not run.

| Route | Before (best) | Now | How it ended | Run |
| --- | --- | --- | --- | --- |
| 4 ConstructionObstacleTwoWays | 28.8 % | 100 % | Finished, 37.9 s, no infraction | `fix3_r4` |
| 6 AccidentTwoWays | 36.9 % | 100 % | Finished, 46.1 s, no infraction | `fix3_r6` |
| 8 SignalizedJunctionRightTurn | 43.3 % | 23-37 % | Watchdog; no collision | `turn1_r8` to `push_r8` |

Routes 4 and 6 pass only with 5 m kept to oncoming traffic at the end of a pass (`oncoming_gap`, 20 m in the core; the
agent sets 5 m, `ONCOMING_GAP` to change it). On route 4 that left about 0.3 s: the ego was back in its lane with the
next oncoming car 6 m ahead. Routes 0 and 1 have not been run with any of these changes.

## Why route 4 failed

- From where it stopped, 20.5 m behind the cones, the pass is 41.5 m (5.9 s at the 8.3 m/s limit). The clear check
  then needs about 113 m of the oncoming lane seen, or a car at the limit could appear from beyond what is seen.
- The car saw 70 m. A traffic-warning sign as wide as the lane hides the oncoming lane beyond it, because that lane is
  only 3 m to the left. From the lane's middle, sight was about 3.4 times the distance to the sign.
- The oncoming cars run at the limit, 70-149 m apart, and never brake (Bench2Drive's `OppositeActorFlow`).

## What changed

- **Backing up and peeking.** Stopped to see, with the lane to go into not seen far enough, the planner asks the agent
  to reverse up to 12 m, steered along the lane. It keeps 1.5 m from anything behind; the NMPC itself plans forwards
  only. The car then creeps forward at 2 m/s to the left edge of its lane and waits there. On route 4, sight rose from
  70 m to a steady 104-106 m.
- **Overtakes up to 20 km/h over the limit.** The pass is judged at that speed, and the car comes back into its lane
  further on, along a ramp rather than a jump. This saved little: from rest, 96 m of sight needed against 100 m.
- **Once going round from a peek, the sight verdict stands.** The sight reading swings as the car turns out, and a
  withdrawn pass left the car askew in the oncoming lane. Seen traffic still stops it.
- **A row of stopped cars is one obstacle.** A stopped car held up only by a parked car is parked too; route 6's
  accident cars had been one blocker and two cars to queue behind. The peek also stays 8 m from any car it follows,
  and creeps forward no further than it backed up.
- **Merging into a stream (route 8).** Cars that join the route's lane from a side road are now "mergers", not
  crossers that hold the lane for good. A new candidate, `merge:<id>`, goes in between two of them. It keeps the ego's
  rear ahead of the car behind (a "chaser" constraint), waits 15 m back for a run-up, backs up to regain one, and
  takes the bend at up to 6 m/s².
- **Turns use the map's junction lane** (`Route.turn_path`), the one SimLingo's line follows. SimLingo's own line is
  tighter and differs from plan to plan.
- **Junction conflicts** (`Route.conflicts`) find the junction lanes that merge with or cross the turn. They are tested
  offline on route 8, but not yet used by the planner.
- **Fixes along the way.** A car following in the ego's lane is no longer kept clear of as an obstacle. "Base" is not
  solved when it cannot stop behind the blocker (it took 100 ms a plan). Slack is logged per constraint group.

## Why route 8 still fails

The stream runs at 13 m/s, about 25 m apart, and never brakes. From a standing start 13-15 m before the entry, the
best predicted margin to the car behind is about 3 m, which is the threshold. The car starts for a gap and drops it
within a second, again and again.

One trial assumed the car behind would brake at 3 m/s² once the ego is in its lane (`MERGE_CHASER_BRAKE`; default 0).
The ego still dropped each gap, now held back by the car ahead of it. When the stream stopped at 26.7 s, the ego's
light turned red with its front 0.2 m past the line. The ego then stopped 5 m into the junction, and SimLingo asked to
stop there until the watchdog. The trial never tested whether a car behind would brake. From the scenario code they
do not: they ignore other vehicles until they hit something.

## Checks

- All 72 tests pass, three of them new.
- Offline scenarios: two new ones pass, `peek_to_see` and `merge_into_stream` (3.2 m clearance). `parked_car_wait`
  fails as it did before.

## Decisions needed

- Whether to keep 5 m to oncoming traffic, about 0.3 s on route 4's return.
- Route 8: the margin accepted to a car that will not brake. About 3 m is all the gap allows from rest.
- Before relying on these, re-run routes 0 and 1, which have not been run since: turns now use the map path, and
  following cars are no longer avoided.
