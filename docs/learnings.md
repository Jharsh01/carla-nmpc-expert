# Learnings

What the work has taught, each with the evidence and what was decided. Newest first within each section. Add to it
whenever a run, a test or a dead end teaches something not obvious from the code. Open work is in
[todo.md](todo.md); how everything fits is in [architecture.pdf](architecture.pdf).

## SimLingo

- **SimLingo's GNSS frame is not CARLA's.** The leaderboard does not give the GNSS reference latitude/longitude, so
  SimLingo estimates it (`_init`, fsolve on the first plan point). On Town12 its frame was 497.6 m off CARLA's in x
  (route 0, 29 Oct). SimLingo does not notice, since its route goes through the same conversion; anything mixing its
  pose with CARLA's ground truth does. Decision: the NMPC plans in the car's own frame and never uses a global pose.
- **Its predictions are car-relative and complete enough to plan on.** `pred_route`: 20 points about 1 m apart (x
  forward, y right); `pred_speed_wps`: 11 points 0.25 s apart, which may or may not start at the car
  (`simlingo_ref.speed_reference` handles both). It predicts every tick (20 Hz).
- **Its path is short.** About 20 m: at speed that is under 2 s, while the NMPC looks 5 s ahead and the behaviour
  layer looks for actors 160 m ahead. Past it the path must go on along the lane (below).
- **Its restart creep forces throttle after a long stop** (`stuck_threshold`), which would override the NMPC.
  Turned off in the agent.
- **`LingoAgent.destroy()` raises** (`cfg.data_module.encoder` is missing from its config). Harmless; caught.

## The path and the frame

- **A straight continuation past SimLingo's path is wrong in bends.** The corridor (from the road) drifts sideways
  along it, so the NMPC treats a bend as a lane shift: `curve@local` drifted 0.54 m starting 40 m before the bend,
  and entered it too fast. A lead in the bend would project metres off the path and be missed. Decision: continue
  along the map's lane for 160 m.
- **The path must reach as far as the behaviour layer looks.** With an 80 m extension, an oncoming car 100 m ahead
  projected past the path's end, its distance counted as a sideways offset, so it was not seen as oncoming and the
  car pulled out too early (`parked_car_wait@local`).
- **Kinked paths stall the NMPC.** An emulated SimLingo path joining the lane with a linear ramp had kinks at the car
  and 10 m on; starting from rest beside a parked car the plan stalled. A smooth (cosine) join fixed it. Real
  SimLingo paths are smooth, but a kinked reference is worth guarding against.
- **The dense route the leaderboard keeps on the agent is privileged** (`org_dense_route_world_coord`, lane centres
  every metre): a SENSORS-track agent is meant to get only the sparse plan (50 m, GPS). The route comes from
  SimLingo; only the route's end is taken from the sparse plan.

## The world model and CARLA

- **CARLA's bounding boxes for `static.prop.mesh` actors are wrong.** Bench2Drive spawns its background parked cars
  that way (`route_scenario.py`, `spawn_parked_vehicles`). A 5 m Charger reports half-extents 1.48 × 1.94 m; on
  Town12 they came out 2.0 × 4.3 m, a car turned sideways across the lane. Decision: take them as 4.8 × 2.0 m parked
  cars along the actor's heading.
- **Actors vanish mid-tick.** The scenario destroys actors in the same tick the world model reads them, and
  `is_alive` can still say True. Any CARLA error on an actor now drops its track.
- **The leaderboard adds GNSS noise** of 5e-6° per axis (about 0.5 m per sample), unbiased.
- **The scenario plant is not the dynamic bicycle.** `sim/urban/scenarios.cpp` uses its own kinematic bicycle with
  a 0.25 s lag on acceleration and a 0.05 s rate-limited lag on steering. The dynamic bicycle and tyre model drove
  nothing and were removed.

## The NMPC and the behaviour layer

- **The car can be steered off the road, and nothing stops it** (route 0, run of 30 Oct 01:22, rebuilt offline
  from Town12.xodr and metric_info.json). At t 12.7 s the cut-in car (3697), parked on the left, pulls into the lane
  5 m ahead. No note says "cut-in": it was a mover, so the NMPC kept its circles clear by steering right, to full lock
  (road-wheel -0.95 rad at 2 m/s), up to 3 m off SimLingo's path. Shoulder at t 14.5 s, sidewalk at 17 s (49 deg
  to the lane), where it stood until the watchdog. Nothing held it on the road: the corridor is only the lane band,
  and (1) `Route.lanes` measures the lane edges across the path, so with the path at an angle to the lane, and the
  +-6-sample union, the band grew from [-1.5, 1.4] to [-6.4, 2.9]; (2) the behaviour layer widens the corridor to
  wherever the car is (`lane_corridor`, z0 -+ half), so it follows the car out; (3) road edges, kerbs and sidewalks
  are not in the scene at all. The 2 s run of the same route kept to the lane (0.2 m at most).
- **In CARLA the NMPC plans 2 s ahead in 25 steps, every 200 ms** (20 steps of 50 ms, then 5 of 200 ms; it was
  5 s in 30 steps every 100 ms). On route 0 (30 Oct, on its own core): the step 33 ms p50, 54 ms p95 (from 40 and
  94), 6 of 248 plans failed (from 13 of 396), 139 ms a tick, and the car got furthest yet (53.4 %, stopped by the
  lane band as before). The library's defaults (5 s), used by `sim/urban`, are unchanged.
- **Circles plus a 0.5 m margin cannot pass a row of parked cars beside a 2.9 m lane.** Ego circles r = 1.31 m
  (4.89 × 2.04 m with mirrors), a parked car's 1.28 m, margin 0.5: 3.09 m needed between centres, 2.88 m available.
  The car held still for 99 s (route 0). Decision: parked cars and objects get 0.2 m and no growth with prediction
  time (`static_margin`); the real side gap is then about 0.75 m.
- **"Any overlap with the lane" makes false blockers.** On a slightly wobbling path a van beside the lane tilted
  0.03 rad and grazed the band by 2 cm; the car stopped for good. Decision: a parked car or object blocks only if it
  reaches 0.3 m into the lane (`blocker_overlap`).
- **The lane band must be measured on the lane, with the road's edges, or the corridor follows the car anywhere.**
  `Route.lanes` measured each lane edge across the path and widened every row to the union of +-6 rows; with the
  path at an angle to the lane that made the band 9-15 m (route 0), and the car left the road (below). Now each edge
  is where the path's normal crosses it, the union is taken only between neighbouring lanes (a lane change), and
  `LaneSection.road_lo/road_hi` (the outermost driving lanes' outer edges; shoulder, parking, sidewalk are not road)
  cap the corridor, also where it is widened to the car and round a blocker. Replayed offline on Town12 at the
  moment route 0 left the road: old [-6.4, +8.4], new [+0.1, +4.0] (the lane, to the car's left). In CARLA route 0
  then completed for the first time (100 %, no infractions; band at most 2.8 m, plan within 0.29 m of SimLingo's
  path). `nmpc.lanes` costs 12 ms a plan (was 5), for the neighbour walks.
- **A fast steering-rate limit makes overtakes overshoot.** At 1.5 rad/s the plan steers back faster than the lagged
  plant follows and the car left the road; 1.0 is borderline; 0.6 chosen. CARLA itself has no rate limit, only a
  0.15 s lag, so this is a design value.
- **Solve time depends on the car.** On the MKZ `parked_car_overtake` takes 56–59 ms p95 against 46.8 ms on the
  Model 3; the 50 ms check fails on it alone.
- **The warm start survives a new path every plan** if the old plan is mapped through the world (or the old frame's
  pose in the new one): `set_route` replans the same within 5 cm (tests).

## Running Bench2Drive

- **Where a tick's time goes** (route 0, 30 Oct, `--no-cot --fast`, L4; `timing.jsonl`, `scripts/timing_report.py`):
  248 ms a tick on average, 0.20x real time. SimLingo's preprocessing (`LingoAgent.tick`: JPEG round trip of the
  camera, UKF) takes 65 ms every tick, 26 % of the route; SimLingo's model 240 ms but only once a second (5 %); the
  NMPC's behaviour step 60 ms p50, 136 ms p95 on a plan tick (13 %); the world model's `Actors.detect` 25 ms a plan;
  the scenario tree 25–35 ms (spikes to 550 ms); CARLA's step 30 ms; waiting for the sensors 16 ms. SimLingo also
  rewrites its whole `metric_info.json` every tick, which grows with the route: 5 ms a tick at the start, 60 ms
  after 900 ticks (0.8 MB).
- **SimLingo in its own process and the NMPC in its own thread, each on its own core, took route 0 from 248 to
  150 ms a tick** (0.33x real time; 30 Oct, `expert/simlingo_worker.py`, `expert/cpus.py`). Their results are taken
  a fixed number of ticks later (6 for SimLingo, 1 for the NMPC), so the run does not depend on the machine's speed;
  the loop never waited for SimLingo (0.7 ms at most), and ticks it answered in were no slower than others. On its
  core SimLingo takes 57 ms to preprocess and 229 ms for the model; the NMPC's step 40 ms p50, 94 ms p95 (from 58
  and 135 ms when it shared the loop's CPUs). Three layouts compared: CARLA and the loop on cores 0–1 (150 ms a
  tick); also on the NMPC's core (145 ms, but its solve doubled to 79 ms); unpinned (128 ms, but SimLingo's
  preprocessing doubled to 116 ms, its model went to 295 ms and the solve to 75 ms). The first is the default: the
  only one whose SimLingo and NMPC times would hold at real time. Pinning costs the loop: CARLA's step 30 → 42 ms,
  the scenario tree 25 → 48 ms (median), `Actors.detect` 25 → 36 ms.
- **`run_bench2drive.sh --video` records a route** from two cameras on the car (chase, and from above; the agent
  never sees them) into `videos/<route>_<scenario>.mp4` with the NMPC's log under them (`scripts/make_video.py`).
  CARLA's image clock runs ahead of the route's (by about 1 s on route 0: ticks before the route starts), so the
  evaluator writes the route's start on the image clock to `video/start.txt`.
- **Real time is out of reach on 4 cores.** With the agent's work off the loop, what is left is CARLA's step
  (42 ms), the sensors (19 ms) and the leaderboard's scenario tree (48 ms), about 110 ms a tick before the world
  model, against 50 ms. SimLingo's restart creep, the DEBUG frames and `metric_info.json` every tick are gone with
  LingoAgent.run_step; the speed the NMPC gets is the speedometer's, no longer rounded to 0.1 m/s.
- **A route takes about ten times its simulated time** with SimLingo `--no-cot --fast` on an L4 (route 0: 42 s
  simulated in 390 s), plus about three minutes of start-up. The first NMPC in a fresh `build/urban_cache` compiles
  the solver for about a minute.
- **An exception in the leaderboard's tick loop is recorded as "Simulation crashed"**, with the route's score so far
  kept; Bench2Drive's merge drops only "Agent crashed" routes. The watchdog uses this.
- **One leaderboard process per route**, so anything served from the agent (the live view) restarts per route.
