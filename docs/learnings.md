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

- **Circles plus a 0.5 m margin cannot pass a row of parked cars beside a 2.9 m lane.** Ego circles r = 1.31 m
  (4.89 × 2.04 m with mirrors), a parked car's 1.28 m, margin 0.5: 3.09 m needed between centres, 2.88 m available.
  The car held still for 99 s (route 0). Decision: parked cars and objects get 0.2 m and no growth with prediction
  time (`static_margin`); the real side gap is then about 0.75 m.
- **"Any overlap with the lane" makes false blockers.** On a slightly wobbling path a van beside the lane tilted
  0.03 rad and grazed the band by 2 cm; the car stopped for good. Decision: a parked car or object blocks only if it
  reaches 0.3 m into the lane (`blocker_overlap`).
- **The ±6-sample union of the lane band (route.py) can make it far too wide.** On route 0 it came out 5.4 m
  ([−1.6, 3.8]), so parked cars 2.4–2.8 m to the left counted as in the lane. Not fixed yet (todo).
- **A fast steering-rate limit makes overtakes overshoot.** At 1.5 rad/s the plan steers back faster than the lagged
  plant follows and the car left the road; 1.0 is borderline; 0.6 chosen. CARLA itself has no rate limit, only a
  0.15 s lag, so this is a design value.
- **Solve time depends on the car.** On the MKZ `parked_car_overtake` takes 56–59 ms p95 against 46.8 ms on the
  Model 3; the 50 ms check fails on it alone.
- **The warm start survives a new path every plan** if the old plan is mapped through the world (or the old frame's
  pose in the new one): `set_route` replans the same within 5 cm (tests).

## Running Bench2Drive

- **A route takes about ten times its simulated time** with SimLingo `--no-cot --fast` on an L4 (route 0: 42 s
  simulated in 390 s), plus about three minutes of start-up. The first NMPC in a fresh `build/urban_cache` compiles
  the solver for about a minute.
- **An exception in the leaderboard's tick loop is recorded as "Simulation crashed"**, with the route's score so far
  kept; Bench2Drive's merge drops only "Agent crashed" routes. The watchdog uses this.
- **One leaderboard process per route**, so anything served from the agent (the live view) restarts per route.
