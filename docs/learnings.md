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

- **Forcing the path through the car makes it weave** (route 0, 31 Oct, results/bench2drive/path_overlay_route0;
  `path_points` joins the car's origin to SimLingo's line). SimLingo's line is steady in the world: each new
  prediction lands 6-8 cm RMS from the last at 2-10 m ahead. The car is 5 cm off it at the median and 15 cm at p95,
  and the join then bends: curvature 0.047 1/m (p95 0.13) over the first 6 m against 0.013 over the next 12, and the
  path's heading at the car 8.06 deg RMS off the car's. On the line as predicted it is 1.42 deg. Re-solving 103 logged
  plans offline with the line kept where it is (no actors, same car states, open loop): steering RMS 0.068 -> 0.037
  rad, plan-to-plan change 0.079 -> 0.046. Replay against logged steering: correlation 0.74, so the size is
  approximate. Not the cause: SimLingo's 1 Hz update (steering moves no more just after it), odometry (3 cm, 5 cm,
  0.02 deg over each hold against the true pose; the signs are right).
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

- **A destroyed actor looks alive, at the world's origin, to every handle but the one that destroyed it** (checked on CARLA 0.9.15,
  1 Nov). `is_alive` stays True for good, `get_location()` gives (0, 0, 0) with no error, and it leaves `get_actors()` one tick
  later. On the destroying handle reads raise. So the list is the only test: tracks not in it are dropped at once (`Actors.detect`).
- **A destroyed actor stayed in the scene for 1.5 s as a phantom crossing car** (fixed 1 Nov; route 0: 15 of 43 stops). The scenario
  destroys the cut-in car after 30 m (`parking_cut_in.py`). Its track was kept for `keep` with its last position and speed,
  so `_follow_lane` ran (`actors.py:197`) on the origin: the waypoint is a road near the origin, and the fading offset takes the prediction from the old position at
  t = 0 to that road. So the "3715 m in 5 s" of actor 3697 is the distance to the origin: predicted metres less distance to
  the origin is 85 m on all of route 0's runs and 81-86 m for each stream car on route 1. It lasts 7 plans each time. The first
  sample's heading is that road's, so it reads as a crosser "in the lane from 0.0 s to 0.0 s", 12 m ahead and standing still; the
  car brakes for nothing, as replayed offline. On route 1 it fires for every stream car, every 3 s.
- **Under about 6.5 m/s the brake is 26 m/s², not 7.** Below it the gearbox drops to first and the car sheds 1.3 m/s a tick
  (results/identify/runs.csv, brake from 7.5 m/s; the same on route 0: -7 asked, -20 measured, the pose agreeing). It holds with the
  brake at 0.03, and in the identify runs brake levels from 0.1 reach 21 m/s². `_clean`'s median hides it: the raw table's
  -17.4 at 7.5 m/s is taken for noise. The plan does not know: a slow-down it asks for ends at rest, and every stop here is
  a skid to a stop in about 0.3 s. Tyre friction is 3.5.
- **CARLA's bounding boxes for `static.prop.mesh` actors are wrong.** Bench2Drive spawns its background parked cars
  that way (`route_scenario.py`, `spawn_parked_vehicles`). A 5 m Charger reports half-extents 1.48 × 1.94 m; on
  Town12 they came out 2.0 × 4.3 m, a car turned sideways across the lane. Decision: take them as 4.8 × 2.0 m parked
  cars along the actor's heading.
- **Actors vanish mid-tick.** The scenario destroys actors in the same tick the world model reads them. Any CARLA
  error on an actor drops its track.
- **The leaderboard adds GNSS noise** of 5e-6° per axis (about 0.5 m per sample), unbiased.
- **The scenario plant is not the dynamic bicycle.** `sim/urban/scenarios.cpp` uses its own kinematic bicycle with
  a 0.25 s lag on acceleration and a 0.05 s rate-limited lag on steering. The dynamic bicycle and tyre model drove
  nothing and were removed.

## The NMPC and the behaviour layer

- **Without the two false stops route 0 does not stop at all; route 1 does not get better** (results/bench2drive/fix_rear_ghost,
  fix_ghost_only, fix_rear_ghost_70s, 1 Nov; one run each). Route 0: no rest after the start, 22.5 s against 30.55, 100 %, plans
  failed 3 of 112 against 13 of 153, slowest 1.1 m/s, for the real cut-in. Route 1, to 70 s: 30.3 % before, 27.3 % with the phantom
  fix, 24.3 % with both, all behind the same parked car (33-41 m), so the fixes do not solve it and may cost a little: the false
  notes let it edge on between stream cars. With the at-rest watchdog on it ends at 43.85 s on 21.3 %, as the odd creep no
  longer resets the watchdog. Cars from behind are now movers, which is what holds it (todo 0). A second run with both fixes
  (results/bench2drive/fix_video, Epic quality, the runs above Low): route 1 30.3 %, 42.5 m by 20 s then at rest for 52 s, so the
  differences above are noise, not a cost of the fixes. Route 0 100 % in 22.7 s, with one stop of 0.5 s at 9.5 s for the real
  cut-in: it can still come to rest, and the zero stops of the first run are not a guarantee.
- **A car 18-22 m behind made the car stop dead in an empty lane** (fixed 1 Nov: `behind_now`; route 0, 25 of 43 stops in the 15
  lane-keeping runs; reproduced offline on the real `Behaviour`). `project` (behaviour.cpp:114) gives a car behind the path's start
  e = its distance, so it is "not in the lane now"; its prediction then enters the lane, and `inside` (:187) compares that
  with where the car's rear is now, as if the car would stay there, so it counts as a cut-in (:198). The lead sits behind the
  car and is clamped to `front_now + gap_min` (urban_nmpc.cpp:457). It bites once `t_in` is inside the 2 s horizon: `a_ref` went
  +0.44 at 2.2 s, -4.15 at 2.0 s. Offline at 7.9 m/s: no effect at under 18 m (a mover) or over 22 m (past the horizon), a
  -7 plan to a stop between, even for a follower at the car's own speed, which can never catch it. Every note `s 0.0 m`.
- **A car at rest inside a parked car's clearance stays there for good** (reproduced offline in closed loop on
  video_route0's logged scene: 0 m in 12 s). A parked car needs 1.31 + 1.28 + 0.2 = 2.79 m between circle centres; in the lane
  at 2.64 m that is e <= -0.15, and the corridor gives e >= -0.23, so the slot past a row of parked cars is 8 cm wide.
  At speed the plan threads it. From rest outside it, going forward deepens the slack, so the plan stays or reverses,
  v(1 s) < 0.2, and the car never moves, with the hold rule on or off. From e -0.15 it drives off; from -0.10 it moves 1 m. Stopping
  where no parked car is alongside is fine (26.4 s, a gap in the row, any e). The old path started at the car, so e was
  always 0 there, which is why those runs stalled and lane keeping does not: the nudge survives. All 15 runs still stop at
  18.7 s beside parked cars at e about -0.14, a few cm from it, so 15 of 15 is not a margin. Full lock is not the cause:
  the scene stays stuck from 0 or -0.95, and neither is gravity in the IMU (`a` reads -0.16 at every rest, `a_ref` -1.00 is the
  hold rule). Not reproduced: video_route0_oct31's own stall (nothing near in the logged scene; that log has no path).
- **Five of route 0's seven 30 s stalls were false blockers.** timing_route0, _async, _b, _c and _h2 (lane bands 3.5-5.3 m
  wide, before the road-edge fix) took a parked car beside or just ahead of the car as a blocker. `stay_behind` puts the lead
  4 m behind the blocker's rear, level with the car or behind it, and it is clamped as above, so the car can never move
  (reproduced on timing_route0_h2's scene, and it drives with the true lane). The other two are the bullet above.
- **In steady traffic the behaviour layer holds the car, not SimLingo** (results/bench2drive/intent_z, 31 Oct). SimLingo
  asks to stop on 17-28 % of plans on routes 1, 8 and 10. Routes 8 and 10 have a stream about 2 s apart, every plan
  has a crosser within 3 s, and `gate_buffer` widens each gate 1 s either side, so no gap is left. On route 8, 145 of 145
  plans also have a gate "to inf" (`t_out` is INF when the last prediction sample is in the lane): the stream runs down
  the exit lane, so it is traffic to merge with, but `crossing_kind` reads the direction at t = 0 only and calls it a
  crosser.
- **Route 8's gaps fit only without margin.** The stream is 13 m/s with about 1.9 s between cars. Merging with 3 m
  behind needs 2.8 s at 3 m/s² to 6 m/s, or 2.1 s at 5 m/s² to 7.5 m/s. A to-scale profile clears 1.3 m ahead and
  0.9 m behind. Plain SimLingo enters this stream (100 %, no infractions, twice), though 78.6 m in 11.5 s looks
  over-counted. ActorFlow cars ignore other vehicles until one collides, so the stream will not brake for the car.
- **Constraints can be switched at run time but not added.** The solver has 34 rows a step (850 in all) and 6 slack
  groups; a row is switched by parameters (`mov_on` per slot, `lead_on`, `cap_on`, `floor_on` per step) and an off row
  is a constant -1. A rebuild takes over a minute. So a dynamic constraint system has to change what fills the fixed
  slots. Design: conflicts, a lifecycle, a phase, and one space-time tube a plan
  (https://claude.ai/artifact/5mgXZSJodoQ7Sk3Cf27chh; not implemented).
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
- **`--video` draws SimLingo's path, the NMPC's path and its plan on both views** when `nmpc.jsonl` has them (the
  agent now logs these and the true pose, for analysis only). `--keep-frames` keeps the frames to remake a video. The
  video runs up to 15 % fast: route 0 lost frames (249 for 29 s), so go by the time in its bar.
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
- **Give `map.get_waypoint` a height.** At z = 0 it returned the wrong lane for 19 % of Town12's road points off junctions
  (roads are 298-380 m up), against 4 % at the road's height. On route 8 this
  put the band on the oncoming lane 2.75 m away, and the NMPC steered the car off the road. Route 8 went from 10 % to 43 %.
  `Frame` now carries the car's z; `Route._waypoint` takes it.
- **Lane keeping (`expertctl.PathIntent`).** SimLingo's line is measured against the map lane from 8 m ahead, not against the
  car's heading, so a bend is not a manoeuvre. Off by over 1 m starts following SimLingo, under 0.5 m ends it. Otherwise the
  path is the lane's centre and the speed is the limit, unless SimLingo drops under 1 m/s within 2 s. Largest offset in
  lane-keeping on routes 0, 1, 8, 10: 0.57 m; a real manoeuvre showed 2.6 m at once (route 1) or 2.7 m (route 8). A turn is
  found the same way only where the lane's branch differs from SimLingo's: on route 10 the lane already follows the left
  turn, so it stays in lane keeping throughout, and that is fine. On route 0 steering RMS fell from 0.060 to 0.019 rad, the
  heading swing from 1.49 to 0.54 deg RMS (6.2 to 1.9 max), and 95th percentile lateral acceleration from 1.36 to 0.37.
- **Repeat runs of a route are mostly the same run.** Thirteen runs of route 0 (seeds 1-6) gave 3 distinct drives: ten ended at
  30.55 s with the same steering and failed plans (the same to 2-3 figures), two at 28.05 s and one at 27.65 s. They
  match up to 12 s. The seed made no difference: seed 2 gave 27.65 s once and 30.55 s once. Which drive a run gets is not
  known. A count of passes on repeats says little, so vary something that matters, not just `--seed`.
- **The last field of an actor in `nmpc.jsonl`'s scene is metres predicted over 5 s, not a speed** (`simlingo_nmpc_agent.py:383`).
  Divide by 5 for m/s: 65 is 13 m/s. Read as km/h it misled the analysis of routes 0, 1, 8 and 10.
