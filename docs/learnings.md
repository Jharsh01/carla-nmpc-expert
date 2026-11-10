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

- **The behaviour layer is three files (9 November):** `sensor` (the Scene into path coordinates, classification,
  traffic-rule state: a `Percept`), `constraints` (the candidate Problems from a Percept: corridor, gates, leads,
  movers, the borrow's clear check) and `behaviour` (the step: those two, the parallel solves, the choice).
  `Behaviour::Options` inherits `Constraints::Options`, which inherits `Sensor::Options`, so the Python
  `BehaviourOptions` is unchanged. Older line references to `behaviour.cpp` below predate the split. Checked: all 22
  offline scenario traces identical to before, timing aside.
- **Route 1 waited for oncoming traffic in a lane it never needed** (split_r01 and sides_r01: 30.3 % and 31.9 %, then
  room_r01: 100 % in 20.5 s, 10 Nov). While SimLingo changes into the lane on its left (our way), the band takes in
  both lanes, but its "left lane" was the neighbour of the lane under the path: the lane inside the band, our way.
  Going round on the left was then judged against the oncoming lane beyond the band, and it never cleared. Three
  changes: (1) `route.py` gives each section the lanes on both sides (width, direction, allowed), each taken beyond
  the band's outermost lane on that side; (2) `constraints` goes round on either side where there is a lane, and a
  lane our way is judged on traffic beside or behind (`lane_gap` over `merge_time`; traffic ahead is followed), one
  the other way on oncoming traffic as before; (3) `sensor` takes a parked car that leaves the car room past it
  within the band ("room beside it") as something to keep clear of, not a blocker. (3) is what fixed route 1: the
  car never stopped. SimLingo turning back to its old lane (9.3 s in split_r01) followed our stopping the car half
  way, not the other way round.
- **Room beside a parked car counts only in lanes running the car's way** (`LaneSection.way_lo/way_hi`, 10 Nov).
  Where SimLingo's path goes into the oncoming lane the band takes that lane in too; the "room beside it" rule then
  saw room there, the parked car stopped being a blocker, so there was no borrow and no oncoming check: the NMPC
  followed SimLingo into the oncoming lane, and an oncoming car in the band was a lead, to stop head-on for (the
  route 4 crash). Found by reasoning, not seen in a run. The behaviour layer's step is now `Planner`
  (core/urban/planner), Python `expertctl.Planner` / `PlannerOptions`.
- **Acceleration limit 0.5 g** (`UrbanNMPC::Options::a_max` 4.905 from 3.0, 10 Nov; g05_r014). The overtake's clear
  check now judges the pass at the same a_max (`pass_accel`, 1.5, is gone). The NMPC asks for up to 4.9 m/s^2 (3.0
  before); measured up to 5.6-6.8 against 4.3-4.8. Routes 0 and 1 still 100 %, route 4 28.8 % as before. Offline:
  no new failures; stops and starts firmer (jerk 11 to 14 m/s^3, against jerk_max 15). The MKZ makes 0.5 g only
  below about 10 m/s at full throttle (5.4-6.8 m/s^2 to 7.5 m/s, 4.6 at 10, 3.4 at 15, 2.9 at 20;
  mkz2020_actuation.json): above that the plan asks for more than the car can give, and the clear check judges a
  faster pass than will be driven. Urban limits here are 8-14 m/s.
- **The acceleration limit follows the speed** (10 Nov): `VehicleParams` carries the measured full-throttle table
  (`tables.a_drive_max` in mkz2020.json, docs/vehicle_parameters.md). Each solve bounds a by min(a_max, the table at
  the speed now), held over the horizon (`UrbanNMPC::a_limit`), from the acceleration the car already has down at the
  jerk limit: a box bound is hard, and the car was logged at 6.8 m/s^2, so a bound below that at the first step would
  make the solve infeasible. A constraint a <= table(v) in the problem was the alternative; per solve is simpler and
  the speed changes little in 0.1 s. The overtake's clear check instead drives the table through the pass
  (`Constraints::drive`). At urban limits it hardly matters (from rest to 13.9 m/s over 30-80 m, 0.02 s against a
  constant 4.9); it does on faster roads, where the table falls to 2-3 m/s^2.
- **Seeing far enough to overtake** (10 Nov). An overtake past a blocker 15-20 m ahead from rest needs 87-117 m of
  oncoming lane seen clear at 30 km/h, 100-140 m at 50 km/h (the pass, oncoming_gap 20 m, and what an oncoming car at
  the limit covers meanwhile). The world model reported vehicles to 60 m only, and nothing told "clear" from "not
  seen". Now: vehicles to 150 m (LOOK_RANGE in the agent); a forward CARLA depth camera (expert/world_model/sight.py,
  a stand-in for a depth sensor, outside the leaderboard's sensor list, not seen by SimLingo) gives how far along the
  lane on each side the car sees: the lane's points from the map, 1 m above the road, projected into the frame, seen
  until one is out of frame or behind something nearer. The scene's sight_left/right is that, capped at LOOK_RANGE;
  the constraints put an oncoming car at the limit at the edge of sight on an oncoming lane ("blocker: left lane seen
  only N m ahead, waiting"). The leaderboard's SENSORS track allows only RGB cameras, lidar and radar; a
  leaderboard-legal version would need depth from RGB (a network) or stereo, both coarse at 100 m.
- **On Town12 a camera image's own transform is wrong** (sight_r410, 10 Nov): `image.transform` of the depth camera
  put it at (173, 319) with the car at (1173, 5319), off by 1000 m and about 5000 m, its rotation right. Every lane
  point then lay about 5000 m ahead: route 4 read 0 m of sight on every plan, route 1 a meaningless 150. The camera is
  now placed from the car's transform and its mount, in double precision (`sight.world_to_camera_of`). The frames
  saved then (SIGHT_DEBUG=<dir> dumps a few) give route 4 74-80 m at the start, cut by something solid across the
  oncoming lane at 70 m.
- **Six new routes, one run each (results/bench2drive/explore_nov8, 8 Nov, with video):** 2 HazardAtSideLane 100 %,
  7 ParkingExit 100 %; 4 ConstructionObstacleTwoWays 22.8 %, 6 AccidentTwoWays 36.1 %, 9 OppositeVehicleRunningRedLight
  46.4 %, 13 NonSignalizedJunctionRightTurn 38.5 %, all four on the watchdog, no collisions.
- **Parked cars count as oncoming traffic, so `borrow_left` is never clear on two-way roads** (routes 4 and 6). Every
  opposed track that is not a blocker, crosser or lead goes into `oncoming` (`behaviour.cpp:218`), parked or not. Both
  routes have a row of parked cars on the far side, 9 of them in the left band (e 2.6-6.0) on every plan of the wait, so
  "blocker: oncoming traffic, waiting" holds for good (227 and 273 plans). The real flow is not the cause: on route 4
  no moving oncoming car was within the 60 m seen from 39.4 to 47.5 s, and the car still waited. SimLingo meanwhile
  wants out (offset 2.4 m, no stop asked on route 4).
- **Routes 4 and 6 after the parked-car fix (9 Nov, one run each):** plain SimLingo + PID finishes both (100 %, 29.8 s
  and 23.8 s; results/bench2drive/r46_simlingo_pid), so the NMPC layer is what fails them. The opposite flow
  (OppositeActorFlow) sends a car every 70-149 m at the limit, under BasicAgents that brake for what is in their lane. What
  held the car, one cause per run on route 4 (r4_*): (1) it crept out at SimLingo's speed, which slows to a stop while it
  waits on the same traffic, switching base/borrow_left every plan or two, and the next car stopped nose to nose with it
  half across (31.9 %; route 6 the same, 36.9 %); (2) the clear check took a 5 m/s pass over the 4 s of the tracks, so
  a car 60 m off counted as clear; (3) an oncoming car on SimLingo's path (already out in the other lane) is a lead, not
  oncoming, and was not checked at all: a borrow began with it 50 m off, the car stopped half across and was struck
  (25.8 %, the only collision); (4) `lane_path` took the oncoming lane as the car's own once it was in it, the blocker
  went out of its lane and the car lane-kept head-on into the flow (36.4 %); (5) between the cones and the cars parked
  opposite (2.9 m apart) the circles leave no room: the car's cover 1.23 m against a 0.92 m half-width, a parked car's
  1.28 against 1.0, plus 0.2 m each, so no plan fits where the car has 0.5 m a side (28.8 %); (6) where SimLingo's path
  changes lane the band takes in both lanes, and the cars parked opposite became blockers: base waited behind them
  through clear gaps, and the row made the pass 86 m long, so any oncoming car called it off (27.3 %, 30.3 %); (7) with
  SimLingo's path already in the left lane at the cones, the lane there has no left, so the pass was never offered
  (27.3 %). Each is fixed in `behaviour.cpp` / `route.py` (pass_accel, pass_progress, opposed leads in `oncoming`,
  `lane_path` back to the car's lane, still things as corridor edges in borrow_left, far-side stills never blockers,
  the left lane looked for back to the car). The last, (8), is open: todo 00.
- **On a left turn, oncoming through traffic is held as a cut-in lead, not a crosser** (route 9, at rest from 25.8 s,
  SimLingo asking to go on every plan: 3 m/s more than the plan). The path turns 12 m left; cars coming the other way at
  e 6.3-6.6 are predicted into it at 3-5 s, and as `crossing_kind` is read at t = 0, when they are opposed, they become
  leads (144 of 149 plans), one after another from the stream. Plans with no note still kept the car within 0.1 m,
  alternating with plans that creep 0.7 m: not explained, needs a replay.
- **Route 13 is held by SimLingo and the behaviour layer together**: SimLingo asks to stop on 142 of the 154 plans of
  the wait; every plan has the stream (flow 8 m/s, 21-26 m apart) as crossers "to inf", the merge case of todo 0.
- **The hard stop under 6.5 m/s shows on every route that stops** (todo 2b): a -20 m/s² on routes 1, 4, 6, 9 (min a -19.7
  to -20.9), each from a_ref near -7.
- **SimLingo's move round the side-lane hazard waits about 10 s in lane** (route 2, still 100 %): SimLingo's line is 3 m
  left from 12.95 s, the car stays within 0.3 m of its lane to 19 s, stops twice and creeps at 1.5 m/s heading 8 deg the
  other way, and is across only by 24.5 s. No blocker or borrow candidate: todo 10's held-back overtake.
- **Route 1's swing at the end starts with a stop beside the parked car** (results/bench2drive/repro_r1_video, 8 Nov, 100 %
  but visibly weaving from about 19 s; the same weave in nokink_hold and repro_r1, so all three runs). At 10.95 s SimLingo
  turns back to the route lane level with the parked car 3697 (its rear 2.6 m ahead of the car's centre). In the new lane 3697
  is a blocker; `borrow_left` is refused ("oncoming traffic, waiting"), so `base` keeps `stay_behind`'s lead at
  `s_lo - 4` m, 1.4 m behind the car's own front: a gap already broken, so a_ref -7 and a stop in 0.2 s (a -20 m/s², todo 2b)
  in the passing lane at 11.55 s. Stopped there, SimLingo's next predictions keep the passing lane, and it stays in it to
  y 5196, 40 m from the route's end (the target point: route 1 is 134 m, under the leaderboard's 200 m downsampling, so
  its only target is the end). The late return is then a closed loop between SimLingo and the NMPC: SimLingo's line goes
  3 m across in 10 m, the NMPC follows it to 10-13 deg of heading, and the next prediction, made at that heading, picks
  the other lane (seen in all three runs: back to the passing lane at 77-80 deg, to the route lane at 93-95). Plain
  SimLingo + PID (results/bench2drive/r1_simlingo_pid, same day) has no stop and no weave: it waits behind 3697, dodges
  round it and is back in the route lane by y 5226, heading under 1.1 deg after. Not a frame error: the lane-keeping line
  lands within 0.3 m of the lane, and 0.3 s of latency at these yaw rates moves a point 10 m ahead about 0.35 m.
- **Keeping SimLingo's line where it is and not driving failed plans finishes route 1** (`path_points`, `expert/plan_hold.py`, candidates ranked solved first; results/bench2drive/nokink_hold, one run each at 2 Hz, 2 Nov). Route 1: 100 % for the
  first time, 21.85 s, no infractions, no stop after the start, largest heading 14.9 deg, in the left lane by about 8 s, past the parked car, and on in lane keeping at 8 m/s. Failed plans 13 of 231 to 3 of 109; while moving in a manoeuvre 8 of 21 to 2 of 48;
  median distance of the plan from the line 1.22 to 0.72 m. Route 0: 100 %. Route 8: 41.0 % (43.3), 2 m less, held by crossers as before; it now stays in a manoeuvre on 197 plans against 42. Route 10: 30.4 % (35.5), but it had no manoeuvre plans and
  no failed plans, so neither fix acted on it: same crossers, run-to-run difference, and at rest 30 s now ends it before 70 s. Both still need todo 0. One run of route 1 is not a margin (repeat runs: below).
- **Route 1's solves fail because the kink makes the path coordinates singular** (route 1 at 2 Hz; replay of 7.35-8.15 s). The model divides by `1 - kappa e` (`urban_nmpc.cpp:228`), which must stay positive: the car
  must be inside the path's radius of curvature. At 7.55 s a new prediction moves the line 1.06 m (+0.37 to -0.69 at the car), the first point kept is (0.2, -0.69), so `path_points` joins at 74 deg: radius 0.2 m, heading error
  61 deg at t = 0, and `1 - kappa e` goes down to -0.85 in that plan, -5.15 in the next (normal: 0.76-1.54). FATROP then takes steps of 1e-3 and stops at `max_iter` 100, the violation still 2-4 m, the status UNKNOWN. With the
  line kept where it is: radius 1.8-6.3 m, `1 - kappa e` 0.87-1.08, the plans of 7.35-7.75 s solve in 11-35 ms, against 38-100; those after that start from the real run's swerve, so they say little.
  Ruled out as the cause: car 3712 (dropped: the same plans fail), the warm start (cold: 2 of 8 solve, against 4), `max_iter` (300: 6 of 8 "solve", at 200-285 ms, 7 m off the path).
- **Driving failed plans is what turns route 1's lane change into a swerve** (route 1 at 2 Hz, results/bench2drive/simlingo_2hz). The first turn is fine: heading steady at 11-12 deg to 7.35 s, -1.9 deg in all
  under the good plans of 6.95-9.35 s. Then seven plans in a row fail (7.55-8.75 s) and the car takes them: +21.1 deg, to 42 deg. A failed plan took 110-153 ms against 30-50 ms, with up to 5.9 m of corridor slack in the
  replay. Its steering is not to be trusted: replayed, the plans of 7.55 and 7.75 s gave the opposite sign (-0.12, -0.11 against +0.13, +0.25). Why they fail, beyond the kink and the mover above, is not shown:
  status SOLVER_RET_UNKNOWN (iterations not reported). Manoeuvre plans failed while moving: 7 of 18 at 2 Hz, 5 of 10 and 1 of 5 at 1 Hz.
- **In a lane change, a faster car behind in the target lane is only a mover, so the plan dodges it instead of deciding the merge** (route 1 at 2 Hz, results/bench2drive/simlingo_2hz, plan of
  6.95 s, 7.1 on the video's bar; replay, open loop, its offsets smaller than the log's). SimLingo merges ahead of 3712, about 5.5 m behind and closing at 4.4 m/s (10.1 against 5.7), so about
  1.3 s. `behind_now` sends it to `add_movers` (`behaviour.cpp:199`). The plan goes full throttle and up to about 2 m right of the line. With 3712 removed: acceleration 2.44 to -0.68,
  offset at 2 s -0.34 to +0.26 m. With 3712 and the kink both removed, every plan to 7.55 s solves with corridor slack 0; with only 3712 removed the failure at 7.55 s stays, so the kink is
  still needed. Lead 3705 removed: no effect on the offset's cause. The car is slow here as the plan held 5.7-6.5 m/s against the 9 or so SimLingo asked; why is not isolated.
- **SimLingo at 2 Hz changes nothing on routes 0, 8 and 10 and leaves route 1 worse** (`SIMLINGO_EVERY` 10, results/bench2drive/simlingo_2hz, one run each, 2 Nov; 1 Hz:
  rate_06, stays_out_body). Route 0 100 %, 8 43.3 %, 10 35.5 %, as before, with 0 failed plans of 350 on 8 and 10 (2 and 3 at 1 Hz). Route 1: 24.3 % at penalty 0.88 for
  driving outside the route's lanes (score 21.3; 28.8 at 1 Hz, no infraction): the plan fails from 7.55 s and the car goes past SimLingo's line, y 5.9 against its 3.5 at 8.75 s, to rest
  in the oncoming lane at y 6.9, 38 deg, 9.35 s. The same swerve as at 1 Hz, sooner. The plan's median distance from the line while moving in a manoeuvre fell, 3.1 to 1.2 m, so the
  faster line does help the plan track it, on only 10-21 plans. Not yet shown why it swerves sooner: every new line is a new join to the car (todo 3), now twice as often.
- **The steering-rate limit held inside a plan but not in the command, and holding it does not fix the kink** (results/bench2drive/stays_out_body, rate_06,
  rate_03; route 1 and 0, one run each, 1 Nov). `delta_rate_max` bounds each plan, but a new plan could start elsewhere: route 1's command reached 8.26 rad/s,
  over 0.6 on 38 % of ticks from 8.4 to 10 s; route 0's 2.52. `rate_limited` in the agent now holds it (0.6 max on both). Route 0 unchanged (100 %). Route 1 no
  better: 28.8 % at 0.6, 22.8 % at 0.3 (plan and command both), and the plan's median distance from SimLingo's line while moving in a
  manoeuvre went up, 2.2 to 3.1-3.2 m. It ended at y 5.7, 72 deg at 0.6 and y 4.6, 33 deg at 0.3, against y 1.5, 11 deg. A rate limit slows the response to
  the kink's false heading error; it does not remove the error, and correcting later needs more room. Kept at 0.6, the value the plans assume; 0.3 rejected.
- **A car counts only if it intends to use a lane the ego is using** (`Actors._stays_out`, `Route.ego_lanes`, `Actor::stays_out`; 1 Nov).
  Decided on the map, not from boxes across the path: its lane at each predicted point against the path's lanes level with it, and whether its body
  reaches 0.3 m into one of them or it moves towards one at over 0.3 m/s. Junctions, cars across their lane, parked cars, bikes, walkers and objects
  are classified as before; `borrow_left` still sees them. **The ego's lanes must include those its body is in, not only the path's:** the first version
  (results/bench2drive/stays_out) took the nose over the double yellow on route 1 and hit oncoming 3713, flagged as staying out, its first collision.
  With that added (results/bench2drive/stays_out_body, one run each): route 0 100 %, one stop for the real cut-in; route 1 28.8 % (30.3), no infractions, the
  7 s false stop and the swerve across the double yellow gone; route 8 43.3 % (43.3); route 10 35.5 % (35.5), both still held by junction crossers, as
  expected. Route 1 now brakes at 9 s, mid lane change, to rest at y 1.5, then waits for good: the blocker holds it, and SimLingo itself asks to stop
  on 123 of 145 plans. Replay at 8.55-9.35 s (less exact here than for the earlier run: steering off by up to 0.35 rad at one plan): keeping the line un-kinked
  takes the plan from -7 to at most -3.2 m/s² with no failed solves; dropping lead 3705 alone changes nothing. So todo 3 is next.
- **A reclaimed solver slot plans from a stale position, and a failed plan can still be chosen** (found reading f14a0aa, 1 Nov;
  reproduced on the real `Behaviour`, a 598-piece straight, fixed route). `behaviour.cpp:391-393` gives a candidate the slot with its
  label and no reset, and that slot's `s_last_` is from when it last ran, so `path_state` searches only -30 to +60 m round it. The
  second time `borrow_left` appeared its plan began at s 163 with the car at s 335; the solve failed and it was chosen anyway (769
  against 940), since the choice reads slack and score, not `Plan.success`. The agent is safe from the first part, as `set_route`
  clears `s_last_` on every slot each plan; fixed-route use, such as the scenarios without `@local`, is not. The second part reaches the agent.
- **The losing candidate sets the step time** (`URBAN_VERBOSE`, f14a0aa, 1 Nov). Threads join before scoring. In
  `parked_car_overtake`, `base` takes 50-57 iterations and 55-61 ms on a problem it cannot meet (slack about 5.2), while the
  chosen `borrow_left` takes 10 iterations and 12 ms; in `crossing_pass` it is 103.8 ms against 12.8. This is what puts both
  `parked_car_overtake` variants over the 50 ms check (56.1 and 50.3 ms p95; the other 20 of 22 runs pass).
- **Eight mover slots are eight circles, about 2.7 cars.** A car takes three. Given 12 circles, `n_movers` came back 8.
- **The model has no guard at `1 - kappa e_y = 0`.** On a path of curvature about 1/3 1/m, e_y 2.0 solved, 2.9 failed at 0
  iterations, and 3.2, past the singularity, reported success after 71.
- **Route 1's lane change is undone by the path, not by the traffic in the lane it goes to** (results/bench2drive/fix_video, analysed
  1 Nov; the logged plans replayed through the real `Behaviour` on a straight 4-lane stand-in for the map, open loop: bands, notes and
  steering match the log, steering within 0.03 rad through the swerve). The road is 2 lanes our way (y 0, 3.2; stream at 8 m/s) and 2
  oncoming (y 6.6, 9.9), so the lane SimLingo moves to is not oncoming, as this file and todo.md said. Earlier reports' "waits beside the parked
  car" is wrong too: the car stands across the double yellow at y 5.3, 41 deg to the road. Three things in turn:
  (1) **Stop, 6.55-7.15 s.** SimLingo moves left and asks for 8-10 m/s. Its line slants about 20 deg, which turns the box of oncoming car
  3709, in its own lane throughout, to `ext_e` 1.58 from 0.90, and the band's left edge jumps from 2.6 to 3.4 at the same place (`Route.lanes`
  takes `max` of offsets measured at other points on the path). One sample, t = 1 s, is in by 0.17 m. That is enough: `opposed` makes it a
  lead, `urban_nmpc.cpp:457` clamps it to the car's front + 0.5 m, so the plan must stop in about 3 m (slack `ahead` 4.5 m, solve failed),
  and the 26 m/s² brake does the rest: 6.5 m/s to rest in 0.4 s. Without oncoming cars the plan keeps 4.5-6.7 m/s at 1 s; without
  the lead in the new lane, 3705, it still stops. Narrowing the band alone did not clear it in a rough test, as the box is most of it.
  (2) **Swerve, 11.75-12.95 s.** No actor causes it: slack is in `corridor` (1.4-5 m), and it stays with the oncoming cars, parked car
  or lead removed. Each new prediction (1 Hz) lands the line 0.2-0.4 m from the car, and `path_points` joins the car to it (the kink
  above): the first metre runs 19-30 deg (44 deg on the spline), the next 2 deg. The plan's model then sees the car 50 deg off the path and
  bound for y 1.0 (11.75 s), so it steers left at the 0.6 rad/s limit; the real car, 7 m/s and 0.5 rad, goes left to y 5.3. SimLingo's line stayed in
  the lane at y 3. With the line kept where it is, at the same logged states: road-wheel 0.02-0.10 from 0.12-0.30 at 11.75-11.95 s, corridor
  slack 0. Failed plans at 12.15 and 12.35 s (4.2 m off the path) were driven anyway.
  (3) **Held for good, 13-70 s.** On a path 41 deg across the road the band is [-4.4, 6.4], every lane, so passing cars are "cut-ins" (note on 281
  of 285 plans); SimLingo asks to stop on 7 of them. With the moving cars removed the replayed plan drives off.
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
