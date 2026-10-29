# Status and hand-off (27 October 2026; scope narrowed 29 October)

Where the work stopped, what has been measured, and how to pick it up. On 29 October the scope was narrowed to
SimLingo + the urban NMPC + the privileged world model. The PDM-Lite baseline and the race-track NMPC, pure pursuit
and directives copied from carla-track-control were removed (they are in git history before that date). What was
left of `track_control` (path, vehicle parameters, RK4) was folded into `core/urban`, and the dynamic bicycle plant
dropped. The MKZ, read from CARLA, is now the only vehicle (`docs/vehicle_parameters.md`): the offline scenarios ran
on the Model 3 before. On it `parked_car_overtake` drives correctly but its p95 solve time is 56–58 ms, over the
50 ms check (46.8 ms on the Model 3), so `urban.scenarios` fails on timing alone. The plan is `docs/plan.md`; the M0/M1 report
is `docs/m0_m1_report.pdf`; the urban NMPC's design reasons are in `core/urban/README.md`.

## Milestones

| | State |
|---|---|
| M0: repo bootstrapped | Done. |
| M1: urban NMPC + offline scenarios | Done. All 11 scenarios pass (`docs/run_logs/urban_scenarios.txt`). The behaviour layer, planned for M3, was built here too. |
| M2: world model, MKZ actuation, expert on an empty route | **In progress.** The parts are written and drive in CARLA outside the leaderboard. The first leaderboard route fails (see the open bug). |
| M3–M5 | Not started (SimLingo + NMPC agent, dev-set iteration against SimLingo + PID, full evaluation). |

## What M2 added (this session, after the report)

- **`scripts/identify_ego.py`** measures Bench2Drive's ego (`vehicle.lincoln.mkz_2020`, default physics) at 20 Hz
  on Town06's longest straight (855 m). It writes `config/vehicles/mkz2020_actuation.json` and the raw ticks to
  `results/identify/runs.csv`.
  - Each test holds a speed, then applies a step. Teleporting the car to speed is invalid: the wheels don't spin
    up and the tyres skid at about −26 m/s².
  - Throttle has a dead zone up to about 0.4, which roughly holds speed. Full throttle gives 5–7 m/s² at low speed
    and about 2 at 25 m/s.
  - Full brake gives about −7 to −8 m/s².
  - Steering gives −1.11 rad per unit steer at 3 m/s and −0.95 at 20 m/s (CARLA's steering curve), with a 0.15 s lag.
  - The 7.5 m/s brake column is noisy; `expert/mkz_actuation.py` smooths it and forces each column monotone.
- **`config/vehicles/mkz2020.json`**: mass 1696 kg, CG 1.172 m behind the front axle and 1.689 m ahead of the
  rear, wheelbase 2.861 m, body 4.89 × 1.84 m. From the physics control; the sources are noted per value.
- **`expert/mkz_actuation.py`**: acceleration and road-wheel angle to throttle, brake and steer (table inverse plus
  PI on the acceleration). Holds the brake at rest.
- **`expert/world_model/route.py`**: reference path from the leaderboard's dense route (2 m spacing, lane changes
  smoothed); lane sections from the map (band, left lane, oncoming or not, borrowable); stop lines on the route
  (traffic-light stop waypoints, stop-sign trigger volumes, route end).
- **`expert/world_model/actors.py`**: perception-shaped actors.
  - Range of 60 m (40 m for walkers), with line of sight checked by ray casts from the roof.
  - Velocity estimated by the tracker from box positions; tracks are kept 1.5 s after last seen.
  - Vehicles predicted along their lane (map), everything else at constant velocity.
  - Parked or queued decided from hazard lights, being off the lane, or 4 s at rest with nothing ahead.
- **`expert/driver.py`**: the per-tick loop shared by the agent and the test script. It plans at 10 Hz, controls
  at 20 Hz, and logs every tick, with a scene snapshot at every plan.
- **`expert/agents/expert_agent.py`**: leaderboard agent, the world model and the NMPC on the map route (no SimLingo yet). `scripts/run_bench2drive.sh --agent expert`.
- **`scripts/drive_expert.py`**: drive the expert outside the leaderboard (`--town --length --traffic`).
- **Core changes:**
  - `Path::project(x, y, s_from, s_to)`: a windowed projection, used by the NMPC and the behaviour layer (long routes).
  - The NMPC is never asked to reverse: leads and stop gates are clamped at the car's current front.
  - The behaviour layer handles points beyond the route's ends (see the open bug).
  - `expertctl` binds the behaviour layer.

## Results so far in CARLA (outside the leaderboard)

`docs/run_logs/drive_Town10HD_0_*.json`, Town10HD, spawn point 0, a 604 m route with traffic lights:

| Run | End | Collisions | p95 / max \|e_y\| | Mean speed | Step p50 / p95 / max | Failed solves |
|---|---|---|---|---|---|---|
| Empty | All 601 m driven (ended by time 0.6 m short of the old end test) | 0 | 0.25 / 0.52 m | 5.0 m/s | 16 / 20 / 24 ms | 0 |
| 40 Traffic Manager cars | Route end, 118 s | 0 | 0.29 / 0.51 m | 5.1 m/s | 25 / 50 / 136 ms | 5 |

In both runs it stopped for red lights and went on green, cruised at 8.2 m/s against an 8.33 m/s limit, and stopped
at the route's end. With traffic it twice chose to pass a crossing car at a junction (`pass:<id>`). Red lights were
not independently checked, but the leaderboard's RunningRedLightTest will check them.

## Bench2Drive

- `scripts/run_bench2drive.sh --agent expert --routes 0 --tries 1` ran route 0 (Town12, ParkingCutIn).
- **Result: 0% route completion, AgentBlockedTest failed**, with no collisions or red lights. The car moved 0.3 m
  and stopped for good.
- Logs: `results/bench2drive/expert/seed1/out/bench2drive_00.log` and `.../viz/bench2drive_00/expert.jsonl`.

### The open bug (start here tomorrow)

The behaviour layer reported "blockers" at s = 0.0 (several) and at 9.4, 14.9 and 20.4 m.

1. **The s = 0 blockers** are actors behind the route's start. An open path clamps their projection onto s = 0,
   keeping only the sideways offset, so they looked in the lane. Combined with the no-reverse clamp, that held the
   car where it was.
   - **Fixed, not yet re-run:** in `core/urban/src/behaviour.cpp`, the `project` lambda uses the true distance when
     a point lies beyond the ends of the projection window.
   - The re-run of route 0 was started and then stopped at your request.
2. **The 9–20 m blockers** are probably the parked cars of ParkingCutIn, which sit beside the lane. Why they count
   as *in* the lane is not yet known. Suspects:
   - the lane band is widened by the ±6-sample union in `Route._lane_sections` (meant for lane changes);
   - the cars' boxes overlap the lane edge;
   - the route starts next to the parking lane.

   The log now records every actor's `[id, kind, parked, s, e_y, length, width, predicted travel]` and the lane band
   at each plan, under `"scene"`. Re-run route 0 and read it:
   ```bash
   scripts/run_bench2drive.sh --agent expert --routes 0 --tries 1
   external/simlingo_venv/bin/python -c "import json; rows=[json.loads(l) for l in open('results/bench2drive/expert/seed1/viz/bench2drive_00/expert.jsonl')]; print([r['scene'] for r in rows if r['scene']][:3])"
   ```
   A likely fix: count an actor as a blocker only if it overlaps the lane by more than a margin (say 0.3 m), and
   take the lane band without the union when no lane change is near.

### Other known issues

- **Step time:** p95 is 50 ms with traffic; the worst steps (about 100–150 ms) are at standstill near other cars
  and on cold starts of new candidates. Idea: start a new candidate from the chosen plan rather than cold.
- **Lateral error in bends:** up to 0.5 m. Steering is sampled 0.15 s ahead for the lag; a feedforward from the
  planned curvature may help.
- **Start-up:** the first `ExpertDriver` in a fresh `build/urban_cache` compiles the solver for about 66 s. The
  cache is keyed by the problem, so changing NMPC options or vehicle parameters recompiles once. The leaderboard's
  timeouts allowed this on route 0.
- **Offline timing:** `build/urban_scenarios` checks p95 < 50 ms. With a CARLA server running on the same machine,
  two parallel-candidate scenarios exceed it (about 55 ms) with unchanged behaviour.
- **Not yet in the world model:** pedestrians predicted only at constant velocity; no junction-branch hypotheses;
  emergency vehicles (YieldToEmergencyVehicle) not handled; no right-side nudging. Lane changes along the route
  rely on the smoothed reference only.

## Picking up

```bash
cd ~/carla-expert-nmpc
cmake -S . -B build && cmake --build build -j                  # builds against external/simlingo_venv (Python 3.8)
ctest --test-dir build --output-on-failure                     # about 2.5 min, includes the 11 offline scenarios
external/simlingo_venv/bin/python -m pytest -q
scripts/start_carla.sh --quality Low                           # for drive_expert.py / identify_ego.py (port 2000)
external/simlingo_venv/bin/python scripts/drive_expert.py --town Town10HD --length 600 --traffic 40
scripts/start_carla.sh --stop                                  # run_bench2drive.sh starts its own servers (port 20000)
scripts/run_bench2drive.sh --agent expert --routes 0 --tries 1
```

`external/` is a symlink to `~/carla-track-control/external` (CARLA, SimLingo, checkpoint, venv) and is not in
git. The SimLingo venv has CasADi 3.7.2, pybind11, imageio-ffmpeg and pytest added.

**Next steps, in order:**
1. Fix the route 0 blockers and get route 0 driving.
2. Run the 22 routes SimLingo already has results for (`results/bench2drive/bench2drive_routes.csv`: routes 0–21)
   and compare.
3. Write the SimLingo + NMPC agent (`docs/plan.md`, section 4).
4. Pick one route per scenario type as the dev set.
