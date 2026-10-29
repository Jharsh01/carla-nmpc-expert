# Plan: SimLingo + urban NMPC + a perception stand-in, on Bench2Drive

## Goal

SimLingo drives Bench2Drive with a hand-tuned PID (`agent_simlingo.py::control_pid`). Here its path and speed
waypoints become the reference of an urban NMPC instead. What the car may not hit, and the rules it must obey, come
from a world model, and they enter the NMPC as constraints acting as a safety filter. The stack has three parts:

```
SimLingo (camera)  ──  reference: path e_ref(s), speed v_ref(t)  ──┐
World model (privileged, perception-shaped)  ──  Scene  ──────────┼──>  behaviour layer + urban NMPC  ──>  MKZ actuation
```

**The rule for privileged info:** the world model may only give what a perception stack on Bench2Drive's sensor
suite could estimate. That means actors visible to sensors within range, the current state of lights and signs, and
the map. It never gives other agents' intentions, scenario triggers, occluded actors or future light phases. All of
it goes through one `Scene`, so a learned perception model can replace the privileged source later.

**Success:** SimLingo + NMPC's Bench2Drive driving score is at least SimLingo + PID's, with fewer collisions, and
the NMPC's overrides of SimLingo's reference are logged and counted.

---

## 1. The world model (`expert/world_model/`, built)

| Scene field | CARLA source (privileged) | What would estimate it | Realism filter |
|---|---|---|---|
| the car's pose | `hero.get_transform()`: only to report the rest in the car's frame | a sensor on the car (detections come car-relative) or HD-map localisation | none |
| route | none: SimLingo's predicted path (section 4); only the route's end from the leaderboard's sparse plan | — | — |
| lanes: band, left lane, oncoming, borrowable; stop lines | `CarlaDataProvider.get_map()` waypoints | HD map or online mapping | local window |
| vehicles, bikes, pedestrians: box, pose, velocity, parked | `world.get_actors()` | 3D detection + tracking | in range (60 m, walkers 40 m), line of sight by ray casts; velocity from tracked boxes |
| static obstacles: cones, warning triangles, debris | props among `get_actors()` | detection | same visibility filter |
| traffic light for the ego lane, stop signs | `ego.get_traffic_light()`, trigger volumes | TL / sign detection + map | current state only |

Prediction: vehicles along their lane (map), everything else at constant velocity. Still to do: junction-branch
hypotheses, emergency vehicles, and a noise/degradation config (position σ, velocity σ, drop rate, latency) for
ablations.

## 2. The urban NMPC and the behaviour layer (`core/urban/`, built)

A kinematic bicycle in the route's path coordinates, with soft constraints for the corridor, movers, leads,
stop/pass gates, grip and headway (CasADi + FATROP). The behaviour layer classifies actors, builds 1–3 candidate
plans (yield or pass each crosser; stay behind or borrow the left lane), solves them in parallel and picks one. It
also holds the traffic-rule state. The design reasons are in `core/urban/README.md`; 11 offline scenarios
(`build/urban_scenarios`) cover it without CARLA.

Still to do: a fallback for a late or failed solve (follow the last plan, then brake at the friction limit);
start new candidates from the chosen plan to cut cold-start solve times.

## 3. CARLA integration (`expert/agents/simlingo_nmpc_agent.py`: plans at 10 Hz, controls at 20 Hz, logs each tick)

- `scripts/identify_ego.py` → `config/vehicles/mkz2020_actuation.json`; `expert/mkz_actuation.py` maps the planned
  acceleration and steering angle to throttle, brake and steer (table inverse + PI).

## 4. SimLingo in the loop (`expert/agents/simlingo_nmpc_agent.py`, built, first runs pending)

- `LingoAgent` subclassed, `control_pid` overridden: SimLingo still predicts every tick; the NMPC replaces its PID.
- **The route is SimLingo's** (`expert/simlingo_ref.py`). Its 20 path points (about 1 m apart) become the NMPC's
  `Path` as they are, in the car's frame, rebuilt at every plan. The Scene's line is that path.
- **The NMPC plans in the car's frame**: the car at the origin, no global position. `Behaviour::set_route` carries
  the previous plan into the new frame, given the old frame's pose in the new one, from odometry (speedometer and
  compass, `simlingo_ref.Odometry`). The world model reports its actors, lanes and stops in the car's frame
  (`frames.Frame`, from the car's true pose: the stand-in for sensors on the car). An earlier version placed the
  path in the world by SimLingo's own GNSS/UKF pose; on Town12 that was 497.6 m off CARLA's frame, since SimLingo
  estimates its GNSS reference itself.
- Past SimLingo's last point the path goes on along the lane that point is in, for 160 m (`world_model/route.py`):
  the behaviour layer looks for actors and stops that far, and on a straight continuation a bend's lane, a lead in
  the bend and the bend's speed are all wrong (`curve@local` drifted 0.54 m that way). Only SimLingo's part is
  asked to be followed.
- `pred_speed_wps` (0.25 s apart) become `v_ref(t)`, capped by the limit and the world model's constraints.
- The car's state: what it measures: the speedometer, the IMU's forward acceleration, the steering it commanded.
- The world model's constraints stay hard. The log keeps, at each plan, how far the plan departs from SimLingo's
  path over its first 2 s and from its speed at 1 s (`override`).
- SimLingo's restart creep (forced throttle after a long stop) is off: the NMPC decides when to move.
- Offline, every scenario also runs as `<name>@local` on SimLingo-like local paths (`sim/urban/scenarios.cpp`).
- Later: with CoT on, SimLingo's commentary may steer the behaviour layer (e.g. which candidate to prefer).

## 5. Evaluation

- Baseline: SimLingo + PID on the same machine, seeds and routes (`run_bench2drive.sh --agent simlingo`).
- Dev set: one route per scenario type, about 44 routes. The full 220 only at milestones, on 3 seeds. Several
  routes run side by side through `scripts/b2d_evaluator.py` (port ranges per job); `scripts/summarize_b2d.py`
  scores a folder of routes and `scripts/route_report.py` maps one.
- Metrics: driving score and success rate, multi-ability scores, infractions by type, solve times, and override
  and fallback rates.
- Perception ablations: no visibility filter (upper bound), then the filter, then noise, latency and drops.

## Milestones

| # | Deliverable | State |
|---|---|---|
| M0 | Repo bootstrapped, builds in the Python 3.8 venv | done |
| M1 | Urban NMPC, behaviour layer, offline scenarios | done |
| M2 | World model + MKZ actuation | done (`docs/STATUS.md`) |
| M3 | SimLingo + NMPC agent | built; first Bench2Drive runs (`docs/STATUS.md`) |
| M4 | Dev set: SimLingo + NMPC ≥ SimLingo + PID; iterate | not started |
| M5 | Full 220 × 3 seeds, perception ablations, report | not started |

## Risks

- **Mixed-integer interaction** (yield or pass, lane borrowing): handled by parallel candidates, each a local NLP.
  Plan for about 4 cores per agent.
- **Solve time with many actors**: fixed slot counts, the nearest by time to conflict.
- **SimLingo's reference fighting the constraints** (e.g. it asks to go through a red light): the override log
  shows it. If overrides are frequent, the reference may need filtering before the NMPC sees it.
- **Python 3.8 + the CasADi wheel's ABI**: keep `_GLIBCXX_USE_CXX11_ABI=0` (top-level `CMakeLists.txt`) and pin CasADi 3.7.
