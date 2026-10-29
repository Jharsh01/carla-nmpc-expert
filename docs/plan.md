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
| ego pose, v, a, yaw rate | `get_transform/velocity/acceleration` | GNSS + IMU + speedometer, with a UKF (SimLingo has one) | optional noise and latency |
| route | the leaderboard's global plan (given to every agent) | the same | none |
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

## 3. CARLA integration (built for the map route)

- `scripts/identify_ego.py` → `config/vehicles/mkz2020_actuation.json`; `expert/mkz_actuation.py` maps the planned
  acceleration and steering angle to throttle, brake and steer (table inverse + PI).
- `expert/driver.py`: plans at 10 Hz, controls at 20 Hz, logs each tick.
- `expert/agents/expert_agent.py`: the world model and the NMPC on the map route, without SimLingo. It is the test
  bed for the NMPC and perception half (`run_bench2drive.sh --agent expert`).

## 4. SimLingo in the loop (`expert/agents/simlingo_nmpc_agent.py`, next)

- Subclass `LingoAgent` and override `control_pid`.
- `pred_route` (spatial path, ego frame) becomes `e_ref(s)` relative to the map route. When SimLingo deviates (a
  nudge or a lane change), the corridor follows it within map-legal lanes.
- `pred_speed_wps` (0.25 s apart) becomes `v_ref(t)`, capped by the limit and the world model's constraints.
- With CoT on, SimLingo's commentary may later steer the behaviour layer (e.g. which candidate to prefer).
- The world model's constraints stay hard. When SimLingo's reference would violate one, the NMPC overrides it and
  logs an `override` event, which measures how often SimLingo alone would have crashed.
- SimLingo runs at its own rate (about 0.4 s a step with `--fast`); the NMPC keeps solving at 10 Hz on the latest
  reference.
- `run_bench2drive.sh --agent simlingo_nmpc`, with its own results folder.

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
| M2 | World model + MKZ actuation; the NMPC drives Bench2Drive routes on the map route | in progress (`docs/STATUS.md`) |
| M3 | SimLingo + NMPC agent | not started |
| M4 | Dev set: SimLingo + NMPC ≥ SimLingo + PID; iterate | not started |
| M5 | Full 220 × 3 seeds, perception ablations, report | not started |

## Risks

- **Mixed-integer interaction** (yield or pass, lane borrowing): handled by parallel candidates, each a local NLP.
  Plan for about 4 cores per agent.
- **Solve time with many actors**: fixed slot counts, the nearest by time to conflict.
- **SimLingo's reference fighting the constraints** (e.g. it asks to go through a red light): the override log
  shows it. If overrides are frequent, the reference may need filtering before the NMPC sees it.
- **Python 3.8 + the CasADi wheel's ABI**: keep `_GLIBCXX_USE_CXX11_ABI=0` (top-level `CMakeLists.txt`) and pin CasADi 3.7.
