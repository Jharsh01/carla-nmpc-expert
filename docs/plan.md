# Plan: expert NMPC for Bench2Drive, in its own repo (SimLingo + perception-estimable privileged info)

## Context

`carla-track-control` has grown two things that don't belong together: a race-track NMPC with a pure-pursuit fallback (`track_control/`, `sim/racetrack/`), and a SimLingo/Bench2Drive harness (`scripts/install_simlingo.sh`, `run_bench2drive.sh`, `simlingo.patch`). SimLingo drives Bench2Drive with a hand-tuned PID (`agent_simlingo.py::control_pid`). PDM-Lite (`external/simlingo/team_code/autopilot.py`) is the rule-based privileged expert the field uses to generate data. It uses IDM for speed, PID on a fixed route for steering and hard-coded route shifts for overtaking.

**Goal:** a new repo with one NMPC core and two front ends:
- **Mode A, expert:** privileged-only, a drop-in replacement for PDM-Lite. It must score higher than PDM-Lite on Bench2Drive, and it doubles as a data generator.
- **Mode B, SimLingo + NMPC:** SimLingo's path and speed waypoints become the NMPC's reference, replacing its PID. Privileged info enters only as constraints that act as a safety filter.

**The rule for privileged info:** the expert may only use what a perception stack on Bench2Drive's sensor suite could estimate. That means actors visible to sensors within range, the current state of lights and signs, and the map. It never uses other agents' intentions, scenario triggers, occluded actors or future light phases. Every input goes through one `WorldModel` interface, so a learned perception stack can replace it later.

Decisions already made: fresh repo with the code copied in (not filter-repo); the C++ CasADi/FATROP NMPC through pybind, built for the Python 3.8 SimLingo venv; both modes.

---

## Phase 0: bootstrap the repo (`~/carla-expert-nmpc`)

1. Commit or stash the current uncommitted work in `carla-track-control` first. The copy is taken from that state, and its commit hash goes in `PROVENANCE.md`.
2. Copy the following, then prune:
   - `track_control/` keeps its whole layout: `nmpc`, `pure_pursuit`, `path`, `model/`, `directive`, the tests and the CMake package. It stays namespace `tc`.
   - `sim/plant/bicycle_plant.{hpp,cpp}`, `filters.hpp` and `interface.hpp` are for offline closed-loop tests. Chrono and the race track are dropped.
   - `sim/racetrack/fallback_control.{hpp,cpp}` keeps the NMPC→pure-pursuit blend and the plan-following fallback. It is renamed `core/fallback`.
   - `carla_bridge/frames.py` is for the CARLA left-handed ↔ `tc` right-handed conversion. `actuation.py` is the base of the inverse throttle/brake map.
   - `scripts/env.sh`, `install_carla.sh`, `start_carla.sh`, `install_simlingo.sh`, `run_bench2drive.sh` and `simlingo.patch`.
   - `docs/nmpc_pure_pursuit_derivations.pdf` and `nmpc_solve_time.pdf` go in as references.
   - `bindings/trackctl.cpp` is rewritten as a smaller module, `expertctl` (see Phase 2).
3. Layout:
   ```
   core/            C++: track_control (copied) + new urban OCP, corridor, prediction, behaviour
   bindings/        pybind11 module expertctl (built against simlingo_venv's Python 3.8)
   expert/          Python: world_model/, reference.py, behaviour.py, actuation.py, agents/
     agents/expert_agent.py         Mode A, a leaderboard AutonomousAgent (MAP track)
     agents/simlingo_nmpc_agent.py  Mode B, subclasses LingoAgent and overrides control_pid
   sim/             offline bicycle plant + scripted urban scenarios (C++), as scenarios/ did for corners
   scripts/         install, run_bench2drive.sh --agent {simlingo,expert,simlingo_nmpc,pdm_lite}
   tests/           pytest (fake carla from tests/fake_carla/carla.py), ctest
   docs/
   ```
4. CI: ctest plus pytest with the fake `carla` module (reuse the `.github/workflows/ci.yml` pattern).

**Exit:** `cmake --build` and the copied ctest pass; `python -c "import expertctl"` works inside `external/simlingo_venv`.

---

## Phase 1: the world model (privileged info, perception-shaped)

`expert/world_model/` turns CARLA ground truth into a `Scene` in exactly the form a perception stack would produce. Each field records its source and the perception stand-in it represents:

| Scene field | CARLA source (privileged) | What would estimate it | Realism filter |
|---|---|---|---|
| ego pose, v, a, yaw rate | `get_transform/velocity/acceleration` | GNSS+IMU+speedometer in B2D's suite, with a UKF (SimLingo already has one) | optional noise and latency |
| route | leaderboard global plan (given to every agent) | the same | none |
| lanes: centre, boundaries, width, lane-change permission, junction areas, stop lines | `CarlaDataProvider.get_map()` waypoints / OpenDRIVE | HD map (Leaderboard MAP track) or online mapping (MapTR-like) | local window only, about 80 m |
| vehicles, bikes, pedestrians: box, pose, velocity, class, flags (emergency, parked, door open) | `world.get_actors()` | 3D detection + tracking | **visible only**: in range (≤ 50 m), in a sensor FOV, ≥ N lidar points or unoccluded ray (the same approach as carla_garage's visibility check); velocity from finite differences of tracked boxes, not `get_velocity` |
| static obstacles: cones, warning triangles, accident debris | props among `get_actors()` | detection | same visibility filter |
| traffic light affecting ego lane: state, stop line | `ego.get_traffic_light()`, trigger volumes | TL detector + map association | current state only, no timers or phase prediction |
| stop signs, speed limit | map landmarks / sign actors | sign detection + map | in range |

**Forbidden, and enforced by the interface rather than by convention:** Traffic Manager targets and routes, scenario trigger data, future light phases, actors that fail the visibility check, and `get_control()` of other actors.

Pieces:
- `Scene` dataclass, plus `PrivilegedSource` (CARLA) and later a `PerceptionSource` (learned) behind the same interface.
- A noise/degradation config: position σ, velocity σ, detection drop rate, latency, range. This is for ablations.
- Prediction (C++, `core/prediction`): each actor is predicted over the horizon with uncertainty growing in time:
  - vehicles: lane-following (map-matched, constant velocity along their lane, every successor branch kept at a junction);
  - walkers and bikes: constant velocity;
  - parked actors: static.

---

## Phase 2: NMPC redesign for urban driving (`core/urban_nmpc`)

The track NMPC (`track_control/src/nmpc.cpp`) is built for racing: a dynamic bicycle with tyre slip, Frenet coordinates on a closed path, and 2 ellipse obstacles. Bench2Drive needs stop-and-go, rules and interaction. What is reused: the cost/constraint term tables, the Directive mechanism (`directive.hpp`, `NMPC::interpret`), variable steps, the FATROP warm start, `Path::from_waypoints` and the pure-pursuit fallback.

**Model.** Kinematic bicycle in Frenet coordinates along the route reference:
- state `[s, e_y, e_psi, v, a, delta]`, inputs `[jerk, delta_rate]`;
- `v ≥ 0` hard, so it is well posed at standstill, which the dynamic model's slip angles are not;
- grip enters as a friction-ellipse constraint `a² + (v² κ_eff)² ≤ (μ g)²`, where `κ_eff` is the path curvature plus the lateral term;
- horizon 5 s at 0.1 s steps (0.05 s for the first 0.3 s); solved at 10 Hz and played at CARLA's 20 Hz.

**Constraints**, each a new entry in the term tables:
1. **Corridor:** `e_min(s) ≤ e_y ± half-width ≤ e_max(s)`, built per solve from the lane boundaries. The behaviour layer can widen it into an adjacent lane, or the opposite lane for the *TwoWays* scenarios. Soft, with a large slack weight.
2. **Leading vehicle (same lane):** `s_k ≤ s_lead(t_k) − d0 − T_h v_k` (time headway).
3. **Crossing conflicts** (pedestrian crossing, junction traffic, cut-ins): for each predicted actor path that crosses the corridor at `s_c` during `[t_in, t_out]`, apply either *yield* (`s_k ≤ s_c − margin` for `t_k ∈ window`) or *pass* (`s_k ≥ s_c + margin` by `t_in`). The behaviour layer chooses which.
4. **Lateral obstacles** (parked cars, cones, open doors): oriented boxes projected into Frenet coordinates, covered by circles; soft.
5. **Stops:** red or amber light, a stop sign (stop for at least the hold time, then release), a blocked junction (don't enter unless the exit is clear). Applied as `s_k ≤ s_stop`.
6. **Speed:** the map or sign limit, a curvature limit and a comfort band: `|a| ≤ 3`, `|jerk| ≤ 5`, `|lateral a| ≤ 3` (tunable).

**Cost:**
- track `v_ref` (Mode A: the speed limit. Mode B: SimLingo's speed waypoints, below);
- track `e_ref` (the lane centre, or the nudge or lane-change target);
- penalties on jerk, steering rate and slack;
- a progress term, to avoid Bench2Drive's min-speed infractions and blocked-agent timeouts.

**Behaviour layer** (`core/behaviour`, the part PDM-Lite does with rules). Each tick it lists 1–4 candidate *homotopies*:
- keep lane and follow;
- nudge within the lane;
- borrow the adjacent or opposite lane, only if the gap in visible oncoming traffic covers the manoeuvre time plus a margin;
- for each crossing conflict, yield or pass;
- creep (a low `v_max` with an extended stop line) for occluded junctions and pedestrians at parked cars.

Each candidate is a separate NMPC instance solved in parallel (one thread each, warm-started from its own last plan). The plan chosen is the lowest-cost feasible one. Traffic rules are hard filters, and switching between homotopies has hysteresis.

**Execution:** the same handover as now, using `FallbackControl`:
- a late or failed solve hands over to pure pursuit following the last plan;
- if no candidate is feasible, it brakes (emergency stop at the friction limit).

**Bindings (`expertctl`):** `UrbanNMPC`, `Scene` structs, `Behaviour.step(scene) → plan, inputs, debug`, and the directive API.

**Offline tests** (`sim/`, with the bicycle plant): a scripted C++ scenario per behaviour, run without CARLA in about 1 s each:
- lead car braking hard;
- cut-in;
- pedestrian crossing from behind a parked car;
- red light then green;
- stop sign;
- parked car needing the opposite lane, with oncoming traffic;
- unprotected left turn with oncoming flow.

Each test asserts no collision, the stop line respected, comfort limits held and solve time p95 < 50 ms.

---

## Phase 3: CARLA integration and actuation

1. **Identify the Bench2Drive ego** (`vehicle.lincoln.mkz_2020`):
   - collect throttle/brake → longitudinal acceleration as a function of speed, and steer → road-wheel angle;
   - script: `scripts/identify_ego.py`, built on the approach of `scripts/fit_chrono_tires.py`;
   - fit the maps used by `expert/actuation.py`.
2. **Inner loop at 20 Hz:** the inverse map from the planned `a` plus PI on acceleration error gives throttle/brake. The planned `delta` interpolated at the current time gives steer. This replaces SimLingo's PID in Mode B too.
3. **Mode A agent** (`expert_agent.py`):
   - sensors: the minimal B2D set (GNSS/IMU/speed), plus one camera for video only;
   - each step: `PrivilegedSource → Scene → Behaviour → NMPC → actuation`;
   - logs per step: Scene, chosen homotopy, plan and solve stats, as JSON lines alongside the leaderboard's `metric_info`.
4. **Mode B agent** (`simlingo_nmpc_agent.py`):
   - subclass `LingoAgent` and override `control_pid`;
   - `pred_route` (spatial path, ego frame) becomes `e_ref(s)` relative to the map route. If SimLingo deviates (a nudge or lane change), the corridor follows it within map-legal lanes;
   - `pred_speed_wps` (0.25 s apart) becomes `v_ref(t)`;
   - SimLingo's commentary, with CoT on, maps to `Intent`s through `NMPC::interpret`;
   - privileged constraints stay hard. When SimLingo's reference violates one, the NMPC overrides it and logs an `override` event, which measures how often SimLingo would have crashed;
   - SimLingo runs at its own rate (about 0.4 s per step with `--fast`); the NMPC keeps solving at 10 Hz on the latest reference.
5. `run_bench2drive.sh --agent {simlingo, simlingo_nmpc, expert, pdm_lite}` with separate result folders. Allow N parallel CARLA servers (ports 20000 + 100 k) to cut a full run from days to hours.

---

## Phase 4: evaluation (beating PDM-Lite)

- **Baselines on the same machine, seeds and routes:**
  - PDM-Lite (the autopilot from carla_garage / SimLingo's `team_code/autopilot.py`, behind a thin agent wrapper);
  - SimLingo + PID: the existing results in `results/bench2drive/simlingo-nocot-fast/` cover 23 routes, so extend them to 220.
- **Dev set:** one route per scenario type, about 44 routes, which takes hours rather than days. It is the iteration loop. The full 220 run happens only at milestones, on 3 seeds.
- **Metrics:**
  - Bench2Drive DS and SR;
  - multi-ability scores (merging, overtaking, emergency brake, give way, traffic signs);
  - efficiency and comfort (B2D's smoothness metrics);
  - infraction counts by type;
  - solve-time distribution;
  - fallback and override rates.
- `scripts/summarize_b2d.py`: per-scenario comparison tables (expert vs PDM-Lite vs SimLingo vs SimLingo+NMPC), extending `results/bench2drive/bench2drive_routes.csv`.
- **Perception-realism ablations** of the expert:
  - GT with no visibility filter (an upper bound), then the visibility filter;
  - plus noise, plus 100 ms latency, plus 10% detection drops.

  These show how much of the expert's gain survives a realistic perception stack, which is the case for "estimable".

**Success criteria:**
1. The expert's DS beats PDM-Lite's on the full 220 routes, mean of 3 seeds, with the visibility filter on.
2. SimLingo+NMPC's DS is at least SimLingo+PID's, with fewer collision infractions.

---

## Milestones (order of work)

| # | Deliverable | Exit check |
|---|---|---|
| M0 | Repo bootstrapped, builds in the py3.8 venv | ctest + `import expertctl` |
| M1 | Urban NMPC + offline scenario suite | all 7 offline scenarios pass, p95 solve < 50 ms |
| M2 | WorldModel + actuation identified, Mode A drives an empty route | lane keeping in CARLA, \|e_y\| < 0.3 m, smooth stops at lights |
| M3 | Behaviour layer (homotopies) + all constraint types | dev set: no collision on static and cut-in scenarios |
| M4 | PDM-Lite baseline + expert on the dev set, iterate | expert ≥ PDM-Lite on the dev set |
| M5 | Mode B (SimLingo + NMPC) | dev-set DS ≥ SimLingo+PID |
| M6 | Full 220 × 3 seeds, all four agents + perception ablations | criteria above, report in `docs/` |

## Risks and mitigations

- **Mixed-integer interaction** (yield vs pass, lane borrowing) makes a single NLP non-convex. The parallel homotopy candidates keep each NLP local. Plan for about 4 cores per agent.
- **Low-speed and standstill solves.** The kinematic model with `v ≥ 0` covers this. Add a dedicated "hold" mode at `v < 0.5` with a stop target, to skip solves.
- **Solve time with many obstacles.** Fixed slot counts as now (pad unused slots far away), with the K nearest by time-to-conflict. Measure using the `SolveStats` pattern.
- **Python 3.8 + CasADi wheel ABI.** Keep `TRACK_CONTROL_OLD_CXX_ABI=ON` as now, and pin `casadi<3.8` in the venv.
- **Min-speed infractions and timeouts** from over-caution. The progress term, the creep mode, and tuning against B2D's efficiency metric.

## Verification

- `ctest --test-dir build` (copied model/path/controller tests, plus the new urban OCP unit tests and the offline scenarios).
- `pytest` with the fake `carla` module: Scene extraction, the visibility filter, frames, actuation maps and the agents' `run_step` plumbing.
- In CARLA:
  - `scripts/run_bench2drive.sh --agent expert --routes <dev set> --debug` produces videos with the plan, corridor and constraints overlaid;
  - `--agent simlingo_nmpc` does the same for Mode B;
  - `scripts/summarize_b2d.py` gives the comparison table against PDM-Lite and SimLingo+PID.
