# carla-expert-nmpc

[SimLingo](https://github.com/RenzKa/simlingo) driving [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive)
through an urban nonlinear MPC in place of its PID. One stack:

- **SimLingo** gives the reference: its path and speed waypoints.
- **The world model** (`expert/world_model/`) stands in for perception. For now it reads privileged ground truth,
  but only what a perception stack on Bench2Drive's sensors could estimate: visible actors in range, the current
  state of lights and signs, and the map. It never reads other agents' intentions, scenario triggers, occluded
  actors or future light phases. A learned perception model can later replace it behind the same `Scene`.
- **The urban NMPC and its behaviour layer** (`core/urban/`) track the reference within the world model's
  constraints, which act as a safety filter.

Until the SimLingo agent is written, `expert/agents/expert_agent.py` runs the world model and the NMPC on the
leaderboard's map route. That tests the NMPC and perception half on its own.

The plan: [docs/plan.md](docs/plan.md). Where the work stopped: **[docs/STATUS.md](docs/STATUS.md)**. Where the
code came from: [PROVENANCE.md](PROVENANCE.md).

## Layout

| | |
|---|---|
| `core/urban/` | the urban NMPC, the behaviour layer, the route path and the vehicle parameters ([its README](core/urban/README.md)) |
| `sim/urban/` | closed-loop scenarios for the urban NMPC without CARLA (`build/urban_scenarios`) |
| `bindings/expertctl.cpp` | the Python module `expertctl` |
| `expert/world_model/` | the perception stand-in: route and lanes from the map, visible actors, stop lines |
| `expert/` | frames, MKZ actuation, the per-tick driver and the leaderboard agents |
| `scripts/` | CARLA, SimLingo and Bench2Drive: install and run; MKZ identification |

## Setup

```bash
scripts/install_carla.sh        # CARLA 0.9.15 into external/carla
scripts/install_simlingo.sh     # SimLingo, its checkpoint, Bench2Drive and the Python 3.8 environment
cmake -S . -B build && cmake --build build -j    # builds against external/simlingo_venv's Python
ctest --test-dir build --output-on-failure
external/simlingo_venv/bin/python -m pytest -q
```

`external/` may be a symlink to another checkout's, to share the downloads (about 25 GB).

## Bench2Drive

```bash
scripts/run_bench2drive.sh --routes 0-19 --no-cot --fast    # SimLingo with its PID, as published (the baseline)
scripts/run_bench2drive.sh --agent expert --routes 0        # the world model and the NMPC on the map route
```
