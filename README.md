# carla-expert-nmpc

[SimLingo](https://github.com/RenzKa/simlingo) driving [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive)
through an urban nonlinear MPC in place of its PID. One stack:

- **SimLingo** gives the reference: its predicted path is the route the NMPC plans along, rebuilt every plan, and
  its speed waypoints the speed asked for. The NMPC plans in the car's own frame, as SimLingo predicts in it: the
  car at the origin, no global position, its state what it measures (speedometer, IMU, its steering). Nothing of
  the route comes from CARLA.
- **The world model** (`expert/world_model/`) stands in for perception. For now it reads privileged ground truth,
  but only what a perception stack on Bench2Drive's sensors could estimate: visible actors in range, the current
  state of lights and signs, and the map, and reports them in the car's frame. It never reads other agents'
  intentions, scenario triggers, occluded actors or future light phases. A learned perception model can later replace it behind the same `Scene`.
- **The urban NMPC and its behaviour layer** (`core/urban/`) track the reference within the world model's
  constraints, which act as a safety filter.

The agent is `expert/agents/simlingo_nmpc_agent.py`: SimLingo's `LingoAgent` with its PID (`control_pid`) replaced
by the NMPC.

The plan: [docs/plan.md](docs/plan.md). How it all works: [docs/architecture.pdf](docs/architecture.pdf). What is
open: **[docs/todo.md](docs/todo.md)**; what was learned: [docs/learnings.md](docs/learnings.md). Older hand-off
notes: [docs/STATUS.md](docs/STATUS.md). Where the code came from: [PROVENANCE.md](PROVENANCE.md).

## Layout

| | |
|---|---|
| `core/urban/` | the urban NMPC, the behaviour layer, the route path and the vehicle parameters ([its README](core/urban/README.md)) |
| `sim/urban/` | closed-loop scenarios for the urban NMPC without CARLA (`build/urban_scenarios`) |
| `bindings/expertctl.cpp` | the Python module `expertctl` |
| `expert/world_model/` | the perception stand-in: route and lanes from the map, visible actors, stop lines |
| `expert/` | frames, SimLingo's reference (`simlingo_ref.py`), MKZ actuation, and the agent (`agents/`) |
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
scripts/run_bench2drive.sh --agent simlingo_nmpc --routes 0-19 --no-cot --fast    # SimLingo through the NMPC
```
