# carla-expert-nmpc

[SimLingo](https://github.com/RenzKa/simlingo) driving [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive)
through an urban nonlinear MPC in place of its PID. One stack:

- **SimLingo** gives the reference: its predicted path is the route the NMPC plans along, rebuilt every plan, and
  its speed waypoints the speed asked for. The NMPC plans in the car's own frame, as SimLingo predicts in it: the
  car at the origin, no global position, its state what it measures (speedometer, IMU, its steering). Nothing of
  the route comes from CARLA.
- **The middle layer** (`middle_layer/`) is perception: its sensors (`perception/sensors/`, Python) stand in for a
  perception stack, and its post-processing (`perception/post_process/`, C++) takes what they report into the
  route's path coordinates and classifies it. For now the sensors read privileged ground truth, but only what a
  perception stack on Bench2Drive's sensors could estimate: visible actors in range, the current state of lights
  and signs, the map, and how far the car can see, and report them in the car's frame. They never read other
  agents' intentions, scenario triggers, occluded actors or future light phases. A learned perception model can
  later replace them behind the same `Scene`.
- **The NMPC** (`nmpc/`) tracks the reference within the constraints of each manoeuvre (`nmpc/src/constraint/`:
  lane, movers, follow, stop, crossing, overtake, turn, grip), which act as a safety filter; its planner solves the
  candidate manoeuvres in parallel and picks one.

The agent is `expert/agents/simlingo_nmpc_agent.py`: SimLingo's `LingoAgent` with its PID (`control_pid`) replaced
by the NMPC.

The plan: [docs/plan.md](docs/plan.md). How it all works: [docs/architecture.pdf](docs/architecture.pdf). What is
open: **[docs/todo.md](docs/todo.md)**; what was learned: [docs/learnings.md](docs/learnings.md); run reports:
[docs/reports](docs/reports/2026-11-01-lane-keeping-and-traffic/README.md). Older hand-off
notes: [docs/STATUS.md](docs/STATUS.md). Where the code came from: [PROVENANCE.md](PROVENANCE.md).

## Layout

| | |
|---|---|
| `nmpc/` | the NMPC: framework, cost, the manoeuvres' constraints, the planner; paths, `shared_param.hpp` ([its README](nmpc/README.md)) |
| `middle_layer/` | perception: sensors (actors, the map's lanes and stops, the depth camera's sight) and their post-processing ([its README](middle_layer/README.md)) |
| `sim/urban/` | closed-loop scenarios for the NMPC without CARLA (`build/urban_scenarios`) |
| `bindings/expertctl.cpp` | the Python module `expertctl` |
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
