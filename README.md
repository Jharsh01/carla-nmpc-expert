# carla-expert-nmpc

An expert driver for [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) built on a nonlinear MPC, meant to
drive better than PDM-Lite, in two modes on one NMPC core:

- **Expert (Mode A):** privileged, but only with what a perception stack on Bench2Drive's sensors could estimate:
  visible actors in range, the current state of lights and signs, the map. Never other agents' intentions,
  scenario triggers, occluded actors or future light phases.
- **SimLingo + NMPC (Mode B):** [SimLingo](https://github.com/RenzKa/simlingo)'s path and speed waypoints as the
  NMPC's reference in place of its PID, the same world information as constraints.

The plan, its phases and milestones: [docs/plan.md](docs/plan.md).

## Status

- **M0, done:** the repository bootstrapped, built against SimLingo's Python 3.8.
- **M1, done offline:** the urban NMPC and the behaviour layer (most of M3 came forward, since the scenarios needed it).
  All 11 offline scenarios pass: a bend, a lead braking hard, a cut-in, a pedestrian from behind a van, a red light,
  the red-light dilemma zone, a stop sign, overtaking a parked car (clear, and waiting for oncoming traffic), and
  yielding to or passing a crossing car. p95 step time is 12 ms with one candidate and 48 ms with three in parallel.
- **M2, in progress:** the MKZ identified, the world model, the expert driving in CARLA (604 m with 40 traffic cars,
  no contact), and the leaderboard agent. Bench2Drive route 0 does not drive yet. Where it stopped and how to pick
  it up: **[docs/STATUS.md](docs/STATUS.md)**. Where the code came from: [PROVENANCE.md](PROVENANCE.md).

## Layout

| | |
|---|---|
| `core/urban/` | the urban NMPC and the behaviour layer ([its README](core/urban/README.md)) |
| `core/track_control/` | from carla-track-control: the race-track NMPC, pure pursuit, paths, the bicycle model, directives |
| `sim/urban/` | closed-loop scenarios for the urban NMPC without CARLA (`build/urban_scenarios`) |
| `sim/plant/` | the bicycle plant |
| `bindings/expertctl.cpp` | the Python module `expertctl` |
| `expert/` | Python: frames, actuation, the world model and the leaderboard agents |
| `scripts/` | CARLA, SimLingo and Bench2Drive: install and run |

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
scripts/run_bench2drive.sh --routes 0-19 --no-cot --fast    # SimLingo with its PID, as published
```
