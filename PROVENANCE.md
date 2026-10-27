# Provenance

Copied from [carla-track-control](https://github.com/Jharsh01/carla-track-control) at commit `8495ae9`
("Corner-outage scenarios, pure pursuit following the NMPC plan, SimLingo on Bench2Drive"), without its
history. Changes since then are this repository's own.

| here | there |
|---|---|
| `core/track_control/` | `track_control/` (unchanged: NMPC, pure pursuit, path, bicycle model, directives) |
| `sim/plant/` | `sim/plant/` (bicycle plant only; Project Chrono left behind) |
| `expert/frames.py`, `expert/actuation.py` | `carla_bridge/` |
| `scripts/` | `scripts/` (CARLA, SimLingo and Bench2Drive; env.sh now points at SimLingo's venv) |
| `config/vehicles/` | `config/vehicles/` |
| `docs/reference/` | `docs/` (the NMPC derivations and solve-time study, vehicle parameters) |

Not copied: the race track (`sim/racetrack/`, `carla_bridge/` apart from the two files above), the Chrono
plant and the corner-outage scenarios. `FallbackControl` (`sim/racetrack/fallback_control.*`, the NMPC to pure
pursuit handover) is tied to the race track's `Driver`; its blend is to be ported to the urban NMPC (docs/plan.md, M1).
