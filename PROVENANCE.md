# Provenance

Copied from [carla-track-control](https://github.com/Jharsh01/carla-track-control) at commit `8495ae9`
("Corner-outage scenarios, pure pursuit following the NMPC plan, SimLingo on Bench2Drive"), without its
history. Changes since then are this repository's own.

| here | there |
|---|---|
| `core/urban/` `path`, `vehicle_params`, `integrate`, `body`, `tests/` | `track_control/` (folded into `core/urban` on 29 October 2026, namespace `ex`; its race-track NMPC, pure pursuit, speed profile, directives, dynamic bicycle and tyre model, and `sim/plant/`'s bicycle plant were removed) |
| `expert/frames.py` | `carla_bridge/` (and `actuation.py`, since replaced by the MKZ's `expert/mkz_actuation.py`) |
| `scripts/` | `scripts/` (CARLA, SimLingo and Bench2Drive; env.sh now points at SimLingo's venv) |
| `config/vehicles/` | `config/vehicles/` |
| `docs/reference/` | `docs/` (vehicle parameters) |

Not copied: the race track (`sim/racetrack/`, `carla_bridge/` apart from the two files above), the Chrono
plant, the corner-outage scenarios and `FallbackControl` (the NMPC to pure pursuit handover).
