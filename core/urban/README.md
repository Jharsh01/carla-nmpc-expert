# core/urban: the urban NMPC and the behaviour layer

- `urban_nmpc.hpp`: the NMPC. A kinematic bicycle in the route's path coordinates, and soft constraints for the
  corridor, movers (circles on predicted trajectories), leads, stop/pass gates, grip and headway. CasADi + FATROP.
- `behaviour.hpp`: classifies actors (lead, blocker, crosser, mover), builds candidate plans (yield or pass each
  crosser; stay behind a blocker or borrow the left lane), solves them in parallel and picks one. It also holds
  the traffic-rule state (stop-sign hold, red-light dilemma zone).
- `path.hpp`: the route as straights and arcs, fitted through waypoints (biarcs), with projection (windowed for
  long routes), pose and curvature at s. Tested by `tests/test_path.cpp`.
- `vehicle_params.hpp`: the MKZ's parameters the NMPC plans with (`config/vehicles/mkz2020.json`,
  `docs/vehicle_parameters.md`); `body.hpp`: its body size.
- `integrate.hpp`: the RK4 step the NMPC discretises its model with.
- `sim/urban/scenarios.cpp` (`build/urban_scenarios`, a ctest): 11 closed-loop scenarios without CARLA, on a
  kinematic plant with lags on acceleration and steering.

## Why it is built this way

Each point below came from a failing scenario.

| Choice | What went wrong without it |
|---|---|
| Speed bounded at −1 m/s, not 0 | With `v ≥ 0` active over many stages at a standstill, the multipliers degenerate and FATROP hits its iteration limit (about 200 ms a solve). |
| `acceptable_tol` 0.1 | Near standstill the dual infeasibility crawls down long after the plan has stopped changing (about 90 iterations against about 12). |
| Terminal room to stop (`a_stop`) | A plan that reaches the stop line just as the horizon ends never brakes. |
| Speed asked for tapers to stops, leads and bends (`a_ref`, `time_gap`, `a_lat_ref`) | A quadratic speed cost would rather creep towards a stop over the whole horizon than drive up and wait (a convex cost spreads the distance), so the car never actually stops. |
| Time gap in the speed asked for, not a position constraint | A position constraint priced high pays the plan to weave, since steering at an angle shortens progress along the route. A cut-in then makes it swerve out of its lane. |
| Slack weight 1e5 | With 1e3, a normalised mover violation was cheaper than braking to a stop, so the car drove into the pedestrian's path. |
| Yield or pass decided by the behaviour layer (gates), not by mover circles | A plan straight through a circle has no gradient along the route, so the solver stays in the local minimum: through the pedestrian, or stuck behind the parked car. |
| Candidates scored on outcomes, not on the NMPC's cost | With tapered speeds, waiting behind a blocker costs almost nothing, so the overtake was never chosen. |
| Blockers kept until passed, and overtakes committed | Otherwise the borrow candidate vanished mid-overtake and the car cut back into the parked car. |
| The corridor merges back from where the car is | Otherwise ending an overtake meant an instant jump back into the lane, at 7 m/s² lateral. |
| Wait 4 m further back behind blockers | From 1 m behind, a kinematic car can only pull out at full lock. |
| JIT with an on-disk cache (`jit`, `cache_dir`) | Half the time per iteration (about 27 → 10 ms a solve), but compiling takes over a minute. The compiled solver is saved with its binary and loaded in later processes in 0.3 s. |

## Running

```bash
cmake --build build -j && build/urban_scenarios [--only NAME] [--out results/urban]
URBAN_VERBOSE=1 build/urban_scenarios --only parked_car_wait    # every candidate, every step
```

The first run compiles the solver into `build/urban_cache/`, which takes about a minute.
