# nmpc: the NMPC and its planner

```
nmpc/
  CMakeLists.txt          libraries nmpc_shared (paths, vehicle parameters; the middle layer uses them too) and nmpc
  include/nmpc/
    shared_param.hpp      every tunable: the car (Body, VehicleParams), PerceptionOptions, NmpcOptions,
                          ConstraintOptions, PlannerOptions
    problem.hpp           what one solve is asked (Problem: corridor, movers, leads, gates) and gives back (Plan)
    framework.hpp         UrbanNMPC: the optimisation, its warm start and set_route
    stage.hpp             one step of the horizon as the cost and the constraint rows see it (CasADi; internal)
    cost.hpp              the cost terms
    planner.hpp           Planner (perceive, build the candidates, solve them in parallel, choose), PathIntent
    path.hpp, integrate.hpp
    constraint/           one header per manoeuvre, as below
  src/
    framework.cpp         builds the problem from cost.cpp and the constraint rows, solves it
    cost.cpp, planner.cpp, path.cpp, vehicle_params.cpp
    constraint/           each manoeuvre's rule and its rows of the optimisation, side by side:
      candidates.cpp      the candidates: base, pass:<id>, borrow_left / borrow_right
      lane.cpp            the corridor along the lane and the line asked for        rows: corridor
      movers.cpp          what to keep clear of, the nearest planned with           rows: movers
      follow.cpp          leads, waiting behind blockers, the speed behind a lead   rows: ahead (lead), headway
      stop.cpp            stop lines and the traffic rules, the speed to a stop     rows: ahead (stop gates)
      crossing.cpp        yield to a crosser or pass ahead of it                    rows: behind
      overtake.cpp        round blockers through either lane, when clear and seen
      turn.cpp            the speed in bends
      grip.cpp            friction circle and speed limit                           rows: grip
  tests/test_path.cpp
```

- The optimisation: a kinematic bicycle in the route's path coordinates; every constraint group soft, its slack
  heavily priced. CasADi + FATROP, compiled once into `build/urban_cache`.
- The middle layer (`middle_layer/`) gives the planner a `Percept`: the actors classified against the lane, the lanes
  either side. What to do about it, stop lines and traffic rules included, is decided here, in `constraint/`.
- `path.hpp`: the route as straights and arcs, fitted through waypoints (biarcs), with projection (windowed for
  long routes), pose and curvature at s.
- The vehicle (`shared_param.hpp`, read from `config/vehicles/mkz2020.json`; `docs/vehicle_parameters.md`).
- `sim/urban/scenarios.cpp` (`build/urban_scenarios`, a ctest): 11 closed-loop scenarios without CARLA, on a
  kinematic plant with lags on acceleration and steering, each run twice: along the scenario's road, and as
  `<name>@local` along a SimLingo-like local path rebuilt every plan (`Planner::set_route`).
- The route can change every plan (`set_route`): SimLingo's path is the route in CARLA. The previous plan is
  carried into the new route's coordinates through the world frame, so the warm start survives.

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
| A parked car or object blocks the lane only if it reaches 0.3 m into it (`blocker_overlap`) | On a SimLingo-like path, which wobbles, a van parked beside the lane tilted 0.03 rad against the path and its box grazed the lane by 2 cm: the car stopped behind it for good (`pedestrian_behind_van@local`). The same mechanism is the suspect for Bench2Drive route 0's blockers. |
| Wait 4 m further back behind blockers | From 1 m behind, a kinematic car can only pull out at full lock. |
| JIT with an on-disk cache (`jit`, `cache_dir`) | Half the time per iteration (about 27 → 10 ms a solve), but compiling takes over a minute. The compiled solver is saved with its binary and loaded in later processes in 0.3 s. |

## Running

```bash
cmake --build build -j && build/urban_scenarios [--only NAME] [--out results/urban]
URBAN_VERBOSE=1 build/urban_scenarios --only parked_car_wait    # every candidate, every step
```

The first run compiles the solver into `build/urban_cache/`, which takes about a minute.
