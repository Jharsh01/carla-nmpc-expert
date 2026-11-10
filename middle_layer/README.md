# middle_layer: perception

Between the simulator and the NMPC (`nmpc/`): what the car senses, and that taken into the route's path coordinates
and interpreted. What to do about it is the NMPC's (`nmpc/src/constraint/`).

```
middle_layer/
  CMakeLists.txt            library perception (the C++ post-processing)
  perception/
    sensors/                Python: CARLA read as sensors on the car would report it (privileged stand-ins)
      actors.py             vehicles, bikes, pedestrians, objects in range (150 m; walkers 40 m) and line of sight,
                            tracked, predicted along their lane or at constant velocity
      map.py                the route's lanes from the map (the ego lane, the lanes either side and which way they
                            run, the road's edges), the path continued along the lane, stop lines, the route's end
      sight.py              how far along the lanes either side the car sees: a forward depth camera, the lane's
                            points from the map projected into it
    include/perception/     the C++ headers
      perception.hpp        the Scene (Actor, LaneSection, StopLine) and the Percept; Perception::perceive
      tracks.hpp, classify.hpp, lanes.hpp, left_dir.hpp, right_dir.hpp
    post_process/           C++: the Scene into a Percept, once per plan
      perception.cpp        Perception::perceive: tracks, then classify
      tracks.cpp            each actor's trajectory projected onto the route; its body as circles
      classify.cpp          lead, blocker, crosser, mover; oncoming, along; parked cars with room beside them
      lanes.cpp             the section at a point, in the lane or not, the band's part running the car's way
      left_dir.cpp          the lane on the left: there or not, how wide, which way it runs
      right_dir.cpp         the lane on the right, the same
```

The agent (`expert/agents/simlingo_nmpc_agent.py`) reads the sensors each plan into an `expertctl.Scene`; the
planner (`nmpc/planner.hpp`) post-processes it with `Perception::perceive` before building its candidates.
