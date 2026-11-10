// The lane: the corridor every candidate starts from, and its rows (the body's ends within it: stage.hpp).
#pragma once

#include "nmpc/problem.hpp"
#include "perception/perception.hpp"

namespace ex {

// The lane along the route, widened where the car is if it is out of it (after an overtake, a swerve), narrowing
// back over the distance a calm lane change takes, never past the road's edges; the line asked for (the Scene's,
// SimLingo's path) kept where the body fits in the lane.
Corridor lane_corridor(const Percept& p, const Scene& scene, const Body& body, const ConstraintOptions& opt,
                       const NmpcOptions& nmpc_opt);

}  // namespace ex
