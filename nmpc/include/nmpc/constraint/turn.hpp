// Bends: the speed asked for no more than takes the bend at a_lat_ref of lateral acceleration (grip allows more,
// comfort less); in a junction at mu g, grip alone (14 Nov: PDM-Lite turns across route 10's stream at up to
// 18 m/s^2; the friction circle stays a constraint). Turns themselves are the route's: SimLingo's path through the junction, the lane past it.
#pragma once

#include "nmpc/problem.hpp"
#include "nmpc/shared_param.hpp"
#include "perception/perception.hpp"

namespace ex {

// The speed asked for v in a bend of curvature kappa [1/m].
double bend_speed(double v, double kappa, double a_lat_ref);

// Whether s is in a junction, by the corridor's samples (false where it has none).
bool in_junction(const Corridor& c, double s);

}  // namespace ex
