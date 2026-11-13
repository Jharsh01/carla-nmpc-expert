// Bends: the speed asked for no more than takes the bend at a_lat_ref of lateral acceleration (grip allows more,
// comfort less). Turns themselves are the route's: SimLingo's path through the junction, the lane past it.
#pragma once

#include "nmpc/shared_param.hpp"

namespace ex {

// The speed asked for v in a bend of curvature kappa [1/m].
double bend_speed(double v, double kappa, double a_lat_ref);

}  // namespace ex
