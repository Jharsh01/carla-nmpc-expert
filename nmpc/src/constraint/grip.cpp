// Grip: the planned longitudinal and lateral accelerations within the friction circle (mu g), and the speed within
// the limit. Rows only (stage.hpp); the strongest acceleration is a bound set at each solve (framework.cpp).
#include "nmpc/stage.hpp"

namespace ex {

using casadi::sq;

std::vector<SX> grip_rows(const Stage& g) {
    const SX a_lat = lateral_acceleration(g.x, g.nmpc->p.L());
    return {(sq(g.x(IA)) + sq(a_lat)) / sq(g.mu_g) - 1.0, g.x(IV) - g.v_max};
}

}  // namespace ex
