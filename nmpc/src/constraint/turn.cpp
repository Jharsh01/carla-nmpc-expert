#include "nmpc/constraint/turn.hpp"

#include <algorithm>
#include <cmath>

namespace ex {

double bend_speed(double v, double kappa, const NmpcOptions& opt) {
    const double k = std::fabs(kappa);
    return k > 1e-4 ? std::min(v, std::sqrt(opt.a_lat_ref / k)) : v;
}

}  // namespace ex
