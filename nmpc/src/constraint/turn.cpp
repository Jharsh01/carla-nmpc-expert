#include "nmpc/constraint/turn.hpp"

#include <algorithm>
#include <cmath>

namespace ex {

double bend_speed(double v, double kappa, double a_lat_ref) {
    const double k = std::fabs(kappa);
    return k > 1e-4 ? std::min(v, std::sqrt(a_lat_ref / k)) : v;
}

bool in_junction(const Corridor& c, double s) {
    if (c.junction.size() != c.s.size() || c.s.empty()) return false;
    const size_t i = std::lower_bound(c.s.begin(), c.s.end(), s) - c.s.begin();
    const size_t j = i == 0 ? 0 : i >= c.s.size() ? c.s.size() - 1 : (s - c.s[i - 1] < c.s[i] - s ? i - 1 : i);
    return c.junction[j] > 0.5;
}

}  // namespace ex
