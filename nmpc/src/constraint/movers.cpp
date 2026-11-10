#include "nmpc/constraint/movers.hpp"

#include <algorithm>
#include <cmath>

#include "nmpc/stage.hpp"

namespace ex {

using casadi::sq;

std::vector<MoverSlot> nearest_movers(const std::vector<Mover>& movers, const std::vector<double>& times,
                                      const std::vector<double>& s_plan, const std::vector<double>& e_plan,
                                      const Body& body, const NmpcOptions& opt) {
    const int N = static_cast<int>(times.size()) - 1;
    const double r_ego = std::hypot(body.length / 6.0, 0.5 * body.width);
    std::vector<MoverSlot> slots;
    for (const Mover& mv : movers) {
        MoverSlot slot{INF, std::vector<double>(N), std::vector<double>(N), std::vector<double>(N)};
        for (int k = 0; k < N; ++k) {
            const double t = times[k + 1];
            slot.s[k] = sample(mv.t, mv.s, t);
            slot.e[k] = sample(mv.t, mv.e_y, t);
            slot.R[k] = r_ego + mv.radius + mv.growth * t + (std::isnan(mv.margin) ? opt.margin : mv.margin);
            slot.distance = std::min(slot.distance, std::hypot(slot.s[k] - s_plan[k + 1], slot.e[k] - e_plan[k + 1]) - slot.R[k]);
        }
        slots.push_back(std::move(slot));
    }
    std::stable_sort(slots.begin(), slots.end(), [](const MoverSlot& a, const MoverSlot& b) { return a.distance < b.distance; });
    if (static_cast<int>(slots.size()) > opt.n_movers) slots.resize(opt.n_movers);
    return slots;
}

std::vector<SX> mover_rows(const Stage& g) {  // each of the body's three circles outside each mover's clearance circle
    const Body& b = g.nmpc->body;
    const SX c = cos(g.x(IPSI)), sn = sin(g.x(IPSI));
    std::array<SX, 3> sc, ec;  // the circles' centres, once for all movers
    for (int i = 0; i < 3; ++i) {
        const double d = -b.cg_to_center + (i - 1) * b.length / 3.0;
        sc[i] = g.x(IS) + d * c, ec[i] = g.x(IE) + d * sn;
    }
    std::vector<SX> out;
    for (const auto& [s, e, R, on] : g.movers)
        for (int i = 0; i < 3; ++i) out.push_back(on * (1.0 - (sq(sc[i] - s) + sq(ec[i] - e)) / sq(R)) + (on - 1.0));
    return out;
}

}  // namespace ex
