#include "nmpc/constraint/crossing.hpp"

#include <algorithm>

#include "nmpc/constraint/overtake.hpp"
#include "nmpc/stage.hpp"

namespace ex {

Gate yield_gate(const Crosser& c, const ConstraintOptions& opt) {
    return Gate{c.s_lo - opt.gate_margin, std::max(0.0, c.t_in - opt.gate_buffer), c.t_out + opt.gate_buffer, true};
}

Gate pass_gate(const Crosser& c, const ConstraintOptions& opt) {
    return Gate{c.s_hi + opt.gate_margin, std::max(0.0, c.t_in - opt.gate_buffer), c.t_out + opt.gate_buffer, false};
}

bool can_pass(const Crosser& c, const Percept& p, double v_max, const VehicleParams& params, const NmpcOptions& nmpc,
              const ConstraintOptions& opt) {
    const double t_by = c.t_in - opt.gate_buffer;
    const double need = c.s_hi + opt.gate_margin - p.rear0;
    const double reach = drive(p.v0, std::max(p.v0, v_max), nmpc.a_max, params).at(t_by);
    return t_by > 0.0 && reach >= need;
}

std::vector<double> pass_profile(const std::vector<Gate>& gates, const std::vector<double>& times) {
    const int N = static_cast<int>(times.size()) - 1;
    std::vector<double> floor(N, -INF);
    for (int k = 0; k < N; ++k) {
        const double t = times[k + 1];
        for (const Gate& gt : gates)
            if (!gt.before && gt.t_from <= t && t <= gt.t_to) floor[k] = std::max(floor[k], gt.s);
    }
    return floor;
}

std::vector<SX> pass_rows(const Stage& g) {  // the rear past the passing gates
    const UrbanNMPC& n = *g.nmpc;
    const SX rear = g.x(IS) - 0.5 * n.body.length - n.body.cg_to_center;
    return {g.floor_on * (g.floor_s - rear) + (g.floor_on - 1.0)};
}

}  // namespace ex
