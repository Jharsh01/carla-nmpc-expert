#include "nmpc/constraint/lane.hpp"

#include <algorithm>
#include <cmath>

#include "nmpc/stage.hpp"

namespace ex {

Corridor lane_corridor(const Percept& p, const Scene& sc, const Body& body, const ConstraintOptions& opt,
                       const NmpcOptions& nmpc_opt) {
    const auto& z0 = p.z0;
    const double s0 = p.s0, v0 = p.v0;
    // the lane, widened where the car is if it is out of it (after an overtake, a swerve), narrowing back to the
    // lane over the distance a calm lane change takes, so that coming back is a manoeuvre and not a jump
    Corridor lane;
    // At rest, where the car is takes in its body's corners at its heading and the edge margin, so that the plan's
    // first step is inside: pedestrian_behind_van at -4.6 m/s^2 swerved to rest at e 0.57, heading 0.08, its front
    // corner 0.12 m past the band, and standing still was the cheapest plan for good. Only at rest: in motion each
    // plan widening round where the car has got to carried a swerve off the road.
    const double arm = 0.5 * body.length + std::fabs(body.cg_to_center);
    const double corners = v0 < 1.0 ? nmpc_opt.edge_margin + arm * std::fabs(std::sin(z0[IPSI])) : 0.0;
    const double half = 0.5 * body.width + 0.3 + corners;
    const double merge = std::max(25.0, 3.0 * v0);
    for (double s = s0 - 10.0; s <= s0 + opt.look_ahead + 60.0; s += 2.0) {
        const LaneSection& l = p.section(s);
        const double w = std::clamp((s - s0) / merge, 0.0, 1.0);
        lane.s.push_back(s);
        // but never past the road's edges: a car off the road is asked back onto it at once
        lane.lo.push_back(std::max(l.road_lo, std::min(l.lo, (1.0 - w) * (z0[IE] - half) + w * l.lo)));
        lane.hi.push_back(std::min(l.road_hi, std::max(l.hi, (1.0 - w) * (z0[IE] + half) + w * l.hi)));
        double ref = 0.5 * (l.lo + l.hi);
        if (!sc.line_s.empty()) {  // the line asked for, kept where the body fits in the lane
            const double margin = 0.5 * body.width + 0.1;
            const size_t i = std::lower_bound(sc.line_s.begin(), sc.line_s.end(), s) - sc.line_s.begin();
            const double asked = i == 0 ? sc.line_e.front() : i >= sc.line_s.size() ? sc.line_e.back()
                                 : sc.line_e[i - 1] + (sc.line_e[i] - sc.line_e[i - 1]) * (s - sc.line_s[i - 1]) /
                                                          (sc.line_s[i] - sc.line_s[i - 1]);
            ref = l.hi - l.lo > 2.0 * margin ? std::clamp(asked, l.lo + margin, l.hi - margin) : ref;
        }
        lane.ref.push_back(ref);
    }
    return lane;
}

std::vector<SX> corridor_rows(const Stage& g) {  // the body's front and rear ends within the band
    const Body& b = g.nmpc->body;
    std::vector<SX> out;
    for (double arm : {0.5 * b.length - b.cg_to_center, -(0.5 * b.length + b.cg_to_center)}) {
        const SX e_end = g.x(IE) + arm * sin(g.x(IPSI));
        out.push_back(e_end + 0.5 * b.width + g.nmpc->opt.edge_margin - g.hi);
        out.push_back(g.lo - (e_end - 0.5 * b.width - g.nmpc->opt.edge_margin));
    }
    return out;
}

}  // namespace ex
