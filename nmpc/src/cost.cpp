#include "nmpc/cost.hpp"

namespace ex {

using casadi::sq;

const std::vector<CostTerm>& cost_terms() {
    static const std::vector<CostTerm> terms = {
        {"speed", &NmpcOptions::w_speed, [](const Stage& g) { return sq(g.x(IV) - g.v_ref); }},
        {"lane", &NmpcOptions::w_lane, [](const Stage& g) { return sq(g.x(IE) - g.e_ref); }},
        {"heading", &NmpcOptions::w_heading, [](const Stage& g) { return sq(g.x(IPSI)); }},
        {"accel", &NmpcOptions::w_accel, [](const Stage& g) { return sq(g.x(IA)); }},
        {"jerk", &NmpcOptions::w_jerk, [](const Stage& g) { return sq(g.u(IJERK)); }},
        {"rate", &NmpcOptions::w_rate, [](const Stage& g) { return sq(g.u(IRATE)); }},
        {"lat", &NmpcOptions::w_lat, [](const Stage& g) { return sq(lateral_acceleration(g.x, g.nmpc->p.L())); }},
        {"slack", &NmpcOptions::w_slack,  // the hard groups: all but the headway
         [](const Stage& g) { const SX s = g.slack(casadi::Slice(0, NS - 1)); return sum1(s) + sumsqr(s); }},
        {"headway", &NmpcOptions::w_headway, [](const Stage& g) { return g.slack(NS - 1) + sq(g.slack(NS - 1)); }},
    };
    return terms;
}

}  // namespace ex
