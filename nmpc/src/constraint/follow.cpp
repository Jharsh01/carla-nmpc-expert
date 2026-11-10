#include "nmpc/constraint/follow.hpp"

#include <algorithm>
#include <cmath>

#include "nmpc/stage.hpp"

namespace ex {

using casadi::sq;

void lead_profile(const std::vector<Lead>& leads, const std::vector<double>& times, double front_now,
                  const NmpcOptions& opt, std::vector<double>& lead, std::vector<double>& lead_speed) {
    const int N = static_cast<int>(times.size()) - 1;
    lead.assign(N, INF);
    lead_speed.assign(N, 0.0);
    for (int k = 0; k < N; ++k) {
        const double t = times[k + 1];
        for (const Lead& l : leads) {  // the nearest at each step, and its speed
            const double s = sample(l.t, l.s_rear, t);
            if (s < lead[k])
                lead[k] = s, lead_speed[k] = std::max(0.0, (sample(l.t, l.s_rear, t + 0.05) - sample(l.t, l.s_rear, t - 0.05)) / 0.1);
        }
        // never asked to go back: a lead already nearer than the gap holds the car where it is (a plan that reverses
        // to restore it is not one the car can drive, and at standstill it leaves the solver crawling)
        if (std::isfinite(lead[k])) lead[k] = std::max(lead[k], front_now + opt.gap_min);
    }
}

double behind_lead_speed(double v, double lead_s, double lead_v, double front, const NmpcOptions& opt) {
    if (!std::isfinite(lead_s)) return v;
    // the time gap, and the envelope to the lead's speed
    const double room = std::max(0.0, lead_s - opt.gap - front);
    return std::min({v, room / opt.time_gap, std::sqrt(lead_v * lead_v + 2.0 * opt.a_ref * room)});
}

void stay_behind(Problem& pr, const Percept& p) {
    for (const Blocker& b : p.blockers) {
        Lead lead;
        lead.t = {0.0};
        lead.s_rear = {b.s_lo - 4.0};  // waiting far enough back to pull out round it
        pr.leads.push_back(lead);
    }
}

std::vector<SX> lead_rows(const Stage& g) {  // the front behind the lead: bodies apart
    const UrbanNMPC& n = *g.nmpc;
    const SX front = g.x(IS) + 0.5 * n.body.length - n.body.cg_to_center;
    return {g.lead_on * (front + n.opt.gap_min - g.lead_s) + (g.lead_on - 1.0)};
}

std::vector<SX> headway_rows(const Stage& g) {
    // at the last step, room to stop behind the lead were it to brake: spacing, which a cut-in may take away, not
    // contact. (The time gap is in the speed asked for, not here: a constraint on position, priced high, pays the
    // plan to weave, as steering at an angle shortens the progress along the route.)
    const UrbanNMPC& n = *g.nmpc;
    const SX front = g.x(IS) + 0.5 * n.body.length - n.body.cg_to_center;
    const SX stop = (sq(g.x(IV)) - sq(g.lead_v)) / (2.0 * n.opt.a_stop);
    return {g.terminal * g.lead_on * (front + n.opt.gap + stop - g.lead_s) + (g.terminal * g.lead_on - 1.0)};
}

}  // namespace ex
