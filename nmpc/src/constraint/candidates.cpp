#include "nmpc/constraint/candidates.hpp"

#include <algorithm>
#include <cmath>

#include "nmpc/constraint/crossing.hpp"
#include "nmpc/constraint/follow.hpp"
#include "nmpc/constraint/lane.hpp"
#include "nmpc/constraint/overtake.hpp"

namespace ex {

Constraints::Result Constraints::build(const Percept& p, const Scene& sc, const std::string& last_label) {
    Result out;
    const std::vector<Gate> stop_gates = stops.gates(p, sc, nmpc_opt.a_ref, opt, out.notes);
    const Corridor lane = lane_corridor(p, sc, body, opt, nmpc_opt);
    auto base_problem = [&] {
        Problem pr;
        pr.t = sc.t;
        pr.ego = sc.ego;
        pr.v_max = sc.v_max;
        pr.v_ref_t = sc.v_ref_t;
        pr.v_ref = sc.v_ref;
        pr.corridor = lane;
        pr.movers = p.movers;
        pr.leads = p.leads;
        pr.gates = stop_gates;
        return pr;
    };

    std::vector<Candidate>& cands = out.candidates;
    {  // yield to every crosser, stay behind every blocker
        Candidate c;
        c.label = "base";
        c.problem = base_problem();
        for (const Crosser& x : p.crossers) c.problem.gates.push_back(yield_gate(x, opt));
        stay_behind(c.problem, p);
        cands.push_back(std::move(c));
    }
    {  // pass ahead of one crosser, where the car can get past in time
        std::vector<const Crosser*> by_time;
        for (const Crosser& x : p.crossers) by_time.push_back(&x);
        std::sort(by_time.begin(), by_time.end(), [](auto* a, auto* b) { return a->t_in < b->t_in; });
        for (const Crosser* x : by_time) {
            if (static_cast<int>(cands.size()) >= opt.pool - 1) break;
            if (!can_pass(*x, p, sc.v_max, params, nmpc_opt, opt)) continue;
            Candidate c;
            c.label = "pass:" + std::to_string(x->id);
            c.problem = base_problem();
            for (const Crosser& y : p.crossers) c.problem.gates.push_back(&y == x ? pass_gate(*x, opt) : yield_gate(y, opt));
            stay_behind(c.problem, p);
            cands.push_back(std::move(c));
        }
    }
    // round the blockers: into the lane on either side, where the map has one, judged by which way it runs
    if (!p.blockers.empty()) {
        const double front0 = p.front0;
        double b_lo = INF, b_hi = -INF;
        for (const Blocker& b : p.blockers)
            if (b.s_lo - front0 < opt.blocker_range) b_lo = std::min(b_lo, b.s_lo), b_hi = std::max(b_hi, b.s_hi);
        for (const Blocker& b : p.blockers)  // and those just beyond them, as in a row of cones
            if (b.s_lo < b_hi + 10.0) b_hi = std::max(b_hi, b.s_hi);
        if (std::isfinite(b_lo))
            for (bool left : {true, false}) overtake(*this, p, sc, left, b_lo, b_hi, last_label, lane, stop_gates, out);
        // SimLingo asks to go round: decided now, not on the way up to the blocker, where the blocker hides more of
        // the lane beyond and there is less room to pull out (route 1: it asked 21 m off, at 7.7 m/s, and the car
        // came to rest 16 m off, beside it). Clear: gone round at once (the planner drives `go`). Not clear: the car
        // stops where it is, as far back as it can, to see down that lane, and goes once it is clear.
        if (!std::isfinite(b_lo)) view_stop = 0;
        else {
            const int side = sc.asks_change != 0 ? sc.asks_change : view_stop;
            if (side != 0) {
                const std::string label = side > 0 ? "borrow_left" : "borrow_right";
                const bool clear = std::any_of(cands.begin(), cands.end(), [&](const Candidate& c) { return c.label == label; });
                if (clear) {
                    out.go = label;
                    view_stop = 0;
                } else {
                    if (view_stop == 0) out.notes.push_back("blocker: SimLingo asks to go round, not clear: stopping to see");
                    view_stop = side;
                }
            }
        }
    } else {
        view_stop = 0;
    }
    if (view_stop != 0) {  // stopping where the car is, at view_stop_decel: from rest, it stays put
        Lead hold;
        hold.t = {0.0};
        hold.s_rear = {p.front0 + nmpc_opt.gap_min + p.v0 * opt.view_stop_delay + p.v0 * p.v0 / (2.0 * opt.view_stop_decel)};
        cands.front().problem.leads.push_back(hold);
    }
    return out;
}

}  // namespace ex
