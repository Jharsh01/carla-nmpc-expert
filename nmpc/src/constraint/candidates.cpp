#include "nmpc/constraint/candidates.hpp"

#include <algorithm>
#include <cmath>

#include "nmpc/constraint/crossing.hpp"
#include "nmpc/constraint/follow.hpp"
#include "nmpc/constraint/lane.hpp"
#include "nmpc/constraint/merge.hpp"
#include "nmpc/constraint/overtake.hpp"
#include "perception/lanes.hpp"

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
        // Back from an overtake above the limit (overtake_over_limit): the limit is a constraint, which a car
        // already over it could only meet by braking as hard as it can. The speed asked for is the limit, and the
        // constraint starts at the speed the car has.
        if (p.v0 > sc.v_max + 0.2) {
            pr.v_max = std::min(p.v0, sc.v_max + opt.overtake_over_limit);
            if (pr.v_ref.empty()) pr.v_ref_t = {0.0}, pr.v_ref = {sc.v_max};
            for (double& v : pr.v_ref) v = std::min(v, sc.v_max);
        }
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
        wait_for_mergers(c.problem, p, opt);
        cands.push_back(std::move(c));
    }
    cross(*this, p, sc, last_label, base_problem(), out);  // across their path, between two of them
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
            wait_for_mergers(c.problem, p, opt);
            cands.push_back(std::move(c));
        }
    }
    merge(*this, p, sc, last_label, base_problem(), out);  // into a stream, between two of its cars
    // round the blockers: into the lane on either side, where the map has one, judged by which way it runs
    double b_lo = INF, b_hi = -INF;
    if (!p.blockers.empty()) {
        const double front0 = p.front0;
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
    // Peeking: stopped to see, and that lane is not seen far enough for the way round to be judged clear. From where
    // the car stopped the blocker itself hides the lane beyond (route 4: 20 m behind a sign as wide as the lane the
    // oncoming lane was seen 70 m, the pass needed 113 m). The car backs up, then comes forward over to that side of
    // its lane, and waits there.
    // Going round from there (3), the car moves out from where it is (overtake.cpp), not back to the lane's middle
    // first, from where it would see less again.
    if (view_stop == 0) {
        if (!out.go.empty() && peek != 0) peek = 3;
        else if (!(peek == 3 && last_label.rfind("borrow_", 0) == 0 && std::isfinite(b_lo))) peek = 0;
    } else {
        if (peek == 3) peek = 0;  // no longer clear: stopped to see again
        const bool left = view_stop > 0;
        const double gap = b_lo - p.front0;  // to the blockers
        // out of its lane on that side, as a way round withdrawn half way out leaves it (route 6: 1.2 m over at
        // 0.33 rad, its nose 1.4 m into the oncoming lane, where the next car met it)
        const LaneSection& here = p.section(p.s0);
        const auto [w_lo, w_hi] = way_band(here);
        const double reach = (0.5 * body.length + std::fabs(body.cg_to_center)) * std::fabs(std::sin(p.z0[IPSI])) + 0.5 * body.width;
        const bool astray = left ? p.z0[IE] + reach > (w_lo < w_hi ? w_hi : here.hi) + opt.peek_over + 0.1
                                 : p.z0[IE] - reach < (w_lo < w_hi ? w_lo : here.lo) - opt.peek_over - 0.1;
        if (peek == 0 && (out.sight_short[left ? 1 : 0] || astray) && p.v0 < 0.3) {
            peek = 1;
            peek_back_gap = gap + opt.peek_back;
            out.notes.push_back(astray ? "peek: out of the lane, backing up into it"
                                       : "peek: the lane to go into is not seen far enough, backing up to see past the blocker");
        }
        if (peek == 1) {
            // room behind: to the nearest thing behind the car and across its width, now and as predicted
            const double half = 0.5 * body.width + 0.5;
            double room = INF;
            for (const Track& tr : p.tracks)
                for (size_t i = 0; i < tr.t.size(); ++i)
                    if (tr.s[0] < p.s0 && std::fabs(tr.e[i] - p.z0[IE]) < tr.ext_e[i] + half)
                        room = std::min(room, p.rear0 - (tr.s[i] + tr.ext_s[i]) - opt.peek_rear_gap);
            const double want = peek_back_gap - gap;
            if (want > 0.3 && room > 0.3) {
                out.reverse = std::min(want, room);
                // Backing up along the lane, not along the car's heading: stopped askew (0.26 rad, on route 4 after
                // a way round was withdrawn) it backed 2 m across into the oncoming lane. Going backwards, a heading
                // to the left takes the car to the right: the heading asked for leans to the side the car is off
                // to, and the wheels turn it there at about 1 rad/s per rad.
                const LaneSection& l = p.section(p.s0);
                const double psi_to = std::clamp(0.15 * (p.z0[IE] - 0.5 * (l.lo + l.hi)), -0.25, 0.25);
                out.reverse_delta = std::clamp(std::atan(params.L() * (p.z0[IPSI] - psi_to) / 1.5), -0.4, 0.4);
            }
            else if (p.v0 < 0.3) {
                peek = 2;
                // forward as far as it backed up (all of the run if it is out of its lane, to straighten)
                const double backed = std::max(0.0, gap - (peek_back_gap - opt.peek_back));
                const double run = astray ? opt.peek_run : std::min(opt.peek_run, backed);
                peek_gap = std::max(gap - run, std::min(gap, opt.peek_min_gap));
            }
        }
        out.peeking = peek != 0;
    }
    if (peek == 3) out.peeking = true;
    if (std::isfinite(b_lo) && last_label.rfind("borrow_", 0) == 0 &&
        std::any_of(cands.begin(), cands.end(), [&](const Candidate& c) { return c.label == last_label; }) &&
        p.front0 + p.v0 * p.v0 / (2.0 * std::fabs(nmpc_opt.a_min)) > b_lo - 4.0)
        cands.front().hopeless = true;  // waiting behind the blocker is no longer possible
    if (view_stop != 0) {
        Problem& pr = cands.front().problem;
        Lead hold;
        hold.t = {0.0};
        if (peek == 2) {  // forward to `peek_gap` short of the blocker, over to the side asked
            const bool left = view_stop > 0;
            double front_to = b_lo - peek_gap;  // and no nearer a car followed than the blocker
            for (const Lead& l : p.leads) front_to = std::min(front_to, l.s_rear.front() - opt.peek_min_gap);
            hold.s_rear = {std::max(p.front0, front_to) + nmpc_opt.gap};  // the speed tapers to `gap` behind a lead
            pr.v_ref_t.clear(), pr.v_ref.clear();
            pr.v_max = std::min(pr.v_max, opt.peek_speed);
            const double fit = 0.5 * body.width + nmpc_opt.edge_margin;
            Corridor& cr = pr.corridor;
            for (size_t i = 0; i < cr.s.size(); ++i) {
                if (cr.s[i] < p.s0 - 10.0 || cr.s[i] > b_lo) continue;
                const LaneSection& l = p.section(cr.s[i]);
                const auto [w_lo, w_hi] = way_band(l);  // the lanes running the car's way
                if (left) {
                    const double line = std::min(l.road_hi, (w_lo < w_hi ? w_hi : l.hi) + opt.peek_over);
                    cr.hi[i] = std::max(cr.hi[i], line);
                    cr.ref[i] = std::max(cr.ref[i], line - fit);
                } else {
                    const double line = std::max(l.road_lo, (w_lo < w_hi ? w_lo : l.lo) - opt.peek_over);
                    cr.lo[i] = std::min(cr.lo[i], line);
                    cr.ref[i] = std::min(cr.ref[i], line + fit);
                }
            }
            out.notes.push_back(std::string("peek: edged out to see down the ") + (left ? "left" : "right") + " lane");
        } else {  // stopping where the car is, at view_stop_decel: from rest, it stays put
            hold.s_rear = {p.front0 + nmpc_opt.gap_min + p.v0 * opt.view_stop_delay + p.v0 * p.v0 / (2.0 * opt.view_stop_decel)};
        }
        pr.leads.push_back(hold);
    }
    return out;
}

}  // namespace ex
