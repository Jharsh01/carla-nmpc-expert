#include "urban/constraints.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ex {

namespace {

std::string fmt_sight(const char* side, double sight) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "blocker: %s lane seen only %.0f m ahead, waiting", side, sight);
    return buf;
}

}  // namespace

Constraints::Drive Constraints::drive(double v0, double v_top) const {
    Drive out;
    double v = v0, d = 0.0;
    out.d.push_back(0.0);
    for (double t = 0.0; t < 30.0; t += out.dt) {  // to the top speed, or far enough
        const double a = v < v_top ? std::min(nmpc_opt.a_max, params.a_drive(v)) : 0.0;
        const double v_next = std::min(v_top, v + a * out.dt);
        d += 0.5 * (v + std::max(v, v_next)) * out.dt;
        v = std::max(v, v_next);
        out.d.push_back(d);
        if (v >= v_top && t > 1.0) break;
    }
    out.v_end = v;
    return out;
}

double Constraints::Drive::at(double t) const {
    if (t <= 0.0) return 0.0;
    const double t_end = dt * (d.size() - 1);
    if (t >= t_end) return d.back() + v_end * (t - t_end);
    const size_t i = static_cast<size_t>(t / dt);
    const double w = t / dt - i;
    return (1.0 - w) * d[i] + w * d[std::min(i + 1, d.size() - 1)];
}

double Constraints::Drive::time_to(double distance) const {
    if (distance <= 0.0) return 0.0;
    if (distance > d.back()) return dt * (d.size() - 1) + (v_end > 0.0 ? (distance - d.back()) / v_end : INF);
    const size_t i = std::lower_bound(d.begin(), d.end(), distance) - d.begin();
    if (i == 0) return 0.0;
    return dt * (i - 1) + dt * (distance - d[i - 1]) / std::max(1e-9, d[i] - d[i - 1]);
}

Constraints::Result Constraints::build(const Percept& p, const Scene& sc, const std::string& last_label) const {
    Result out;
    const auto& z0 = p.z0;
    const double s0 = p.s0, v0 = p.v0, front0 = p.front0, rear0 = p.rear0;
    auto section = [&](double s) -> const LaneSection& { return p.section(s); };

    std::vector<Gate> stop_gates;
    for (double s : p.stops) stop_gates.push_back({s});

    // the lane, widened where the car is if it is out of it (after an overtake, a swerve), narrowing back to the
    // lane over the distance a calm lane change takes, so that coming back is a manoeuvre and not a jump
    Corridor lane_corridor;
    const double half = 0.5 * body.width + 0.3, merge = std::max(25.0, 3.0 * v0);
    for (double s = s0 - 10.0; s <= s0 + opt.look_ahead + 60.0; s += 2.0) {
        const LaneSection& l = section(s);
        const double w = std::clamp((s - s0) / merge, 0.0, 1.0);
        lane_corridor.s.push_back(s);
        // but never past the road's edges: a car off the road is asked back onto it at once
        lane_corridor.lo.push_back(std::max(l.road_lo, std::min(l.lo, (1.0 - w) * (z0[IE] - half) + w * l.lo)));
        lane_corridor.hi.push_back(std::min(l.road_hi, std::max(l.hi, (1.0 - w) * (z0[IE] + half) + w * l.hi)));
        double ref = 0.5 * (l.lo + l.hi);
        if (!sc.line_s.empty()) {  // the line asked for, kept where the body fits in the lane
            const double margin = 0.5 * body.width + 0.1;
            const size_t i = std::lower_bound(sc.line_s.begin(), sc.line_s.end(), s) - sc.line_s.begin();
            const double asked = i == 0 ? sc.line_e.front() : i >= sc.line_s.size() ? sc.line_e.back()
                                 : sc.line_e[i - 1] + (sc.line_e[i] - sc.line_e[i - 1]) * (s - sc.line_s[i - 1]) /
                                                          (sc.line_s[i] - sc.line_s[i - 1]);
            ref = l.hi - l.lo > 2.0 * margin ? std::clamp(asked, l.lo + margin, l.hi - margin) : ref;
        }
        lane_corridor.ref.push_back(ref);
    }

    auto base_problem = [&] {
        Problem pr;
        pr.t = sc.t;
        pr.ego = sc.ego;
        pr.v_max = sc.v_max;
        pr.v_ref_t = sc.v_ref_t;
        pr.v_ref = sc.v_ref;
        pr.corridor = lane_corridor;
        pr.movers = p.movers;
        pr.leads = p.leads;
        pr.gates = stop_gates;
        return pr;
    };
    auto yield_gate = [&](const Crosser& c) {
        return Gate{c.s_lo - opt.gate_margin, std::max(0.0, c.t_in - opt.gate_buffer), c.t_out + opt.gate_buffer, true};
    };
    auto stay_behind = [&](Problem& pr) {
        for (const Blocker& b : p.blockers) {
            Lead lead;
            lead.t = {0.0};
            lead.s_rear = {b.s_lo - 4.0};  // waiting far enough back to pull out round it
            pr.leads.push_back(lead);
        }
    };

    std::vector<Candidate>& cands = out.candidates;
    {  // yield to every crosser, stay behind every blocker
        Candidate c;
        c.label = "base";
        c.problem = base_problem();
        for (const Crosser& x : p.crossers) c.problem.gates.push_back(yield_gate(x));
        stay_behind(c.problem);
        cands.push_back(std::move(c));
    }
    {  // pass ahead of one crosser, where the car can get past in time
        std::vector<const Crosser*> by_time;
        for (const Crosser& x : p.crossers) by_time.push_back(&x);
        std::sort(by_time.begin(), by_time.end(), [](auto* a, auto* b) { return a->t_in < b->t_in; });
        for (const Crosser* x : by_time) {
            if (static_cast<int>(cands.size()) >= opt.pool - 1) break;
            const double t_by = x->t_in - opt.gate_buffer;
            const double need = x->s_hi + opt.gate_margin - rear0;
            const double reach = drive(v0, std::max(v0, sc.v_max)).at(t_by);
            if (t_by <= 0.0 || reach < need) continue;
            Candidate c;
            c.label = "pass:" + std::to_string(x->id);
            c.problem = base_problem();
            for (const Crosser& y : p.crossers)
                c.problem.gates.push_back(&y == x ? Gate{x->s_hi + opt.gate_margin, std::max(0.0, x->t_in - opt.gate_buffer),
                                                         x->t_out + opt.gate_buffer, false}
                                                  : yield_gate(y));
            stay_behind(c.problem);
            cands.push_back(std::move(c));
        }
    }
    // ---- round the blockers: into the lane on either side, where the map has one, judged by which way it runs

    if (!p.blockers.empty()) {
        double b_lo = INF, b_hi = -INF;
        for (const Blocker& b : p.blockers)
            if (b.s_lo - front0 < opt.blocker_range) b_lo = std::min(b_lo, b.s_lo), b_hi = std::max(b_hi, b.s_hi);
        for (const Blocker& b : p.blockers)  // and those just beyond them, as in a row of cones
            if (b.s_lo < b_hi + 10.0) b_hi = std::max(b_hi, b.s_hi);
        if (std::isfinite(b_lo))
            for (bool left : {true, false}) borrow(p, sc, left, b_lo, b_hi, last_label, out);
    }
    return out;
}

// The candidate going round the blockers [b_lo, b_hi] through the lane on one side, if there is one and it is clear.
void Constraints::borrow(const Percept& p, const Scene& sc, bool left, double b_lo, double b_hi,
                         const std::string& last_label, Result& out) const {
    const auto& z0 = p.z0;
    const double s0 = p.s0, v0 = p.v0, front0 = p.front0, rear0 = p.rear0;
    const double sign = left ? 1.0 : -1.0;
    const std::string label = left ? "borrow_left" : "borrow_right";
    const char* side = left ? "left" : "right";
    auto section = [&](double s) -> const LaneSection& { return p.section(s); };
    auto width = [&](const LaneSection& l) { return l.side_width(left); };
    // the lane beside the ego lane on that side, as [lo, hi] across the route
    auto target = [&](const LaneSection& l) {
        return left ? std::make_pair(l.hi, l.hi + width(l)) : std::make_pair(l.lo - width(l), l.lo);
    };

    // the lane to go into: beside the blockers, or nearer if SimLingo's path is already out in it there (then the
    // lane under the path is the other one: route 4 never offered the overtake)
    const LaneSection* lp = &section(b_lo);
    for (double s = b_lo; width(*lp) <= 0.0 && s > front0 - 2.0; s -= 2.0)
        if (width(section(s)) > 0.0) lp = &section(s);
    if (width(*lp) <= 0.0) return;  // no lane there on that side
    const LaneSection& l = *lp;
    const bool oncoming_lane = l.side_oncoming(left);

    // The pass as it will be driven: from the speed now, as hard as the car can (drive()) up to the limit, until the
    // front is `done`.
    // Bench2Drive's opposite flows (routes 4 and 6) send a car every 70-149 m at the limit: a pass begun at 5 m/s,
    // judged over the 4 s the tracks reach, met the next car half way and both stopped nose to nose.
    const Drive pass = drive(v0, std::max(v0, sc.v_max));
    const double done = b_hi + 10.0, T = pass.time_to(std::max(0.0, done - front0));
    auto ours = [&](double t) { return std::min(front0 + pass.at(t), done); };  // where the front is, t from now
    const double length = front0 - rear0;
    auto in_band = [](double e, double ext_e, std::pair<double, double> band) {
        return e - ext_e < band.second && e + ext_e > band.first;
    };
    // Traffic coming the other way, in that lane or down the car's own (where the path, SimLingo's, has it): kept
    // `oncoming_gap` ahead of the front until the pass is done.
    auto head_on = [&](double t, double s, double e, double ext_s, double ext_e) {
        const LaneSection& li = section(s);
        const auto band = target(li);
        return in_band(e, ext_e, {std::min(band.first, li.lo), std::max(band.second, li.hi)}) && s + ext_s > rear0 &&
               s - ext_s < ours(t) + opt.oncoming_gap;
    };
    // Traffic going the car's way in that lane, beside or behind the car now: it must not be beside the car, or
    // within `lane_gap` of its rear, while the car moves over (`merge_time`). Traffic ahead is followed (it is kept
    // clear of in the candidate), not waited for.
    auto cut_off = [&](double t, double s, double e, double ext_s, double ext_e) {
        const double front = ours(t), rear = front - length;
        return in_band(e, ext_e, target(section(s))) && s + ext_s > rear - opt.lane_gap && s - ext_s < front;
    };
    auto any_conflict = [&](const std::vector<size_t>& tracks, double horizon, auto conflict, bool from_behind) {
        for (size_t k : tracks) {
            const Track& tr = p.tracks[k];
            const size_t n = tr.t.size();
            if (from_behind && tr.s[0] - tr.ext_s[0] >= front0) continue;  // ahead of the car now
            for (size_t i = 0; i < n && tr.t[i] <= horizon; ++i)
                if (conflict(tr.t[i], tr.s[i], tr.e[i], tr.ext_s[i], tr.ext_e[i])) return true;
            if (n >= 2 && tr.t[n - 1] < horizon) {  // beyond the prediction, on at the speed it ends with
                const double ds = (tr.s[n - 1] - tr.s[n - 2]) / (tr.t[n - 1] - tr.t[n - 2]);
                if (conflict(horizon, tr.s[n - 1] + ds * (horizon - tr.t[n - 1]), tr.e[n - 1], tr.ext_s[n - 1], tr.ext_e[n - 1]))
                    return true;
            }
        }
        return false;
    };
    // What the car cannot see may hold a car: one coming down an oncoming lane at the limit from the edge of sight, its
    // front there now, checked as a seen one is. (A lane the car's way needs no such car: one there comes from behind,
    // which the car has seen, or is ahead, and followed.)
    const double sight = left ? sc.sight_left : sc.sight_right;
    bool unseen_traffic = false;
    if (oncoming_lane && std::isfinite(sight)) {
        const double half = 2.4;  // half a car's length [m]
        for (double t = 0.0; t <= T + 1e-9 && !unseen_traffic; t += 0.1) {
            const double s = s0 + sight + half - sc.v_max * t;
            unseen_traffic = s + half > rear0 && s - half < ours(t) + opt.oncoming_gap;
        }
    }
    const bool head_on_traffic = any_conflict(p.oncoming, T, head_on, false);
    const bool cut_off_traffic = any_conflict(p.along, std::min(T, opt.merge_time), cut_off, true);
    // once out in that lane and past where it would wait behind the blocker, the car finishes going round; short of
    // that it goes back behind it (route 6 kept on, as committed, into a car 29 m off)
    const double centre = 0.5 * (l.lo + l.hi);
    const bool committed = last_label == label && sign * (z0[IE] - centre) > 0.5 && front0 > b_lo - 4.0;
    if (committed) out.committed = true;
    if (!committed && (head_on_traffic || cut_off_traffic || unseen_traffic)) {
        out.notes.push_back(head_on_traffic   ? std::string("blocker: oncoming traffic, waiting")
                            : cut_off_traffic ? std::string("blocker: traffic in the ") + side + " lane (" +
                                                    (oncoming_lane ? "oncoming" : "our way") + "), waiting"
                                              : fmt_sight(side, sight));
        return;
    }

    Candidate c;
    c.label = label;
    c.problem = Problem();
    c.problem.t = sc.t;
    c.problem.ego = sc.ego;
    c.problem.v_max = sc.v_max;
    // at the limit, as the pass was judged: not SimLingo's speeds, which slow to a stop while it waits on the same
    // traffic (a crawl into the other lane on route 4); the actors and the rules still bring it down
    c.problem.corridor = out.candidates.front().problem.corridor;
    c.problem.leads = p.leads;
    for (double s : p.stops) c.problem.gates.push_back({s});
    Corridor& cr = c.problem.corridor;
    for (size_t i = 0; i < cr.s.size(); ++i) {
        const LaneSection& li = section(cr.s[i]);
        const auto band = target(li);
        if (cr.s[i] > s0 - 10.0 && cr.s[i] < b_hi + 15.0 && width(li) > 0.0) {
            if (left) cr.hi[i] = std::min(li.road_hi, band.second);
            else cr.lo[i] = std::max(li.road_lo, band.first);
        }
        if (cr.s[i] > b_lo - 12.0 && cr.s[i] < b_hi + 6.0 && width(li) > 0.0) cr.ref[i] = 0.5 * (band.first + band.second);
    }
    // The blockers and the still things beside the way round are the corridor's edges, boxes and not circles: the
    // circles cover a car's side 0.3 m wide of it, and the ego's too, so between route 4's cones and the cars parked
    // opposite (2.9 m apart, room for the car with 0.5 m a side) no plan fitted and the overtake stalled half way.
    // The blockers bound it on the near side; still things beyond the blockers' far edge bound it on the far side.
    double blockers_edge = -INF;  // the blockers' furthest extent towards that side, as sign * e
    for (const Blocker& b : p.blockers) {
        const Track& tr = p.tracks[b.track];
        blockers_edge = std::max(blockers_edge, sign * tr.e[0] + tr.ext_e[0]);
    }
    const double reach = 0.5 * body.length + body.cg_to_center + 2.0;  // the body's ends, and the sampling
    auto edge = [&](const Track& tr, bool far_side) {
        const bool upper = far_side == left;  // bounds hi (the left) or lo (the right)
        for (size_t i = 0; i < cr.s.size(); ++i)
            if (cr.s[i] > tr.s[0] - tr.ext_s[0] - reach && cr.s[i] < tr.s[0] + tr.ext_s[0] + reach) {
                if (upper) cr.hi[i] = std::min(cr.hi[i], tr.e[0] - tr.ext_e[0] - opt.static_margin);
                else cr.lo[i] = std::max(cr.lo[i], tr.e[0] + tr.ext_e[0] + opt.static_margin);
            }
    };
    for (const Blocker& b : p.blockers) edge(p.tracks[b.track], false);
    for (size_t k : p.still)
        if (std::fabs(p.tracks[k].e[0]) < 10.0) edge(p.tracks[k], sign * p.tracks[k].e[0] > blockers_edge);
    const double fit = 0.5 * body.width + nmpc_opt.edge_margin;
    for (size_t i = 0; i < cr.s.size(); ++i)  // the line through the middle where it is narrow
        cr.ref[i] = cr.hi[i] - cr.lo[i] > 2.0 * fit ? std::clamp(cr.ref[i], cr.lo[i] + fit, cr.hi[i] - fit)
                                                   : 0.5 * (cr.lo[i] + cr.hi[i]);
    c.problem.movers = p.moving;
    for (const Crosser& x : p.crossers)
        c.problem.gates.push_back(
            Gate{x.s_lo - opt.gate_margin, std::max(0.0, x.t_in - opt.gate_buffer), x.t_out + opt.gate_buffer, true});
    // the lanes it goes into count here
    c.problem.movers.insert(c.problem.movers.end(), p.stay_out.begin(), p.stay_out.end());
    if (static_cast<int>(out.candidates.size()) >= opt.pool) out.candidates.pop_back();
    out.candidates.push_back(std::move(c));
}

}  // namespace ex
