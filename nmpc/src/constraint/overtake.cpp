#include "nmpc/constraint/overtake.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "nmpc/constraint/crossing.hpp"
#include "nmpc/constraint/merge.hpp"
#include "perception/lanes.hpp"

namespace ex {

namespace {

std::string fmt_sight(const char* side, double sight) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "blocker: %s lane seen only %.0f m ahead, waiting", side, sight);
    return buf;
}

}  // namespace

Drive drive(double v0, double v_top, double a_max, const VehicleParams& params) {
    Drive out;
    double v = v0, d = 0.0;
    out.d.push_back(0.0);
    for (double t = 0.0; t < 30.0; t += out.dt) {  // to the top speed, or far enough
        const double a = v < v_top ? std::min(a_max, params.a_drive(v)) : 0.0;
        const double v_next = std::min(v_top, v + a * out.dt);
        d += 0.5 * (v + std::max(v, v_next)) * out.dt;
        v = std::max(v, v_next);
        out.d.push_back(d);
        if (v >= v_top && t > 1.0) break;
    }
    out.v_end = v;
    return out;
}

double Drive::at(double t) const {
    if (t <= 0.0) return 0.0;
    const double t_end = dt * (d.size() - 1);
    if (t >= t_end) return d.back() + v_end * (t - t_end);
    const size_t i = static_cast<size_t>(t / dt);
    const double w = t / dt - i;
    return (1.0 - w) * d[i] + w * d[std::min(i + 1, d.size() - 1)];
}

double Drive::time_to(double distance) const {
    if (distance <= 0.0) return 0.0;
    if (distance > d.back()) return dt * (d.size() - 1) + (v_end > 0.0 ? (distance - d.back()) / v_end : INF);
    const size_t i = std::lower_bound(d.begin(), d.end(), distance) - d.begin();
    if (i == 0) return 0.0;
    return dt * (i - 1) + dt * (distance - d[i - 1]) / std::max(1e-9, d[i] - d[i - 1]);
}

// The candidate going round the blockers [b_lo, b_hi] through the lane on one side, if there is one and it is clear.
void overtake(const Constraints& cs, const Percept& p, const Scene& sc, bool left, double b_lo, double b_hi,
              const std::string& last_label, const Corridor& lane, const std::vector<Gate>& stop_gates,
              Constraints::Result& out) {
    const Body& body = cs.body;
    const ConstraintOptions& opt = cs.opt;
    const auto& z0 = p.z0;
    const double s0 = p.s0, v0 = p.v0, front0 = p.front0, rear0 = p.rear0;
    const double sign = left ? 1.0 : -1.0;
    const std::string label = left ? "borrow_left" : "borrow_right";
    const char* side = left ? "left" : "right";
    auto section = [&](double s) -> const LaneSection& { return p.section(s); };
    // the lane beside the ego lane on that side (left_dir, right_dir): its width (0: none), [lo, hi] across the route.
    // Where SimLingo's path changes into an oncoming lane the band takes that lane in (sensors/map.py), and the lane
    // to go into is the oncoming part of the band on that side: beyond the band there is no lane, so on route 4 no
    // way round was ever offered, nothing judged the oncoming traffic, and the car waited behind the cones for good.
    auto beside = [&](const LaneSection& l) {
        NeighbourLane n = neighbour_lane(l, left);
        if (n.exists()) return n;
        const auto [w_lo, w_hi] = way_band(l);
        if (w_lo >= w_hi) return n;  // none of the band runs the car's way: no lane of its own to go round from
        const double lo = left ? w_hi : l.lo, hi = left ? l.hi : w_lo;
        if (hi - lo >= opt.min_borrow_width) n.width = hi - lo, n.lo = lo, n.hi = hi, n.oncoming = true;
        return n;
    };
    auto width = [&](const LaneSection& l) { return beside(l).width; };
    auto target = [&](const LaneSection& l) {
        const NeighbourLane n = beside(l);
        return std::make_pair(n.lo, n.hi);
    };

    // the lane to go into: beside the blockers, or nearer if SimLingo's path is already out in it there (then the
    // lane under the path is the other one: route 4 never offered the overtake)
    const LaneSection* lp = &section(b_lo);
    for (double s = b_lo; width(*lp) <= 0.0 && s > front0 - 2.0; s -= 2.0)
        if (width(section(s)) > 0.0) lp = &section(s);
    if (width(*lp) <= 0.0) return;  // no lane there on that side
    const LaneSection& l = *lp;
    const bool oncoming_lane = beside(l).oncoming;

    // The pass as it will be driven: from the speed now, as hard as the car can (drive()) up to the limit, until the
    // front is `done`.
    // Bench2Drive's opposite flows (routes 4 and 6) send a car every 70-149 m at the limit: a pass begun at 5 m/s,
    // judged over the 4 s the tracks reach, met the next car half way and both stopped nose to nose.
    const double v_pass = sc.v_max + opt.overtake_over_limit;  // out of the other lane sooner
    const Drive pass = drive(v0, std::max(v0, v_pass), cs.nmpc_opt.a_max, cs.params);
    // Back in its lane the faster it goes the further past the blockers: the same time to move over, at the speed
    // of the pass (10 and 15 m at 30 km/h, which they were set at).
    const double done = b_hi + std::max(10.0, 1.0 * v_pass), back_by = b_hi + std::max(15.0, 1.6 * v_pass);
    const double T = pass.time_to(std::max(0.0, done - front0));
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
    // Going round from where the car edged out to see (peek 3), what it saw then stands: on the way out the sight
    // read swings (route 4: 106 m, then 92 and 80 as the car turned and closed on the sign), and a way round
    // withdrawn half way out leaves the car askew in its lane. Traffic it does see still holds it.
    if (cs.peek == 3 && last_label == label) unseen_traffic = false;
    if (unseen_traffic) out.sight_short[left ? 1 : 0] = true;
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
    c.problem.v_max = v_pass;
    // at the speed the pass was judged at: not SimLingo's speeds, which slow to a stop while it waits on the same
    // traffic (a crawl into the other lane on route 4); the actors and the rules still bring it down
    c.problem.corridor = lane;
    c.problem.leads = p.leads;
    c.problem.gates = stop_gates;
    Corridor& cr = c.problem.corridor;
    // out into that lane beside the blockers; from where the car is when it has edged out to see (peeking): back
    // in the middle of its lane on the way up to the blocker it would see less again, and the way round be withdrawn
    const double out_from = cs.peek != 0 ? s0 : b_lo - 12.0;
    for (size_t i = 0; i < cr.s.size(); ++i) {
        const LaneSection& li = section(cr.s[i]);
        const auto band = target(li);
        if (cr.s[i] > s0 - 10.0 && cr.s[i] < back_by && width(li) > 0.0) {
            if (left) cr.hi[i] = std::min(li.road_hi, band.second);
            else cr.lo[i] = std::max(li.road_lo, band.first);
        }
        if (cr.s[i] > out_from && cr.s[i] < back_by && width(li) > 0.0) {
            // in that lane to 6 m past the blockers, then back over to the car's own by `back_by`, not at once: at
            // 50 km/h a line that jumps back asks for all the grip there is
            const double w = std::clamp((cr.s[i] - (b_hi + 6.0)) / std::max(1.0, back_by - 4.0 - (b_hi + 6.0)), 0.0, 1.0);
            cr.ref[i] = (1.0 - w) * 0.5 * (band.first + band.second) + w * cr.ref[i];
        }
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
    const double fit = 0.5 * body.width + cs.nmpc_opt.edge_margin;
    for (size_t i = 0; i < cr.s.size(); ++i)  // the line through the middle where it is narrow
        cr.ref[i] = cr.hi[i] - cr.lo[i] > 2.0 * fit ? std::clamp(cr.ref[i], cr.lo[i] + fit, cr.hi[i] - fit)
                                                   : 0.5 * (cr.lo[i] + cr.hi[i]);
    c.problem.movers = p.moving;
    for (const Crosser& x : p.crossers) c.problem.gates.push_back(yield_gate(x, opt));
    wait_for_mergers(c.problem, p, opt);
    // the lanes it goes into count here
    c.problem.movers.insert(c.problem.movers.end(), p.stay_out.begin(), p.stay_out.end());
    if (static_cast<int>(out.candidates.size()) >= opt.pool) out.candidates.pop_back();
    out.candidates.push_back(std::move(c));
}

}  // namespace ex
