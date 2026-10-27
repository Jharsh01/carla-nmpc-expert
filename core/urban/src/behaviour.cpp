#include "urban/behaviour.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <thread>

namespace ex {

namespace {

// An actor's predicted trajectory in path coordinates, with the half extents of its body along and across the route.
struct Track {
    const Actor* actor = nullptr;
    std::vector<double> t, s, e, ext_s, ext_e;
    double cos_rel = 1.0;  // of its heading relative to the route, now
    double speed = 0.0;  // now [m/s]
};

double sample_at(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const size_t i = std::upper_bound(xs.begin(), xs.end(), x) - xs.begin();
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
}

std::string fmt(const char* format, double a, double b = 0.0, double c = 0.0) {
    char buf[160];
    std::snprintf(buf, sizeof buf, format, a, b, c);
    return buf;
}

}  // namespace

struct Behaviour::Impl {
    std::vector<std::unique_ptr<UrbanNMPC>> pool;
    std::vector<std::string> slot_label;
    std::string last_label;
    // traffic-rule state, by stop id
    std::map<int, double> stopped_since;  // stop signs: when the car came to rest at the line
    std::set<int> released;  // stop signs held at, and lights driven through on amber or in the dilemma zone
    std::set<int> seen;  // lights already decided on
};

Behaviour::Behaviour(Path route_, VehicleParams params, Body body_, UrbanNMPC::Options nmpc_options, Options o)
    : route(std::move(route_)), body(body_), opt(o), impl_(std::make_unique<Impl>()) {
    for (int i = 0; i < o.pool; ++i) {
        impl_->pool.push_back(std::make_unique<UrbanNMPC>(route, params, body, nmpc_options));
        impl_->slot_label.emplace_back();
    }
}

Behaviour::~Behaviour() = default;

void Behaviour::reset() {
    Impl& m = *impl_;
    for (auto& n : m.pool) n->reset();
    std::fill(m.slot_label.begin(), m.slot_label.end(), "");
    m.last_label.clear();
    m.stopped_since.clear();
    m.released.clear();
    m.seen.clear();
}

Decision Behaviour::step(const Scene& sc) {
    Impl& m = *impl_;
    const auto start = std::chrono::steady_clock::now();
    Decision out;
    const UrbanNMPC& nmpc = *m.pool[0];
    const auto z0 = nmpc.path_state(sc.ego);
    const double s0 = z0[IS], v0 = sc.ego.v;
    const double front0 = s0 + 0.5 * body.length - body.cg_to_center, rear0 = s0 - 0.5 * body.length - body.cg_to_center;
    const double a_max = nmpc.opt.a_max;

    std::vector<LaneSection> lanes = sc.lanes;
    if (lanes.empty()) lanes.push_back(LaneSection());
    auto section = [&](double s) -> const LaneSection& {
        for (const LaneSection& l : lanes)
            if (l.s_from <= s && s < l.s_to) return l;
        return s < lanes.front().s_from ? lanes.front() : lanes.back();
    };
    auto in_lane = [&](double s, double e, double ext_e) {
        const LaneSection& l = section(s);
        return e - ext_e < l.hi && e + ext_e > l.lo;
    };

    // ---- the actors in path coordinates, classified (projected onto the stretch of route around the car)

    const double s_lo = s0 - 50.0, s_hi = s0 + opt.look_ahead + 80.0;
    // a point beyond either end of that stretch is clamped onto the end with only its sideways offset, so a car
    // straight behind the route's start would look in the lane at s = 0: there its distance stands in for the offset
    auto project = [&](double x, double y) {
        auto [s, e] = route.project(x, y, s_lo, s_hi);
        const tc::Pose q = route.pose(s);
        const double d = std::hypot(x - q.x, y - q.y);
        if (d > std::fabs(e) + 0.5) e = e >= 0.0 ? d : -d;
        return std::make_pair(s, e);
    };
    std::vector<Track> tracks;
    for (const Actor& a : sc.actors) {
        if (a.t.empty() || a.x.size() != a.t.size() || a.y.size() != a.t.size() || a.psi.size() != a.t.size()) continue;
        Track tr;
        tr.actor = &a;
        for (size_t i = 0; i < a.t.size(); ++i) {
            const auto [s, e] = project(a.x[i], a.y[i]);
            const double rel = tc::wrap_angle(a.psi[i] - route.pose(s).psi);
            tr.t.push_back(a.t[i]);
            tr.s.push_back(s);
            tr.e.push_back(e);
            tr.ext_s.push_back(0.5 * (std::fabs(a.length * std::cos(rel)) + std::fabs(a.width * std::sin(rel))));
            tr.ext_e.push_back(0.5 * (std::fabs(a.length * std::sin(rel)) + std::fabs(a.width * std::cos(rel))));
            if (i == 0) tr.cos_rel = std::cos(rel);
        }
        if (a.t.size() > 1) tr.speed = std::hypot(a.x[1] - a.x[0], a.y[1] - a.y[0]) / (a.t[1] - a.t[0]);
        // near the route, and not all of it behind the car or beyond the look-ahead
        bool relevant = false;
        for (size_t i = 0; i < tr.s.size(); ++i)
            relevant = relevant || (tr.s[i] + tr.ext_s[i] > rear0 - 10.0 && tr.s[i] - tr.ext_s[i] < s0 + opt.look_ahead &&
                                    std::fabs(tr.e[i]) < 30.0);
        if (relevant) tracks.push_back(std::move(tr));
    }

    std::vector<Lead> leads;
    std::vector<Mover> movers;
    struct Crosser {
        int id;
        double t_in, t_out, s_lo, s_hi;  // its window in the lane, and the span of the route it takes up there
    };
    std::vector<Crosser> crossers;
    struct Blocker {
        double s_lo, s_hi;
    };
    std::vector<Blocker> blockers;
    std::vector<const Track*> blocker_tracks, oncoming;

    auto add_movers = [&](const Track& tr, std::vector<Mover>& into) {
        const Actor& a = *tr.actor;
        const bool walker = a.kind == Actor::walker;
        const int n = a.length > 3.0 ? 3 : 1;
        const double r = n == 3 ? std::hypot(a.length / 6.0, 0.5 * a.width) : 0.5 * std::max(a.length, a.width);
        for (int c = 0; c < n; ++c) {
            const double d = n == 1 ? 0.0 : (c - 1) * a.length / 3.0;
            Mover mv;
            mv.radius = r;
            mv.growth = walker ? 0.3 : 0.1;
            for (size_t i = 0; i < a.t.size(); ++i) {
                const auto [s, e] = project(a.x[i] + d * std::cos(a.psi[i]), a.y[i] + d * std::sin(a.psi[i]));
                mv.t.push_back(a.t[i]);
                mv.s.push_back(s);
                mv.e_y.push_back(e);
            }
            into.push_back(std::move(mv));
        }
    };

    for (const Track& tr : tracks) {
        const Actor& a = *tr.actor;
        const size_t n = tr.t.size();
        const bool aligned = tr.cos_rel > 0.5, opposed = tr.cos_rel < -0.5;
        const bool crossing_kind = a.kind == Actor::walker || (!aligned && !opposed);
        const bool ahead_now = tr.s[0] - tr.ext_s[0] > front0 - 0.5;
        const bool in_now = in_lane(tr.s[0], tr.e[0], tr.ext_e[0]);
        // the samples in the lane, not all behind the car
        std::vector<size_t> inside;
        for (size_t i = 0; i < n; ++i)
            if (in_lane(tr.s[i], tr.e[i], tr.ext_e[i]) && tr.s[i] + tr.ext_s[i] > rear0) inside.push_back(i);

        if ((a.kind == Actor::object || a.parked) && in_now && tr.s[0] + tr.ext_s[0] > rear0 - 3.0) {  // until passed
            blockers.push_back({tr.s[0] - tr.ext_s[0], tr.s[0] + tr.ext_s[0]});
            blocker_tracks.push_back(&tr);
            out.notes.push_back(fmt("blocker at s %.1f m", tr.s[0]));
        } else if (crossing_kind && !inside.empty()) {
            Crosser c{a.id, tr.t[inside.front()], inside.back() == n - 1 ? INF : tr.t[inside.back()], INF, -INF};
            for (size_t i : inside) c.s_lo = std::min(c.s_lo, tr.s[i] - tr.ext_s[i]), c.s_hi = std::max(c.s_hi, tr.s[i] + tr.ext_s[i]);
            crossers.push_back(c);
            out.notes.push_back(fmt("crosser in the lane from %.1f s to %.1f s at s %.1f m", c.t_in, c.t_out, c.s_lo));
        } else if (!crossing_kind && !inside.empty() && (ahead_now || !in_now)) {
            // a lead (or, opposed, one coming the wrong way down the lane); one cutting in counts from when it does
            Lead lead;
            const double t_in = tr.t[inside.front()];
            for (size_t i = 0; i < n; ++i) {
                lead.t.push_back(tr.t[i]);
                lead.s_rear.push_back(tr.s[i] - tr.ext_s[i] + (tr.t[i] < t_in ? 300.0 : 0.0));
            }
            leads.push_back(std::move(lead));
            if (!in_now) out.notes.push_back(fmt("cut-in from %.1f s at s %.1f m", t_in, tr.s[0]));
        } else {
            add_movers(tr, movers);
            if (opposed) oncoming.push_back(&tr);
        }
    }

    // ---- stop lines, with the rules' state

    std::vector<Gate> stop_gates;
    std::set<int> present;
    for (const StopLine& st : sc.stops) {
        present.insert(st.id);
        const double to_line = st.s - front0;
        if (to_line > opt.look_ahead || to_line < -2.0) continue;
        const double decel = v0 * v0 / (2.0 * std::max(0.1, to_line));
        switch (st.kind) {
            case StopLine::route_end: stop_gates.push_back({st.s}); break;
            case StopLine::stop_sign:
                if (m.released.count(st.id)) break;
                if (v0 < 0.1 && to_line < 4.0) {
                    if (!m.stopped_since.count(st.id)) m.stopped_since[st.id] = sc.t;
                    if (sc.t - m.stopped_since[st.id] >= opt.stop_hold) {
                        m.released.insert(st.id);
                        out.notes.push_back("stop sign: held, going on");
                        break;
                    }
                }
                stop_gates.push_back({st.s});
                break;
            case StopLine::amber_light:
            case StopLine::red_light: {
                // decided once per light: through if stopping would take more than the limit (on amber, more
                // than comfortable braking), else stop
                if (!m.seen.count(st.id)) {
                    m.seen.insert(st.id);
                    const double limit = st.kind == StopLine::amber_light ? nmpc.opt.a_ref * 1.5 : opt.dilemma_decel;
                    if (decel > limit) {
                        m.released.insert(st.id);
                        out.notes.push_back(fmt("light: %.1f m/s^2 to stop, going through", decel));
                    }
                }
                if (!m.released.count(st.id)) stop_gates.push_back({st.s});
                break;
            }
        }
    }
    for (auto* ids : {&m.seen, &m.released})
        for (auto it = ids->begin(); it != ids->end();) it = present.count(*it) ? std::next(it) : ids->erase(it);
    for (auto it = m.stopped_since.begin(); it != m.stopped_since.end();)
        it = present.count(it->first) ? std::next(it) : m.stopped_since.erase(it);

    // ---- the candidates

    // the lane, widened where the car is if it is out of it (after an overtake, a swerve), narrowing back to the
    // lane over the distance a calm lane change takes, so that coming back is a manoeuvre and not a jump
    Corridor lane_corridor;
    const double half = 0.5 * body.width + 0.3, merge = std::max(25.0, 3.0 * v0);
    for (double s = s0 - 10.0; s <= s0 + opt.look_ahead + 60.0; s += 2.0) {
        const LaneSection& l = section(s);
        const double w = std::clamp((s - s0) / merge, 0.0, 1.0);
        lane_corridor.s.push_back(s);
        lane_corridor.lo.push_back(std::min(l.lo, (1.0 - w) * (z0[IE] - half) + w * l.lo));
        lane_corridor.hi.push_back(std::max(l.hi, (1.0 - w) * (z0[IE] + half) + w * l.hi));
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
        pr.movers = movers;
        pr.leads = leads;
        pr.gates = stop_gates;
        return pr;
    };
    auto yield_gate = [&](const Crosser& c) {
        return Gate{c.s_lo - opt.gate_margin, std::max(0.0, c.t_in - opt.gate_buffer), c.t_out + opt.gate_buffer, true};
    };
    auto stay_behind = [&](Problem& pr) {
        for (const Blocker& b : blockers) {
            Lead lead;
            lead.t = {0.0};
            lead.s_rear = {b.s_lo - 4.0};  // waiting far enough back to pull out round it
            pr.leads.push_back(lead);
        }
    };

    std::vector<Candidate> cands;
    {  // yield to every crosser, stay behind every blocker
        Candidate c;
        c.label = "base";
        c.problem = base_problem();
        for (const Crosser& x : crossers) c.problem.gates.push_back(yield_gate(x));
        stay_behind(c.problem);
        cands.push_back(std::move(c));
    }
    {  // pass ahead of one crosser, where the car can get past in time
        std::vector<const Crosser*> by_time;
        for (const Crosser& x : crossers) by_time.push_back(&x);
        std::sort(by_time.begin(), by_time.end(), [](auto* a, auto* b) { return a->t_in < b->t_in; });
        for (const Crosser* x : by_time) {
            if (static_cast<int>(cands.size()) >= opt.pool - 1) break;
            const double t_by = x->t_in - opt.gate_buffer;
            const double need = x->s_hi + opt.gate_margin - rear0;
            const double t_acc = std::max(0.0, (sc.v_max - v0) / a_max);
            const double reach = t_by <= t_acc ? v0 * t_by + 0.5 * a_max * t_by * t_by
                                               : v0 * t_acc + 0.5 * a_max * t_acc * t_acc + sc.v_max * (t_by - t_acc);
            if (t_by <= 0.0 || reach < need) continue;
            Candidate c;
        c.label = "pass:" + std::to_string(x->id);
        c.problem = base_problem();
            for (const Crosser& y : crossers)
                c.problem.gates.push_back(&y == x ? Gate{x->s_hi + opt.gate_margin, std::max(0.0, x->t_in - opt.gate_buffer),
                                                         x->t_out + opt.gate_buffer, false}
                                                  : yield_gate(y));
            stay_behind(c.problem);
            cands.push_back(std::move(c));
        }
    }
    if (!blockers.empty()) {  // borrow the left lane round the nearest blockers, if allowed and clear
        double b_lo = INF, b_hi = -INF;
        for (const Blocker& b : blockers)
            if (b.s_lo - front0 < opt.blocker_range) b_lo = std::min(b_lo, b.s_lo), b_hi = std::max(b_hi, b.s_hi);
        for (const Blocker& b : blockers)  // and those just beyond them, as in a row of cones
            if (b.s_lo < b_hi + 10.0) b_hi = std::max(b_hi, b.s_hi);
        const LaneSection& l = section(b_lo);
        if (std::isfinite(b_lo) && l.left_allowed && l.left_width > 0.0) {
            const double v_pass = std::max(v0, 5.0);
            const double done = b_hi + 10.0;
            const double T = (done - front0) / v_pass + 2.0;
            bool clear = true;
            for (const Track* tr : oncoming)
                for (size_t i = 0; i < tr->t.size() && tr->t[i] <= T; ++i) {
                    const LaneSection& li = section(tr->s[i]);
                    const bool in_left = tr->e[i] - tr->ext_e[i] < li.hi + li.left_width && tr->e[i] + tr->ext_e[i] > li.hi;
                    const double ours = std::min(front0 + v_pass * tr->t[i], done);
                    if (in_left && tr->s[i] + tr->ext_s[i] > rear0 && tr->s[i] - tr->ext_s[i] < ours + opt.oncoming_gap)
                        clear = false;
                }
            // once out in the other lane, the car finishes the overtake
            const bool committed = m.last_label == "borrow_left" && z0[IE] > 0.5 * (l.lo + l.hi) + 0.5;
            clear = clear || committed;
            if (!clear) out.notes.push_back("blocker: oncoming traffic, waiting");
            if (clear) {
                Candidate c;
        c.label = "borrow_left";
        c.problem = base_problem();
                Corridor& cr = c.problem.corridor;
                for (size_t i = 0; i < cr.s.size(); ++i) {
                    const LaneSection& li = section(cr.s[i]);
                    if (cr.s[i] > s0 - 10.0 && cr.s[i] < b_hi + 15.0 && li.left_allowed) cr.hi[i] = li.hi + li.left_width;
                    if (cr.s[i] > b_lo - 12.0 && cr.s[i] < b_hi + 6.0) cr.ref[i] = li.hi + 0.5 * li.left_width;
                }
                for (const Crosser& x : crossers) c.problem.gates.push_back(yield_gate(x));
                for (const Track* tr : blocker_tracks) add_movers(*tr, c.problem.movers);  // gone round, not stopped behind
                if (static_cast<int>(cands.size()) >= opt.pool) cands.pop_back();
                cands.push_back(std::move(c));
            }
        }
    }

    // ---- solve them in parallel, each candidate on the NMPC that solved it last time if there is one

    std::vector<int> slot_of(cands.size(), -1);
    std::vector<bool> taken(m.pool.size(), false);
    for (size_t i = 0; i < cands.size(); ++i)
        for (size_t j = 0; j < m.pool.size(); ++j)
            if (!taken[j] && m.slot_label[j] == cands[i].label) slot_of[i] = static_cast<int>(j), taken[j] = true;
    for (size_t i = 0; i < cands.size(); ++i)
        if (slot_of[i] < 0)
            for (size_t j = 0; j < m.pool.size(); ++j)
                if (!taken[j]) {
                    slot_of[i] = static_cast<int>(j), taken[j] = true;
                    m.slot_label[j] = cands[i].label;
                    m.pool[j]->reset();
                    break;
                }
    std::vector<std::thread> threads;
    for (size_t i = 1; i < cands.size(); ++i)
        threads.emplace_back([&, i] { cands[i].plan = m.pool[slot_of[i]]->solve(cands[i].problem); });
    cands[0].plan = m.pool[slot_of[0]]->solve(cands[0].problem);
    for (auto& th : threads) th.join();

    // ---- choose: the cheapest of those within their constraints, the last choice favoured

    const UrbanNMPC::Options& no = nmpc.opt;
    auto score = [&](const Candidate& c) {
        const Plan& pl = c.plan;
        double J = 0.0;
        for (int k = 0; k < nmpc.N; ++k) {
            const double dt = (pl.times[k + 1] - pl.times[k]) / no.h, t = pl.times[k + 1];
            const double v = pl.x(k + 1, IV), a = pl.x(k + 1, IA), s = pl.x(k + 1, IS);
            const double v_des = std::min(sc.v_max, sc.v_ref.empty() ? sc.v_max : sample_at(sc.v_ref_t, sc.v_ref, t));
            const double a_lat = v * v * std::tan(pl.x(k + 1, IDELTA)) / nmpc.p.L();
            const LaneSection& l = section(s);
            J += dt * (no.w_speed * (v - v_des) * (v - v_des) + no.w_accel * a * a + no.w_lat * a_lat * a_lat +
                       opt.w_out_of_lane * std::pow(pl.x(k + 1, IE) - 0.5 * (l.lo + l.hi), 2));
        }
        for (size_t i = 0; i < pl.slack_max.size(); ++i)
            J += (i + 1 == pl.slack_max.size() ? no.w_headway : no.w_slack) * pl.slack_max[i];
        if (c.label.rfind("pass:", 0) == 0) J += opt.pass_penalty;
        return J;
    };
    int best = -1;
    bool best_ok = false;
    for (size_t i = 0; i < cands.size(); ++i) {
        Candidate& c = cands[i];
        // within the hard constraints (the headway, the last group, is spacing, not contact)
        const double slack = *std::max_element(c.plan.slack_max.begin(), c.plan.slack_max.end() - 1);
        const bool ok = slack <= opt.slack_ok;
        c.score = score(c) * (c.label == m.last_label ? opt.hysteresis : 1.0);
        if (best < 0 || (ok && !best_ok) || (ok == best_ok && c.score < cands[best].score)) best = static_cast<int>(i), best_ok = ok;
    }
    out.label = cands[best].label;
    out.plan = cands[best].plan;
    if (out.label != m.last_label && !m.last_label.empty()) out.notes.push_back("switched from " + m.last_label + " to " + out.label);
    m.last_label = out.label;
    out.candidates = std::move(cands);
    out.wall_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return out;
}

}  // namespace ex
