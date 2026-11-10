#include "urban/sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ex {

namespace {

std::string fmt(const char* format, double a, double b = 0.0, double c = 0.0) {
    char buf[160];
    std::snprintf(buf, sizeof buf, format, a, b, c);
    return buf;
}

}  // namespace

const LaneSection& Percept::section(double s) const {
    for (const LaneSection& l : lanes)
        if (l.s_from <= s && s < l.s_to) return l;
    return s < lanes.front().s_from ? lanes.front() : lanes.back();
}

void Sensor::reset() {
    stopped_since_.clear();
    released_.clear();
    seen_.clear();
}

Percept Sensor::sense(const Scene& sc, const Path& route, const UrbanNMPC& nmpc) {
    Percept p;
    p.z0 = nmpc.path_state(sc.ego);
    const auto& z0 = p.z0;
    const double s0 = p.s0 = z0[IS], v0 = p.v0 = sc.ego.v;
    const double front0 = p.front0 = s0 + 0.5 * body.length - body.cg_to_center;
    const double rear0 = p.rear0 = s0 - 0.5 * body.length - body.cg_to_center;

    p.lanes = sc.lanes;
    if (p.lanes.empty()) p.lanes.push_back(LaneSection());
    auto in_lane = [&](double s, double e, double ext_e) {
        const LaneSection& l = p.section(s);
        return e - ext_e < l.hi && e + ext_e > l.lo;
    };

    // ---- the actors in path coordinates (projected onto the stretch of route around the car)

    const double s_lo = s0 - 50.0, s_hi = s0 + opt.look_ahead + 80.0;
    // a point beyond either end of that stretch is clamped onto the end with only its sideways offset, so a car
    // straight behind the route's start would look in the lane at s = 0: there its distance stands in for the offset
    auto project = [&](double x, double y) {
        auto [s, e] = route.project(x, y, s_lo, s_hi);
        const Pose q = route.pose(s);
        const double d = std::hypot(x - q.x, y - q.y);
        if (d > std::fabs(e) + 0.5) e = e >= 0.0 ? d : -d;
        return std::make_pair(s, e);
    };
    for (const Actor& a : sc.actors) {
        if (a.t.empty() || a.x.size() != a.t.size() || a.y.size() != a.t.size() || a.psi.size() != a.t.size()) continue;
        Track tr;
        tr.actor = &a;
        for (size_t i = 0; i < a.t.size(); ++i) {
            const auto [s, e] = project(a.x[i], a.y[i]);
            const double rel = wrap_angle(a.psi[i] - route.pose(s).psi);
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
        if (relevant) p.tracks.push_back(std::move(tr));
    }

    // ---- classified

    // an actor as circles along its body
    auto add_movers = [&](const Track& tr, std::vector<Mover>& into) {
        const Actor& a = *tr.actor;
        const bool walker = a.kind == Actor::walker;
        const bool still = a.kind == Actor::object || a.parked;
        const int n = a.length > 3.0 ? 3 : 1;
        const double r = n == 3 ? std::hypot(a.length / 6.0, 0.5 * a.width) : 0.5 * std::max(a.length, a.width);
        for (int c = 0; c < n; ++c) {
            const double d = n == 1 ? 0.0 : (c - 1) * a.length / 3.0;
            Mover mv;
            mv.radius = r;
            mv.growth = still ? 0.0 : walker ? 0.3 : 0.1;
            if (still) mv.margin = opt.static_margin;
            for (size_t i = 0; i < a.t.size(); ++i) {
                const auto [s, e] = project(a.x[i] + d * std::cos(a.psi[i]), a.y[i] + d * std::sin(a.psi[i]));
                mv.t.push_back(a.t[i]);
                mv.s.push_back(s);
                mv.e_y.push_back(e);
            }
            into.push_back(std::move(mv));
        }
    };

    for (size_t k = 0; k < p.tracks.size(); ++k) {
        const Track& tr = p.tracks[k];
        const Actor& a = *tr.actor;
        const size_t n = tr.t.size();
        const bool aligned = tr.cos_rel > 0.5, opposed = tr.cos_rel < -0.5;
        // traffic coming the other way, to wait for before borrowing its lane; not a parked car facing the ego (Bench2Drive
        // routes 4 and 6: a row parked on the far side held the car for good), which stays a mover to keep clear of
        const bool oncoming_traffic = opposed && a.kind != Actor::object && !a.parked;
        // and traffic going the ego's way, for changing into a lane that runs its way
        if (aligned && a.kind != Actor::object && a.kind != Actor::walker && !a.parked) p.along.push_back(k);
        if (a.stays_out) {  // nothing to the ego, unless it goes into that car's lane
            add_movers(tr, p.stay_out);
            if (oncoming_traffic) p.oncoming.push_back(k);
            continue;
        }
        const bool crossing_kind = a.kind == Actor::walker || (!aligned && !opposed);
        const bool ahead_now = tr.s[0] - tr.ext_s[0] > front0 - 0.5;
        const bool behind_now = tr.s[0] + tr.ext_s[0] < rear0;  // all of it
        const double reach = a.kind == Actor::object || a.parked ? opt.blocker_overlap : 0.0;  // into the lane
        const bool in_now = in_lane(tr.s[0], tr.e[0], tr.ext_e[0] - reach);
        // the samples in the lane, not all behind the car
        std::vector<size_t> inside;
        for (size_t i = 0; i < n; ++i)
            if (in_lane(tr.s[i], tr.e[i], tr.ext_e[i] - reach) && tr.s[i] + tr.ext_s[i] > rear0) inside.push_back(i);

        // A parked car or an object wholly left of the car's body is on the far side of the road, whatever the band
        // takes in: where SimLingo's path changes lane the band takes in both lanes, and on route 4 the cars parked
        // opposite became blockers, to stay behind (base waited through clear gaps) and to go round (a row of them
        // 80 m long to be clear of, so any oncoming car called the overtake off). Kept clear of, never followed.
        const bool still_kind = a.kind == Actor::object || a.parked;
        const bool far_side = still_kind && tr.e[0] - tr.ext_e[0] > z0[IE] + 0.5 * body.width + 0.3;
        // Nor is one that leaves the car room past it within the band: where the band takes in two lanes (SimLingo
        // changing lane), a car parked in one leaves the other free. Route 1: it held the car behind it for good,
        // waiting to go round into the lane beyond the band, which is oncoming. Kept clear of, as an edge or circles.
        // Only room in lanes running the car's way counts: room in an oncoming lane the band takes in is a borrow,
        // to be judged on the oncoming traffic (constraints.hpp), or the car follows SimLingo into it unchecked.
        const LaneSection& here = p.section(tr.s[0]);
        const double way_lo = std::max(here.lo, here.way_lo), way_hi = std::min(here.hi, here.way_hi);
        const double room = way_lo >= way_hi ? 0.0
                            : std::max(way_hi - (tr.e[0] + tr.ext_e[0]), (tr.e[0] - tr.ext_e[0]) - way_lo);
        const bool room_beside = still_kind && in_now && !far_side && room >= body.width + 2.0 * (opt.static_margin + 0.3);
        if (far_side || room_beside) inside.clear();
        if (room_beside) p.notes.push_back(fmt("parked at s %.1f m, room beside it", tr.s[0]));
        if (still_kind && in_now && !far_side && !room_beside && tr.s[0] + tr.ext_s[0] > rear0 - 3.0) {  // until passed
            p.blockers.push_back({tr.s[0] - tr.ext_s[0], tr.s[0] + tr.ext_s[0], k});
            p.notes.push_back(fmt("blocker at s %.1f m", tr.s[0]));
        } else if (crossing_kind && !inside.empty()) {
            Crosser c{a.id, tr.t[inside.front()], inside.back() == n - 1 ? INF : tr.t[inside.back()], INF, -INF};
            for (size_t i : inside) c.s_lo = std::min(c.s_lo, tr.s[i] - tr.ext_s[i]), c.s_hi = std::max(c.s_hi, tr.s[i] + tr.ext_s[i]);
            p.crossers.push_back(c);
            p.notes.push_back(fmt("crosser in the lane from %.1f s to %.1f s at s %.1f m", c.t_in, c.t_out, c.s_lo));
        } else if (!crossing_kind && !inside.empty() && !behind_now && (ahead_now || !in_now)) {
            // a lead (or, opposed, one coming the wrong way down the lane); one cutting in counts from when it does.
            // Not one behind the car: reaching where the car is now is not cutting in, the car will have moved on.
            // One that overtakes and cuts in counts once it is level.
            Lead lead;
            const double t_in = tr.t[inside.front()];
            for (size_t i = 0; i < n; ++i) {
                lead.t.push_back(tr.t[i]);
                lead.s_rear.push_back(tr.s[i] - tr.ext_s[i] + (tr.t[i] < t_in ? 300.0 : 0.0));
            }
            p.leads.push_back(std::move(lead));
            // one coming the wrong way down the path is oncoming traffic all the same: SimLingo's path swings into
            // the other lane before the car does, and the oncoming car on it went unseen by the clear check (route 4,
            // a borrow begun with it 50 m off at 10 m/s; the car stopped half across and was struck)
            if (oncoming_traffic) p.oncoming.push_back(k);
            if (!in_now) p.notes.push_back(fmt("cut-in from %.1f s at s %.1f m", t_in, tr.s[0]));
        } else {
            add_movers(tr, p.movers);
            if (still_kind) p.still.push_back(k);
            else add_movers(tr, p.moving);
            if (oncoming_traffic) p.oncoming.push_back(k);
        }
    }

    // ---- stop lines, with the rules' state

    std::set<int> present;
    for (const StopLine& st : sc.stops) {
        present.insert(st.id);
        const double to_line = st.s - front0;
        if (to_line > opt.look_ahead || to_line < -2.0) continue;
        const double decel = v0 * v0 / (2.0 * std::max(0.1, to_line));
        switch (st.kind) {
            case StopLine::route_end: p.stops.push_back(st.s); break;
            case StopLine::stop_sign:
                if (released_.count(st.id)) break;
                if (v0 < 0.1 && to_line < 4.0) {
                    if (!stopped_since_.count(st.id)) stopped_since_[st.id] = sc.t;
                    if (sc.t - stopped_since_[st.id] >= opt.stop_hold) {
                        released_.insert(st.id);
                        p.notes.push_back("stop sign: held, going on");
                        break;
                    }
                }
                p.stops.push_back(st.s);
                break;
            case StopLine::amber_light:
            case StopLine::red_light: {
                // decided once per light: through if stopping would take more than the limit (on amber, more
                // than comfortable braking), else stop
                if (!seen_.count(st.id)) {
                    seen_.insert(st.id);
                    const double limit = st.kind == StopLine::amber_light ? nmpc.opt.a_ref * 1.5 : opt.dilemma_decel;
                    if (decel > limit) {
                        released_.insert(st.id);
                        p.notes.push_back(fmt("light: %.1f m/s^2 to stop, going through", decel));
                    }
                }
                if (!released_.count(st.id)) p.stops.push_back(st.s);
                break;
            }
        }
    }
    for (auto* ids : {&seen_, &released_})
        for (auto it = ids->begin(); it != ids->end();) it = present.count(*it) ? std::next(it) : ids->erase(it);
    for (auto it = stopped_since_.begin(); it != stopped_since_.end();)
        it = present.count(it->first) ? std::next(it) : stopped_since_.erase(it);
    return p;
}

}  // namespace ex
