#include "perception/classify.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "perception/lanes.hpp"

namespace ex {

namespace {

std::string fmt(const char* format, double a, double b = 0.0, double c = 0.0) {
    char buf[160];
    std::snprintf(buf, sizeof buf, format, a, b, c);
    return buf;
}

}  // namespace

void classify(Percept& p, const Body& body, const PerceptionOptions& opt, const Projector& project) {
    const auto& z0 = p.z0;
    const double front0 = p.front0, rear0 = p.rear0;
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
            add_circles(tr, project, opt.static_margin, p.stay_out);
            if (oncoming_traffic) p.oncoming.push_back(k);
            continue;
        }
        const bool crossing_kind = a.kind == Actor::walker || (!aligned && !opposed);
        const bool ahead_now = tr.s[0] - tr.ext_s[0] > front0 - 0.5;
        const bool behind_now = tr.s[0] + tr.ext_s[0] < rear0;  // all of it
        const double reach = a.kind == Actor::object || a.parked ? opt.blocker_overlap : 0.0;  // into the lane
        const bool in_now = in_lane(p.section(tr.s[0]), tr.e[0], tr.ext_e[0] - reach);
        // the samples in the lane, not all behind the car
        std::vector<size_t> inside;
        for (size_t i = 0; i < n; ++i)
            if (in_lane(p.section(tr.s[i]), tr.e[i], tr.ext_e[i] - reach) && tr.s[i] + tr.ext_s[i] > rear0) inside.push_back(i);

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
        // to be judged on the oncoming traffic (nmpc/constraint/overtake), or the car follows SimLingo into it unchecked.
        const LaneSection& here = p.section(tr.s[0]);
        const auto [way_lo, way_hi] = way_band(here);
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
            add_circles(tr, project, opt.static_margin, p.movers);
            if (still_kind) p.still.push_back(k);
            else add_circles(tr, project, opt.static_margin, p.moving);
            if (oncoming_traffic) p.oncoming.push_back(k);
        }
    }

}

}  // namespace ex
