#include "nmpc/constraint/merge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "nmpc/constraint/crossing.hpp"
#include "nmpc/constraint/follow.hpp"
#include "nmpc/constraint/overtake.hpp"
#include "nmpc/constraint/turn.hpp"

namespace ex {

namespace {

constexpr double DT = 0.1, T_CHECK = 10.0;  // the ego and the stream are compared this far ahead [s]

// A track's value at time t: interpolated, and beyond its end carried on at the rate it ends with.
double at(const std::vector<double>& ts, const std::vector<double>& xs, double t) {
    const size_t n = ts.size();
    if (n >= 2 && t > ts[n - 1]) return xs[n - 1] + (xs[n - 1] - xs[n - 2]) / (ts[n - 1] - ts[n - 2]) * (t - ts[n - 1]);
    return sample(ts, xs, t);
}

// Where a car following `ts, xs` is at t when it brakes at `brake` from t_brake on, to a stop (as predicted before).
double braked(const std::vector<double>& ts, const std::vector<double>& xs, double t_brake, double brake, double t) {
    if (brake <= 0.0 || t <= t_brake) return at(ts, xs, t);
    const double s0 = at(ts, xs, t_brake), v0 = std::max(0.0, (at(ts, xs, t_brake + 0.05) - at(ts, xs, t_brake - 0.05)) / 0.1);
    const double tau = std::min(t - t_brake, v0 / brake);
    return s0 + v0 * tau - 0.5 * brake * tau * tau;
}

double gate_of(const Percept& p, const ConstraintOptions& opt) {
    double s = INF;
    for (const Merger& m : p.mergers) s = std::min(s, m.s_in - opt.gate_margin);
    return s;
}

}  // namespace

void wait_for_mergers(Problem& pr, const Percept& p, const ConstraintOptions& opt) {
    const double s_gate = gate_of(p, opt) - opt.merge_run_up;  // back from the entry, with a run-up to it
    for (const Merger& m : p.mergers) {
        pr.gates.push_back(Gate{s_gate, 0.0, m.t_in + opt.merge_lag, true});
        pr.leads.push_back(m.lead);
    }
}

void merge(const Constraints& cs, const Percept& p, const Scene& sc, const std::string& last_label,
           const Problem& base, Constraints::Result& out) {
    const ConstraintOptions& opt = cs.opt;
    const NmpcOptions& no = cs.nmpc_opt;
    const double length = p.front0 - p.rear0;
    const int last_id = last_label.rfind("merge:", 0) == 0 ? std::atoi(last_label.c_str() + 6) : 0;

    // Once the ego has gone into the stream (the last plan was a merge), the cars of it now behind the ego in its
    // lane stay chasers while they are close: they do not slow for the ego, which is not to slow in front of them
    // (for the limit, or the route's end).
    // (Not the cars queued behind the ego on the road it came by: only once it is past where the stream comes in.)
    double entry = INF;
    for (const Merger& m : p.mergers) entry = std::min(entry, m.s_in - opt.gate_margin);
    std::vector<const Track*> behind_all;
    if (last_label.rfind("merge:", 0) == 0 && (p.mergers.empty() || p.front0 >= entry))
        for (const Track& tr : p.tracks)
            if ((tr.actor->kind == Actor::vehicle || tr.actor->kind == Actor::bike) && !tr.actor->parked &&
                tr.s[0] < p.s0 && p.rear0 - (tr.s[0] + tr.ext_s[0]) < 40.0 && tr.cos_rel > 0.5 && tr.speed > 1.0 &&
                std::fabs(tr.e[0] - p.z0[IE]) < 2.5 &&
                std::none_of(p.mergers.begin(), p.mergers.end(), [&](const Merger& m) { return m.id == tr.actor->id; }))
                behind_all.push_back(&tr);
    std::sort(behind_all.begin(), behind_all.end(), [](auto* a, auto* b) { return a->s[0] > b->s[0]; });
    const Track* behind = behind_all.empty() ? nullptr : behind_all.front();
    // and those of it still coming in behind the ego
    // (only once the ego is past the entry, as for `behind`: before it, a drive that goes in after every car seen has
    // no chasers, and was taken as clear for these, kept the last gap's label and called base hopeless; route 8,
    // lane8s_r8 at 11.3 s: "merge:3726" held until 4.8 s, behind 3726, and stopped 5 m before the entry)
    const bool joining = last_label.rfind("merge:", 0) == 0 && !p.joining.empty() && (p.mergers.empty() || p.front0 >= entry);
    if (p.mergers.empty() && behind == nullptr && !joining) return;

    std::vector<const Merger*> stream;
    for (const Merger& m : p.mergers) stream.push_back(&m);
    std::sort(stream.begin(), stream.end(), [](auto* a, auto* b) { return a->t_in < b->t_in; });
    const double s_gate = stream.empty() ? -INF : gate_of(p, opt);
    double v_stream = behind ? behind->speed : 0.0;
    for (const Merger* m : stream) v_stream = std::max(v_stream, m->speed);
    for (size_t j = 0; joining && j < p.joining.size(); ++j) v_stream = std::max(v_stream, p.tracks[p.joining[j].first].speed);
    // up to the limit and overtake_over_limit beyond it, not held to the stream's speed: PDM-Lite goes on to 17.9 m/s
    // after route 8's turn, so the car behind falls back (13 Nov; was max(v_max, stream + 1), 13.9 m/s there)
    const double v_top = std::max(sc.v_max + opt.overtake_over_limit, v_stream + 1.0);

    // The ego from now, held before the gate until t_hold, then as hard as it can: within the bends' speed, braking
    // for them (and for the gate while it is held) at a_ref. Its front's position every DT.
    auto bend_cap = [&](double s) {
        double v = v_top;
        for (size_t i = 0; i < p.bend_s.size(); ++i)
            if (p.bend_s[i] >= s - 1.0) {
                const double vb = bend_speed(v_top, p.bend_kappa[i], opt.merge_a_lat);
                v = std::min(v, std::sqrt(vb * vb + 2.0 * no.a_ref * std::max(0.0, p.bend_s[i] - s)));
            }
        return v;
    };
    // Held before the gate until t_hold, the ego does best to come up to it slowly and be rolling when it opens, not
    // to stop at it: from rest the car behind in the stream closes 17 m while the ego gets up to its 13 m/s, from
    // 5.5 m/s under 6 m. `run` caps the speed at v_c until t_hold; the largest v_c that is not at the gate before
    // t_hold is taken. Slowing at 1.5 a_ref, gaining speed at 0.85 of what the car can (the plan is driven a little
    // late).
    auto run = [&](double t_hold, double v_c) {
        std::vector<double> front{p.front0};
        double v = p.v0, f = p.front0;
        const double launch = p.v0 < 0.5 ? 0.5 : 0.0;  // from rest the car is under way half a second late
        for (double t = 0.0; t < T_CHECK; t += DT) {
            double cap = bend_cap(f - 0.5 * length);
            if (t < t_hold) cap = std::min(cap, v_c);
            if (t < launch) cap = 0.0;
            const double a = 0.85 * cs.params.a_drive(v);  // full throttle (Problem::a_max INF)
            v = std::max(0.0, std::min(v + a * DT, std::max(cap, v - 1.5 * no.a_ref * DT)));
            // behind a car of the stream that is in ahead of it: no nearer than merge_lead_gap to its rear, at its
            // speed (the plan follows it so: Problem::follow_gap), not driven on through it
            const double t1 = t + DT;
            for (const Merger& m : p.mergers) {
                if (t1 < m.t_in) continue;
                const Track& tr = p.tracks[m.track];
                const double rear = at(tr.t, tr.s, t1) - tr.ext_s.back();
                if (rear > f && f + v * DT > rear - opt.merge_lead_gap) v = std::max(0.0, (rear - opt.merge_lead_gap - f) / DT);
            }
            f += v * DT;
            front.push_back(f);
        }
        return front;
    };
    auto arrival = [&](const std::vector<double>& front) {
        size_t i = 0;
        while (i < front.size() && front[i] < s_gate) ++i;
        return i * DT;
    };
    auto drive_in = [&](double t_hold) {
        std::vector<double> best = run(t_hold, v_top);
        if (arrival(best) >= t_hold) return best;
        if (arrival(run(t_hold, 0.0)) < t_hold) return std::vector<double>();  // cannot hold back that long
        double lo = 0.0, hi = v_top;
        for (int i = 0; i < 12; ++i) {
            const double mid = 0.5 * (lo + hi);
            (arrival(run(t_hold, mid)) >= t_hold ? lo : hi) = mid;
        }
        return run(t_hold, lo);
    };
    // Whether that drive keeps clear of the stream, and which of its cars it goes in ahead of (the chasers).
    // (Behind a car it has gone in after, the ego follows: its distance is checked over the first 1.5 s only. A
    // gap chosen last time is kept on half the margins, so that it is not dropped for a few centimetres.)
    std::vector<std::string> why;  // what stood in the way of the first few entries tried
    auto clear = [&](const std::vector<double>& front, std::vector<const Merger*>& chasers) {
        size_t i_e = 0;
        while (i_e < front.size() && front[i_e] < s_gate) ++i_e;
        if (i_e >= front.size()) return false;  // never in
        const double t_e = i_e * DT;
        chasers.clear();
        for (const Merger* m : stream) {
            const Track& tr = p.tracks[m->track];
            const bool ahead = m->t_in < t_e;  // in before the ego: followed
            if (!ahead) chasers.push_back(m);
            for (size_t i = 0; i < front.size(); ++i) {
                const double t = i * DT;
                if (t < m->t_in || i < i_e) continue;  // before the gate the ego is out of its way
                if (ahead && t > t_e + 1.5) break;
                // a gap the ego has started for is kept while the two stay 0.5 m apart: the entry times are read off
                // tracks sampled every 0.25 s (3 m of the stream's travel), and a start called off loses the gap
                const bool kept = last_id != 0 && !chasers.empty() && chasers.front()->id == last_id;
                const double lead_gap = kept ? 0.5 : opt.merge_lead_gap, chase_gap = kept ? 0.5 : opt.merge_gap;
                // a car the ego goes in ahead of brakes from when the ego is in (merge_chaser_brake; 0: it does not)
                const double s = ahead ? at(tr.t, tr.s, t) : braked(tr.t, tr.s, t_e, opt.merge_chaser_brake, t), ext = tr.ext_s.back();
                if (ahead ? front[i] > s - ext - lead_gap : front[i] - length < s + ext + chase_gap) {
                    char buf[160];
                    std::snprintf(buf, sizeof buf, "in at %.1f s, %s %d (in %.1f s): %.1f m apart at %.1f s", t_e,
                                  ahead ? "behind" : "ahead of", m->id, m->t_in,
                                  ahead ? s - ext - front[i] : front[i] - length - s - ext, t);
                    if (why.size() < 3) why.push_back(buf);
                    return false;
                }
            }
        }
        return !chasers.empty() || behind != nullptr || joining;
    };

    std::vector<double> holds{0.0};
    for (const Merger* m : stream)
        for (double lag = opt.merge_lag; lag < 1.25; lag += 0.1) holds.push_back(m->t_in + lag);  // how far behind it
    std::sort(holds.begin(), holds.end());
    std::vector<const Merger*> chasers, found;
    double t_hold = -1.0;
    for (double h : holds) {
        if (stream.empty() || !clear(drive_in(h), chasers)) continue;
        if (t_hold < 0.0 || (!chasers.empty() && chasers.front()->id == last_id)) t_hold = h, found = chasers;
        if (!found.empty() && found.front()->id == last_id) break;
    }
    // Past where it could still stop before the entry, the ego goes on with the gap it chose, clear or not: ahead of
    // the car it meant to be ahead of (stopping in the stream's lane is the worst of it).
    const bool in_already = !stream.empty() && p.front0 + p.v0 * p.v0 / (2.0 * std::fabs(no.a_min)) > s_gate;
    if (in_already && (t_hold < 0.0 || found.empty() || found.front()->id != last_id) && last_id != 0) {
        const auto it = std::find_if(stream.begin(), stream.end(), [&](const Merger* m) { return m->id == last_id; });
        if (it != stream.end()) {
            t_hold = 0.0;
            found.assign(it, stream.end());
            out.notes.push_back("merge: no stopping before the entry now, going on ahead of the car chosen");
        }
    }
    if (stream.empty()) t_hold = 0.0;  // only the cars behind
    if (!stream.empty() && p.front0 > s_gate + 1.0) {  // past the entry: every car of the stream comes in behind
        t_hold = 0.0;
        found = stream;
    }
    // At rest too near the entry for a run-up (it rolled on after a gap it had started for closed), the ego backs up
    // to where it has one, as far as there is room behind (the agent drives this: Decision::reverse), and starts for
    // no gap meanwhile: the drive a gap is judged on is not the one it would make from there, going backwards.
    const double short_by = stream.empty() ? 0.0 : opt.merge_run_up - (s_gate - p.front0);
    double room = INF;
    for (const Track& tr : p.tracks)
        for (size_t i = 0; i < tr.t.size(); ++i)
            if (tr.s[0] < p.s0 && std::fabs(tr.e[i] - p.z0[IE]) < tr.ext_e[i] + 0.5 * cs.body.width + 0.5)
                room = std::min(room, p.rear0 - (tr.s[i] + tr.ext_s[i]) - opt.peek_rear_gap);
    const bool back_up = p.v0 < 0.3 && short_by > 2.0 && room > 0.5 && !in_already &&
                         (found.empty() || found.front()->id != last_id);
    if (back_up) {
        t_hold = -1.0, found.clear();
        out.reverse = std::min(short_by, room);
        const LaneSection& l = p.section(p.s0);
        const double psi_to = std::clamp(0.15 * (p.z0[IE] - 0.5 * (l.lo + l.hi)), -0.25, 0.25);
        out.reverse_delta = std::clamp(std::atan(cs.params.L() * (p.z0[IPSI] - psi_to) / 1.5), -0.4, 0.4);
        out.notes.push_back("merge: no run-up left to the entry, backing up");
    } else if (t_hold < 0.0 && !stream.empty()) {
        char buf[120];
        std::snprintf(buf, sizeof buf, "merge: no gap clear (%zu cars seen, entry %.1f m ahead, bend %.1f m/s)", stream.size(),
                      s_gate - p.front0, bend_cap(s_gate));
        std::string note = buf;
        for (const std::string& w : why) note += "; " + w;
        out.notes.push_back(note);
    }
    if (t_hold < 0.0 || (found.empty() && behind == nullptr && !joining)) return;

    Candidate c;
    const int id = !found.empty() ? found.front()->id : behind != nullptr ? behind->actor->id : last_id;
    c.label = "merge:" + std::to_string(id);
    c.problem = base;
    Problem& pr = c.problem;
    pr.v_max = v_top;
    pr.a_lat_ref = opt.merge_a_lat;
    pr.follow_gap = opt.merge_lead_gap, pr.follow_time_gap = 0.0;  // close behind the car it goes in after
    pr.a_max = INF;  // full throttle
    // the speed asked for is the drive the gap was judged on: slow up to the entry so as to be rolling as it opens,
    // then as hard as the car can (not SimLingo's speed, which waits)
    pr.v_ref_t.clear(), pr.v_ref.clear();
    const std::vector<double> front = stream.empty() ? std::vector<double>() : drive_in(t_hold);
    for (size_t i = 0; i + 1 < front.size(); ++i) {
        pr.v_ref_t.push_back((i + 0.5) * DT);
        pr.v_ref.push_back(std::max(i * DT < t_hold ? 0.0 : 0.5 * v_top, (front[i + 1] - front[i]) / DT));
    }
    // not stopped at the route's end in front of a car that will not slow
    pr.gates.erase(std::remove_if(pr.gates.begin(), pr.gates.end(), [](const Gate& g) { return g.route_end; }), pr.gates.end());
    // when the ego is in, on the drive the gap was judged on (the chasers brake from then: merge_chaser_brake)
    double t_in_ego = 0.0;
    {
        const std::vector<double> f = stream.empty() ? std::vector<double>() : drive_in(t_hold);
        while (t_in_ego / DT < f.size() && f[static_cast<size_t>(t_in_ego / DT)] < s_gate) t_in_ego += DT;
    }
    for (const Merger* m : stream) {
        if (std::find(found.begin(), found.end(), m) != found.end()) {
            const Track& tr = p.tracks[m->track];
            Chaser ch;
            ch.t_from = m->t_in;
            for (double t = 0.0; t <= T_CHECK + 1e-9; t += 0.25)
                ch.t.push_back(t), ch.s_front.push_back(braked(tr.t, tr.s, t_in_ego, opt.merge_chaser_brake, t) + tr.ext_s.back());
            pr.chasers.push_back(std::move(ch));
        } else {
            Gate gate{s_gate, 0.0, std::max(m->t_in + opt.merge_lag, t_hold), true};
            gate.taper = false;  // rolled through as it opens
            pr.gates.push_back(gate);
            pr.leads.push_back(m->lead);
        }
    }
    for (size_t j = 0; joining && j < p.joining.size(); ++j) {
        const Track& tr = p.tracks[p.joining[j].first];
        Chaser ch;
        ch.t_from = p.joining[j].second;
        for (size_t i = 0; i < tr.t.size(); ++i) ch.t.push_back(tr.t[i]), ch.s_front.push_back(tr.s[i] + tr.ext_s[i]);
        pr.chasers.push_back(std::move(ch));
    }
    for (const Track* tr : behind_all) {
        Chaser ch;
        for (size_t i = 0; i < tr->t.size(); ++i) ch.t.push_back(tr->t[i]), ch.s_front.push_back(tr->s[i] + tr->ext_s[i]);
        pr.chasers.push_back(std::move(ch));
    }
    for (const Crosser& x : p.crossers) pr.gates.push_back(yield_gate(x, opt));
    stay_behind(pr, p);
    char buf[120];
    std::snprintf(buf, sizeof buf, "merge: clear ahead of %d, in from %.1f s, up to %.1f m/s", id, t_hold, v_top);
    out.notes.push_back(buf);
    out.merge = c.label;
    const bool going = last_label == c.label;
    if (static_cast<int>(out.candidates.size()) >= opt.pool) {
        if (out.candidates.size() <= 1) return;  // base is kept
        out.candidates.pop_back();
    }
    out.candidates.push_back(std::move(c));
    // waiting is over once the ego is in the stream's lane or ahead of a car that will not slow
    if (in_already || behind != nullptr || joining || going) out.candidates.front().hopeless = true;
}

}  // namespace ex
