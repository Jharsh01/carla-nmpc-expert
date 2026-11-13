#include "nmpc/constraint/crossing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "nmpc/constraint/follow.hpp"
#include "nmpc/constraint/merge.hpp"
#include "nmpc/constraint/overtake.hpp"
#include "nmpc/constraint/turn.hpp"
#include "perception/lanes.hpp"
#include "nmpc/stage.hpp"

namespace ex {

Gate yield_gate(const Crosser& c, const ConstraintOptions& opt) {
    return Gate{c.s_lo - opt.gate_margin, std::max(0.0, c.t_in - opt.gate_buffer), c.t_out + opt.gate_buffer, true};
}

Gate pass_gate(const Crosser& c, const ConstraintOptions& opt) {
    return Gate{c.s_hi + opt.gate_margin, std::max(0.0, c.t_in - opt.gate_buffer), c.t_out + opt.gate_buffer, false};
}

bool can_pass(const Crosser& c, const Percept& p, double v_max, const VehicleParams& params, const NmpcOptions& nmpc,
              const ConstraintOptions& opt) {
    const double t_by = c.t_in - opt.gate_buffer;
    const double need = c.s_hi + opt.gate_margin - p.rear0;
    const double reach = drive(p.v0, std::max(p.v0, v_max), nmpc.a_max, params).at(t_by);
    return t_by > 0.0 && reach >= need;
}

namespace {

// A box in the plane: centre, half length and width, heading.
struct Box {
    double x, y, hl, hw, psi;
};

bool overlap(const Box& a, const Box& b) {  // separating axes
    const double ax[2][2] = {{std::cos(a.psi), std::sin(a.psi)}, {-std::sin(a.psi), std::cos(a.psi)}};
    const double bx[2][2] = {{std::cos(b.psi), std::sin(b.psi)}, {-std::sin(b.psi), std::cos(b.psi)}};
    const double d[2] = {b.x - a.x, b.y - a.y};
    auto radius = [](const Box& o, const double (&u)[2][2], const double* n) {
        return o.hl * std::fabs(u[0][0] * n[0] + u[0][1] * n[1]) + o.hw * std::fabs(u[1][0] * n[0] + u[1][1] * n[1]);
    };
    for (const auto* axes : {&ax, &bx})
        for (int i = 0; i < 2; ++i) {
            const double* n = (*axes)[i];
            if (std::fabs(d[0] * n[0] + d[1] * n[1]) > radius(a, ax, n) + radius(b, bx, n)) return false;
        }
    return true;
}

}  // namespace

void cross(const Constraints& cs, const Percept& p, const Scene& sc, const std::string& last_label,
           const Problem& base, Constraints::Result& out) {
    constexpr double DT = 0.1, T_CHECK = 10.0, HALF_STEP = 0.125;  // tracks are sampled every 0.25 s
    // the boxes grown as PDM-Lite grows them: its own x1.3 in length and x1.2 in width when going over 5 m/s now (not
    // from rest), others' x1.2-1.5 in length at speed (their_*: each end, each side) [m]
    constexpr double THEIR_END = 0.5, THEIR_SIDE = 0.2;
    constexpr double GIVE = 1.0;  // the plan may be this far off the judged drive at a crossing moment [m]
    const ConstraintOptions& opt = cs.opt;
    const NmpcOptions& no = cs.nmpc_opt;
    if (p.route == nullptr) return;
    const double OWN_END = p.v0 > 5.0 ? 0.75 : 0.0, OWN_SIDE = p.v0 > 5.0 ? 0.2 : 0.0;
    const double length = p.front0 - p.rear0;
    const bool going = last_label.rfind("cross", 0) == 0;
    const int last_id = last_label.rfind("cross:", 0) == 0 ? std::atoi(last_label.c_str() + 6) : 0;

    // Each crosser at the moments it is near the ego's path (its body within the ego's width and cross_side of the
    // path, as an envelope in path coordinates): when, its box then, and the stretch of the path its envelope takes
    // up. The ego is judged against its box at the same moment, as PDM-Lite does: the envelope alone, where the path
    // slants across the oncoming lane (route 10's left turn), took up 5-16 m for 1-2 s, and every gap read as shut.
    struct Spot { double t, lo, hi, s; Box box; };
    struct Way { const Crosser* x; std::vector<Spot> spots; };
    std::vector<Way> ways;
    const double reach = 0.5 * cs.body.width + opt.cross_side;
    for (const Crosser& x : p.crossers) {
        const Track* tr = nullptr;
        for (const Track& t : p.tracks)
            if (t.actor->id == x.id) tr = &t;
        if (tr == nullptr) continue;
        const Actor& a = *tr->actor;
        Way w{&x, {}};
        const double until = tr->speed < opt.cross_slow_speed ? opt.cross_slow_horizon : INF;
        for (size_t i = 0; i < tr->t.size() && i < a.t.size(); ++i)
            if (tr->t[i] <= until && std::fabs(tr->e[i]) - tr->ext_e[i] < reach && tr->s[i] + tr->ext_s[i] + opt.gate_margin > p.rear0)
                w.spots.push_back({tr->t[i], tr->s[i] - tr->ext_s[i] - opt.gate_margin, tr->s[i] + tr->ext_s[i] + opt.gate_margin,
                                   tr->s[i], Box{a.x[i], a.y[i], 0.5 * a.length + THEIR_END, 0.5 * a.width + THEIR_SIDE, a.psi[i]}});
        if (!w.spots.empty()) ways.push_back(std::move(w));
    }
    if (ways.empty()) return;
    double s_gate = INF;  // held, the front stays behind the nearest any of them comes
    for (const Way& w : ways)
        for (const Spot& sp : w.spots) s_gate = std::min(s_gate, sp.lo);
    s_gate = std::max(s_gate, p.front0);
    const double v_top = sc.v_max;

    // The ego from now, its speed capped at v_c until t_hold (and not past the gate before then), then as hard as it
    // can within the bends' speed; its front every DT, rolling up to the gate so as to be moving as it opens. Launched
    // as PDM-Lite launches the same car on full throttle (route 10: 0.05 to 5.0 m/s in 0.95 s, under way in 0.15 s,
    // 4-7 m/s^2): from rest 0.2 s late, at 0.95 of what the car can. With merge.cpp's 0.5 s and 0.85 the car was 5.6 m
    // on at 2 s where PDM-Lite is 11, and none of the stream's gaps (free 1.0-1.9 s at the line) fitted.
    auto bend_cap = [&](double s) {
        double v = v_top;
        for (size_t i = 0; i < p.bend_s.size(); ++i)
            if (p.bend_s[i] >= s - 1.0) {
                const double vb = bend_speed(v_top, p.bend_kappa[i], opt.merge_a_lat);
                v = std::min(v, std::sqrt(vb * vb + 2.0 * no.a_ref * std::max(0.0, p.bend_s[i] - s)));
            }
        return v;
    };
    auto run = [&](double t_hold, double v_c) {
        std::vector<double> front{p.front0};
        double v = p.v0, f = p.front0;
        const double launch = p.v0 < 0.5 ? 0.2 : 0.0;
        for (double t = 0.0; t < T_CHECK; t += DT) {
            double cap = bend_cap(f - 0.5 * length);
            if (t < t_hold) cap = std::min(cap, v_c);
            if (t < launch) cap = 0.0;
            const double a = 0.95 * cs.params.a_drive(v);
            v = std::max(0.0, std::min(v + a * DT, std::max(cap, v - 1.5 * no.a_ref * DT)));
            if (t < t_hold && f + v * DT > s_gate) v = std::max(0.0, (s_gate - f) / DT);
            f += v * DT;
            front.push_back(f);
        }
        return front;
    };
    auto front_at = [&](const std::vector<double>& front, double t) {
        const double i = std::clamp(t / DT, 0.0, static_cast<double>(front.size() - 1));
        const size_t k = static_cast<size_t>(i);
        return k + 1 < front.size() ? front[k] + (i - k) * (front[k + 1] - front[k]) : front.back();
    };
    auto drive_from = [&](double t_hold) {  // rolling as the gate opens, if it can be
        if (t_hold <= 0.0) return run(0.0, v_top);
        double lo = 0.0, hi = v_top;
        for (int i = 0; i < 12; ++i) {
            const double mid = 0.5 * (lo + hi);
            (front_at(run(t_hold, mid), t_hold) <= s_gate + 1e-6 ? lo : hi) = mid;
        }
        return run(t_hold, lo);
    };
    auto own_box = [&](double front) {  // on the path, where its front is
        const Pose q = p.route->pose(front - 0.5 * length);
        return Box{q.x, q.y, 0.5 * length + OWN_END, 0.5 * cs.body.width + OWN_SIDE, q.psi};
    };
    // A gap the ego has started across is kept on half the buffer (a start called off leaves it standing in their path).
    auto buffer_of = [&](const Way& w) {
        return going && (last_id == 0 || last_id == w.x->id) ? 0.5 * opt.cross_buffer : opt.cross_buffer;
    };
    std::vector<std::string> why;
    auto judge = [&](const std::vector<double>& front) {
        for (const Way& w : ways) {
            const double b = buffer_of(w);
            for (const Spot& sp : w.spots)
                for (double t = std::max(0.0, sp.t - b - HALF_STEP); t <= sp.t + b + HALF_STEP + 1e-9; t += 0.05)
                    // not where it stands now (waiting there is base's too: at the line, its grown box and those of
                    // the cars passing 2-3 m ahead touched, and no hold was ever clear)
                    if (front_at(front, t) > p.front0 + 0.2 && overlap(own_box(front_at(front, t)), sp.box)) {
                        char buf[160];
                        std::snprintf(buf, sizeof buf, "%d at %.2f s, %.1f m on, the ego's front %.1f m on at %.2f s",
                                      w.x->id, sp.t, sp.s - p.front0, front_at(front, t) - p.front0, t);
                        why.push_back(buf);
                        if (why.size() > 2) why.erase(why.begin());  // the last holds tried before the one taken
                        return false;
                    }
        }
        return true;
    };

    // past where it could stop short of their path, the ego goes on with the gap it chose
    const bool in_already = p.front0 + p.v0 * p.v0 / (2.0 * std::fabs(no.a_min)) > s_gate + 0.5;
    std::vector<double> holds{0.0};
    for (const Way& w : ways)
        for (double lag = -0.5; lag < 1.05; lag += 0.1) {
            const double h = w.spots.back().t + lag;
            if (h > 0.0) holds.push_back(h);
        }
    std::sort(holds.begin(), holds.end());
    double t_hold = -1.0;
    for (double h : holds)
        if (judge(drive_from(h))) {
            t_hold = h;
            break;
        }
    if (t_hold < 0.0 && going && in_already) {
        t_hold = 0.0;
        out.notes.push_back("cross: no stopping short of their path now, going on");
    }
    if (t_hold < 0.0) {
        std::string note = "cross: no gap clear";
        for (const std::string& w : why) note += "; " + w;
        out.notes.push_back(note);
        return;
    }
    const std::vector<double> front = drive_from(t_hold);
    // which it goes ahead of: its centre past theirs when they are on the path
    int first_ahead = 0;
    double t_first = INF;
    for (const Way& w : ways)
        if (front_at(front, w.spots.front().t) - 0.5 * length > w.spots.front().s && w.spots.front().t < t_first)
            t_first = w.spots.front().t, first_ahead = w.x->id;

    Candidate c;
    c.label = first_ahead != 0 ? "cross:" + std::to_string(first_ahead) : "cross";
    c.problem = base;
    Problem& pr = c.problem;
    pr.v_max = v_top;
    pr.a_lat_ref = opt.merge_a_lat;
    pr.a_max = INF;  // full throttle, as PDM-Lite goes
    pr.w_accel = 0.02, pr.w_lat = 0.05;  // and not held back by the cost (route 10: it trailed the drive judged on)
    // the speed asked for is the drive the gap was judged on (not SimLingo's, which waits)
    pr.v_ref_t.clear(), pr.v_ref.clear();
    for (size_t i = 0; i + 1 < front.size(); ++i) {
        pr.v_ref_t.push_back((i + 0.5) * DT);
        pr.v_ref.push_back(std::max(i * DT < t_hold ? 0.0 : 0.5 * v_top, (front[i + 1] - front[i]) / DT));
    }
    // at each moment one is on the path, the plan within GIVE of the judged drive: its front no further on where the
    // drive is behind the crosser (rolled through as it moves on), its rear no less far where the drive is past it
    for (const Way& w : ways) {
        const double b = buffer_of(w) + HALF_STEP;
        for (const Spot& sp : w.spots) {
            const bool past = front_at(front, sp.t) - 0.5 * length > sp.s;
            // no give where the drive stands: each plan's give, taken, moved the next plan's standing point on, and
            // the car crept into the stream's lane at 0.5-1 m/s (cross2_r10, 15-22 s: a car's centre 0.5 m behind
            // the ego's front)
            const double ahead_at = front_at(front, sp.t + b);
            const double give = ahead_at > p.front0 + 0.2 ? GIVE : 0.0;
            Gate g{past ? front_at(front, std::max(0.0, sp.t - b)) - length - GIVE : ahead_at + give,
                   std::max(0.0, sp.t - b), sp.t + b, !past};
            g.taper = false;
            pr.gates.push_back(g);
        }
    }
    stay_behind(pr, p);
    wait_for_mergers(pr, p, opt);
    char buf[120];
    std::snprintf(buf, sizeof buf, "cross: clear, from %.1f s, ahead of %d", t_hold, first_ahead);
    std::string note = buf;
    for (const std::string& w : why) note += "; sooner: " + w;
    out.notes.push_back(note);
    if (static_cast<int>(out.candidates.size()) >= opt.pool) return;
    out.cross = c.label;
    out.candidates.push_back(std::move(c));
    // waiting is over once the ego is on its way across and cannot stop short
    if (going && in_already) out.candidates.front().hopeless = true;
}

std::vector<double> pass_profile(const std::vector<Gate>& gates, const std::vector<Chaser>& chasers,
                                 const std::vector<double>& times, double chaser_margin) {
    const int N = static_cast<int>(times.size()) - 1;
    std::vector<double> floor(N, -INF);
    for (int k = 0; k < N; ++k) {
        const double t = times[k + 1];
        for (const Gate& gt : gates)
            if (!gt.before && gt.t_from <= t && t <= gt.t_to) floor[k] = std::max(floor[k], gt.s);
        for (const Chaser& c : chasers)
            if (t >= c.t_from) floor[k] = std::max(floor[k], sample(c.t, c.s_front, t) + chaser_margin);
    }
    return floor;
}

std::vector<SX> pass_rows(const Stage& g) {  // the rear past the passing gates
    const UrbanNMPC& n = *g.nmpc;
    const SX rear = g.x(IS) - 0.5 * n.body.length - n.body.cg_to_center;
    return {g.floor_on * (g.floor_s - rear) + (g.floor_on - 1.0)};
}

}  // namespace ex
