#include "urban/urban_nmpc.hpp"

#include <algorithm>
#include <array>
#include <casadi/casadi.hpp>
#include <casadi/config.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <functional>
#include <stdexcept>

#include "urban/integrate.hpp"

namespace ex {

using casadi::SX;
static constexpr int NS = 6;  // slacks per step, one per constraint group

namespace {

std::vector<double> make_steps(const UrbanNMPC::Options& o) {
    if (!(o.h > 0.0) || !(o.horizon > 0.0) || o.h_fine < 0.0 || o.fine < 0.0)
        throw std::invalid_argument("UrbanNMPC: the steps and the horizon must be positive");
    const int n_fine = o.h_fine > 0.0 ? static_cast<int>(std::lround(std::min(o.fine, o.horizon) / o.h_fine)) : 0;
    const int n = static_cast<int>(std::lround((o.horizon - n_fine * o.h_fine) / o.h));
    std::vector<double> steps(n_fine, o.h_fine);
    steps.insert(steps.end(), std::max(0, n), o.h);
    if (steps.empty()) throw std::invalid_argument("UrbanNMPC: a horizon shorter than one step");
    return steps;
}

std::vector<double> start_times(const std::vector<double>& steps) {
    std::vector<double> times{0.0};
    for (double dt : steps) times.push_back(times.back() + dt);
    return times;
}

// The step of `times` (ascending, the last the horizon's end) that t falls in; N from the end on.
int step_at(const std::vector<double>& times, double t) {
    const double eps = 1e-9;  // a time on a step's start, give or take rounding, is in it
    return std::max(0, static_cast<int>(std::upper_bound(times.begin(), times.end(), t + eps) - times.begin()) - 1);
}

// Linear interpolation in (xs, ys), held beyond the ends; xs ascending.
double sample(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (xs.empty() || xs.size() != ys.size()) throw std::invalid_argument("UrbanNMPC: samples of unequal length");
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const size_t i = std::upper_bound(xs.begin(), xs.end(), x) - xs.begin();
    const double w = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return (1.0 - w) * ys[i - 1] + w * ys[i];
}

// One step of the horizon as the cost and constraint terms see it.
struct Stage {
    SX x, u, slack;  // x: the state at the end of the step
    SX v_ref, e_ref, lo, hi, v_max, mu_g;
    std::vector<std::array<SX, 4>> movers;  // s, e_y, clearance radius (ego circle + mover + margin), 1 (0: unused)
    SX lead_s, lead_v, lead_on, cap_s, cap_on, floor_s, floor_on;
    double terminal;  // 1 at the last step, else 0
    const UrbanNMPC* nmpc;
};

SX lateral_acceleration(const SX& x, double L) { return sq(x(IV)) * tan(x(IDELTA)) / L; }

struct CostTerm {
    const char* name;
    double UrbanNMPC::Options::*weight;
    std::function<SX(const Stage&)> f;
};
const std::vector<CostTerm>& cost_terms() {
    static const std::vector<CostTerm> terms = {
        {"speed", &UrbanNMPC::Options::w_speed, [](const Stage& g) { return sq(g.x(IV) - g.v_ref); }},
        {"lane", &UrbanNMPC::Options::w_lane, [](const Stage& g) { return sq(g.x(IE) - g.e_ref); }},
        {"heading", &UrbanNMPC::Options::w_heading, [](const Stage& g) { return sq(g.x(IPSI)); }},
        {"accel", &UrbanNMPC::Options::w_accel, [](const Stage& g) { return sq(g.x(IA)); }},
        {"jerk", &UrbanNMPC::Options::w_jerk, [](const Stage& g) { return sq(g.u(IJERK)); }},
        {"rate", &UrbanNMPC::Options::w_rate, [](const Stage& g) { return sq(g.u(IRATE)); }},
        {"lat", &UrbanNMPC::Options::w_lat, [](const Stage& g) { return sq(lateral_acceleration(g.x, g.nmpc->p.L())); }},
        {"slack", &UrbanNMPC::Options::w_slack,  // the hard groups: all but the headway
         [](const Stage& g) { const SX s = g.slack(casadi::Slice(0, NS - 1)); return sum1(s) + sumsqr(s); }},
        {"headway", &UrbanNMPC::Options::w_headway, [](const Stage& g) { return g.slack(NS - 1) + sq(g.slack(NS - 1)); }},
    };
    return terms;
}

// Each group softened by one slack per step: every expression <= that slack. A switched-off row (on = 0) is the
// constant -1, which costs the solver nothing.
struct ConstraintGroup {
    const char* name;
    std::function<std::vector<SX>(const Stage&)> f;
};
const std::vector<ConstraintGroup>& constraint_groups() {
    static const std::vector<ConstraintGroup> groups = {
        {"corridor",
         [](const Stage& g) {  // the body's front and rear ends within the band
             const Body& b = g.nmpc->body;
             std::vector<SX> out;
             for (double arm : {0.5 * b.length - b.cg_to_center, -(0.5 * b.length + b.cg_to_center)}) {
                 const SX e_end = g.x(IE) + arm * sin(g.x(IPSI));
                 out.push_back(e_end + 0.5 * b.width + g.nmpc->opt.edge_margin - g.hi);
                 out.push_back(g.lo - (e_end - 0.5 * b.width - g.nmpc->opt.edge_margin));
             }
             return out;
         }},
        {"movers",
         [](const Stage& g) {  // each of the body's three circles outside each mover's clearance circle
             const Body& b = g.nmpc->body;
             const SX c = cos(g.x(IPSI)), sn = sin(g.x(IPSI));
             std::array<SX, 3> sc, ec;  // the circles' centres, once for all movers
             for (int i = 0; i < 3; ++i) {
                 const double d = -b.cg_to_center + (i - 1) * b.length / 3.0;
                 sc[i] = g.x(IS) + d * c, ec[i] = g.x(IE) + d * sn;
             }
             std::vector<SX> out;
             for (const auto& [s, e, R, on] : g.movers)
                 for (int i = 0; i < 3; ++i) out.push_back(on * (1.0 - (sq(sc[i] - s) + sq(ec[i] - e)) / sq(R)) + (on - 1.0));
             return out;
         }},
        {"ahead",
         [](const Stage& g) {  // the front behind the lead (bodies apart), and behind the stop gates
             // (at the last step with the distance to stop in)
             const UrbanNMPC& n = *g.nmpc;
             const SX front = g.x(IS) + 0.5 * n.body.length - n.body.cg_to_center;
             const SX stop = g.terminal * sq(g.x(IV)) / (2.0 * n.opt.a_stop);
             return std::vector<SX>{g.lead_on * (front + n.opt.gap_min - g.lead_s) + (g.lead_on - 1.0),
                                    g.cap_on * (front + stop - g.cap_s) + (g.cap_on - 1.0)};
         }},
        {"behind",
         [](const Stage& g) {  // the rear past the passing gates
             const UrbanNMPC& n = *g.nmpc;
             const SX rear = g.x(IS) - 0.5 * n.body.length - n.body.cg_to_center;
             return std::vector<SX>{g.floor_on * (g.floor_s - rear) + (g.floor_on - 1.0)};
         }},
        {"grip",
         [](const Stage& g) {  // within the friction circle, and the speed limit
             const SX a_lat = lateral_acceleration(g.x, g.nmpc->p.L());
             return std::vector<SX>{(sq(g.x(IA)) + sq(a_lat)) / sq(g.mu_g) - 1.0, g.x(IV) - g.v_max};
         }},
        {"headway",  // last: softer than the others (w_headway)
         [](const Stage& g) {  // at the last step, room to stop behind the lead were it to brake: spacing, which a
             // cut-in may take away, not contact. (The time gap is in the speed asked for, not here: a constraint on
             // position, priced high, pays the plan to weave, as steering at an angle shortens the progress along
             // the route.)
             const UrbanNMPC& n = *g.nmpc;
             const SX front = g.x(IS) + 0.5 * n.body.length - n.body.cg_to_center;
             const SX stop = (sq(g.x(IV)) - sq(g.lead_v)) / (2.0 * n.opt.a_stop);
             return std::vector<SX>{g.terminal * g.lead_on * (front + n.opt.gap + stop - g.lead_s) + (g.terminal * g.lead_on - 1.0)};
         }},
    };
    return groups;
}

}  // namespace

const std::vector<std::string>& UrbanNMPC::groups() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> out;
        for (const auto& g : constraint_groups()) out.push_back(g.name);
        return out;
    }();
    return names;
}

struct UrbanNMPC::Impl {
    casadi::Function solver;
    std::vector<double> lbw, ubw, lbg, ubg;
    int n_w = 0;
    // previous solution, planned at time t, to start the next solve from
    std::vector<double> w0;
    double t = 0.0;
    bool warm = false;

    int x_at(int k) const { return k * (NX + NU + NS); }
    int u_at(int k) const { return x_at(k) + NX; }
};

UrbanNMPC::UrbanNMPC(Path route_, VehicleParams params, Body body_, Options o)
    : route(std::move(route_)),
      p(params),
      body(body_),
      opt(o),
      steps(make_steps(o)),
      N(static_cast<int>(steps.size())),
      times(start_times(steps)),
      a_min(std::isnan(o.a_min) ? -std::min(params.F_brake_max / params.m, o.mu * params.g) : o.a_min),
      impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    if (static_cast<int>(constraint_groups().size()) != NS)
        throw std::logic_error("UrbanNMPC: NS must be the number of constraint groups");
    const int n_m = o.n_movers;
    const int n_terms = static_cast<int>(cost_terms().size());

    // Parameters, set at every solve, in this order (solve() fills them the same way).
    const SX x_init = SX::sym("x_init", NX);
    const SX kappa = SX::sym("kappa", N), v_ref = SX::sym("v_ref", N), e_ref = SX::sym("e_ref", N);
    const SX lo = SX::sym("lo", N), hi = SX::sym("hi", N);
    const SX v_max = SX::sym("v_max"), mu_g = SX::sym("mu_g");
    const SX weight = SX::sym("weight", n_terms);
    const SX mov_s = SX::sym("mov_s", n_m, N), mov_e = SX::sym("mov_e", n_m, N), mov_R = SX::sym("mov_R", n_m, N);
    const SX mov_on = SX::sym("mov_on", n_m);
    const SX lead_s = SX::sym("lead_s", N), lead_v = SX::sym("lead_v", N), lead_on = SX::sym("lead_on", N);
    const SX cap_s = SX::sym("cap_s", N), cap_on = SX::sym("cap_on", N);
    const SX floor_s = SX::sym("floor_s", N), floor_on = SX::sym("floor_on", N);
    const SX par = SX::vertcat({x_init, kappa, v_ref, e_ref, lo, hi, v_max, mu_g, weight, vec(mov_s), vec(mov_e),
                                vec(mov_R), mov_on, lead_s, lead_v, lead_on, cap_s, cap_on, floor_s, floor_on});

    // decision variables stage by stage, [x_0 u_0 s_0  x_1 u_1 s_1 ... x_N], each constraint and cost term on one
    // stage's variables only, as FATROP needs
    std::vector<SX> w, g;
    std::vector<bool> equality;
    SX J = 0.0;
    std::vector<SX> xs;
    for (int k = 0; k <= N; ++k) xs.push_back(SX::sym("x_" + std::to_string(k), NX));
    const double L = p.L(), b = p.b;
    for (int k = 0; k < N; ++k) {
        const SX xk = xs[k], uk = SX::sym("u_" + std::to_string(k), NU), sk = SX::sym("s_" + std::to_string(k), NS);
        w.insert(w.end(), {xk, uk, sk});
        const SX kap = kappa(k);
        auto f = [&](const SX& z, const SX& v) {
            const SX beta = atan(b * tan(z(IDELTA)) / L);
            const SX s_dot = z(IV) * cos(z(IPSI) + beta) / (1.0 - kap * z(IE));
            return SX::vertcat({s_dot, z(IV) * sin(z(IPSI) + beta), z(IV) * cos(beta) * tan(z(IDELTA)) / L - kap * s_dot,
                                z(IA), v(IJERK), v(IRATE)});
        };
        const SX nxt = rk4_step(f, xk, uk, steps[k]);
        g.push_back(xs[k + 1] - nxt);  // gap closing first
        equality.insert(equality.end(), NX, true);
        if (k == 0) {
            g.push_back(xk - x_init);
            equality.insert(equality.end(), NX, true);
        }

        Stage stage{nxt, uk, sk, v_ref(k), e_ref(k), lo(k), hi(k), v_max, mu_g, {},
                    lead_s(k), lead_v(k), lead_on(k), cap_s(k), cap_on(k), floor_s(k), floor_on(k),
                    k == N - 1 ? 1.0 : 0.0, this};
        for (int j = 0; j < n_m; ++j) stage.movers.push_back({mov_s(j, k), mov_e(j, k), mov_R(j, k), mov_on(j)});
        // each term weighted by the step's length, the cost a sum over time whatever the steps
        for (int i = 0; i < n_terms; ++i) J += (steps[k] / o.h) * weight(i) * cost_terms()[i].f(stage);
        std::vector<SX> in;
        for (int i = 0; i < NS; ++i)
            for (const SX& c : constraint_groups()[i].f(stage)) in.push_back(c - sk(i));
        g.push_back(SX::vertcat(in));
        equality.insert(equality.end(), in.size(), false);
    }
    w.push_back(xs[N]);

    for (bool e : equality) {
        m.lbg.push_back(e ? 0.0 : -INF);
        m.ubg.push_back(0.0);
    }
    for (int k = 0; k <= N; ++k) {
        const bool bounded = k > 0;
        for (int i = 0; i < NX; ++i) {
            double lb = -INF, ub = INF;
            // not v >= 0: at standstill that bound would be active over many stages in a row, all tied together by
            // the dynamics, and the multipliers degenerate (FATROP then crawls to its iteration limit). A stop gate
            // and the speed cost already hold the plan at rest without it; this only keeps reversing out of reach
            if (bounded && i == IV) lb = -1.0;
            if (bounded && i == IA) lb = a_min, ub = o.a_max;
            if (bounded && i == IDELTA) lb = -p.delta_max, ub = p.delta_max;
            m.lbw.push_back(lb);
            m.ubw.push_back(ub);
        }
        if (k == N) break;
        m.lbw.insert(m.lbw.end(), {-o.jerk_max, -p.delta_rate_max});
        m.ubw.insert(m.ubw.end(), {o.jerk_max, p.delta_rate_max});
        m.lbw.insert(m.lbw.end(), NS, 0.0);
        m.ubw.insert(m.ubw.end(), NS, INF);
    }
    m.n_w = static_cast<int>(m.lbw.size());

    // common subexpressions merged: the stage terms rebuild the same trigonometry many times over
    const casadi::SXDict nlp = {{"x", SX::vertcat(w)}, {"p", par}, {"f", cse(J)}, {"g", cse(SX::vertcat(g))}};
    casadi::Dict opts = {{"print_time", false}, {"structure_detection", "auto"}, {"equality", equality}};
    opts["fatrop"] = casadi::Dict{{"print_level", std::getenv("URBAN_PRINT") ? 10 : 0}, {"max_iter", o.max_iter}, {"tol", o.tol},
                                  {"acceptable_tol", o.acceptable_tol}, {"acceptable_iter", o.acceptable_iter}};
    std::string cached, dir;
    if (o.jit) {
        opts["jit"] = true;
        opts["compiler"] = "shell";
        casadi::Dict jit_options{{"flags", std::vector<std::string>{"-O1"}}, {"verbose", false}};
        if (!o.cache_dir.empty()) {
            dir = std::filesystem::absolute(o.cache_dir).string();
            // the key: the problem as built (its expressions carry every option, parameter and the body), the solver
            // options and the CasADi version
            std::ostringstream key;
            key << SX::vertcat(w) << '|' << par << '|' << J << '|' << SX::vertcat(g) << '|' << casadi::Dict(opts) << '|'
                << CASADI_VERSION_STRING;
            const std::string name = "urban_nmpc_" + std::to_string(std::hash<std::string>()(key.str()));
            cached = dir + "/" + name + ".casadi";
            if (std::ifstream(cached).good()) {
                m.solver = casadi::Function::load(cached);
                return;
            }
            // the compiled library kept in the cache under a fixed name, and the saved solver only pointing at it
            // ("embed" would carry the binary, but loading writes it out to the working directory and leaves it)
            opts["jit_name"] = name;
            opts["jit_temp_suffix"] = false;
            opts["jit_serialize"] = "link";
            jit_options["directory"] = dir + "/";
            jit_options["cleanup"] = false;  // the compiled library stays
            opts["jit_cleanup"] = false;  // and the generated source is removed below (a loaded solver would retry it)
        }
        opts["jit_options"] = jit_options;
    }
    if (cached.empty()) {
        m.solver = casadi::nlpsol("urban_nmpc", "fatrop", nlp, opts);
    } else {
        // CasADi writes the generated source to the working directory but removes it from `directory`, so it is built
        // from inside the cache (the working directory is the process's: this happens once, at construction)
        const auto here = std::filesystem::current_path();
        std::filesystem::current_path(dir);
        try {
            m.solver = casadi::nlpsol("urban_nmpc", "fatrop", nlp, opts);
        } catch (...) {
            std::filesystem::current_path(here);
            throw;
        }
        std::filesystem::current_path(here);
        std::filesystem::remove(cached.substr(0, cached.size() - std::string(".casadi").size()) + ".c");
    }
    if (!cached.empty()) {  // written aside and moved into place, so a process loading it never sees half a file
        const std::string part = cached + ".part" + std::to_string(::getpid());
        m.solver.save(part);
        std::rename(part.c_str(), cached.c_str());
    }
}

UrbanNMPC::~UrbanNMPC() = default;

void UrbanNMPC::reset() {
    impl_->warm = false;
    s_last_ = NAN;
}

void UrbanNMPC::set_route(Path next, Pose old_frame) {
    Impl& m = *impl_;
    if (m.warm) {  // each stage of the previous plan: where it is in the new frame, then on the new route
        const double c = std::cos(old_frame.psi), sn = std::sin(old_frame.psi);
        for (int k = 0; k <= N; ++k) {
            double* z = m.w0.data() + m.x_at(k);
            const Pose q = route.pose(z[IS], z[IE]);
            const double x = old_frame.x + c * q.x - sn * q.y, y = old_frame.y + sn * q.x + c * q.y;
            const double psi = route.pose(z[IS]).psi + z[IPSI] + old_frame.psi;
            const auto [s, e_y] = next.project(x, y);
            z[IS] = s;
            z[IE] = e_y;
            z[IPSI] = wrap_angle(psi - next.pose(s).psi);
        }
    }
    route = std::move(next);
    s_last_ = NAN;
}

Eigen::Matrix<double, NX, 1> UrbanNMPC::path_state(const Ego& ego) const {
    // near where the car was last (a long route is not searched whole, nor mistaken where it passes near itself)
    const auto [s, e_y] = std::isnan(s_last_) ? route.project(ego.x, ego.y) : route.project(ego.x, ego.y, s_last_ - 30.0, s_last_ + 60.0);
    s_last_ = s;
    Eigen::Matrix<double, NX, 1> z;
    z << s, e_y, wrap_angle(ego.psi - route.pose(s).psi), ego.v, ego.a, ego.delta;
    return z;
}

Plan UrbanNMPC::solve(const Problem& pr) {
    Impl& m = *impl_;
    const int n_m = opt.n_movers;
    const Eigen::Matrix<double, NX, 1> z0 = path_state(pr.ego);
    const double s0 = z0[IS];
    const Corridor& c = pr.corridor;
    if (c.s.empty() || c.lo.size() != c.s.size() || c.hi.size() != c.s.size() || c.ref.size() != c.s.size())
        throw std::invalid_argument("UrbanNMPC: the corridor needs s, lo, hi and ref of the same, nonzero length");

    // The starting point: at first the measured state carried on at its speed; after that the previous plan moved
    // on by the time since it was planned, each stage taking the previous plan's values at times[k] + since
    // (interpolated), the stages past its horizon its last values, coasting on at its last speed.
    if (m.warm && opt.warm_start) {
        const double since = std::max(0.0, pr.t - m.t);
        const std::vector<double> old = m.w0;
        auto value = [&](int j, int i, bool state) {  // stage j's state or input entry i, j past the end held
            const int last = state ? N : N - 1;
            return old[(state ? m.x_at(std::min(j, last)) : m.u_at(std::min(j, last))) + i];
        };
        for (int k = 0; k <= N; ++k) {
            const double t = times[k] + since;
            int j = step_at(times, t);
            double a = 0.0;
            if (j >= N) j = N, a = 0.0;
            else a = (t - times[j]) / steps[j];
            for (int i = 0; i < NX; ++i) {
                double v = a > 0.0 ? (1.0 - a) * value(j, i, true) + a * value(j + 1, i, true) : value(j, i, true);
                if (i == IS && t > times[N]) v += value(N, IV, true) * (t - times[N]);
                m.w0[m.x_at(k) + i] = v;
            }
            if (k == N) break;
            for (int i = 0; i < NU + NS; ++i) m.w0[m.u_at(k) + i] = value(std::min(j, N - 1), i, false);
        }
        std::copy(z0.data(), z0.data() + NX, m.w0.begin() + m.x_at(0));
    } else {
        m.w0.assign(m.n_w, 0.0);
        for (int k = 0; k <= N; ++k) {
            Eigen::Matrix<double, NX, 1> guess = z0;
            guess[IS] = s0 + z0[IV] * times[k];
            if (k > 0) guess[IPSI] = guess[IA] = guess[IDELTA] = 0.0;
            std::copy(guess.data(), guess.data() + NX, m.w0.begin() + m.x_at(k));
        }
    }
    std::vector<double> s_plan(N + 1), e_plan(N + 1);
    for (int k = 0; k <= N; ++k) s_plan[k] = m.w0[m.x_at(k) + IS], e_plan[k] = m.w0[m.x_at(k) + IE];

    // movers: the n_m nearest to the plan at any step
    const Body& bd = body;
    const double r_ego = std::hypot(bd.length / 6.0, 0.5 * bd.width);
    struct Slot {
        double distance;
        std::vector<double> s, e, R;
    };
    std::vector<Slot> slots;
    for (const Mover& mv : pr.movers) {
        Slot slot{INF, std::vector<double>(N), std::vector<double>(N), std::vector<double>(N)};
        for (int k = 0; k < N; ++k) {
            const double t = times[k + 1];
            slot.s[k] = sample(mv.t, mv.s, t);
            slot.e[k] = sample(mv.t, mv.e_y, t);
            slot.R[k] = r_ego + mv.radius + mv.growth * t + (std::isnan(mv.margin) ? opt.margin : mv.margin);
            slot.distance = std::min(slot.distance, std::hypot(slot.s[k] - s_plan[k + 1], slot.e[k] - e_plan[k + 1]) - slot.R[k]);
        }
        slots.push_back(std::move(slot));
    }
    std::stable_sort(slots.begin(), slots.end(), [](const Slot& a, const Slot& b) { return a.distance < b.distance; });
    if (static_cast<int>(slots.size()) > n_m) slots.resize(n_m);
    const int n_used = static_cast<int>(slots.size());

    std::vector<double> lead(N, INF), lead_speed(N, 0.0), cap(N, INF), floor(N, -INF);
    for (int k = 0; k < N; ++k) {
        const double t = times[k + 1];
        for (const Lead& l : pr.leads) {  // the nearest at each step, and its speed
            const double s = sample(l.t, l.s_rear, t);
            if (s < lead[k])
                lead[k] = s, lead_speed[k] = std::max(0.0, (sample(l.t, l.s_rear, t + 0.05) - sample(l.t, l.s_rear, t - 0.05)) / 0.1);
        }
        for (const Gate& gt : pr.gates)
            if (gt.t_from <= t && t <= gt.t_to) {
                if (gt.before) cap[k] = std::min(cap[k], gt.s);
                else floor[k] = std::max(floor[k], gt.s);
            }
        // never asked to go back: a lead already nearer than the gap, or a stop line the front is already over,
        // holds the car where it is (a plan that reverses to restore them is not one the car can drive, and at
        // standstill it leaves the solver crawling)
        const double front_now = s0 + 0.5 * body.length - body.cg_to_center;
        if (std::isfinite(lead[k])) lead[k] = std::max(lead[k], front_now + opt.gap_min);
        if (std::isfinite(cap[k])) cap[k] = std::max(cap[k], front_now);
    }
    // in the order of the parameters in the constructor
    std::vector<double> par(z0.data(), z0.data() + NX);
    for (int k = 0; k < N; ++k) {  // mean curvature over each step
        double sum = 0.0;
        for (int i = 0; i < 5; ++i) sum += route.curvature(s_plan[k] + (s_plan[k + 1] - s_plan[k]) * i / 4.0);
        par.push_back(sum / 5.0);
    }
    const double front_arm = 0.5 * body.length - body.cg_to_center;
    for (int k = 0; k < N; ++k) {  // what is asked for, tapered to the bends, stops and leads ahead of the shifted plan
        double v = std::min(pr.v_max, pr.v_ref.empty() ? pr.v_max : sample(pr.v_ref_t, pr.v_ref, times[k + 1]));
        const double kappa_k = std::fabs(par[NX + k]);
        if (kappa_k > 1e-4) v = std::min(v, std::sqrt(opt.a_lat_ref / kappa_k));
        const double front = s_plan[k + 1] + front_arm;
        if (std::isfinite(cap[k])) v = std::min(v, std::sqrt(2.0 * opt.a_ref * std::max(0.0, cap[k] - front)));
        if (std::isfinite(lead[k])) {  // the time gap, and the envelope to the lead's speed
            const double room = std::max(0.0, lead[k] - opt.gap - front);
            v = std::min({v, room / opt.time_gap, std::sqrt(lead_speed[k] * lead_speed[k] + 2.0 * opt.a_ref * room)});
        }
        par.push_back(v);
    }
    for (const auto* ys : {&c.ref, &c.lo, &c.hi})
        for (int k = 0; k < N; ++k) par.push_back(sample(c.s, *ys, s_plan[k + 1]));
    par.push_back(pr.v_max);
    par.push_back(opt.mu * p.g);
    for (const CostTerm& term : cost_terms()) par.push_back(opt.*term.weight);
    auto slot_values = [&](std::vector<double> Slot::*field, double unused) {
        for (int k = 0; k < N; ++k)
            for (int j = 0; j < n_m; ++j) par.push_back(j < n_used ? (slots[j].*field)[k] : unused);
    };
    slot_values(&Slot::s, s0 + 1e4);
    slot_values(&Slot::e, 0.0);
    slot_values(&Slot::R, 1.0);
    for (int j = 0; j < n_m; ++j) par.push_back(j < n_used ? 1.0 : 0.0);

    for (double v : lead) par.push_back(std::isfinite(v) ? v : 0.0);
    par.insert(par.end(), lead_speed.begin(), lead_speed.end());
    for (double v : lead) par.push_back(std::isfinite(v) ? 1.0 : 0.0);
    for (const auto* values : {&cap, &floor}) {
        for (double v : *values) par.push_back(std::isfinite(v) ? v : 0.0);
        for (double v : *values) par.push_back(std::isfinite(v) ? 1.0 : 0.0);
    }

    casadi::DMDict args = {{"p", par}, {"lbx", m.lbw}, {"ubx", m.ubw}, {"lbg", m.lbg}, {"ubg", m.ubg}, {"x0", m.w0}};
    const auto start = std::chrono::steady_clock::now();
    const casadi::DMDict sol = m.solver(args);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    solve_times.push_back(elapsed);
    const casadi::Dict st = m.solver.stats();

    Plan plan;
    plan.success = st.count("success") && st.at("success").as_bool();
    plan.status = st.count("return_status") && st.at("return_status").is_string() ? st.at("return_status").to_string()
                  : st.count("unified_return_status")                            ? st.at("unified_return_status").to_string()
                                                                                  : "";
    plan.iterations = st.count("iter_count") ? st.at("iter_count").to_int() : 0;
    plan.solve_time = elapsed;
    plan.cost = sol.at("f").scalar();
    plan.n_movers = n_used;
    if (!plan.success) ++failures;

    m.w0 = sol.at("x").get_elements();
    m.t = pr.t;
    m.warm = true;
    plan.times = times;
    plan.x.resize(N + 1, NX);
    plan.u.resize(N, NU);
    plan.slack_max.assign(NS, 0.0);
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < NX; ++i) plan.x(k, i) = m.w0[m.x_at(k) + i];
        if (k == N) break;
        for (int i = 0; i < NU; ++i) plan.u(k, i) = m.w0[m.u_at(k) + i];
        for (int i = 0; i < NS; ++i) plan.slack_max[i] = std::max(plan.slack_max[i], m.w0[m.u_at(k) + NU + i]);
    }
    return plan;
}

}  // namespace ex
