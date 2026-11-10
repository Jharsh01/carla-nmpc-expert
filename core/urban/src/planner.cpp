#include "urban/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace ex {

namespace {

double sample_at(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const size_t i = std::upper_bound(xs.begin(), xs.end(), x) - xs.begin();
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
}

}  // namespace

struct Planner::Impl {
    Sensor sensor;
    Constraints constraints;
    std::vector<std::unique_ptr<UrbanNMPC>> pool;
    std::vector<std::string> slot_label;
    std::string last_label;
};

Planner::Planner(Path route_, VehicleParams params, Body body_, UrbanNMPC::Options nmpc_options, Options o)
    : route(std::move(route_)), body(body_), opt(o),
      impl_(std::make_unique<Impl>(Impl{Sensor(body_, o), Constraints(body_, params, nmpc_options, o), {}, {}, {}})) {
    for (int i = 0; i < o.pool; ++i) {
        impl_->pool.push_back(std::make_unique<UrbanNMPC>(route, params, body, nmpc_options));
        impl_->slot_label.emplace_back();
    }
}

bool PathIntent::update(const std::vector<double>& s, const std::vector<double>& e) {
    double largest = -1.0;
    for (size_t i = 0; i < s.size() && i < e.size(); ++i)
        if (s[i] >= from) largest = std::max(largest, std::abs(e[i]));
    if (largest < 0.0) return manoeuvre;  // too short to tell
    offset = largest;
    if (offset > enter) manoeuvre = true;
    else if (offset < exit) manoeuvre = false;
    return manoeuvre;
}

Planner::~Planner() = default;

void Planner::reset() {
    Impl& m = *impl_;
    for (auto& n : m.pool) n->reset();
    std::fill(m.slot_label.begin(), m.slot_label.end(), "");
    m.last_label.clear();
    m.sensor.reset();
}

void Planner::set_route(Path next, Pose old_frame) {
    for (auto& n : impl_->pool) n->set_route(next, old_frame);
    route = std::move(next);
}

Decision Planner::step(const Scene& sc) {
    Impl& m = *impl_;
    const auto start = std::chrono::steady_clock::now();
    Decision out;
    const UrbanNMPC& nmpc = *m.pool[0];

    // ---- sense the scene, and build the candidates from it

    const Percept p = m.sensor.sense(sc, route, nmpc);
    Constraints::Result built = m.constraints.build(p, sc, m.last_label);
    out.notes = p.notes;
    out.notes.insert(out.notes.end(), built.notes.begin(), built.notes.end());
    std::vector<Candidate>& cands = built.candidates;
    const bool committed = built.committed;
    const double s0 = p.s0;
    auto section = [&](double s) -> const LaneSection& { return p.section(s); };

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
    int best = -1, best_rank = -1;
    for (size_t i = 0; i < cands.size(); ++i) {
        Candidate& c = cands[i];
        // within the hard constraints (the headway, the last group, is spacing, not contact)
        const double slack = *std::max_element(c.plan.slack_max.begin(), c.plan.slack_max.end() - 1);
        const bool ok = slack <= opt.slack_ok;
        c.score = score(c) * (c.label == m.last_label ? opt.hysteresis : 1.0);
        const int rank = (c.plan.success ? 2 : 0) + (ok ? 1 : 0);  // solved first: a failed plan is not to be driven
        if (rank > best_rank || (rank == best_rank && c.score < cands[best].score)) best = static_cast<int>(i), best_rank = rank;
    }
    // A clear way round a blocker is taken over waiting behind it, and kept once out in the other lane: the scores
    // weigh speeds against different references (base keeps SimLingo's, which stops while it waits), so on route 4
    // the choice flipped every plan or two and the car stopped half across.
    const auto progress = [&](const Plan& pl) { return pl.x(nmpc.N, IS) - s0; };
    for (size_t i = 0; i < cands.size(); ++i) {
        const Candidate& c = cands[i];
        if (c.label.rfind("borrow_", 0) != 0 || !c.plan.success || static_cast<int>(i) == best) continue;
        const double slack = *std::max_element(c.plan.slack_max.begin(), c.plan.slack_max.end() - 1);
        if (slack > opt.slack_ok) continue;
        const bool kept = committed && c.label == m.last_label;  // out in that lane, going on
        if (kept || progress(c.plan) > progress(cands[best].plan) + opt.pass_progress) {
            best = static_cast<int>(i);
            out.notes.push_back(kept ? "borrow: out in the other lane, going on" : "borrow: clear, going round");
        }
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
