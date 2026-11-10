#include "nmpc/constraint/stop.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "nmpc/stage.hpp"

namespace ex {

using casadi::sq;

void StopRules::reset() {
    stopped_since_.clear();
    released_.clear();
    seen_.clear();
}

std::vector<Gate> StopRules::gates(const Percept& p, const Scene& sc, double a_ref, const ConstraintOptions& opt,
                                   std::vector<std::string>& notes) {
    std::vector<Gate> out;
    std::set<int> present;
    const double front0 = p.front0, v0 = p.v0;
    for (const StopLine& st : sc.stops) {
        present.insert(st.id);
        const double to_line = st.s - front0;
        if (to_line > opt.look_ahead || to_line < -2.0) continue;
        const double decel = v0 * v0 / (2.0 * std::max(0.1, to_line));
        switch (st.kind) {
            case StopLine::route_end: out.push_back({st.s}); break;
            case StopLine::stop_sign:
                if (released_.count(st.id)) break;
                if (v0 < 0.1 && to_line < 4.0) {
                    if (!stopped_since_.count(st.id)) stopped_since_[st.id] = sc.t;
                    if (sc.t - stopped_since_[st.id] >= opt.stop_hold) {
                        released_.insert(st.id);
                        notes.push_back("stop sign: held, going on");
                        break;
                    }
                }
                out.push_back({st.s});
                break;
            case StopLine::amber_light:
            case StopLine::red_light: {
                // decided once per light: through if stopping would take more than the limit (on amber, more
                // than comfortable braking), else stop
                if (!seen_.count(st.id)) {
                    seen_.insert(st.id);
                    const double limit = st.kind == StopLine::amber_light ? a_ref * 1.5 : opt.dilemma_decel;
                    if (decel > limit) {
                        released_.insert(st.id);
                        char buf[96];
                        std::snprintf(buf, sizeof buf, "light: %.1f m/s^2 to stop, going through", decel);
                        notes.push_back(buf);
                    }
                }
                if (!released_.count(st.id)) out.push_back({st.s});
                break;
            }
        }
    }
    for (auto* ids : {&seen_, &released_})
        for (auto it = ids->begin(); it != ids->end();) it = present.count(*it) ? std::next(it) : ids->erase(it);
    for (auto it = stopped_since_.begin(); it != stopped_since_.end();)
        it = present.count(it->first) ? std::next(it) : stopped_since_.erase(it);
    return out;
}

std::vector<double> stop_profile(const std::vector<Gate>& gates, const std::vector<double>& times, double front_now) {
    const int N = static_cast<int>(times.size()) - 1;
    std::vector<double> cap(N, INF);
    for (int k = 0; k < N; ++k) {
        const double t = times[k + 1];
        for (const Gate& gt : gates)
            if (gt.before && gt.t_from <= t && t <= gt.t_to) cap[k] = std::min(cap[k], gt.s);
        // never asked to go back: a stop line the front is already over holds the car where it is
        if (std::isfinite(cap[k])) cap[k] = std::max(cap[k], front_now);
    }
    return cap;
}

double stop_speed(double v, double cap, double front, const NmpcOptions& opt) {
    return std::isfinite(cap) ? std::min(v, std::sqrt(2.0 * opt.a_ref * std::max(0.0, cap - front))) : v;
}

std::vector<SX> stop_rows(const Stage& g) {  // the front behind the stop gates, at the last step with the distance to stop in
    const UrbanNMPC& n = *g.nmpc;
    const SX front = g.x(IS) + 0.5 * n.body.length - n.body.cg_to_center;
    const SX stop = g.terminal * sq(g.x(IV)) / (2.0 * n.opt.a_stop);
    return {g.cap_on * (front + stop - g.cap_s) + (g.cap_on - 1.0)};
}

}  // namespace ex
