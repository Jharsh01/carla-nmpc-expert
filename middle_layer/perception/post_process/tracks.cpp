#include "perception/tracks.hpp"

#include <algorithm>
#include <cmath>

namespace ex {

std::pair<double, double> Projector::operator()(double x, double y) const {
    auto [s, e] = route.project(x, y, s_lo, s_hi);
    const Pose q = route.pose(s);
    const double d = std::hypot(x - q.x, y - q.y);
    if (d > std::fabs(e) + 0.5) e = e >= 0.0 ? d : -d;
    return {s, e};
}

bool make_track(const Actor& a, const Projector& project, Track& tr) {
    if (a.t.empty() || a.x.size() != a.t.size() || a.y.size() != a.t.size() || a.psi.size() != a.t.size()) return false;
    tr = Track();
    tr.actor = &a;
    for (size_t i = 0; i < a.t.size(); ++i) {
        const auto [s, e] = project(a.x[i], a.y[i]);
        const double rel = wrap_angle(a.psi[i] - project.route.pose(s).psi);
        tr.t.push_back(a.t[i]);
        tr.s.push_back(s);
        tr.e.push_back(e);
        tr.ext_s.push_back(0.5 * (std::fabs(a.length * std::cos(rel)) + std::fabs(a.width * std::sin(rel))));
        tr.ext_e.push_back(0.5 * (std::fabs(a.length * std::sin(rel)) + std::fabs(a.width * std::cos(rel))));
        if (i == 0) tr.cos_rel = std::cos(rel);
        // (its heading at the last sample on the stretch of path: past its end, s is clamped there)
        if (i == 0 || s < std::min(project.s_hi, project.route.length()) - 0.1) tr.cos_end = std::cos(rel);
    }
    if (a.t.size() > 1) tr.speed = std::hypot(a.x[1] - a.x[0], a.y[1] - a.y[0]) / (a.t[1] - a.t[0]);
    return true;
}

bool near_route(const Track& tr, double s0, double rear0, double look_ahead) {
    for (size_t i = 0; i < tr.s.size(); ++i)
        if (tr.s[i] + tr.ext_s[i] > rear0 - 10.0 && tr.s[i] - tr.ext_s[i] < s0 + look_ahead && std::fabs(tr.e[i]) < 30.0)
            return true;
    return false;
}

void add_circles(const Track& tr, const Projector& project, double static_margin, std::vector<Mover>& into) {
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
        if (still) mv.margin = static_margin;
        for (size_t i = 0; i < a.t.size(); ++i) {
            const auto [s, e] = project(a.x[i] + d * std::cos(a.psi[i]), a.y[i] + d * std::sin(a.psi[i]));
            mv.t.push_back(a.t[i]);
            mv.s.push_back(s);
            mv.e_y.push_back(e);
        }
        into.push_back(std::move(mv));
    }
}

}  // namespace ex
