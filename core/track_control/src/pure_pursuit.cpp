#include "track_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

#include "track_control/model/bicycle.hpp"
#include "track_control/speed_profile.hpp"

namespace tc {

// Linear interpolation of the samples (xp, fp); periodic with `period` on a lap (as numpy.interp with `period`).
static double interp_path(double x, const std::vector<double>& xp, const std::vector<double>& fp, double period,
                          bool periodic) {
    const size_t n = xp.size();
    if (!periodic) {
        if (x <= xp.front()) return fp.front();
        if (x >= xp.back()) return fp.back();
    }
    x = periodic ? pymod(x, period) : x;
    const size_t i = std::upper_bound(xp.begin(), xp.end(), x) - xp.begin();  // xp[i - 1] <= x < xp[i]
    const size_t lo = (i + n - 1) % n, hi = i % n;
    const double x_lo = i == 0 ? xp[lo] - period : xp[lo];
    const double x_hi = i == n ? xp[hi] + period : xp[hi];
    return fp[lo] + (x - x_lo) / (x_hi - x_lo) * (fp[hi] - fp[lo]);
}

PurePursuit::PurePursuit(Path path_, Road road_, VehicleParams params, Body body_, Options options)
    : path(std::move(path_)),
      road(road_),
      p(params),
      body(body_),
      opt(options),
      e_target_(options.lane_offset),
      a_grip_(options.grip_use * road_.mu * params.g),
      a_brake_(options.brake_use * params.F_brake_max / params.m),
      c_rear_(params.C_alpha_r_at(params.m * params.g * params.a / params.L())),
      limited_(std::isfinite(options.road_range)) {
    const double c_front = p.C_alpha_f_at(p.m * p.g * p.b / p.L());
    k_us_ = p.m / p.L() * (p.b / c_front - p.a / c_rear_);
    // the profile must hold on whichever line the car is on
    const std::vector<double> offsets = {opt.lane_offset, opt.pass_offset};
    if (limited_) {
        CorneringLimit limit = cornering_limit(path, offsets, a_grip_, road.v_max);
        k_line_ = std::move(limit.k_line);
        v_limit_ = std::move(limit.v);
    } else {
        std::tie(s_grid_, v_grid_) =
            grip_speed_profile(path, offsets, road.mu, road.v_max, opt.grip_use, opt.a_accel, a_brake_, 0.5, p.g);
    }
}

std::pair<double, double> PurePursuit::speed_reference(double s) const {
    const double length = path.length();
    const bool loop = path.closed();
    if (!limited_) {
        const double v = interp_path(s, s_grid_, v_grid_, length, loop);
        return {v, interp_path(s + 1.0, s_grid_, v_grid_, length, loop) - v};
    }
    const int n = static_cast<int>(v_limit_.size());
    const double step = length / n;
    const int first = static_cast<int>(s / step), count = static_cast<int>(opt.road_range / step) + 1;
    std::vector<double> v(count), k(count);
    for (int i = 0; i < count; ++i) {
        const int j = loop ? ((first + i) % n + n) % n : std::clamp(first + i, 0, n - 1);
        v[i] = v_limit_[j];
        k[i] = k_line_[j];
    }
    const double v_beyond = *std::min_element(v_limit_.begin(), v_limit_.end());
    const auto [v0, v1] = speed_within_sight(std::move(v), k, step, a_grip_, a_brake_, v_beyond);
    return {v0, (v1 - v0) / step};
}

bool PurePursuit::obstacle_in_the_way(const std::vector<Obstacle>& obstacles, double s, double speed) const {
    for (const Obstacle& car : obstacles) {
        if (std::fabs(car.e_y - opt.lane_offset) > 0.5 * road.lane_width) continue;  // not on the home line
        const double ahead = path.difference(car.s, s);
        const double reach = body.length + opt.pass_time * std::max(speed - car.speed, 0.0);
        if (-opt.window_behind <= ahead && ahead <= std::max(opt.window_ahead, reach)) return true;
    }
    return false;
}

void PurePursuit::follow(Reference reference) {
    ref_ = Reference();
    if (reference.s.empty()) return;
    const bool steering = reference.e_psi.size() == reference.s.size() && reference.delta.size() == reference.s.size();
    ref_.t_end = reference.t_end;
    ref_s0_ = path.wrap(reference.s.front());
    for (size_t k = 0; k < reference.s.size(); ++k) {  // up to where s stops increasing
        const double s = reference.s[k] - reference.s.front();
        if (k > 0 && !(s > ref_.s.back())) break;
        ref_.s.push_back(s);
        ref_.e_y.push_back(reference.e_y[k]);
        ref_.v.push_back(reference.v[k]);
        ref_.e_psi.push_back(steering ? reference.e_psi[k] : NAN);
        ref_.delta.push_back(steering ? reference.delta[k] : NAN);
    }
    if (ref_.s.size() < 2) ref_ = Reference();
}

bool PurePursuit::reference_at(double s, ReferencePoint& point) const {
    if (ref_.s.size() < 2) return false;
    const double along = path.difference(s, ref_s0_);
    if (along < 0.0 || along > ref_.s.back()) return false;
    const size_t i = std::min<size_t>(std::upper_bound(ref_.s.begin(), ref_.s.end(), along) - ref_.s.begin(),
                                      ref_.s.size() - 1);  // ref_.s[i - 1] <= along < ref_.s[i]
    const double w = (along - ref_.s[i - 1]) / (ref_.s[i] - ref_.s[i - 1]);
    auto at = [&](const std::vector<double>& f) { return f[i - 1] + w * (f[i] - f[i - 1]); };
    point = {at(ref_.e_y), at(ref_.v), at(ref_.e_psi), at(ref_.delta)};
    return true;
}

double PurePursuit::axle_force_limit(double a_y, bool brake, double Fy_front, double Fy_rear) const {
    if (!(opt.axle_grip_use > 0.0)) return INF;
    const double L = p.L();
    // per axle: its share of Fx, its static load times L, the sign of its load change with |Fx|, its friction, and
    // the lateral force it carries
    struct Axle {
        double share, load, transfer, mu, Fy;
    };
    const double front = brake ? p.brake_front : p.drive_front;
    const Axle axles[2] = {
        {front, p.m * p.g * p.b, brake ? 1.0 : -1.0, opt.axle_grip_use * p.mu_f(),
         std::max(p.m * std::fabs(a_y) * p.b / L, std::fabs(Fy_front))},
        {1.0 - front, p.m * p.g * p.a, brake ? -1.0 : 1.0, opt.axle_grip_use * p.mu_r(),
         std::max(p.m * std::fabs(a_y) * p.a / L, std::fabs(Fy_rear))}};
    double limit = INF;
    for (const Axle& axle : axles) {
        if (!(axle.share > 0.0)) continue;
        // (share F)^2 + Fy^2 <= (mu (load + transfer h F) / L)^2: A F^2 - 2 B F + C <= 0
        const double F0 = axle.mu * axle.load / L, k = axle.transfer * axle.mu * p.h / L;
        if (axle.Fy >= F0) return 0.0;  // the cornering alone takes all its grip
        const double A = axle.share * axle.share - k * k, B = F0 * k, C = axle.Fy * axle.Fy - F0 * F0;
        const double root = A > 0.0 ? (B + std::sqrt(B * B - A * C)) / A : INF;  // C < 0: a real root
        limit = std::min(limit, root);
    }
    return limit;
}

Input PurePursuit::compute(const Observation& obs) {
    const State& x = obs.x;
    const double t = obs.t;
    const double s = path.project(x[IX], x[IY]).first;

    // passing rule: slide the target line toward the one it should be on
    const double goal = obstacle_in_the_way(obs.obstacles, s, x[IUX]) ? opt.pass_offset : opt.lane_offset;
    const double dt = t_prev_ ? t - *t_prev_ : 0.0;
    t_prev_ = t;
    e_target_ += std::clamp(goal - e_target_, -opt.lateral_rate * dt, opt.lateral_rate * dt);

    const bool following = t <= ref_.t_end;
    ReferencePoint here;
    const bool on_reference = following && reference_at(s, here);
    if (on_reference) e_target_ = here.e_y;  // so that the line slides on from the reference's once it ends
    double delta_cmd;
    if (opt.feedforward_steering) {
        const double e_psi = wrap_angle(x[IPSI] - path.pose(s).psi);
        double feedforward, heading_error;
        const double preview = std::max(x[IUX], 0.0) * opt.preview_time;
        if (opt.plan_steering && on_reference && !std::isnan(here.delta)) {  // the plan's own steering and heading
            ReferencePoint led;  // the steering lags its command
            feedforward = reference_at(s + std::max(x[IUX], 0.0) * opt.tau_steer, led) ? led.delta : here.delta;
            heading_error = e_psi - here.e_psi;
        } else {  // steady state on the target line
            double kappa = 0.0;
            for (int i = 0; i < 5; ++i) kappa += path.curvature(s + preview * i / 4.0) / 5.0;
            const double k_line = kappa / (1.0 - kappa * e_target_);
            const double beta_ss = p.b * k_line - p.a * p.m * x[IUX] * x[IUX] * k_line / (p.L() * c_rear_);
            feedforward = p.L() * k_line + k_us_ * x[IUX] * x[IUX] * k_line;
            heading_error = e_psi + beta_ss;
        }
        const double e_y = path.project(x[IX], x[IY]).second;
        delta_cmd = feedforward - opt.k_lateral * (e_y - e_target_ + opt.x_lookahead * heading_error);
    } else {
        const double psi = x[IPSI];
        const double rear_x = x[IX] - p.b * std::cos(psi), rear_y = x[IY] - p.b * std::sin(psi);
        const double L_d = std::min(std::max(opt.k_lookahead * x[IUX], opt.lookahead_min), opt.road_range);
        const double s_target = path.project(rear_x, rear_y).first + L_d;
        ReferencePoint ahead;
        const Pose target = path.pose(s_target, following && reference_at(s_target, ahead) ? ahead.e_y : e_target_);
        const double alpha = std::atan2(target.y - rear_y, target.x - rear_x) - psi;
        const double chord = std::hypot(target.x - rear_x, target.y - rear_y);
        delta_cmd = std::atan(2.0 * p.L() * std::sin(alpha) / chord);
    }
    delta_cmd = std::clamp(delta_cmd, -p.delta_max, p.delta_max);
    const double rate = std::clamp((delta_cmd - x[IDELTA]) / opt.tau_steer, -p.delta_rate_max, p.delta_rate_max);

    // curvature of the target line at s
    auto line_curvature = [&](double at) {
        const double kappa = path.curvature(at);
        return kappa / (1.0 - kappa * e_target_);
    };
    double accel;
    ReferencePoint next;
    if (on_reference && reference_at(s + 1.0, next)) {
        // The plan's speed, but no faster than the force cap's share of the grip holds the line at: the NMPC plans
        // at the limit, and tracking that within the cap would leave it no force, then a speed error to make up
        // with the throttle at the limit.
        auto held = [&](double at, double v) {
            if (!(opt.axle_grip_use > 0.0)) return v;
            double k = 0.0;
            for (int i = 0; i < 5; ++i) k = std::max(k, std::fabs(line_curvature(at + 0.25 * i * opt.preview_time *
                                                                                   std::max(x[IUX], 0.0))));
            return k > 0.0 ? std::min(v, std::sqrt(opt.axle_grip_use * road.mu * p.g / k)) : v;
        };
        const double v_ref = held(s, here.v), v_next = held(s + 1.0, next.v);
        accel = v_ref * (v_next - v_ref) + opt.k_speed * (v_ref - x[IUX]);
    } else {
        const auto [v_ref, dv_ds] = speed_reference(s);
        accel = v_ref * dv_ds + opt.k_speed * (v_ref - x[IUX]);
        if (limited_) accel = std::min(accel, opt.a_accel);  // the full profile has the drive's limit built in; this one has not
    }
    const double drag = 0.5 * p.rho * p.CdA * x[IUX] * x[IUX] + p.Crr * p.m * p.g;
    const auto [fx_min, fx_max] = fx_limits(x[IUX], p);  // what the car can deliver, as the NMPC is bounded
    // the grip the cornering leaves each axle: the lateral acceleration measured, or that the line asks for if more
    const double a_y = std::max(std::fabs(x[IUX] * x[IR]), x[IUX] * x[IUX] * std::fabs(line_curvature(s)));
    const AxleForces<double> now = axle_forces(x, Input::Zero(), p);  // from the slip angles
    return {rate, std::clamp(p.m * accel + drag, std::max(fx_min, -axle_force_limit(a_y, true, now.Fyf, now.Fyr)),
                             std::min(fx_max, axle_force_limit(a_y, false, now.Fyf, now.Fyr)))};
}

}  // namespace tc
