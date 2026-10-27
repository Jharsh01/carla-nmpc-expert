// Pure pursuit with a grip-limited speed profile and a lane-change rule for passing.
//
// Steering (`feedforward_steering`, the default) is a feedforward of the angle
// that holds the target line's curvature in steady state plus a lookahead
// feedback on the lateral and heading errors (Kapania and Gerdes, 2015):
//
//     delta = L kappa + K_us U_x^2 kappa - k_lateral (e + x_lookahead (e_psi + beta_ss))
//
// with e the CG's offset from the target line, K_us the understeer gradient
// and beta_ss the steady-state sideslip, both from the axles' cornering
// stiffness. Otherwise it is the pure-pursuit law: the lookahead point is on
// the current target line, L_d = max(k_lookahead * U_x, lookahead_min) further
// along the path than the rear axle. Speed follows a
// grip-limited profile (speed_profile.hpp): computed once for the whole path if
// the road is known everywhere (`road_range` infinite), otherwise at every call
// over the `road_range` metres ahead. The longitudinal force is the profile's
// acceleration plus a proportional speed correction plus drag. Both outputs are
// kept within the vehicle's limits (steering rate, model/bicycle.hpp fx_limits).
//
// Pure pursuit has no notion of other cars, so passing is a rule: move from
// `lane_offset` to `pass_offset` while an obstacle near `lane_offset` is
// between `window_behind` behind and `window_ahead` ahead of the ego (centre to
// centre, along the path), and come back afterwards. The window ahead grows with
// the closing speed, so that `pass_time` seconds are left to change lane before
// reaching the car. The target line moves at `lateral_rate` [m/s].
//
// With a limited road range the car keeps to a speed from which it can brake,
// within the known road, to the speed of the path's tightest corner: the
// tightest radius is taken as known in advance (a property of the road).
//
// The longitudinal force is kept within the grip the cornering leaves on each
// axle (`axle_grip_use`): an axle's share of Fx (brake_front, drive_front) and
// the lateral force the measured U_x r asks of it, m U_x r b / L at the front
// and m U_x r a / L at the rear, must stay within its friction times its load,
// with the load transfer h Fx / L; or, if more, the lateral force its slip
// angle has it carry now (the tire model at the measured state), which is what
// a sliding rear carries. Braking shifts load off the rear, which
// carries 39% of the car and half the brake force, so in a corner the rear
// sets the limit: braking at the brakes' 7568 N at 0.7 g would ask 117% of its
// grip and spin the car.
//
// Given a reference (follow), such as the NMPC's last plan, it steers onto the
// reference's line instead of its own and tracks the reference's speed instead
// of its profile, wherever along the path the reference reaches and until its
// end time; elsewhere it drives as above. Following a plan made at the limit,
// its speed is capped at what `axle_grip_use` of the grip holds the line at.
// With `plan_steering`, a reference with the plan's heading and steering angle
// has those replace the steady-state feedforward: delta = delta_ref -
// k_lateral (e + x_lookahead (e_psi - e_psi_ref)). It is off: in the corner
// scenarios (scenarios/) it spun the car after 14 of 240 outages and left the
// road after 2, mostly at the corner exit, the heading feedback against the
// plan's body heading steering out while the car still yawed in; with the
// steady-state feedforward on the plan's line none spun or left the road.
#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include "track_control/controller.hpp"

namespace tc {

class PurePursuit : public Controller {
public:
    struct Options {
        double lane_offset = 0.0;  // line followed when nothing is in the way [m]
        double pass_offset = 3.5;  // line to pass on [m]
        double road_range = INF;  // how far ahead the road is known [m]
        double grip_use = 0.4;  // share of mu*g the speed profile may use
        double a_accel = 3.0;  // [m/s^2]
        double brake_use = 0.85;  // share of the brakes' peak force the profile plans with
        double k_lookahead = 0.6;  // [s]
        double lookahead_min = 5.0;  // [m]
        double tau_steer = 0.05;  // time constant the steering rate closes the angle error with [s]
        double k_speed = 3.0;  // [1/s]
        double window_ahead = 30.0;  // [m]
        double window_behind = 15.0;  // [m]
        double pass_time = 3.5;  // [s]
        double lateral_rate = 1.2;  // [m/s]
        bool feedforward_steering = true;  // the feedforward and lookahead law; false: the pure-pursuit law
        double k_lateral = 0.05;  // lookahead feedback gain [rad/m]
        double x_lookahead = 12.0;  // [m]
        // The feedforward's curvature is the mean over the next preview_time U_x metres, so the rate-limited steering
        // starts turning before a corner (or a change of direction) rather than at it. [s]
        double preview_time = 0.5;
        double axle_grip_use = 0.9;  // share of each axle's friction the force cap allows; 0: no cap
        bool plan_steering = false;  // steer with a reference's own steering angle and heading (see above)
    };

    PurePursuit(Path path, Road road, VehicleParams params, Body body, Options options);
    PurePursuit(Path path, Road road, VehicleParams params)
        : PurePursuit(std::move(path), road, params, Body(), Options()) {}

    // A line and speed along the path to follow, valid until plant time t_end: samples of s (increasing, as the
    // NMPC's s, unwrapped), e_y [m] and U_x [m/s], and optionally the heading to the path e_psi and the steering
    // angle delta [rad] (both or neither). An empty one (the default) follows nothing.
    struct Reference {
        double t_end = -INF;
        std::vector<double> s, e_y, v, e_psi, delta;
    };
    void follow(Reference reference);

    Input compute(const Observation& obs) override;
    std::string name() const override { return "pure_pursuit"; }

    // Profile speed [m/s] and its slope dv/ds [1/s] at arc length `s`.
    std::pair<double, double> speed_reference(double s) const;
    // The largest Fx braking (`brake`) or driving that leaves each axle, within axle_grip_use of its friction, the
    // lateral force a lateral acceleration `a_y` asks of it in steady state, or `Fy_front` / `Fy_rear` if more (what
    // its slip angle has it carry now, which a sliding rear makes larger) [N]; INF without the cap.
    double axle_force_limit(double a_y, bool brake, double Fy_front = 0.0, double Fy_rear = 0.0) const;
    double target_offset() const { return e_target_; }  // the line it is steering onto now [m]

    const Path path;
    const Road road;
    const VehicleParams p;
    const Body body;
    const Options opt;

private:
    bool obstacle_in_the_way(const std::vector<Obstacle>& obstacles, double s, double speed) const;
    // The reference at path position s, into `at`, if it reaches there; e_psi and delta NaN if it has none.
    struct ReferencePoint {
        double e_y = NAN, v = NAN, e_psi = NAN, delta = NAN;
    };
    bool reference_at(double s, ReferencePoint& at) const;

    double e_target_;
    double a_grip_, a_brake_;
    double k_us_;  // understeer gradient [rad / (m/s^2)]
    double c_rear_;  // rear cornering stiffness at the static load [N/rad]
    bool limited_;  // the road is known only a limited distance ahead
    std::vector<double> k_line_, v_limit_;  // cornering limit along the path (limited road range)
    std::vector<double> s_grid_, v_grid_;  // speed profile of the path (whole road known)
    std::optional<double> t_prev_;
    Reference ref_;  // its s relative to its first
    double ref_s0_ = 0.0;  // where along the path the reference starts
};

}  // namespace tc
