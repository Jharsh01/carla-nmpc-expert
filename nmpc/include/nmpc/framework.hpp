// The NMPC framework, for urban driving: a kinematic bicycle in path coordinates along the route (CasADi + FATROP).
//
//     state  [s, e_y, e_psi, v, a, delta]      input  [jerk, delta_rate]
//     beta   = atan(b tan(delta) / L)          (sideslip at the centre of gravity, b behind it to the rear axle)
//     s'     = v cos(e_psi + beta) / (1 - kappa e_y)
//     e_y'   = v sin(e_psi + beta)
//     e_psi' = v cos(beta) tan(delta) / L - kappa s'
//     v' = a,  a' = jerk,  delta' = delta_rate
//
// There are no tire slip angles, so the model holds down to
// standstill (v >= 0 is a bound), which stop-and-go traffic needs; grip enters as a friction circle on the planned
// longitudinal and lateral accelerations instead.
//
// What is asked of the plan comes in a `Problem` per solve (problem.hpp).
//
// The steps need not be equal (h_fine for the first `fine` seconds, h after), and each solve starts from the
// previous plan shifted forward by the time since.
//
// The problem is assembled here from its parts: the cost (cost.hpp) and the constraint rows of each manoeuvre
// (constraint/: lane, movers, follow, stop, crossing, grip), each group softened by one slack per step.
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nmpc/path.hpp"
#include "nmpc/problem.hpp"
#include "nmpc/shared_param.hpp"

namespace ex {

class UrbanNMPC {
public:
    using Options = NmpcOptions;  // shared_param.hpp

    UrbanNMPC(Path route, VehicleParams params, Body body, Options options);
    UrbanNMPC(Path route, VehicleParams params) : UrbanNMPC(std::move(route), params, Body(), Options()) {}
    ~UrbanNMPC();

    Plan solve(const Problem& problem);
    void reset();  // forget the previous plan: the next solve starts cold
    // Plan along another route from now on (a new path from SimLingo each update), given in a frame that may differ
    // from the old route's: `old_frame` is where the old frame's origin and x axis are in the new one (identity: the
    // same frame; for routes in the car's frame, where the car was at the last plan as seen from where it is now).
    // The previous plan, kept to start the next solve from, is carried over into the new route's coordinates. The
    // route should reach as far as the horizon goes, and a little behind the car.
    void set_route(Path route, Pose old_frame = {0.0, 0.0, 0.0});

    // The ego's path state [s, e_y, e_psi, v, a, delta] for a measurement: projected onto the route near where it
    // was last (the whole route the first time and after reset()).
    Eigen::Matrix<double, NX, 1> path_state(const Ego& ego) const;
    // Names of the constraint groups, in the order of Plan::slack_max.
    static const std::vector<std::string>& groups();

    Path route;  // set_route() changes it
    // The strongest acceleration the last solve allowed: min(opt.a_max, the vehicle's a_drive_max at the speed) [m/s^2]
    double a_limit = NAN;
    const VehicleParams p;
    const Body body;
    const Options opt;
    const std::vector<double> steps;  // each step's length [s]
    const int N;
    const std::vector<double> times;  // N + 1 [s]
    const double a_min;
    std::vector<double> solve_times;
    int failures = 0;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    mutable double s_last_ = NAN;
};

}  // namespace ex
