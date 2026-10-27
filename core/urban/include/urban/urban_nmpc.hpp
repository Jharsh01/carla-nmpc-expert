// NMPC for urban driving: a kinematic bicycle in path coordinates along the route (CasADi + FATROP).
//
//     state  [s, e_y, e_psi, v, a, delta]      input  [jerk, delta_rate]
//     beta   = atan(b tan(delta) / L)          (sideslip at the centre of gravity, b behind it to the rear axle)
//     s'     = v cos(e_psi + beta) / (1 - kappa e_y)
//     e_y'   = v sin(e_psi + beta)
//     e_psi' = v cos(beta) tan(delta) / L - kappa s'
//     v' = a,  a' = jerk,  delta' = delta_rate
//
// Unlike the race-track NMPC (track_control/nmpc.hpp) there are no tire slip angles, so the model holds down to
// standstill (v >= 0 is a bound), which stop-and-go traffic needs; grip enters as a friction circle on the planned
// longitudinal and lateral accelerations instead.
//
// What the world asks of the plan comes in a `Problem` per solve, all in the route's path coordinates:
//
//   corridor  where the body may be along the route: lower and upper e_y bounds and the line asked for, sampled
//             along s; a lane, or a lane and its neighbour when the behaviour layer lets the car borrow it
//   movers    things to keep clear of (cars, cyclists, pedestrians, cones), each a circle along a predicted
//             trajectory, its radius growing with the prediction's age; the body is covered by three circles
//   leads     cars ahead in the lane: the front stays a standstill gap plus a time gap behind each one's rear
//   gates     the front kept behind s over a time window (a stop line, yielding to a crossing car), or the rear
//             past s over one (passing ahead of a crossing car)
//
// Every one of these is a soft constraint, its slack heavily penalised, so a plan exists even when a violation can
// no longer be avoided, and then the least-violating one. Which gates to set (yield or pass, stop or go) is the
// behaviour layer's decision, not the NMPC's.
//
// The steps need not be equal (h_fine for the first `fine` seconds, h after), and each solve starts from the
// previous plan shifted forward by the time since, as in track_control's NMPC.
#pragma once

#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "track_control/controller.hpp"
#include "track_control/path.hpp"
#include "track_control/vehicle_params.hpp"

namespace ex {

using tc::Body;
using tc::Path;
using tc::VehicleParams;

constexpr double INF = std::numeric_limits<double>::infinity();
constexpr int NX = 6;
constexpr int NU = 2;
enum StateIndex { IS, IE, IPSI, IV, IA, IDELTA };
enum InputIndex { IJERK, IRATE };

// The ego car as measured, in the library's world frame (track_control/types.hpp) at the centre of gravity.
struct Ego {
    double x = 0.0, y = 0.0, psi = 0.0;  // [m], [rad]
    double v = 0.0;  // speed [m/s]
    double a = 0.0;  // longitudinal acceleration [m/s^2]
    double delta = 0.0;  // road-wheel angle [rad]
};

// Values sampled along the route, linear in between and held beyond the ends.
struct Corridor {
    std::vector<double> s;  // ascending [m]
    std::vector<double> lo, hi;  // the drivable band, e_y of its right and left edges [m]
    std::vector<double> ref;  // the line asked for [m]
};

// Something to keep clear of: a circle whose centre follows (s, e_y) at times t from the measurement, held before
// the first sample and after the last.
struct Mover {
    std::vector<double> t, s, e_y;  // [s], [m], [m]
    double radius = 1.0;  // [m]
    double growth = 0.0;  // the radius grows by this much per second of prediction [m/s]
};

// A car ahead in the lane: where its rear is along the route at times t (its speed is taken from these).
struct Lead {
    std::vector<double> t, s_rear;  // [s], [m]
};

// The ego's front kept behind s (`before`: a stop line, yielding) or its rear kept past s (passing ahead of a
// crossing car), from t_from to t_to after the measurement.
struct Gate {
    double s = 0.0;  // [m]
    double t_from = 0.0, t_to = INF;  // [s]
    bool before = true;
};

struct Problem {
    double t = 0.0;  // time of the measurement [s]
    Ego ego;
    Corridor corridor;
    double v_max = 50.0 / 3.6;  // speed limit [m/s]
    std::vector<double> v_ref_t, v_ref;  // speed asked for over time (e.g. SimLingo's); empty: v_max
    std::vector<Mover> movers;
    std::vector<Lead> leads;
    std::vector<Gate> gates;
};

struct Plan {
    std::vector<double> times;  // N + 1, from the measurement [s]
    Eigen::MatrixXd x;  // (N + 1) x NX
    Eigen::MatrixXd u;  // N x NU, each held over its step
    bool success = false;
    std::string status;
    int iterations = 0;
    double solve_time = 0.0;  // wall time [s]
    double cost = 0.0;
    std::vector<double> slack_max;  // largest slack of each constraint group over the horizon (UrbanNMPC::groups())
    int n_movers = 0;  // movers planned with (the nearest of those given)
};

class UrbanNMPC {
public:
    struct Options {
        double horizon = 5.0;  // [s]
        double h = 0.2;  // step [s]
        double h_fine = 0.1;  // step for the first `fine` seconds [s]
        double fine = 1.0;  // [s]
        int n_movers = 8;  // mover slots, the nearest to the plan filled
        double margin = 0.5;  // clearance kept to movers [m]
        double edge_margin = 0.15;  // kept between the body and the corridor's edges [m]
        // Behind a lead: never closer than gap_min (a hard constraint, as for any obstacle). The spacing kept,
        // gap + time_gap v, is asked for through the speed (the speed asked for tapers to it), and at the end of
        // the horizon room to stop behind the lead is softer (w_headway): a cut-in can take spacing away, and
        // braking hard to restore it at once is worse than restoring it over a few seconds.
        double gap_min = 0.5;  // [m]
        double gap = 3.0;  // standstill gap to a lead [m]
        double time_gap = 1.0;  // [s]
        // At the end of the horizon the car must be able to stop behind a stop gate, and behind a lead were it to
        // brake, at this deceleration: without it a plan reaching the line just as the horizon ends never brakes.
        double a_stop = 4.0;  // [m/s^2]
        // The speed asked for tapers to standstill at stop gates and behind leads along sqrt(2 a_ref d), d the
        // distance left: asked for the speed limit right up to them, a quadratic speed cost would rather creep
        // towards a stop over the whole horizon than drive up to it and wait (a convex cost spreads the distance).
        double a_ref = 2.0;  // [m/s^2]
        // and in bends to the speed that takes them at this lateral acceleration (grip allows more, comfort less)
        double a_lat_ref = 3.0;  // [m/s^2]
        double a_max = 3.0;  // [m/s^2]
        // The strongest braking [m/s^2]; NaN: min(F_brake_max / m, mu g). CARLA's cars brake at about 8 m/s^2 on
        // dry asphalt, more than the race-track Model 3 parameters' brake force gives.
        double a_min = -7.0;
        double jerk_max = 15.0;  // [m/s^3]
        double mu = 0.9;  // friction the plans may use
        double w_speed = 1.0;
        double w_lane = 3.0;
        double w_heading = 1.0;
        double w_accel = 0.5;
        double w_jerk = 0.05;
        double w_rate = 20.0;
        double w_lat = 0.3;  // lateral acceleration
        double w_headway = 1e2;
        double w_slack = 1e5;  // high enough that no violation is cheaper than braking to a stop from the speed limit
        double tol = 1e-3;
        // Stop once the iterate has been within acceptable_tol for acceptable_iter iterations in a row. Near
        // standstill the dual infeasibility crawls down long after the plan has stopped changing.
        double acceptable_tol = 1e-1;
        int acceptable_iter = 3;
        int max_iter = 100;
        // Compile the problem's functions to native code (with the system's C compiler): about half the time per
        // iteration, but over a minute to build. With a cache directory the compiled solver is saved there, keyed by
        // everything the problem depends on, and loaded by later instances and processes instead of compiled again.
        bool jit = false;
        std::string cache_dir;
        bool warm_start = true;  // start from the previous plan shifted; false: from the state carried on at its speed
    };

    UrbanNMPC(Path route, VehicleParams params, Body body, Options options);
    UrbanNMPC(Path route, VehicleParams params) : UrbanNMPC(std::move(route), params, Body(), Options()) {}
    ~UrbanNMPC();

    Plan solve(const Problem& problem);
    void reset();  // forget the previous plan: the next solve starts cold

    // The ego's path state [s, e_y, e_psi, v, a, delta] for a measurement.
    Eigen::Matrix<double, NX, 1> path_state(const Ego& ego) const;
    // Names of the constraint groups, in the order of Plan::slack_max.
    static const std::vector<std::string>& groups();

    const Path route;
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
};

}  // namespace ex
