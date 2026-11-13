// What the NMPC is asked to solve and what it gives back: the Problem of one solve, all in the route's path
// coordinates, and its Plan. Shared by the solver (framework.hpp), the manoeuvres that build problems (constraint/)
// and the middle layer (middle_layer/perception), whose leads and movers are these.
//
// What the world asks of the plan comes in a `Problem` per solve, all in the route's path coordinates:
//
//   corridor  where the body may be along the route: lower and upper e_y bounds and the line asked for, sampled
//             along s; a lane, or a lane and its neighbour when the overtake (constraint/overtake) lets the car borrow it
//   movers    things to keep clear of (cars, cyclists, pedestrians, cones), each a circle along a predicted
//             trajectory, its radius growing with the prediction's age; the body is covered by three circles
//   leads     cars ahead in the lane: the front stays a standstill gap plus a time gap behind each one's rear
//   gates     the front kept behind s over a time window (a stop line, yielding to a crossing car), or the rear
//             past s over one (passing ahead of a crossing car)
//   chasers   cars coming up behind in a lane the ego is merging into: the rear kept ahead of each one's front
//
// Every one of these is a soft constraint, its slack heavily penalised, so a plan exists even when a violation can
// no longer be avoided, and then the least-violating one. Which gates to set (yield or pass, stop or go) is decided
// by the manoeuvres (constraint/), not by the optimisation.
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

#include "nmpc/shared_param.hpp"

namespace ex {

constexpr int NX = 6;
constexpr int NU = 2;
enum StateIndex { IS, IE, IPSI, IV, IA, IDELTA };
enum InputIndex { IJERK, IRATE };

// The ego car as measured, in a right-handed world frame (heading counter-clockwise) at the centre of gravity.
struct Ego {
    double x = 0.0, y = 0.0, psi = 0.0;  // [m], [rad]
    double v = 0.0;  // speed [m/s]
    double a = 0.0;  // longitudinal acceleration [m/s^2]
    double delta = 0.0;  // road-wheel angle [rad]
};

// Linear interpolation in (xs, ys), held beyond the ends; xs ascending.
double sample(const std::vector<double>& xs, const std::vector<double>& ys, double x);

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
    double margin = NAN;  // clearance kept to it [m]; NaN: NmpcOptions::margin
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
    bool route_end = false;
    // the speed asked for tapers to a stop at it (stop.cpp). Not at a gate the ego means to roll through as it opens
    // (merge.cpp: timed by the speed asked for)
    bool taper = true;  // the route's end, not a rule of the road: not stopped at with a car close behind (merge.cpp)
};

// A car coming up behind in the lane the ego is entering (constraint/merge): where its front is along the route at
// times t, from `t_from` on (before that it is not in the lane). The ego's rear is kept ahead of it.
struct Chaser {
    std::vector<double> t, s_front;  // [s], [m]
    double t_from = 0.0;  // [s]
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
    std::vector<Chaser> chasers;
    double a_lat_ref = NAN;  // the lateral acceleration bends are taken at [m/s^2]; NaN: NmpcOptions::a_lat_ref
    // the spacing the speed asked for tapers to behind a lead, gap + time_gap v; NaN: NmpcOptions::gap, time_gap
    // (merge.cpp: 1 m and 0 s, close behind the stream's car at its speed)
    double follow_gap = NAN, follow_time_gap = NAN;  // [m], [s]
    // the strongest acceleration asked for (still within the vehicle's a_drive_max); NaN: NmpcOptions::a_max
    // (merge.cpp: INF, full throttle, as PDM-Lite goes on route 8: 5-8 m/s^2 from rest)
    double a_max = NAN;  // [m/s^2]
    // the cost's weights on acceleration and on lateral acceleration; NaN: NmpcOptions::w_accel, w_lat (crossing.cpp:
    // light, so that the plan launches as the gap was judged: with the usual ones it trailed a full-throttle drive by
    // 1.5-2 m/s within a second, on route 10's left turn, and each gap was lost)
    double w_accel = NAN, w_lat = NAN;
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

}  // namespace ex
