// The parameters shared by the NMPC (nmpc/) and the middle layer (middle_layer/): the car (its body and the vehicle
// parameters it is planned with), and every tunable of each part, in one place:
//
//   PerceptionOptions   the middle layer: what is looked at and how it is classified (middle_layer/perception)
//   NmpcOptions         the optimisation: horizon, limits, weights, solver (framework.hpp)
//   ConstraintOptions   the manoeuvres: crossers, blockers, overtakes, stops (constraint/)
//   PlannerOptions      the choice between the candidates (planner.hpp)
//
// ConstraintOptions extends PerceptionOptions (the constraints use its look-ahead and margins) and PlannerOptions
// extends ConstraintOptions, so one PlannerOptions carries everything above the optimisation (Python:
// expertctl.PlannerOptions).
#pragma once

#include <limits>
#include <string>
#include <vector>

namespace ex {

constexpr double INF = std::numeric_limits<double>::infinity();

// ---- the car

// Size of the ego car's body, for the corridor and clearance constraints; the defaults are the MKZ's
// (docs/vehicle_parameters.md).
struct Body {
    double length = 4.89;  // [m]
    double width = 1.84;  // [m]
    double cg_to_center = 0.31;  // centre of gravity ahead of the body centre [m]
};

// The defaults are CARLA 0.9.15's vehicle.lincoln.mkz_2020, Bench2Drive's ego (config/vehicles/mkz2020.json,
// with where each value comes from: docs/vehicle_parameters.md).
struct VehicleParams {
    double m = 1696.0;  // mass [kg]
    double a = 1.172;  // centre of gravity to front axle [m]
    double b = 1.689;  // centre of gravity to rear axle [m]
    double F_brake_max = 13000.0;  // peak brake force [N]
    double delta_max = 0.95;  // road-wheel angle limit [rad]
    double delta_rate_max = 0.6;  // steering rate limit [rad/s]
    double g = 9.81;  // [m/s^2]
    // The strongest drive acceleration against speed (full throttle, measured): linear between the speeds, held
    // beyond them. The file's "tables": {"a_drive_max": {"speed": [...], "value": [...]}}.
    std::vector<double> a_drive_speed = {0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0};  // [m/s]
    std::vector<double> a_drive_max = {5.427, 6.291, 6.8, 5.173, 4.6, 3.831, 3.431, 2.957, 2.902, 2.629, 2.102};  // [m/s^2]

    double a_drive(double v) const;  // the table at speed v [m/s^2]

    // Read from a JSON file of the form {"vehicle": {"parameters": {"m": {"value": ...}, ...}}}; every field above
    // must be present.
    static VehicleParams from_file(const std::string& path);

    double L() const { return a + b; }  // wheelbase [m]

    struct Field {
        const char* name;
        double VehicleParams::* member;
    };
    static const std::vector<Field>& fields();
};

// ---- the middle layer

struct PerceptionOptions {
    double look_ahead = 80.0;  // actors and stops further along the route than this are left out [m]
    // A parked car or an object is in the lane (a blocker, or a lead) only if it reaches this far into it; one
    // that only grazes the lane is kept clear of as a mover, by nudging within the lane. Without it a car parked
    // beside the lane, its box tilted a little against a wobbling path (SimLingo's), blocks the lane.
    double blocker_overlap = 0.3;  // [m]
    // Kept to parked cars and objects instead of the NMPC's margin, without the growth with prediction time a
    // moving actor's clearance has: nothing about where they will be is uncertain. With the NMPC's 0.5 m and the
    // circles' cover, a car parked beside a 2.9 m lane (2.65 m off the path) could not be driven past at all
    // (Bench2Drive route 0).
    double static_margin = 0.2;  // [m]
};

// ---- the optimisation

struct NmpcOptions {
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
    // The strongest acceleration, 0.5 g [m/s^2]; less where the vehicle cannot give it at the speed (its
    // a_drive_max table), taken at the speed at the start of each solve and held over the horizon
    double a_max = 0.5 * 9.81;
    // The strongest braking the plans may ask for [m/s^2]; NaN: min(F_brake_max / m, mu g). The MKZ brakes at about
    // 7.7 m/s^2 on dry asphalt (docs/vehicle_parameters.md); the limit set for the car is 4.6 (10 Nov).
    double a_min = -4.6;
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

// ---- the manoeuvres

struct ConstraintOptions : PerceptionOptions {
    int pool = 4;  // NMPC instances, the most candidates solved at once
    double gate_margin = 1.0;  // kept between the ego's body and a crosser's [m]
    double gate_buffer = 1.0;  // a crosser's window widened by this much either side [s]
    double blocker_range = 40.0;  // a blocker nearer than this may be overtaken [m]
    double oncoming_gap = 20.0;  // kept to oncoming traffic when overtaking [m]
    // an oncoming lane inside the band (SimLingo's path changing into it) counts as one to go round through when at
    // least this much of the band is oncoming on that side [m]
    double min_borrow_width = 2.5;
    // Into a lane, traffic going the car's way there must stay this far behind its rear, judged over merge_time
    // (the time to move over; after that the car is in the lane, ahead of it). Traffic ahead is followed.
    double lane_gap = 3.0;  // [m]
    double merge_time = 4.0;  // [s]
    // SimLingo asking to go round a blocker that is not clear to go round: the car stops at once, at this
    // deceleration, where it is, not up behind the blocker: the further back, the further past it the car sees
    // down the lane it would go into, and the more room it has to pull out from rest.
    double view_stop_decel = 3.0;  // [m/s^2]
    // after this long at the speed now: the plan is driven a tick late and the brake builds up (on route 4 a stop
    // judged at 3 m/s^2 from the first instant asked for 6, which the MKZ's brake under 6.5 m/s turned into 20)
    double view_stop_delay = 0.5;  // [s]
    double stop_hold = 1.0;  // [s] at a stop sign
    double dilemma_decel = 4.6;  // a light turning red that needs more than this to stop for is driven through [m/s^2]
                                 // (no more than the braking limit, -a_min: a stop needing more cannot be made)
};

// ---- the choice

struct PlannerOptions : ConstraintOptions {
    double pass_progress = 5.0;  // a clear overtake is taken over a candidate that gets less far than it by this [m]
    double hysteresis = 0.9;  // the last choice's score is multiplied by this
    double w_out_of_lane = 0.2;  // on the squared distance from the ego lane's centre, in the score
    double pass_penalty = 300.0;  // added to the score for each crosser gone ahead of
    double slack_ok = 0.05;  // a candidate with any slack above this is only chosen when none is within it
};

}  // namespace ex
