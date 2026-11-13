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
    // Going across a stream between two of its cars (constraint/crossing.cpp: cross), on the drive the gap is judged
    // on: each car's window widened by this much either side, not gate_buffer. PDM-Lite keeps only its boxes grown
    // (x1.2-1.5 the other's length, x1.3 its own when moving), some 0.1-0.25 s at 10 m/s; with 1 s either side no gap
    // of route 10's stream (10 m/s, clear 0.7-1.5 s at the crossing) was ever left
    // (13 Nov: 0.1; the stream's cars keep their speed, and its gaps, free 1.0-1.9 s at the line, take the ego ~2.6 s)
    double cross_buffer = 0.1;  // [s]
    // A crosser slower than this now (a car inching up to its stop line) counts only over the next cross_slow_horizon:
    // whether it then stops or goes is not known, and it is judged again each plan. PDM-Lite, 2 s ahead from their
    // controls, saw route 10's cross-road queue brake; predicted 8 s on at its creep, it shut most of the stream's gaps.
    double cross_slow_speed = 5.0;  // [m/s] (route 10's queue crept at up to 4.4 m/s after its stop)
    double cross_slow_horizon = 2.5;  // [s]
    // and is on the ego's path where its body comes within this of the ego's sides (the ego on the path) [m]
    double cross_side = 0.5;
    // The corridor is the lane's band and this much either side where the band has no road edges (a junction, where
    // no lines are painted, or where the nearest lane crosses the path): a car takes a turn wider or tighter than the
    // lane's centre. With the lane alone (route 8's right turn: +-1.4 m) the car's ends had +-0.23 m to follow the
    // bend, and the NMPC slowed to 2-3 m/s to keep in it; 0.3 m more let the same plans (replaylog_r8) go to 7-8 m/s.
    // Only the corridor: the band the actors are classified on stays the lane (on it, route 10's oncoming lane beside
    // the band overlapped it, and the cars in it were crossers all along the approach).
    double junction_slack = 0.4;  // [m]
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
    // Out in another lane round a blocker the car may go this much over the limit, to be out of that lane sooner
    // (the pass is judged at that speed too); back in its lane it comes down to the limit at the speed cost's pace.
    double overtake_over_limit = 20.0 / 3.6;  // [m/s]
    // Peeking: stopped to see (view_stop) and the lane to go into is not seen far enough, the car backs up to
    // `peek_back` further from the blocker (never nearer than `peek_rear_gap` to anything behind it; the agent drives
    // this, the NMPC does not reverse), then comes forward `peek_run` at `peek_speed` over to that side of its lane,
    // its body's edge margin `peek_over` past the lane's edge, and waits there: a blocker hides less of the lane
    // beyond from further back and from nearer that side. Never nearer the blocker, or a car followed, than
    // `peek_min_gap`, and forward no further than it backed up (route 6: with a car behind it there was no backing
    // up, and 8 m forward took it to 3 m behind the car ahead, with no room left to pull out). A car left out of
    // its lane by a way round that was withdrawn is brought back the same way: backed up along the lane, or, with
    // no room behind, forward `peek_run` to straighten.
    double peek_back = 12.0;  // [m] (route 4: sight grew about 4 m for each metre further back, the pass 1 m)
    double peek_run = 8.0;  // [m] (in CARLA the car moved over 0.15 m in the first 2.5 m)
    double peek_over = 0.3;  // [m]
    double peek_min_gap = 8.0;  // [m]
    double peek_rear_gap = 1.5;  // [m]
    double peek_speed = 2.0;  // [m/s]
    // Merging into a stream (constraint/merge): the ego goes in behind one car and ahead of the next when, driven
    // as hard as it can out of the bend, its rear stays `merge_gap` ahead of that next car's front (and its front
    // `merge_lead_gap` behind the rear of the one before) for as long as the two are predicted; it enters no
    // sooner than `merge_lag` after the car it follows. It may match the stream's speed, up to
    // `overtake_over_limit` above the limit: the stream's cars do not slow for it.
    double merge_gap = 2.0;  // [m] (13 Nov: 2, as the NMPC keeps behind a chaser, framework.cpp pass_profile)
    double merge_lead_gap = 1.0;  // [m] (13 Nov: 1, close behind it at its speed; was 2)
    double merge_lag = 0.0;  // [s] (13 Nov: 0, the 1 m behind it is what keeps them apart; was 0.2)
    // Waiting for the stream the ego stays this far back from where it comes in, as far as it can: from rest at
    // the entry the car behind in the stream closes 17 m while the ego gets up to its 13 m/s, with this run-up 1 m.
    double merge_run_up = 15.0;  // [m]
    // and takes the bend into the stream at this lateral acceleration, not the comfortable a_lat_ref: the faster it
    // comes out of the bend the less the car behind closes on it (route 8: 9 m at 3 m/s^2 round a 6 m bend)
    double merge_a_lat = 8.5;  // [m/s^2] (13 Nov: 8.5, within mu g = 8.8; PDM-Lite takes route 8's bend at 10-18; was 6)
    // Going in ahead of a car, expect it to brake at this from when the ego is in its lane (0: it does not slow,
    // the default; Bench2Drive's streams do not, until they hit something). A trial setting, 12 Nov.
    double merge_chaser_brake = 0.0;  // [m/s^2]
    // Turn into the lane without waiting for or fitting between cars coming down it from a side road (Percept::
    // mergers, and those joining behind): once the stop line allows, the ego goes. A car of the stream already in
    // the lane ahead is still followed. Chosen 13 Nov for route 8 (turn right on green, the stream not yielded to);
    // the stream's cars do not slow for the ego.
    bool ignore_mergers = false;
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
