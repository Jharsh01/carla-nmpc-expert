// The constraints half of the behaviour layer: what the NMPC is asked to do, from the scene as sensed (sensor.hpp).
//
// The NMPC (urban_nmpc.hpp) plans locally: it cannot decide by itself whether to wait for a pedestrian or go before
// them, or whether to stay behind a parked car or overtake it in the oncoming lane. A plan through the middle of an
// obstacle has no gradient towards either side, and the solver stays there. So candidates are built from discrete
// choices, each a `Problem` for its own NMPC: yield to each crosser (a gate keeping the front behind where it crosses
// while it does) or pass ahead of it (a gate keeping the rear past it), stay behind a blocker or go round it through
// the lane on its left or its right. Whether there is such a lane, and which way it runs, is read from the map
// along the route (LaneSection), never assumed. A lane is gone into only when clear: of traffic coming the other
// way until the car is past the blockers, and of traffic going its way that it would cut off. Stop lines (red lights, stop signs, the end
// of the route) are gates in every candidate. All of them keep to the lane's corridor, along the line asked for
// (SimLingo's path) where the car fits.
//
// Which candidate is driven is planner.hpp's.
#pragma once

#include <string>
#include <vector>

#include "urban/sensor.hpp"

namespace ex {

// One candidate: the problem built here, the plan and score filled in when it is solved and judged.
struct Candidate {
    std::string label;  // "base", "pass:<id>", "borrow_left", "borrow_right"
    Problem problem;
    Plan plan;
    double score = INF;  // what the choice is made on (lower is better)
};

class Constraints {
public:
    struct Options : Sensor::Options {
        int pool = 4;  // NMPC instances, the most candidates solved at once
        double gate_margin = 1.0;  // kept between the ego's body and a crosser's [m]
        double gate_buffer = 1.0;  // a crosser's window widened by this much either side [s]
        double blocker_range = 40.0;  // a blocker nearer than this may be overtaken [m]
        double oncoming_gap = 20.0;  // kept to oncoming traffic when overtaking [m]
        // Into a lane, traffic going the car's way there must stay this far behind its rear, judged over merge_time
        // (the time to move over; after that the car is in the lane, ahead of it). Traffic ahead is followed.
        double lane_gap = 3.0;  // [m]
        double merge_time = 4.0;  // [s]
    };

    struct Result {
        std::vector<Candidate> candidates;  // "base" first, at most opt.pool
        bool committed = false;  // out in the other lane on a borrow (the one chosen last time), to be finished
        std::vector<std::string> notes;  // for the log
    };

    Constraints(Body body, VehicleParams params, UrbanNMPC::Options nmpc_options, Options options)
        : body(body), params(std::move(params)), nmpc_opt(nmpc_options), opt(options) {}

    // The candidates for the scene `p` was sensed from, `last_label` the one chosen last time.
    Result build(const Percept& p, const Scene& scene, const std::string& last_label) const;

    const Body body;
    const VehicleParams params;  // its a_drive_max: how fast the car can gain speed
    const UrbanNMPC::Options nmpc_opt;
    const Options opt;

    // The car driven from v0 as hard as it can, min(a_max, the vehicle's a_drive_max at each speed), up to v_top: how
    // far it has gone t from now, and how long it takes to go a distance. Over a pass of several seconds the speed
    // changes too much to hold the acceleration at the start's, as a single solve does.
    struct Drive {
        double dt = 0.05, v_end = 0.0;
        std::vector<double> d;  // every dt from 0
        double at(double t) const;
        double time_to(double distance) const;
    };
    Drive drive(double v0, double v_top) const;

private:
    void borrow(const Percept& p, const Scene& scene, bool left, double b_lo, double b_hi, const std::string& last_label,
                Result& out) const;
};

}  // namespace ex
