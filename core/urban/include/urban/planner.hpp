// The planner: the behaviour layer's step, from a perceived `Scene` to the plan driven. The layer is three parts:
// the sensor (sensor.hpp), the constraints (constraints.hpp) and this, which owns the NMPCs, solves the candidates
// and chooses one. Each step:
//
// 1. Sense (sensor.hpp): every actor's predicted trajectory is taken into the route's path coordinates and classified
//    against the ego lane (lead, blocker, crosser, mover); the traffic rules leave the stop lines to stop at.
// 2. Constrain (constraints.hpp): candidates are built from discrete choices (yield to or pass a crosser, stay behind
//    or borrow the lane round a blocker), each a Problem for its own NMPC.
// 3. The candidates are solved in parallel, each by its own NMPC (warm-started from its own last plan), and the
//    best is chosen, the one chosen last time favoured, so the choice does not flicker. They are compared on what
//    their plans do, not on the NMPC's own costs, which differ between them: speed against the speed asked for
//    (not tapered to the stops: waiting is not free), comfort, time out of the ego lane and constraint violation,
//    with a penalty for going before a crosser rather than after.
//
// Before a step, PathIntent says whether the line asked for is a manoeuvre, which decides the route planned along.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "urban/constraints.hpp"

namespace ex {

struct Decision {
    std::string label;  // the candidate chosen
    Plan plan;
    std::vector<Candidate> candidates;
    double wall_time = 0.0;  // for the whole step [s]
    std::vector<std::string> notes;  // classification and rule events, for the log
};

// Whether a line asked for (SimLingo's path) is a manoeuvre: a lane change or a turn. Given as offsets from the lane
// the car is in, not from its heading, so a bend in the road is not one, nor is the car being a little off the lane
// (a line starts at the car, and rejoins the lane within `from`). With hysteresis, as the line is a little different
// each time. Decided before the step, as the route to plan along depends on it (Planner::set_route).
class PathIntent {
public:
    double from = 8.0;  // only the line this far ahead of the car counts [m]
    double enter = 1.0;  // off the lane's centre by more than this, a manoeuvre begins [m]
    double exit = 0.5;  // and ends when within this [m]

    // The line's points, as s ahead of the car and offset from the lane's centre [m]: whether it is a manoeuvre.
    bool update(const std::vector<double>& s, const std::vector<double>& e);
    void reset() { manoeuvre = false; offset = 0.0; }

    bool manoeuvre = false;
    double offset = 0.0;  // the largest offset at the last update [m]
};

class Planner {
public:
    // the sensor's and the constraints' (constraints.hpp), and the choice's
    struct Options : Constraints::Options {
        double pass_progress = 5.0;  // a clear overtake is taken over a candidate that gets less far than it by this [m]
        double hysteresis = 0.9;  // the last choice's score is multiplied by this
        double w_out_of_lane = 0.2;  // on the squared distance from the ego lane's centre, in the score
        double pass_penalty = 300.0;  // added to the score for each crosser gone ahead of
        double slack_ok = 0.05;  // a candidate with any slack above this is only chosen when none is within it
    };

    Planner(Path route, VehicleParams params, Body body, UrbanNMPC::Options nmpc_options, Options options);
    Planner(Path route, VehicleParams params) : Planner(std::move(route), params, Body(), {}, {}) {}
    ~Planner();

    Decision step(const Scene& scene);
    void reset();  // forget plans and rule state (a new route)
    // Plan along another route from the next step on (UrbanNMPC::set_route, with the old route's frame as seen in
    // the new one), keeping the plans and the rule state: the scene is then in the new frame, along the new route.
    void set_route(Path route, Pose old_frame = {0.0, 0.0, 0.0});

    Path route;  // set_route() changes it
    const Body body;
    const Options opt;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ex
