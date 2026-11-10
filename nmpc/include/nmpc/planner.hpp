// The planner: one step from a perceived `Scene` to the plan driven, beside the NMPC framework (framework.hpp),
// which it runs several of. Each step:
//
// 1. Perceive (middle_layer/perception): every actor's predicted trajectory is taken into the route's path
//    coordinates and classified against the ego lane (lead, blocker, crosser, mover).
// 2. Constrain (constraint/candidates.hpp): candidates are built from discrete choices (yield to or pass a crosser,
//    stay behind or go round a blocker; the stop lines and traffic rules in each), each a Problem for its own NMPC.
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

#include "nmpc/constraint/candidates.hpp"
#include "nmpc/framework.hpp"

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
    // Off by more than this, the line asks to change lane to that side (Scene::asks_change): sooner than `enter`, so
    // that an overtake is decided while the car is still far back. SimLingo keeping its lane stayed within 0.57 m
    // on routes 0, 1, 8 and 10; on route 1 its overtake was 0.69 m off at 26 m from the blocker, 2.45 m at 21 m.
    double ask = 0.6;  // [m]

    // The line's points, as s ahead of the car and offset from the lane's centre [m]: whether it is a manoeuvre.
    bool update(const std::vector<double>& s, const std::vector<double>& e);
    void reset() { manoeuvre = false; offset = 0.0; side = 0; }
    int asks() const { return offset > ask ? side : 0; }  // +1 left, -1 right, 0 none

    bool manoeuvre = false;
    double offset = 0.0;  // the largest offset at the last update [m]
    int side = 0;  // which side that offset is to: +1 left, -1 right
};

class Planner {
public:
    using Options = PlannerOptions;  // shared_param.hpp: perception's, the constraints' and the choice's

    Planner(Path route, VehicleParams params, Body body, NmpcOptions nmpc_options, Options options);
    Planner(Path route, VehicleParams params) : Planner(std::move(route), params, Body(), {}, {}) {}
    ~Planner();

    Decision step(const Scene& scene);
    void reset();  // forget the plans and the traffic rules' state (a new route)
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
