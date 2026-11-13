// What the NMPC is asked to do, from the scene as perceived (middle_layer/perception): candidate problems, each
// for its own NMPC, built from discrete choices. The optimisation plans locally: it cannot decide by itself whether
// to wait for a pedestrian or go before them, or whether to stay behind a parked car or overtake it, as a plan
// through the middle of an obstacle has no gradient towards either side. Each manoeuvre is a file in constraint/,
// its rule and the rows it adds to the optimisation (stage.hpp) side by side:
//
//   lane.cpp      the corridor along the lane and the line asked for (SimLingo's path)
//   movers.cpp    what to keep clear of, and which of it the optimisation plans with
//   follow.cpp    leads, and waiting behind blockers
//   stop.cpp      stop lines: red lights, stop signs (held, then released), the route's end
//   crossing.cpp  yield to a crosser, or pass ahead of it
//   merge.cpp     into a stream at a junction: behind one of its cars and ahead of the next
//   overtake.cpp  round blockers through the lane on either side, when clear and seen to be
//   turn.cpp      the speed in bends
//   grip.cpp      the friction circle and the speed limit
//
// This file builds the candidates: "base" (yield to every crosser, stay behind every blocker), "pass:<id>" (go
// ahead of one crosser), "borrow_left" / "borrow_right" (round the blockers). Which is driven is the planner's.
#pragma once

#include <string>
#include <vector>

#include "nmpc/constraint/stop.hpp"
#include "nmpc/framework.hpp"
#include "perception/perception.hpp"

namespace ex {

// One candidate: the problem built here, the plan and score filled in when it is solved and judged.
struct Candidate {
    std::string label;  // "base", "pass:<id>", "merge:<id>", "borrow_left", "borrow_right"
    Problem problem;
    Plan plan;
    double score = INF;  // what the choice is made on (lower is better)
    // Not to be solved or chosen: "base" while the car is going round a blocker it can no longer stop behind. Its
    // plan could only break its constraints, and finding the least bad way to takes the solver its iteration limit
    // (at 50 km/h past a blocker, 100 ms a plan against the 15 ms of the way round).
    bool hopeless = false;
};

class Constraints {
public:
    using Options = ConstraintOptions;  // shared_param.hpp

    struct Result {
        std::vector<Candidate> candidates;  // "base" first, at most opt.pool
        bool committed = false;  // out in the other lane on a borrow (the one chosen last time), to be finished
        // SimLingo asks to go round a blocker and that way is clear: this borrow is driven now, if its plan holds
        std::string go;
        // going into a stream ahead of a car ("merge:<id>"), judged clear (merge.cpp): driven if its plan holds
        std::string merge;
        // going across the crossers' path between two of them ("cross:<id>"), judged clear (crossing.cpp): driven
        // if its plan holds
        std::string cross;
        std::vector<std::string> notes;  // rule events, for the log
        // the way round on a side (0 right, 1 left) is held because that lane is not seen far enough (overtake.cpp)
        bool sight_short[2] = {false, false};
        double reverse = 0.0;  // back up this far now, to see past the blocker (peeking) [m]; the agent drives it
        double reverse_delta = 0.0;  // the road-wheel angle to back up with, towards the lane's middle and along it [rad]
        bool peeking = false;  // backing up or edging out to see: the plans keep to the lane's path meanwhile
    };

    Constraints(Body body, VehicleParams params, NmpcOptions nmpc_options, Options options)
        : body(body), params(std::move(params)), nmpc_opt(nmpc_options), opt(options) {}

    // The candidates for the scene `p` was perceived from, `last_label` the one chosen last time. Keeps the traffic
    // rules' state (stop signs held at, lights decided on).
    Result build(const Percept& p, const Scene& scene, const std::string& last_label);
    void reset() { stops.reset(); view_stop = 0; peek = 0; }  // forget the rules' state (a new route)

    const Body body;
    const VehicleParams params;  // its a_drive_max: how fast the car can gain speed
    const NmpcOptions nmpc_opt;
    const Options opt;
    StopRules stops;
    // Stopped to see past a blocker SimLingo asked to go round (the side asked, +1 left, -1 right; 0 not): held while
    // the blocker is in range, whatever SimLingo asks meanwhile (at rest it may ask to wait in the lane).
    int view_stop = 0;
    // Peeking while stopped to see (ConstraintOptions::peek_back): 0 not, 1 backing up until the blocker is
    // `peek_back_gap` ahead of the front, 2 edged out and waiting with the front `peek_gap` short of the blocker.
    // 3 going round from there.
    int peek = 0;
    double peek_back_gap = 0.0, peek_gap = 0.0;  // [m]
};

}  // namespace ex
