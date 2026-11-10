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
    std::string label;  // "base", "pass:<id>", "borrow_left", "borrow_right"
    Problem problem;
    Plan plan;
    double score = INF;  // what the choice is made on (lower is better)
};

class Constraints {
public:
    using Options = ConstraintOptions;  // shared_param.hpp

    struct Result {
        std::vector<Candidate> candidates;  // "base" first, at most opt.pool
        bool committed = false;  // out in the other lane on a borrow (the one chosen last time), to be finished
        // SimLingo asks to go round a blocker and that way is clear: this borrow is driven now, if its plan holds
        std::string go;
        std::vector<std::string> notes;  // rule events, for the log
    };

    Constraints(Body body, VehicleParams params, NmpcOptions nmpc_options, Options options)
        : body(body), params(std::move(params)), nmpc_opt(nmpc_options), opt(options) {}

    // The candidates for the scene `p` was perceived from, `last_label` the one chosen last time. Keeps the traffic
    // rules' state (stop signs held at, lights decided on).
    Result build(const Percept& p, const Scene& scene, const std::string& last_label);
    void reset() { stops.reset(); view_stop = 0; }  // forget the rules' state (a new route)

    const Body body;
    const VehicleParams params;  // its a_drive_max: how fast the car can gain speed
    const NmpcOptions nmpc_opt;
    const Options opt;
    StopRules stops;
    // Stopped to see past a blocker SimLingo asked to go round (the side asked, +1 left, -1 right; 0 not): held while
    // the blocker is in range, whatever SimLingo asks meanwhile (at rest it may ask to wait in the lane).
    int view_stop = 0;
};

}  // namespace ex
