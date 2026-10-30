// The behaviour layer: what the urban NMPC is asked to do, decided from a perceived `Scene`.
//
// The NMPC (urban_nmpc.hpp) plans locally: it cannot decide by itself whether to wait for a pedestrian or go
// before them, or whether to stay behind a parked car or overtake it in the oncoming lane. A plan through the
// middle of an obstacle has no gradient towards either side, and the solver stays there. So each step:
//
// 1. Every actor's predicted trajectory is taken into the route's path coordinates and classified against the ego
//    lane: a *lead* (in the lane ahead, going along it), a *blocker* (in the lane ahead and parked: an accident,
//    construction, a parked car), a *crosser* (entering the lane, or in it going across: pedestrians, junction
//    traffic, cut-ins) or a *mover* (anything else nearby: kept clear of as circles).
// 2. Candidates are built from discrete choices: yield to each crosser (a gate keeping the front behind where it
//    crosses while it does) or pass ahead of it (a gate keeping the rear past it), stay behind a blocker or borrow
//    the neighbouring lane round it (only with a gap in the oncoming traffic). Stop lines (red lights, stop signs,
//    the end of the route) are gates in every candidate.
// 3. The candidates are solved in parallel, each by its own NMPC (warm-started from its own last plan), and the
//    best is chosen, the one chosen last time favoured, so the choice does not flicker. They are compared on what
//    their plans do, not on the NMPC's own costs, which differ between them: speed against the speed asked for
//    (not tapered to the stops: waiting is not free), comfort, time out of the ego lane and constraint violation,
//    with a penalty for going before a crosser rather than after.
//
// Traffic-rule state lives here too: a stop sign is released once the car has stood at it for `stop_hold`, and a
// light turning red when the car could only stop at more than `dilemma_decel` is driven through.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "urban/urban_nmpc.hpp"

namespace ex {

// Another road user or object, with its predicted motion.
struct Actor {
    enum Kind { vehicle, bike, walker, object };
    int id = 0;
    Kind kind = vehicle;
    double length = 4.8, width = 2.0;  // [m]
    bool parked = false;  // a vehicle that will not move off: parked, crashed, broken down
    // the centre's predicted trajectory in the world frame, at times t from the scene's time; t[0] = 0 is now
    std::vector<double> t, x, y, psi;
};

// The road along a stretch of the route [s_from, s_to): the ego lane's boundaries and what lies beyond them.
struct LaneSection {
    double s_from = 0.0, s_to = INF;  // [m]
    double lo = -1.75, hi = 1.75;  // e_y of the ego lane's right and left boundaries [m]
    double left_width = 0.0;  // drivable width beyond hi: the lane to the left [m], 0 if none
    bool left_oncoming = true;  // that lane runs the other way
    bool left_allowed = false;  // it may be driven into, to go round a blocker
    // The road's edges: the outer edges of its outermost driving lanes either side (beyond them a shoulder, a kerb,
    // a sidewalk). The corridor never goes past them, not when it is widened to where the car is, nor round a blocker.
    double road_lo = -INF, road_hi = INF;  // [m]
};

struct StopLine {
    enum Kind { red_light, amber_light, stop_sign, route_end };
    int id = 0;
    Kind kind = red_light;
    double s = 0.0;  // along the route [m]
};

struct Scene {
    double t = 0.0;  // [s]
    Ego ego;
    double v_max = 50.0 / 3.6;  // speed limit [m/s]
    std::vector<double> v_ref_t, v_ref;  // speed asked for over time (e.g. SimLingo's); empty: the limit
    std::vector<double> line_s, line_e;  // the line asked for along the route (e.g. SimLingo's path); empty: lane centre
    std::vector<LaneSection> lanes;  // empty: one 3.5 m lane centred on the route
    std::vector<Actor> actors;
    std::vector<StopLine> stops;
};

// One candidate as solved.
struct Candidate {
    std::string label;  // "base", "pass:<id>", "borrow_left"
    Problem problem;
    Plan plan;
    double score = INF;  // what the choice is made on (lower is better)
};

struct Decision {
    std::string label;  // the candidate chosen
    Plan plan;
    std::vector<Candidate> candidates;
    double wall_time = 0.0;  // for the whole step [s]
    std::vector<std::string> notes;  // classification and rule events, for the log
};

class Behaviour {
public:
    struct Options {
        int pool = 4;  // NMPC instances, the most candidates solved at once
        double look_ahead = 80.0;  // actors and stops further along the route than this are left out [m]
        double gate_margin = 1.0;  // kept between the ego's body and a crosser's [m]
        double gate_buffer = 1.0;  // a crosser's window widened by this much either side [s]
        double blocker_range = 40.0;  // a blocker nearer than this may be overtaken [m]
        // A parked car or an object is in the lane (a blocker, or a lead) only if it reaches this far into it; one
        // that only grazes the lane is kept clear of as a mover, by nudging within the lane. Without it a car parked
        // beside the lane, its box tilted a little against a wobbling path (SimLingo's), blocks the lane.
        double blocker_overlap = 0.3;  // [m]
        // Kept to parked cars and objects instead of the NMPC's margin, without the growth with prediction time a
        // moving actor's clearance has: nothing about where they will be is uncertain. With the NMPC's 0.5 m and the
        // circles' cover, a car parked beside a 2.9 m lane (2.65 m off the path) could not be driven past at all
        // (Bench2Drive route 0).
        double static_margin = 0.2;  // [m]
        double oncoming_gap = 20.0;  // kept to oncoming traffic when overtaking [m]
        double stop_hold = 1.0;  // [s] at a stop sign
        double dilemma_decel = 5.0;  // a light turning red that needs more than this to stop for is driven through [m/s^2]
        double hysteresis = 0.9;  // the last choice's score is multiplied by this
        double w_out_of_lane = 0.2;  // on the squared distance from the ego lane's centre, in the score
        double pass_penalty = 300.0;  // added to the score for each crosser gone ahead of
        double slack_ok = 0.05;  // a candidate with any slack above this is only chosen when none is within it
    };

    Behaviour(Path route, VehicleParams params, Body body, UrbanNMPC::Options nmpc_options, Options options);
    Behaviour(Path route, VehicleParams params) : Behaviour(std::move(route), params, Body(), {}, {}) {}
    ~Behaviour();

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
