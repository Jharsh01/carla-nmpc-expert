// The sensor half of the behaviour layer: the perceived `Scene` (CARLA's privileged information, gathered by
// expert/world_model) taken into the route's path coordinates and interpreted, before anything is asked of the NMPC.
//
// Every actor's predicted trajectory is projected onto the route and classified against the ego lane: a *lead* (in
// the lane ahead, going along it), a *blocker* (in the lane ahead and parked: an accident, construction, a parked
// car), a *crosser* (entering the lane, or in it going across: pedestrians, junction traffic, cut-ins) or a *mover*
// (anything else nearby: kept clear of as circles). A car that stays out of the lanes the ego is using
// (Actor::stays_out) is none of these. Traffic coming the other way, and traffic going the ego's way, is noted
// whatever its class, for the checks that a neighbouring lane is clear.
//
// Traffic-rule state lives here too: a stop sign is released once the car has stood at it for `stop_hold`, and a
// light turning red when the car could only stop at more than `dilemma_decel` is driven through. What is left are
// the stop lines to stop at.
//
// What the NMPC is then asked to do, from this `Percept`, is constraints.hpp's.
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "urban/urban_nmpc.hpp"

namespace ex {

// ---- the scene as perceived

// Another road user or object, with its predicted motion.
struct Actor {
    enum Kind { vehicle, bike, walker, object };
    int id = 0;
    Kind kind = vehicle;
    double length = 4.8, width = 2.0;  // [m]
    bool parked = false;  // a vehicle that will not move off: parked, crashed, broken down
    // A car with no intent to use the lanes the ego is using (on the map: expert/world_model): left out of the plan,
    // but for what would take the ego into its lane (borrowing the left lane).
    bool stays_out = false;
    // the centre's predicted trajectory in the world frame, at times t from the scene's time; t[0] = 0 is now
    std::vector<double> t, x, y, psi;
};

// The road along a stretch of the route [s_from, s_to): the ego lane's boundaries and what lies beyond them, as the
// map has it there (expert/world_model/route.py): whether there is a lane either side, and which way it runs.
struct LaneSection {
    double s_from = 0.0, s_to = INF;  // [m]
    double lo = -1.75, hi = 1.75;  // e_y of the ego lane's right and left boundaries [m]
    double left_width = 0.0;  // drivable width beyond hi: the lane to the left [m], 0 if none
    bool left_oncoming = true;  // that lane runs the other way
    bool left_allowed = false;  // it may be driven into, to go round a blocker
    // The road's edges: the outer edges of its outermost driving lanes either side (beyond them a shoulder, a kerb,
    // a sidewalk). The corridor never goes past them, not when it is widened to where the car is, nor round a blocker.
    double road_lo = -INF, road_hi = INF;  // [m]
    double right_width = 0.0;  // drivable width beyond lo: the lane to the right [m], 0 if none
    bool right_oncoming = false;  // that lane runs the other way
    bool right_allowed = false;  // it may be driven into, to go round a blocker
    // The part of [lo, hi] in lanes that run the ego's way. Where SimLingo's path goes into an oncoming lane the band
    // takes that lane in too, but room there is no way past a parked car: it is the oncoming traffic's, and going
    // round through it is a borrow, judged on that traffic. Empty (way_lo > way_hi) if the band is all oncoming.
    double way_lo = -INF, way_hi = INF;  // [m]

    // the lane on one side: its width (0 if there is none or it may not be driven into), and which way it runs
    double side_width(bool left) const { return (left ? left_allowed : right_allowed) ? (left ? left_width : right_width) : 0.0; }
    bool side_oncoming(bool left) const { return left ? left_oncoming : right_oncoming; }
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
    // How far ahead of the car it can see along the lane beside its own on each side [m] (expert/world_model/sight.py):
    // a car not reported may be anywhere beyond. INF: not limited (or no lane there).
    double sight_left = INF, sight_right = INF;
};

// ---- the scene as interpreted

// An actor's predicted trajectory in path coordinates, with the half extents of its body along and across the route.
struct Track {
    const Actor* actor = nullptr;  // in the Scene sensed
    std::vector<double> t, s, e, ext_s, ext_e;
    double cos_rel = 1.0;  // of its heading relative to the route, now
    double speed = 0.0;  // now [m/s]
};

struct Crosser {
    int id = 0;
    double t_in, t_out, s_lo, s_hi;  // its window in the lane, and the span of the route it takes up there
};

struct Blocker {
    double s_lo, s_hi;
    size_t track;  // into Percept::tracks
};

struct Percept {
    Eigen::Matrix<double, NX, 1> z0;  // the ego in path coordinates
    double s0 = 0.0, v0 = 0.0, front0 = 0.0, rear0 = 0.0;  // where it is, its speed, its body's ends along the route
    std::vector<LaneSection> lanes;  // the Scene's, or the default lane
    const LaneSection& section(double s) const;  // the one at s (the nearest end's beyond them)

    std::vector<Track> tracks;  // the actors near the route
    std::vector<Lead> leads;
    std::vector<Blocker> blockers;
    std::vector<Crosser> crossers;
    std::vector<Mover> movers;  // what is kept clear of as circles
    std::vector<Mover> moving;  // of those, the moving ones (less the parked cars and objects)
    std::vector<Mover> stay_out;  // the circles of the cars that stay out of the ego's lanes
    std::vector<size_t> still;  // the tracks of the parked cars and objects among the movers
    std::vector<size_t> oncoming;  // the tracks of the traffic coming the other way
    std::vector<size_t> along;  // the tracks of the traffic going the ego's way (vehicles and bikes, not parked)
    std::vector<double> stops;  // the stop lines to stop at, along the route [m]
    std::vector<std::string> notes;  // classification and rule events, for the log
};

class Sensor {
public:
    struct Options {
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
        double stop_hold = 1.0;  // [s] at a stop sign
        double dilemma_decel = 5.0;  // a light turning red that needs more than this to stop for is driven through [m/s^2]
    };

    Sensor(Body body, Options options) : body(body), opt(options) {}

    // The scene along `route`, with the ego placed on it by `nmpc` (whose comfortable braking decides an amber light).
    // The Percept's tracks point into `scene`, which must outlive it.
    Percept sense(const Scene& scene, const Path& route, const UrbanNMPC& nmpc);
    void reset();  // forget the rule state (a new route)

    const Body body;
    const Options opt;

private:
    // traffic-rule state, by stop id
    std::map<int, double> stopped_since_;  // stop signs: when the car came to rest at the line
    std::set<int> released_;  // stop signs held at, and lights driven through on amber or in the dilemma zone
    std::set<int> seen_;  // lights already decided on
};

}  // namespace ex
