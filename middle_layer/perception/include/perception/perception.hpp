// The middle layer's perception: the scene the sensors report (middle_layer/perception/sensors: the actors, the map's
// lanes and stop lines, the depth camera's sight, all in the car's frame), post-processed into the route's path
// coordinates and interpreted, before anything is asked of the NMPC (nmpc/).
//
//   post_process/tracks.cpp    each actor's predicted trajectory projected onto the route, and its circles
//   post_process/classify.cpp  each track against the ego lane: lead, blocker, crosser, mover; oncoming, along
//   post_process/lanes.cpp     the lane section at a point, in the lane or not, the part running the car's way
//   post_process/left_dir.cpp  the lane on the left: there or not, how wide, which way it runs
//   post_process/right_dir.cpp the lane on the right, the same
//   post_process/perception.cpp  all of it, once per plan: Perception::perceive
//
// A *lead* is in the lane ahead, going along it; a *blocker* in the lane ahead and parked (an accident,
// construction, a parked car), unless the band leaves room past it in lanes running the car's way; a *crosser*
// enters the lane or is in it going across (pedestrians, junction traffic, cut-ins); a *mover* is anything else
// nearby, kept clear of as circles. A car that stays out of the lanes the ego is using (Actor::stays_out) is none
// of these. Traffic coming the other way, and traffic going the ego's way, is noted whatever its class, for the
// checks that a neighbouring lane is clear (nmpc/constraint/overtake).
//
// What to do about it is the NMPC's manoeuvres' (nmpc/constraint/), stop lines and traffic rules included.
#pragma once

#include <string>
#include <vector>

#include "nmpc/path.hpp"
#include "nmpc/problem.hpp"
#include "nmpc/shared_param.hpp"

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
};

struct StopLine {
    enum Kind { red_light, amber_light, stop_sign, route_end };
    int id = 0;
    Kind kind = red_light;
    double s = 0.0;  // along the route [m]
};

// A lane of the junction ahead that merges with the route (it ends in the lane the route leaves the junction by) or
// crosses it, as the map has it (middle_layer/perception/sensors/map.py: Route.conflicts): where along the route the
// two overlap, how far its traffic comes along it from the junction's edge to there, and how far up the road before
// the junction that lane is seen. A car not seen may be coming just beyond.
struct Conflict {
    bool merge = false;
    double s_lo = 0.0, s_hi = 0.0;  // [m]
    double lead_in = 0.0;  // [m]
    double sight = INF;  // [m]; INF: all of it that matters
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
    // The side SimLingo's line leaves the lane to (+1 left, -1 right, 0 it keeps to the lane; PathIntent::asks): with
    // a blocker ahead, the overtake on that side is decided at once, gone round now if clear, else the car stops
    // where it is, as far back as it can see past the blocker from (nmpc/constraint/candidates.cpp).
    int asks_change = 0;
    std::vector<Conflict> conflicts;  // of the junction the route turns at; empty: none, or not turning
};

// ---- the scene as interpreted

// An actor's predicted trajectory in path coordinates, with the half extents of its body along and across the route.
struct Track {
    const Actor* actor = nullptr;  // in the Scene sensed
    std::vector<double> t, s, e, ext_s, ext_e;
    double cos_rel = 1.0;  // of its heading relative to the route, now
    double cos_end = 1.0;  // and at the end of its track, where it is then
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

// A car joining the route's lane from a side road and going on along it (the stream the ego turns into at a
// junction): not in the lane now, in it from t_in for as long as it is predicted, moving along the route there.
// Before it is in, the ego either waits for it (then follows it: `lead`) or goes in ahead of it (constraint/merge).
struct Merger {
    int id = 0;
    size_t track = 0;  // in Percept::tracks
    double t_in = 0.0;  // [s]
    double s_in = 0.0;  // where its rear is along the route as it comes in [m]
    double speed = 0.0;  // along the route once in [m/s]
    Lead lead;  // its rear from t_in on (far ahead before)
};

struct Percept {
    const Path* route = nullptr;  // the path perceived along (the planner's; it outlives the Percept)
    Eigen::Matrix<double, NX, 1> z0;  // the ego in path coordinates
    double s0 = 0.0, v0 = 0.0, front0 = 0.0, rear0 = 0.0;  // where it is, its speed, its body's ends along the route
    std::vector<LaneSection> lanes;  // the Scene's, or the default lane
    const LaneSection& section(double s) const;  // the one at s (the nearest end's beyond them; lanes.hpp)

    std::vector<Track> tracks;  // the actors near the route
    std::vector<Lead> leads;
    std::vector<Blocker> blockers;
    std::vector<Crosser> crossers;
    std::vector<Merger> mergers;
    std::vector<std::pair<size_t, double>> joining;  // tracks joining the lane behind the ego, and when [s]
    std::vector<double> bend_s, bend_kappa;  // the route's curvature ahead of the car, every 2 m [m], [1/m]
    std::vector<Mover> movers;  // what is kept clear of as circles
    std::vector<Mover> moving;  // of those, the moving ones (less the parked cars and objects)
    std::vector<Mover> stay_out;  // the circles of the cars that stay out of the ego's lanes
    std::vector<size_t> still;  // the tracks of the parked cars and objects among the movers
    std::vector<size_t> oncoming;  // the tracks of the traffic coming the other way
    std::vector<size_t> along;  // the tracks of the traffic going the ego's way (vehicles and bikes, not parked)
    std::vector<std::string> notes;  // classification events, for the log
};

class Perception {
public:
    using Options = PerceptionOptions;  // nmpc/shared_param.hpp

    Perception(Body body, Options options) : body(body), opt(options) {}

    // The scene along `route`, with the ego at path state z0 there (UrbanNMPC::path_state). The Percept's tracks point
    // into `scene`, which must outlive it.
    Percept perceive(const Scene& scene, const Path& route, const Eigen::Matrix<double, NX, 1>& z0) const;

    const Body body;
    const Options opt;
};

}  // namespace ex
