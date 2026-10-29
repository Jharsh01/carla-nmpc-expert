// Offline closed-loop scenarios for the behaviour layer and the urban NMPC (core/urban): the situations Bench2Drive
// is made of, scripted on a straight two-lane road (one on a curve), without CARLA.
//
// The plant is a kinematic bicycle with a first-order lag on the acceleration and a rate-limited lag on the steering,
// so the NMPC drives something it does not model exactly. It solves every 0.1 s and its plan is played at 20 Hz, as
// in the leaderboard. Other actors follow scripts; the NMPC sees each one from when it becomes visible, predicted at
// constant velocity (one braking also at constant deceleration, to standstill), as the world model will in CARLA.
// The behaviour layer (urban/behaviour.hpp) decides what the NMPC is asked to do.
//
//   build/urban_scenarios [--only NAME] [--out DIR]     exit status 1 if any scenario fails its checks
//
// Each scenario also runs as <name>@local: the behaviour layer then plans along a short path rebuilt every plan,
// as SimLingo's predicted path will be in CARLA (Behaviour::set_route), not along the scenario's road. The path
// starts at the car, joins the lane centre smoothly over 15 m with a different wobble each time, reaches 20 m ahead and
// goes on along the lane centre for 160 m (the map's lane, past SimLingo's last point). The lanes and stop lines
// are taken along it from the road, as the world model does from the map.
//
// Each scenario is checked for: no contact between the bodies, the body on the road, its own goal (stopped at
// the line, through by some time), and the solve time (p95 under 50 ms). --out writes a trace per scenario.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "urban/vehicle_params.hpp"
#include "urban/behaviour.hpp"

using namespace ex;

namespace {

constexpr double LANE = 3.5;  // [m]

struct Box;
double distance(const Box& a, const Box& b);

struct Box {  // an oriented rectangle in the world frame
    double x, y, psi, length, width;
};

std::array<std::array<double, 2>, 4> corners(const Box& b) {
    const double c = std::cos(b.psi), s = std::sin(b.psi), l = 0.5 * b.length, w = 0.5 * b.width;
    std::array<std::array<double, 2>, 4> out;
    const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
    for (int i = 0; i < 4; ++i) out[i] = {b.x + sx[i] * l * c - sy[i] * w * s, b.y + sx[i] * l * s + sy[i] * w * c};
    return out;
}

bool overlap(const Box& a, const Box& b) {  // separating axis test
    const auto ca = corners(a), cb = corners(b);
    for (double psi : {a.psi, a.psi + M_PI_2, b.psi, b.psi + M_PI_2}) {
        const double ax = std::cos(psi), ay = std::sin(psi);
        double amin = INF, amax = -INF, bmin = INF, bmax = -INF;
        for (const auto& p : ca) amin = std::min(amin, p[0] * ax + p[1] * ay), amax = std::max(amax, p[0] * ax + p[1] * ay);
        for (const auto& p : cb) bmin = std::min(bmin, p[0] * ax + p[1] * ay), bmax = std::max(bmax, p[0] * ax + p[1] * ay);
        if (amax < bmin || bmax < amin) return false;
    }
    return true;
}

// Distance between two rectangles that do not overlap: the least from a corner of one to an edge of the other.
double distance(const Box& a, const Box& b) {
    const auto ca = corners(a), cb = corners(b);
    auto to_edges = [](const std::array<double, 2>& p, const std::array<std::array<double, 2>, 4>& c) {
        double d = INF;
        for (int i = 0; i < 4; ++i) {
            const auto& u = c[i];
            const auto& v = c[(i + 1) % 4];
            const double ex = v[0] - u[0], ey = v[1] - u[1];
            const double w = std::clamp(((p[0] - u[0]) * ex + (p[1] - u[1]) * ey) / (ex * ex + ey * ey), 0.0, 1.0);
            d = std::min(d, std::hypot(p[0] - u[0] - w * ex, p[1] - u[1] - w * ey));
        }
        return d;
    };
    double d = INF;
    for (const auto& p : ca) d = std::min(d, to_edges(p, cb));
    for (const auto& p : cb) d = std::min(d, to_edges(p, ca));
    return d;
}

// An actor's motion on the straight road, where s = x and e_y = y.
struct Motion {
    double s, e, psi;  // [m], [m], heading [rad]
    double vs, ve, as;  // velocity along and across the road [m/s], acceleration along it [m/s^2]
};

struct Scripted {
    std::string name;
    double length, width;
    std::function<Motion(double)> at;
    double visible_from = 0.0;  // [s]: occluded before
    Actor::Kind kind = Actor::vehicle;
    bool parked = false;
};


struct Scenario {
    std::string name;
    Path route;
    double v0, v_max, t_end;
    std::vector<Scripted> actors;
    std::vector<LaneSection> lanes;  // empty: one lane on the route
    std::function<std::vector<StopLine>(double t)> stops;
    // the scenario's own goal, from the trace: an empty string when met, else what went wrong
    std::function<std::string(const std::vector<std::array<double, 8>>&)> goal;
    double road_lo = -0.5 * LANE, road_hi = 0.5 * LANE;  // where the body must stay [m]
    bool local = false;  // plan along a local path rebuilt every plan (<name>@local)
};

// A path as SimLingo predicts one, for the car at (s_car, e_car) on the road: from 15 m behind to 20 m ahead,
// starting at the car and joining the lane centre (with a wobble of up to `wobble`) smoothly over 15 m; then on
// along the lane centre for 160 m (as the agent goes on along the map's lane past SimLingo's last point), as far as
// the behaviour layer looks for actors (look_ahead + 80 m).
Path local_path(const Path& road, double s_car, double e_car, double wobble, std::mt19937& rng) {
    std::uniform_real_distribution<double> phase(0.0, 2.0 * M_PI);
    const double phi = phase(rng);
    auto offset = [&](double d) {  // from the road's lane centre
        const double join = d <= 0.0 ? 1.0 : d >= 15.0 ? 0.0 : 0.5 * (1.0 + std::cos(M_PI * d / 15.0));
        const double fade = d <= 20.0 ? 1.0 : d >= 30.0 ? 0.0 : 0.5 * (1.0 + std::cos(M_PI * (d - 20.0) / 10.0));
        return join * e_car + (1.0 - join) * fade * wobble * std::sin(2.0 * M_PI * d / 30.0 + phi);
    };
    std::vector<Point> points;
    for (double d = -15.0; d <= 180.0 + 1e-9; d += d < 40.0 ? 1.0 : 5.0) {
        const Pose q = road.pose(s_car + d, offset(d));
        points.push_back({q.x, q.y});
    }
    return Path::from_waypoints(points, false);
}

// The scenario's lane, every 2 m along a local path, relative to it (the road's lane is where it is on the road).
std::vector<LaneSection> lanes_along(const Path& local, const Path& road, const LaneSection& lane) {
    std::vector<LaneSection> out;
    for (double s = 0.0; s < local.length(); s += 2.0) {
        const Pose q = local.pose(s);
        const double e_road = road.project(q.x, q.y).second;
        LaneSection l = lane;
        l.s_from = s, l.s_to = s + 2.0;
        l.lo = lane.lo - e_road, l.hi = lane.hi - e_road;
        out.push_back(l);
    }
    out.back().s_to = INF;
    return out;
}

Motion constant(double s, double e, double psi, double vs, double ve, double t) {
    return {s + vs * t, e + ve * t, psi, vs, ve, 0.0};
}

// From speed v, braking at `decel` from t_brake on, to standstill.
Motion braking(double s0, double e, double v, double t_brake, double decel, double t) {
    if (t <= t_brake) return {s0 + v * t, e, 0.0, v, 0.0, 0.0};
    const double tau = std::min(t - t_brake, v / decel);
    const double moving = t - t_brake < v / decel;
    return {s0 + v * t_brake + v * tau - 0.5 * decel * tau * tau, e, 0.0, v - decel * tau, 0.0, moving ? -decel : 0.0};
}

// The actors as the world model would give them at time t: those visible, each with its trajectory predicted at
// constant velocity (one braking, to standstill).
std::vector<Actor> perceive(const Scenario& sc, double t) {
    std::vector<Actor> out;
    int id = 0;
    for (const Scripted& a : sc.actors) {
        ++id;
        if (t < a.visible_from) continue;
        const Motion m = a.at(t);
        Actor act;
        act.id = id;
        act.kind = a.kind;
        act.parked = a.parked;
        act.length = a.length;
        act.width = a.width;
        for (double tau = 0.0; tau <= 5.0 + 1e-9; tau += 0.25) {
            const double stop = m.as < 0.0 ? std::min(tau, -m.vs / m.as) : tau;
            act.t.push_back(tau);
            act.x.push_back(m.s + m.vs * stop + 0.5 * std::min(m.as, 0.0) * stop * stop);
            act.y.push_back(m.e + m.ve * tau);
            act.psi.push_back(m.psi);
        }
        out.push_back(std::move(act));
    }
    return out;
}

struct Result {
    std::string name;
    bool pass = true;
    std::vector<std::string> problems;
    double min_clearance = INF;  // smallest distance between the ego's body and another's [m]
    double max_jerk = 0.0, max_lat = 0.0, p95 = 0.0, s_end = 0.0;
    int failures = 0, solves = 0;
    std::vector<std::string> choices;  // the candidates chosen, in order, each once
};

Result run(const Scenario& sc, const VehicleParams& p, const Body& body, const std::string& out_dir) {
    UrbanNMPC::Options nmpc_options;
    nmpc_options.jit = true;  // compiled once into the cache, loaded after that
    nmpc_options.cache_dir = URBAN_CACHE_DIR;
    Behaviour behaviour(sc.route, p, body, nmpc_options, Behaviour::Options());
    std::mt19937 rng(1);
    std::vector<double> step_times;
    const double L = p.L(), lr = p.b;
    const double dt_ctrl = 0.05, dt_plant = 0.005, tau_a = 0.25, tau_d = 0.05;
    const int solve_every = 2;
    // plant state, in the world frame at the centre of gravity
    Pose start = sc.route.pose(10.0);
    double x = start.x, y = start.y, psi = start.psi, v = sc.v0, a = 0.0, delta = 0.0;
    Plan plan;
    double t_plan = 0.0;
    Result res{sc.name};
    std::vector<std::array<double, 8>> trace;  // t, s, e_y, v, a, delta, jerk, a_lat
    double a_prev = 0.0;
    std::ofstream csv;
    if (!out_dir.empty()) {
        csv.open(out_dir + "/" + sc.name + ".csv");
        csv << "t,x,y,psi,s,e_y,v,a,delta,step_ms,success\n";
    }
    for (int tick = 0; tick * dt_ctrl <= sc.t_end + 1e-9; ++tick) {
        const double t = tick * dt_ctrl;
        Ego ego{x, y, psi, v, a, delta};
        const auto [s, e] = sc.route.project(x, y);
        double solve_ms = 0.0;
        if (tick % solve_every == 0) {
            Scene scene;
            scene.t = t;
            scene.ego = ego;
            scene.v_max = sc.v_max;
            scene.lanes = sc.lanes;
            scene.actors = perceive(sc, t);
            if (sc.stops) scene.stops = sc.stops(t);
            if (sc.local) {
                Path local = local_path(sc.route, s, e, 0.15, rng);
                const LaneSection lane = sc.lanes.empty() ? LaneSection{} : sc.lanes.front();
                scene.lanes = lanes_along(local, sc.route, lane);
                for (StopLine& st : scene.stops) {
                    const Pose q = sc.route.pose(st.s);
                    st.s = local.project(q.x, q.y).first;
                }
                for (double ls = 0.0; ls < local.length(); ls += 2.0) {  // SimLingo's line: the path itself
                    scene.line_s.push_back(ls);
                    scene.line_e.push_back(0.0);
                }
                behaviour.set_route(std::move(local));
            }
            const Decision d = behaviour.step(scene);
            plan = d.plan;
            t_plan = t;
            solve_ms = 1e3 * d.wall_time;
            step_times.push_back(d.wall_time);
            if (!d.plan.success) ++res.failures;
            if (res.choices.empty() || res.choices.back() != d.label) res.choices.push_back(d.label);
            if (std::getenv("URBAN_VERBOSE")) {
                std::printf("t %5.2f s %6.1f v %5.2f  %-12s %5.1f ms |", t, s, v, d.label.c_str(), 1e3 * d.wall_time);
                for (const Candidate& c : d.candidates) {
                    const double slack = *std::max_element(c.plan.slack_max.begin(), c.plan.slack_max.end());
                    std::printf(" %s: %d it %.1f ms cost %.0f slack %.2f%s |", c.label.c_str(), c.plan.iterations,
                                1e3 * c.plan.solve_time, c.plan.cost, slack, c.plan.success ? "" : " FAILED");
                }
                for (const auto& n : d.notes) std::printf(" [%s]", n.c_str());
                std::printf("\n");
            }
        }
        // the plan's acceleration a lag ahead (the plant reaches it that much later) and its steering angle
        auto plan_at = [&](double tau, int i) {
            const auto& ts = plan.times;
            if (tau >= ts.back()) return plan.x(plan.x.rows() - 1, i);
            const size_t k = std::upper_bound(ts.begin(), ts.end(), tau) - ts.begin();
            const double w = (tau - ts[k - 1]) / (ts[k] - ts[k - 1]);
            return (1.0 - w) * plan.x(k - 1, i) + w * plan.x(k, i);
        };
        const double since = t - t_plan;
        const double a_cmd = plan_at(since + tau_a, IA), d_cmd = plan_at(since + dt_ctrl, IDELTA);

        const double a_lat = v * v * std::tan(delta) / L;
        const double jerk = tick > 0 ? (a - a_prev) / dt_ctrl : 0.0;
        a_prev = a;
        trace.push_back({t, s, e, v, a, delta, jerk, a_lat});
        res.max_jerk = std::max(res.max_jerk, std::fabs(jerk));
        res.max_lat = std::max(res.max_lat, std::fabs(a_lat));
        if (csv) csv << t << ',' << x << ',' << y << ',' << psi << ',' << s << ',' << e << ',' << v << ',' << a << ','
                     << delta << ',' << solve_ms << ',' << plan.success << '\n';

        // checks: contact with every actor (visible or not), and the body on the road
        const Box ego_box{x - body.cg_to_center * std::cos(psi), y - body.cg_to_center * std::sin(psi), psi, body.length, body.width};
        for (const Scripted& act : sc.actors) {
            const Motion m = act.at(t);
            const Box other{m.s, m.e, m.psi, act.length, act.width};
            const bool contact = overlap(ego_box, other);
            res.min_clearance = std::min(res.min_clearance, contact ? 0.0 : distance(ego_box, other));
            if (contact) {
                res.pass = false;
                char buf[128];
                std::snprintf(buf, sizeof buf, "contact with %s at t = %.2f s", act.name.c_str(), t);
                if (res.problems.empty() || res.problems.back().find("contact") == std::string::npos)
                    res.problems.push_back(buf);
            }
        }
        double e_min = INF, e_max = -INF;
        for (const auto& c : corners(ego_box)) {
            const auto [cs, ce] = sc.route.project(c[0], c[1]);
            (void)cs;
            e_min = std::min(e_min, ce), e_max = std::max(e_max, ce);
        }
        if (e_min < sc.road_lo - 0.1 || e_max > sc.road_hi + 0.1) {
            if (std::none_of(res.problems.begin(), res.problems.end(), [](const std::string& q) { return q.find("road") != std::string::npos; })) {
                char buf[128];
                std::snprintf(buf, sizeof buf, "off the road at t = %.2f s (e_y of the body %.2f to %.2f m)", t, e_min, e_max);
                res.problems.push_back(buf);
            }
            res.pass = false;
        }

        // the plant over one control period
        for (int i = 0; i < static_cast<int>(std::lround(dt_ctrl / dt_plant)); ++i) {
            const double beta = std::atan(lr * std::tan(delta) / L);
            x += dt_plant * v * std::cos(psi + beta);
            y += dt_plant * v * std::sin(psi + beta);
            psi += dt_plant * v * std::cos(beta) * std::tan(delta) / L;
            v = std::max(0.0, v + dt_plant * a);
            if (v == 0.0 && a < 0.0) a = 0.0;  // brakes hold a car at standstill, they do not reverse it
            a += dt_plant * (a_cmd - a) / tau_a;
            const double rate = std::clamp((d_cmd - delta) / tau_d, -p.delta_rate_max, p.delta_rate_max);
            delta = std::clamp(delta + dt_plant * rate, -p.delta_max, p.delta_max);
        }
    }
    std::vector<double> times = step_times;
    std::sort(times.begin(), times.end());
    res.solves = static_cast<int>(times.size());
    res.p95 = times.empty() ? 0.0 : times[static_cast<size_t>(0.95 * (times.size() - 1))];
    res.s_end = trace.back()[1];
    if (res.p95 > 0.05) {
        res.pass = false;
        res.problems.push_back("solve time p95 " + std::to_string(1e3 * res.p95) + " ms");
    }
    if (sc.goal) {
        const std::string why = sc.goal(trace);
        if (!why.empty()) res.pass = false, res.problems.push_back(why);
    }
    return res;
}

Path straight() { return Path::from_waypoints({{0.0, 0.0}, {400.0, 0.0}}, false); }

// Goal helpers on the trace (t, s, e_y, v, a, delta, jerk, a_lat).
std::function<std::string(const std::vector<std::array<double, 8>>&)> reach(double s_goal, double by) {
    return [=](const std::vector<std::array<double, 8>>& tr) -> std::string {
        for (const auto& r : tr)
            if (r[1] >= s_goal && r[0] <= by) return "";
        return "did not reach s = " + std::to_string(s_goal) + " m by t = " + std::to_string(by) + " s";
    };
}

std::vector<Scenario> scenarios(const Body& body) {
    const double front = 0.5 * body.length - body.cg_to_center;
    std::vector<Scenario> out;
    // the lane to the left: the same way (no crossing it) or oncoming (that may be borrowed)
    const LaneSection two_way{0.0, INF, -0.5 * LANE, 0.5 * LANE, LANE, true, true};
    const LaneSection same_way{0.0, INF, -0.5 * LANE, 0.5 * LANE, LANE, false, false};

    {  // lane keeping round a 30 m radius right-angle bend
        Scenario sc{"curve", Path::from_pieces({0.0, 0.0, 0.0}, {60.0, 0.5 * M_PI * 30.0, 150.0}, {0.0, 1.0 / 30.0, 0.0}, false),
                    12.0, 13.9, 15.0};
        sc.goal = [](const std::vector<std::array<double, 8>>& tr) -> std::string {
            for (const auto& r : tr)
                if (std::fabs(r[2]) > 0.3) return "off the lane centre by " + std::to_string(r[2]) + " m";
            return tr.back()[1] > 150.0 ? "" : "too slow round the bend";
        };
        out.push_back(sc);
    }
    {  // the car ahead brakes hard to standstill
        Scenario sc{"lead_brake", straight(), 12.0, 13.9, 12.0};
        sc.actors.push_back({"lead", 4.8, 2.0, [](double t) { return braking(40.0, 0.0, 12.0, 3.0, 6.0, t); }});
        sc.goal = [](const std::vector<std::array<double, 8>>& tr) -> std::string {
            return tr.back()[3] < 0.2 ? "" : "not stopped behind the lead";
        };
        out.push_back(sc);
    }
    {  // a slower car in the left lane cuts in close ahead
        Scenario sc{"cut_in", straight(), 12.0, 13.9, 12.0};
        sc.lanes = {same_way};
        sc.actors.push_back({"cutter", 4.8, 2.0, [](double t) {
                                 const double e = t < 1.0 ? LANE : t < 3.5 ? LANE * (1.0 - (t - 1.0) / 2.5) : 0.0;
                                 const double ve = t >= 1.0 && t < 3.5 ? -LANE / 2.5 : 0.0;
                                 return Motion{25.0 + 8.0 * t, e, std::atan2(ve, 8.0), 8.0, ve, 0.0};
                             }});
        sc.road_hi = 1.5 * LANE;
        sc.goal = reach(80.0, 12.0);
        out.push_back(sc);
    }
    {  // a pedestrian steps out from behind a parked van
        Scenario sc{"pedestrian_behind_van", straight(), 10.0, 11.1, 12.0};
        sc.actors.push_back({"van", 5.5, 2.2, [](double t) { return constant(42.0, -0.5 * LANE - 1.2, 0.0, 0.0, 0.0, t); },
                             0.0, Actor::vehicle, true});
        sc.actors.push_back({"pedestrian", 0.5, 0.5,
                             [](double t) {
                                 const double go = std::max(0.0, t - 1.6);
                                 return Motion{46.0, -0.5 * LANE - 1.0 + 1.4 * go, M_PI_2, 0.0, t > 1.6 ? 1.4 : 0.0, 0.0};
                             },
                             1.6, Actor::walker});
        sc.road_lo = -0.5 * LANE - 2.4;  // the parking strip
        sc.goal = reach(70.0, 12.0);
        out.push_back(sc);
    }
    {  // a red light at s = 60 m turns green at t = 9 s
        Scenario sc{"red_light", straight(), 12.0, 13.9, 16.0};
        sc.stops = [](double t) { return t < 9.0 ? std::vector<StopLine>{{1, StopLine::red_light, 60.0}} : std::vector<StopLine>{}; };
        sc.goal = [front](const std::vector<std::array<double, 8>>& tr) -> std::string {
            bool stopped = false;
            for (const auto& r : tr) {
                if (r[0] < 9.0 && r[1] + front > 60.3) return "over the stop line on red";
                if (r[0] < 9.0 && r[3] < 0.1) stopped = true;
            }
            if (!stopped) return "never stopped at the light";
            return tr.back()[1] > 90.0 ? "" : "did not go on green";
        };
        out.push_back(sc);
    }
    {  // the light turns red as the car is 15 m from it at 12 m/s: too late to stop comfortably, so through
        Scenario sc{"red_light_dilemma", straight(), 12.0, 13.9, 8.0};
        sc.stops = [](double t) { return t >= 2.4 ? std::vector<StopLine>{{1, StopLine::red_light, 60.0}} : std::vector<StopLine>{}; };
        sc.goal = reach(80.0, 8.0);
        out.push_back(sc);
    }
    {  // a stop sign at s = 50 m: stop at the line for a second, then go
        Scenario sc{"stop_sign", straight(), 10.0, 11.1, 16.0};
        sc.stops = [](double) { return std::vector<StopLine>{{2, StopLine::stop_sign, 50.0}}; };
        sc.goal = [front](const std::vector<std::array<double, 8>>& tr) -> std::string {
            double rest = 0.0;
            for (size_t i = 1; i < tr.size(); ++i) {
                if (tr[i][1] + front > 50.3 && rest < 1.0) return "past the stop line before stopping";
                if (tr[i][3] < 0.1 && tr[i][1] + front > 46.0) rest += tr[i][0] - tr[i - 1][0];
            }
            return tr.back()[1] > 80.0 ? "" : "did not go on after stopping";
        };
        out.push_back(sc);
    }
    for (double oncoming_s : {220.0, 110.0}) {  // a parked car blocks the lane; overtake in the oncoming lane when clear
        Scenario sc{oncoming_s > 150.0 ? "parked_car_overtake" : "parked_car_wait", straight(), 8.0, 8.3, 25.0};
        sc.lanes = {two_way};
        sc.actors.push_back({"parked", 4.8, 2.0, [](double t) { return constant(55.0, 0.0, 0.0, 0.0, 0.0, t); }, 0.0,
                             Actor::vehicle, true});
        sc.actors.push_back({"oncoming", 4.8, 2.0, [oncoming_s](double t) { return constant(oncoming_s, LANE, M_PI, -8.0, 0.0, t); }});
        sc.road_hi = 1.5 * LANE;
        sc.goal = reach(100.0, 25.0);
        out.push_back(sc);
    }
    {  // a car crosses at a junction at s = 45 m, arriving as the car would
        Scenario sc{"crossing_yield", straight(), 10.0, 11.1, 14.0};
        sc.actors.push_back({"crossing", 4.8, 2.0, [](double t) { return constant(45.0, -40.0, M_PI_2, 0.0, 9.0, t); }});
        sc.goal = reach(70.0, 14.0);
        out.push_back(sc);
    }
    {  // a crossing car far enough ahead in time to go before it
        Scenario sc{"crossing_pass", straight(), 10.0, 11.1, 10.0};
        sc.actors.push_back({"crossing", 4.8, 2.0, [](double t) { return constant(45.0, -75.0, M_PI_2, 0.0, 9.0, t); }});
        sc.goal = reach(80.0, 10.0);
        out.push_back(sc);
    }
    const size_t n = out.size();
    for (size_t i = 0; i < n; ++i) {  // each again, planned along SimLingo-like local paths
        if (out[i].lanes.size() > 1) continue;  // lanes_along takes one lane section
        Scenario sc = out[i];
        sc.name += "@local";
        sc.local = true;
        out.push_back(sc);
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string only, out_dir;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--only") && i + 1 < argc) only = argv[++i];
        else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out_dir = argv[++i];
        else {
            std::fprintf(stderr, "usage: %s [--only NAME] [--out DIR]\n", argv[0]);
            return 2;
        }
    }
    const VehicleParams p = VehicleParams::from_file(SIM_CONFIG_DIR "/vehicles/mkz2020.json");
    const Body body;
    bool all = true;
    std::printf("%-24s %-5s %9s %8s %8s %8s %9s %6s  %-28s %s\n", "scenario", "", "clear [m]", "jerk", "a_lat", "s_end",
                "p95 [ms]", "fails", "chosen", "problems");
    for (const Scenario& sc : scenarios(body)) {
        if (!only.empty() && sc.name != only) continue;
        const Result r = run(sc, p, body, out_dir);
        all = all && r.pass;
        std::string problems;
        for (const auto& q : r.problems) problems += (problems.empty() ? "" : "; ") + q;
        std::string choices;
        for (const auto& c : r.choices) choices += (choices.empty() ? "" : ">") + c;
        std::printf("%-24s %-5s %9.2f %8.1f %8.2f %8.1f %9.1f %6d  %-28s %s\n", r.name.c_str(), r.pass ? "ok" : "FAIL",
                    r.min_clearance, r.max_jerk, r.max_lat, r.s_end, 1e3 * r.p95, r.failures, choices.c_str(), problems.c_str());
    }
    return all ? 0 : 1;
}
