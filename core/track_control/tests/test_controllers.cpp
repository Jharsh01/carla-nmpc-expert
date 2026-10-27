// Both controllers in closed loop on the library's bicycle model: they follow the lane, hold speed, and pass.
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "check.hpp"
#include "track_control/model/bicycle.hpp"
#include "track_control/model/integrate.hpp"
#include "track_control/nmpc.hpp"
#include "track_control/pure_pursuit.hpp"

using namespace tc;

namespace {

const Path PATH = Path::from_corners({{0, 0}, {200, 0}, {200, 80}, {0, 80}}, {30, 30, 30, 30}, {60, 0});
const double RIGHT = -1.75;

struct Run {
    double worst_offset = 0.0;  // furthest from the right lane centre [m]
    double min_gap = INFINITY;  // closest approach to the obstacle, along the path [m]
    double end_speed = 0.0;
    double distance = 0.0;
    double end_offset = 0.0;  // e_y at the end [m]
};

// Drive for `duration` seconds; an obstacle, if `obstacle_speed` is set, starts 40 m ahead in the right lane.
// `watch` sees the time, path position and offset after each step.
Run drive(Controller& controller, double duration, double obstacle_speed = -1.0,
          const std::function<void(double, double, double)>& watch = {}) {
    const VehicleParams p;
    const Pose start = PATH.pose(0.0, RIGHT);
    State x = State::Zero();
    x << start.x, start.y, start.psi, 12.0, 0.0, 0.0, 0.0;
    Run run;
    double s_prev = 0.0;
    for (double t = 0.0; t < duration; t += 0.05) {
        Observation obs;
        obs.t = t;
        obs.x = x;
        const double obstacle_s = 40.0 + obstacle_speed * t;
        if (obstacle_speed >= 0.0) obs.obstacles.push_back({PATH.wrap(obstacle_s), RIGHT, obstacle_speed});
        Input u = controller.compute(obs);
        const auto [fx_min, fx_max] = fx_limits(x[IUX], p);
        u[IRATE] = std::clamp(u[IRATE], -p.delta_rate_max, p.delta_rate_max);
        u[IFX] = std::clamp(u[IFX], fx_min, fx_max);
        for (int i = 0; i < 50; ++i)
            x = rk4_step([&](const State& z, const Input& v) { return dynamics(z, v, p); }, x, u, 1e-3);
        const auto [s, e_y] = PATH.project(x[IX], x[IY]);
        run.distance += PATH.difference(s, s_prev);
        s_prev = s;
        run.end_offset = e_y;
        if (watch) watch(t, s, e_y);
        if (obstacle_speed >= 0.0) {
            const double gap = std::fabs(PATH.difference(PATH.wrap(obstacle_s), s));
            if (std::fabs(e_y - RIGHT) < 2.0) run.min_gap = std::min(run.min_gap, gap);  // only while in its lane
        } else {
            run.worst_offset = std::max(run.worst_offset, std::fabs(e_y - RIGHT));
        }
    }
    run.end_speed = x[IUX];
    return run;
}

Road road() {
    Road r;
    r.v_max = 15.0;
    return r;
}

}  // namespace

static void pure_pursuit_follows() {
    PurePursuit::Options o;
    o.lane_offset = RIGHT;
    o.pass_offset = -RIGHT;
    PurePursuit controller(PATH, road(), VehicleParams(), Body(), o);
    const Run run = drive(controller, 55.0);  // more than a lap
    CHECK(run.distance > PATH.length());
    CHECK(run.worst_offset < 0.8);
}

static void pure_pursuit_passes() {
    PurePursuit::Options o;
    o.lane_offset = RIGHT;
    o.pass_offset = -RIGHT;
    PurePursuit controller(PATH, road(), VehicleParams(), Body(), o);
    const Run run = drive(controller, 25.0, 5.0);
    CHECK(run.distance > 40.0 + 5.0 * 25.0 + 10.0);  // got past it
    CHECK(run.min_gap > 5.0);  // never close behind it in its lane
}

// Given a reference, it drives the reference's line and speed instead of its own lane and profile, and its own
// again once the reference has ended.
static void pure_pursuit_follows_a_reference() {
    PurePursuit::Options o;
    o.lane_offset = RIGHT;
    o.pass_offset = -RIGHT;
    PurePursuit controller(PATH, road(), VehicleParams(), Body(), o);
    PurePursuit::Reference ref;
    ref.t_end = 12.0;
    for (double s = 0.0; s <= 150.0; s += 5.0) {
        ref.s.push_back(s);
        ref.e_y.push_back(-RIGHT);
        ref.v.push_back(10.0);
    }
    controller.follow(ref);
    double e_y_at_10 = NAN, s_at_9 = NAN, s_at_10 = NAN;
    const Run run = drive(controller, 20.0, -1.0, [&](double t, double s, double e_y) {
        if (std::fabs(t - 9.0) < 0.025) s_at_9 = s;
        if (std::fabs(t - 10.0) < 0.025) s_at_10 = s, e_y_at_10 = e_y;
    });
    CHECK_NEAR(e_y_at_10, -RIGHT, 0.3);  // on the reference's line
    CHECK_NEAR(s_at_10 - s_at_9, 10.0, 0.5);  // at its speed
    CHECK_NEAR(run.end_offset, RIGHT, 0.5);  // back in its own lane
}

// Braking hard in a corner, the force stays within the grip the cornering leaves the rear axle, which carries the
// least load and loses more of it as the car brakes.
static void pure_pursuit_brakes_within_the_grip() {
    const VehicleParams p;
    PurePursuit controller(PATH, road(), p, Body(), PurePursuit::Options());
    const Pose start = PATH.pose(0.0, 0.0);
    State x = State::Zero();
    x << start.x, start.y, start.psi, 25.0, 0.0, 0.25, 0.0;  // far over the 15 m/s limit, at 0.64 g
    Observation obs;
    obs.x = x;
    const double fx = controller.compute(obs)[IFX];
    CHECK(fx < -500.0);  // it does brake
    const double a_y = x[IUX] * x[IR], L = p.L();
    const double Fzr = (p.m * p.g * p.a + p.h * fx) / L, Fxr = (1.0 - p.brake_front) * fx;
    const double Fyr = p.m * a_y * p.a / L;
    CHECK(std::hypot(Fxr, Fyr) <= 0.9 * p.mu_r() * Fzr + 1e-6);  // the rear within axle_grip_use of its friction
    CHECK(-fx < 0.7 * p.F_brake_max);  // the brakes alone would allow more
    // laterally saturated already, it does not brake at all
    x[IR] = 0.33;
    obs.x = x;
    CHECK(controller.compute(obs)[IFX] >= 0.0);
}

static void nmpc_follows_and_passes() {
    NMPC::Options o;
    o.lane_offset = RIGHT;
    o.horizon = 3.0;
    {
        NMPC controller(PATH, road(), VehicleParams(), Body(), o);
        const Run run = drive(controller, 20.0);
        CHECK(run.distance > 250.0);
        CHECK(run.worst_offset < 0.8);
        CHECK_NEAR(run.end_speed, 15.0, 1.5);
        CHECK(controller.failures == 0);
    }
    {
        NMPC controller(PATH, road(), VehicleParams(), Body(), o);
        const Run run = drive(controller, 20.0, 5.0);
        CHECK(run.distance > 40.0 + 5.0 * 20.0 + 10.0);
        CHECK(run.min_gap > 5.0);
    }
}

// A 2 s horizon at about 20 m/s ends near the end of the 40 m of road known: the speed limit there is the braking
// envelope down to the unseen speed, continuous in s, so the plan does not swing between accelerating and braking
// with whether its end falls just inside or just outside the known road.
static void nmpc_steady_with_the_road_known_as_far_as_it_plans() {
    const Path straight = Path::from_corners({{0, 0}, {1000, 0}, {1000, 80}, {0, 80}}, {30, 30, 30, 30}, {60, 0});
    Road r = road();
    r.v_max = 25.0;
    NMPC::Options o;
    o.lane_offset = RIGHT;
    o.horizon = 2.0;
    o.road_range = 40.0;
    NMPC controller(straight, r, VehicleParams(), Body(), o);
    const VehicleParams p;
    State x = State::Zero();
    const Pose start = straight.pose(0.0, RIGHT);
    x << start.x, start.y, start.psi, 20.0, 0.0, 0.0, 0.0;
    int sign_changes = 0;
    double fx_prev = NAN;
    for (double t = 0.0; t < 12.0; t += 0.05) {
        Observation obs;
        obs.t = t;
        obs.x = x;
        const Input u = controller.compute(obs);
        if (t > 3.0 && !std::isnan(fx_prev) && (u[IFX] > 0.0) != (fx_prev > 0.0)) ++sign_changes;
        if (t > 3.0) fx_prev = u[IFX];
        for (int i = 0; i < 50; ++i)
            x = rk4_step([&](const State& z, const Input& v) { return dynamics(z, v, p); }, x, u, 1e-3);
    }
    CHECK(sign_changes < 10);  // 9 s on the straight, 180 solves
    CHECK(controller.failures == 0);
}

// Steps of 20 ms for the first 0.2 s, then 0.1 s: the grid as asked, and the car driven as on equal steps.
static void nmpc_variable_steps() {
    NMPC::Options o;
    o.lane_offset = RIGHT;
    o.horizon = 2.0;
    o.h_fine = 0.02;
    o.fine = 0.2;
    NMPC controller(PATH, road(), VehicleParams(), Body(), o);
    CHECK(controller.N == 10 + 18);
    CHECK_NEAR(controller.times[10], 0.2, 1e-12);
    CHECK_NEAR(controller.times.back(), 2.0, 1e-12);
    CHECK(controller.step_at(-0.01) == -1 && controller.step_at(0.0) == 0 && controller.step_at(0.039) == 1);
    CHECK(controller.step_at(0.25) == 10 && controller.step_at(0.31) == 11 && controller.step_at(2.0) == 28);
    const Run run = drive(controller, 20.0);
    CHECK(run.distance > 250.0);
    CHECK(run.worst_offset < 0.8);
    CHECK_NEAR(run.end_speed, 15.0, 1.5);
    CHECK(controller.failures == 0);
    CHECK(controller.plan.rows() == 29 && controller.plan_inputs.rows() == 28);
    bool refused = false;
    try {  // shorter than one step
        o.h_fine = 0.0;
        o.horizon = 0.04;
        NMPC(PATH, road(), VehicleParams(), Body(), o);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    CHECK(refused);
}

static void nmpc_follows_directives() {
    NMPC::Options o;
    o.lane_offset = RIGHT;
    o.horizon = 3.0;
    auto make = [&] { return NMPC(PATH, road(), VehicleParams(), Body(), o); };
    {  // an empty directive changes nothing
        NMPC plain = make(), directed = make();
        CHECK(directed.set_directive(Directive()).empty());
        const Run a = drive(plain, 3.0), b = drive(directed, 3.0);
        CHECK(a.distance == b.distance && a.end_speed == b.end_speed);
        CHECK(directed.solve_stats.front().directive && !directed.solve_stats.back().directive);  // lapsed after 2 s
    }
    {  // a speed cap, until it lapses
        NMPC controller = make();
        controller.set_directive(controller.interpret({{Intent::set_speed, 9.0}}, NAN, 12.0));
        double speed_at_10 = 0.0;
        const Run run = drive(controller, 25.0, -1.0, [&](double t, double, double) {
            if (std::fabs(t - 10.0) < 1e-6) speed_at_10 = controller.plan(0, IUX);
        });
        CHECK_NEAR(speed_at_10, 9.0, 0.7);
        CHECK(!controller.directed());
        CHECK_NEAR(run.end_speed, 15.0, 1.5);
    }
    {  // the other lane
        NMPC controller = make();
        controller.set_directive(controller.interpret({{Intent::lane_left}}, NAN, 100.0));
        const Run run = drive(controller, 10.0);
        CHECK_NEAR(run.end_offset, -RIGHT, 0.5);
    }
    {  // a zone to keep out of in the lane ahead: passed beside it
        o.n_zones = 4;
        NMPC controller = make();
        o.n_zones = 0;
        Intent avoid{Intent::avoid};
        avoid.zone = {60.0, RIGHT, 4.0, 2.0, 0.0};
        controller.set_directive(controller.interpret({avoid}, NAN, 100.0));
        const Body body;
        double clearance = INFINITY;  // between the zone's side and the body's, beside it [m]
        const Run run = drive(controller, 12.0, -1.0, [&](double, double s, double e_y) {
            if (std::fabs(s - 60.0) < 2.0 + 0.5 * body.length)
                clearance = std::min(clearance, std::fabs(e_y - RIGHT) - 1.0 - 0.5 * body.width);
        });
        CHECK(run.distance > 100.0);
        CHECK(clearance > 0.2);
        CHECK(controller.failures == 0);
    }
    {  // values clamped to make the car no less careful than the defaults; unknown names refused
        NMPC controller = make();
        Directive d;
        d.limits = {{"speed_cap", 100.0}, {"margin_lat", 0.0}, {"sideslip_max", 1.0}};
        d.weights = {{"slack", 1.0}, {"speed", 2.0}};
        CHECK(controller.set_directive(d).size() == 4);  // all but the speed weight
        d.limits = {{"friction", 2.0}};
        bool refused = false;
        try {
            controller.set_directive(d);
        } catch (const std::invalid_argument&) {
            refused = true;
        }
        CHECK(refused);
        CHECK(intent_kind("lane_left") == Intent::lane_left);
        CHECK(controller.set_directive(controller.interpret({{Intent::avoid}})).size() == 1);  // no zone slots
    }
}

int main() {
    pure_pursuit_follows();
    pure_pursuit_passes();
    pure_pursuit_follows_a_reference();
    pure_pursuit_brakes_within_the_grip();
    nmpc_follows_and_passes();
    nmpc_steady_with_the_road_known_as_far_as_it_plans();
    nmpc_variable_steps();
    nmpc_follows_directives();
    return check_report("test_controllers");
}
