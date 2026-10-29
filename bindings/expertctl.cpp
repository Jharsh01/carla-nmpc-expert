// Python module `expertctl`: the NMPC core for the Bench2Drive agents (expert/agents/).
//
// Positions are in a right-handed world frame (heading counter-clockwise) at the centre of gravity; expert/frames.py
// converts from CARLA's.
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "urban/body.hpp"
#include "urban/path.hpp"
#include "urban/vehicle_params.hpp"
#include "urban/behaviour.hpp"
#include "urban/urban_nmpc.hpp"

namespace py = pybind11;
using namespace ex;

PYBIND11_MODULE(expertctl, m) {
    m.doc() = "NMPC core of carla-expert-nmpc: paths, vehicle parameters, the urban NMPC and the behaviour layer";

    py::class_<VehicleParams> params(m, "VehicleParams");
    params.def(py::init<>()).def_static("from_file", &VehicleParams::from_file).def("L", &VehicleParams::L);
    for (const auto& field : VehicleParams::fields()) params.def_readwrite(field.name, field.member);

    py::class_<Pose>(m, "Pose")
        .def(py::init([](double x, double y, double psi) { return Pose{x, y, psi}; }), py::arg("x") = 0.0,
             py::arg("y") = 0.0, py::arg("psi") = 0.0)
        .def_readonly("x", &Pose::x)
        .def_readonly("y", &Pose::y)
        .def_readonly("psi", &Pose::psi);

    py::class_<Path>(m, "Path")
        .def_static("from_waypoints", &Path::from_waypoints, py::arg("points"), py::arg("closed"))
        .def_property_readonly("length", &Path::length)
        .def_property_readonly("closed", &Path::closed)
        .def("curvature", &Path::curvature)
        .def("pose", &Path::pose, py::arg("s"), py::arg("e_y") = 0.0)
        .def("project", py::overload_cast<double, double>(&Path::project, py::const_), py::arg("x"), py::arg("y"))
        .def("project", py::overload_cast<double, double, double, double>(&Path::project, py::const_), py::arg("x"),
             py::arg("y"), py::arg("s_from"), py::arg("s_to"))
        .def("wrap", &Path::wrap)
        .def("difference", &Path::difference)
        .def_property_readonly("piece_lengths", &Path::piece_lengths)
        .def_property_readonly("piece_curvatures", &Path::piece_curvatures)
        .def_property_readonly("piece_poses", &Path::piece_poses);

    py::class_<Body>(m, "Body")
        .def(py::init([](double length, double width, double cg_to_center) { return Body{length, width, cg_to_center}; }),
             py::arg("length") = 4.89, py::arg("width") = 1.84, py::arg("cg_to_center") = 0.31)
        .def_readwrite("length", &Body::length)
        .def_readwrite("width", &Body::width)
        .def_readwrite("cg_to_center", &Body::cg_to_center);

    // ---- the urban NMPC (core/urban/include/urban/urban_nmpc.hpp)

    py::class_<ex::Ego>(m, "Ego")
        .def(py::init([](double x, double y, double psi, double v, double a, double delta) {
                 return ex::Ego{x, y, psi, v, a, delta};
             }),
             py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("psi") = 0.0, py::arg("v") = 0.0, py::arg("a") = 0.0,
             py::arg("delta") = 0.0)
        .def_readwrite("x", &ex::Ego::x)
        .def_readwrite("y", &ex::Ego::y)
        .def_readwrite("psi", &ex::Ego::psi)
        .def_readwrite("v", &ex::Ego::v)
        .def_readwrite("a", &ex::Ego::a)
        .def_readwrite("delta", &ex::Ego::delta);

    py::class_<ex::Corridor>(m, "Corridor")
        .def(py::init([](std::vector<double> s, std::vector<double> lo, std::vector<double> hi, std::vector<double> ref) {
                 return ex::Corridor{std::move(s), std::move(lo), std::move(hi), std::move(ref)};
             }),
             py::arg("s"), py::arg("lo"), py::arg("hi"), py::arg("ref"))
        .def_readwrite("s", &ex::Corridor::s)
        .def_readwrite("lo", &ex::Corridor::lo)
        .def_readwrite("hi", &ex::Corridor::hi)
        .def_readwrite("ref", &ex::Corridor::ref);

    py::class_<ex::Mover>(m, "Mover")
        .def(py::init([](std::vector<double> t, std::vector<double> s, std::vector<double> e_y, double radius, double growth) {
                 return ex::Mover{std::move(t), std::move(s), std::move(e_y), radius, growth};
             }),
             py::arg("t"), py::arg("s"), py::arg("e_y"), py::arg("radius") = 1.0, py::arg("growth") = 0.0)
        .def_readwrite("t", &ex::Mover::t)
        .def_readwrite("s", &ex::Mover::s)
        .def_readwrite("e_y", &ex::Mover::e_y)
        .def_readwrite("radius", &ex::Mover::radius)
        .def_readwrite("growth", &ex::Mover::growth)
        .def_readwrite("margin", &ex::Mover::margin);

    py::class_<ex::Lead>(m, "Lead")
        .def(py::init([](std::vector<double> t, std::vector<double> s_rear) { return ex::Lead{std::move(t), std::move(s_rear)}; }),
             py::arg("t"), py::arg("s_rear"))
        .def_readwrite("t", &ex::Lead::t)
        .def_readwrite("s_rear", &ex::Lead::s_rear);

    py::class_<ex::Gate>(m, "Gate")
        .def(py::init([](double s, double t_from, double t_to, bool before) { return ex::Gate{s, t_from, t_to, before}; }),
             py::arg("s"), py::arg("t_from") = 0.0, py::arg("t_to") = ex::INF, py::arg("before") = true)
        .def_readwrite("s", &ex::Gate::s)
        .def_readwrite("t_from", &ex::Gate::t_from)
        .def_readwrite("t_to", &ex::Gate::t_to)
        .def_readwrite("before", &ex::Gate::before);

    py::class_<ex::Problem>(m, "Problem")
        .def(py::init<>())
        .def_readwrite("t", &ex::Problem::t)
        .def_readwrite("ego", &ex::Problem::ego)
        .def_readwrite("corridor", &ex::Problem::corridor)
        .def_readwrite("v_max", &ex::Problem::v_max)
        .def_readwrite("v_ref_t", &ex::Problem::v_ref_t)
        .def_readwrite("v_ref", &ex::Problem::v_ref)
        .def_readwrite("movers", &ex::Problem::movers)
        .def_readwrite("leads", &ex::Problem::leads)
        .def_readwrite("gates", &ex::Problem::gates);

    py::class_<ex::Plan>(m, "Plan", "x: [s, e_y, e_psi, v, a, delta] per step, u: [jerk, delta_rate] per step")
        .def_readonly("times", &ex::Plan::times)
        .def_readonly("x", &ex::Plan::x)
        .def_readonly("u", &ex::Plan::u)
        .def_readonly("success", &ex::Plan::success)
        .def_readonly("status", &ex::Plan::status)
        .def_readonly("iterations", &ex::Plan::iterations)
        .def_readonly("solve_time", &ex::Plan::solve_time)
        .def_readonly("cost", &ex::Plan::cost)
        .def_readonly("slack_max", &ex::Plan::slack_max)
        .def_readonly("n_movers", &ex::Plan::n_movers);

    using Opt = ex::UrbanNMPC::Options;
    py::class_<Opt> opt(m, "UrbanOptions");
    opt.def(py::init<>());
#define FIELD(name) opt.def_readwrite(#name, &Opt::name)
    FIELD(horizon); FIELD(h); FIELD(h_fine); FIELD(fine); FIELD(n_movers); FIELD(margin); FIELD(edge_margin); FIELD(gap_min); FIELD(gap); FIELD(time_gap); FIELD(a_stop); FIELD(a_ref); FIELD(a_lat_ref);
    FIELD(a_max); FIELD(a_min); FIELD(jerk_max); FIELD(mu); FIELD(w_speed); FIELD(w_lane); FIELD(w_heading);
    FIELD(w_accel); FIELD(w_jerk); FIELD(w_rate); FIELD(w_lat); FIELD(w_headway); FIELD(w_slack); FIELD(tol); FIELD(acceptable_tol);
    FIELD(acceptable_iter); FIELD(max_iter); FIELD(jit); FIELD(cache_dir); FIELD(warm_start);
#undef FIELD

    py::class_<ex::UrbanNMPC>(m, "UrbanNMPC")
        .def(py::init<Path, VehicleParams, Body, Opt>(), py::arg("route"), py::arg("params"), py::arg("body") = Body(),
             py::arg("options") = Opt())
        .def("solve", &ex::UrbanNMPC::solve, py::call_guard<py::gil_scoped_release>())
        .def("reset", &ex::UrbanNMPC::reset)
        .def("set_route", &ex::UrbanNMPC::set_route, py::arg("route"), py::arg("old_frame") = Pose{0.0, 0.0, 0.0})
        .def("path_state", &ex::UrbanNMPC::path_state)
        .def_static("groups", &ex::UrbanNMPC::groups)
        .def_readonly("times", &ex::UrbanNMPC::times)
        .def_readonly("N", &ex::UrbanNMPC::N)
        .def_readonly("a_min", &ex::UrbanNMPC::a_min)
        .def_readonly("solve_times", &ex::UrbanNMPC::solve_times)
        .def_readonly("failures", &ex::UrbanNMPC::failures);

    // ---- the behaviour layer (core/urban/include/urban/behaviour.hpp)

    py::class_<ex::Actor> actor(m, "Actor");
    py::enum_<ex::Actor::Kind>(actor, "Kind")
        .value("vehicle", ex::Actor::vehicle)
        .value("bike", ex::Actor::bike)
        .value("walker", ex::Actor::walker)
        .value("object", ex::Actor::object);
    actor.def(py::init<>())
        .def_readwrite("id", &ex::Actor::id)
        .def_readwrite("kind", &ex::Actor::kind)
        .def_readwrite("length", &ex::Actor::length)
        .def_readwrite("width", &ex::Actor::width)
        .def_readwrite("parked", &ex::Actor::parked)
        .def_readwrite("t", &ex::Actor::t)
        .def_readwrite("x", &ex::Actor::x)
        .def_readwrite("y", &ex::Actor::y)
        .def_readwrite("psi", &ex::Actor::psi);

    py::class_<ex::LaneSection>(m, "LaneSection")
        .def(py::init([](double s_from, double s_to, double lo, double hi, double left_width, bool left_oncoming,
                         bool left_allowed) {
                 return ex::LaneSection{s_from, s_to, lo, hi, left_width, left_oncoming, left_allowed};
             }),
             py::arg("s_from") = 0.0, py::arg("s_to") = ex::INF, py::arg("lo") = -1.75, py::arg("hi") = 1.75,
             py::arg("left_width") = 0.0, py::arg("left_oncoming") = true, py::arg("left_allowed") = false)
        .def_readwrite("s_from", &ex::LaneSection::s_from)
        .def_readwrite("s_to", &ex::LaneSection::s_to)
        .def_readwrite("lo", &ex::LaneSection::lo)
        .def_readwrite("hi", &ex::LaneSection::hi)
        .def_readwrite("left_width", &ex::LaneSection::left_width)
        .def_readwrite("left_oncoming", &ex::LaneSection::left_oncoming)
        .def_readwrite("left_allowed", &ex::LaneSection::left_allowed);

    py::class_<ex::StopLine> stop(m, "StopLine");
    py::enum_<ex::StopLine::Kind>(stop, "Kind")
        .value("red_light", ex::StopLine::red_light)
        .value("amber_light", ex::StopLine::amber_light)
        .value("stop_sign", ex::StopLine::stop_sign)
        .value("route_end", ex::StopLine::route_end);
    stop.def(py::init([](int id, ex::StopLine::Kind kind, double s) { return ex::StopLine{id, kind, s}; }),
             py::arg("id"), py::arg("kind"), py::arg("s"))
        .def_readwrite("id", &ex::StopLine::id)
        .def_readwrite("kind", &ex::StopLine::kind)
        .def_readwrite("s", &ex::StopLine::s);

    py::class_<ex::Scene>(m, "Scene")
        .def(py::init<>())
        .def_readwrite("t", &ex::Scene::t)
        .def_readwrite("ego", &ex::Scene::ego)
        .def_readwrite("v_max", &ex::Scene::v_max)
        .def_readwrite("v_ref_t", &ex::Scene::v_ref_t)
        .def_readwrite("v_ref", &ex::Scene::v_ref)
        .def_readwrite("line_s", &ex::Scene::line_s)
        .def_readwrite("line_e", &ex::Scene::line_e)
        .def_readwrite("lanes", &ex::Scene::lanes)
        .def_readwrite("actors", &ex::Scene::actors)
        .def_readwrite("stops", &ex::Scene::stops);

    py::class_<ex::Candidate>(m, "Candidate")
        .def_readonly("label", &ex::Candidate::label)
        .def_readonly("plan", &ex::Candidate::plan)
        .def_readonly("problem", &ex::Candidate::problem)
        .def_readonly("score", &ex::Candidate::score);

    py::class_<ex::Decision>(m, "Decision")
        .def_readonly("label", &ex::Decision::label)
        .def_readonly("plan", &ex::Decision::plan)
        .def_readonly("candidates", &ex::Decision::candidates)
        .def_readonly("wall_time", &ex::Decision::wall_time)
        .def_readonly("notes", &ex::Decision::notes);

    using BOpt = ex::Behaviour::Options;
    py::class_<BOpt> bopt(m, "BehaviourOptions");
    bopt.def(py::init<>());
#define BFIELD(name) bopt.def_readwrite(#name, &BOpt::name)
    BFIELD(pool); BFIELD(look_ahead); BFIELD(gate_margin); BFIELD(gate_buffer); BFIELD(blocker_range); BFIELD(blocker_overlap); BFIELD(static_margin);
    BFIELD(oncoming_gap); BFIELD(stop_hold); BFIELD(dilemma_decel); BFIELD(hysteresis); BFIELD(w_out_of_lane);
    BFIELD(pass_penalty); BFIELD(slack_ok);
#undef BFIELD

    py::class_<ex::Behaviour>(m, "Behaviour")
        .def(py::init<Path, VehicleParams, Body, Opt, BOpt>(), py::arg("route"), py::arg("params"),
             py::arg("body") = Body(), py::arg("nmpc_options") = Opt(), py::arg("options") = BOpt(),
             py::call_guard<py::gil_scoped_release>())
        .def("step", &ex::Behaviour::step, py::call_guard<py::gil_scoped_release>())
        .def("reset", &ex::Behaviour::reset)
        .def("set_route", &ex::Behaviour::set_route, py::arg("route"), py::arg("old_frame") = Pose{0.0, 0.0, 0.0},
             py::call_guard<py::gil_scoped_release>())
        .def_readonly("route", &ex::Behaviour::route);
}
