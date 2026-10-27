// Python module `expertctl`: the NMPC core for the Bench2Drive agents (expert/agents/).
//
// States and inputs are numpy arrays in the library's convention (core/track_control/include/track_control/types.hpp):
// right-handed world frame, centre of gravity. expert/frames.py converts from CARLA's.
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "track_control/directive.hpp"
#include "track_control/path.hpp"
#include "track_control/vehicle_params.hpp"

namespace py = pybind11;
using namespace tc;

PYBIND11_MODULE(expertctl, m) {
    m.doc() = "NMPC core of carla-expert-nmpc (track_control, the urban NMPC)";

    py::class_<VehicleParams> params(m, "VehicleParams");
    params.def(py::init<>()).def_static("from_file", &VehicleParams::from_file).def("L", &VehicleParams::L);
    for (const auto& field : VehicleParams::fields()) params.def_readwrite(field.name, field.member);

    py::class_<Pose>(m, "Pose").def_readonly("x", &Pose::x).def_readonly("y", &Pose::y).def_readonly("psi", &Pose::psi);

    py::class_<Path>(m, "Path")
        .def_static("from_waypoints", &Path::from_waypoints, py::arg("points"), py::arg("closed"))
        .def_property_readonly("length", &Path::length)
        .def_property_readonly("closed", &Path::closed)
        .def("curvature", &Path::curvature)
        .def("pose", &Path::pose, py::arg("s"), py::arg("e_y") = 0.0)
        .def("project", &Path::project)
        .def("wrap", &Path::wrap)
        .def("difference", &Path::difference)
        .def_property_readonly("piece_lengths", &Path::piece_lengths)
        .def_property_readonly("piece_curvatures", &Path::piece_curvatures)
        .def_property_readonly("piece_poses", &Path::piece_poses);

    py::class_<Zone>(m, "Zone", "a place for the NMPC to keep out of, in path coordinates (track_control/directive.hpp)")
        .def(py::init([](double s, double e_y, double length, double width, double speed) {
                 return Zone{s, e_y, length, width, speed};
             }),
             py::arg("s"), py::arg("e_y"), py::arg("length") = 1.0, py::arg("width") = 1.0, py::arg("speed") = 0.0)
        .def_readwrite("s", &Zone::s)
        .def_readwrite("e_y", &Zone::e_y)
        .def_readwrite("length", &Zone::length)
        .def_readwrite("width", &Zone::width)
        .def_readwrite("speed", &Zone::speed);
}
