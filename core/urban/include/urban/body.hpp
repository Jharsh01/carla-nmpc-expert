// The ego car's body, shared by the urban NMPC and the behaviour layer.
#pragma once

namespace ex {

// Size of the ego car's body, for the corridor and clearance constraints; the defaults are the MKZ's
// (docs/vehicle_parameters.md).
struct Body {
    double length = 4.89;  // [m]
    double width = 1.84;  // [m]
    double cg_to_center = 0.31;  // centre of gravity ahead of the body centre [m]
};

}  // namespace ex
