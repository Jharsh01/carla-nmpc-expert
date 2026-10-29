// The ego car's parameters the urban NMPC plans with.
#pragma once

#include <string>
#include <vector>

namespace ex {

// The defaults are CARLA 0.9.15's vehicle.lincoln.mkz_2020, Bench2Drive's ego (config/vehicles/mkz2020.json,
// with where each value comes from: docs/vehicle_parameters.md).
struct VehicleParams {
    double m = 1696.0;  // mass [kg]
    double a = 1.172;  // centre of gravity to front axle [m]
    double b = 1.689;  // centre of gravity to rear axle [m]
    double F_brake_max = 13000.0;  // peak brake force [N]
    double delta_max = 0.95;  // road-wheel angle limit [rad]
    double delta_rate_max = 0.6;  // steering rate limit [rad/s]
    double g = 9.81;  // [m/s^2]

    // Read from a JSON file of the form {"vehicle": {"parameters": {"m": {"value": ...}, ...}}}; every field above
    // must be present.
    static VehicleParams from_file(const std::string& path);

    double L() const { return a + b; }  // wheelbase [m]

    struct Field {
        const char* name;
        double VehicleParams::* member;
    };
    static const std::vector<Field>& fields();
};

}  // namespace ex
