#include "urban/vehicle_params.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace ex {

const std::vector<VehicleParams::Field>& VehicleParams::fields() {
    using P = VehicleParams;
    static const std::vector<Field> all = {
        {"m", &P::m},
        {"a", &P::a},
        {"b", &P::b},
        {"F_brake_max", &P::F_brake_max},
        {"delta_max", &P::delta_max},
        {"delta_rate_max", &P::delta_rate_max},
        {"g", &P::g},
    };
    return all;
}

VehicleParams VehicleParams::from_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path);
    const nlohmann::json data = nlohmann::json::parse(file);
    VehicleParams p;
    const auto& values = data.at("vehicle").at("parameters");
    for (const auto& field : fields()) {
        if (!values.contains(field.name)) throw std::runtime_error(path + ": no value for " + field.name);
        p.*field.member = values.at(field.name).at("value");
    }
    return p;
}

}  // namespace ex
