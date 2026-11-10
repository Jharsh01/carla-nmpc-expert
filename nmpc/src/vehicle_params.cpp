#include "nmpc/shared_param.hpp"

#include <algorithm>
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
    const auto& table = data.at("vehicle").at("tables").at("a_drive_max");
    p.a_drive_speed = table.at("speed").get<std::vector<double>>();
    p.a_drive_max = table.at("value").get<std::vector<double>>();
    if (p.a_drive_speed.size() != p.a_drive_max.size() || p.a_drive_speed.size() < 2 ||
        !std::is_sorted(p.a_drive_speed.begin(), p.a_drive_speed.end()))
        throw std::runtime_error(path + ": a_drive_max needs two or more ascending speeds, one value each");
    return p;
}

double VehicleParams::a_drive(double v) const {
    const auto& xs = a_drive_speed;
    const auto& ys = a_drive_max;
    if (v <= xs.front()) return ys.front();
    if (v >= xs.back()) return ys.back();
    const size_t i = std::upper_bound(xs.begin(), xs.end(), v) - xs.begin();
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (v - xs[i - 1]) / (xs[i] - xs[i - 1]);
}

}  // namespace ex
