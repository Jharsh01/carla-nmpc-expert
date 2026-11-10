// Crossers: yield to one (a gate keeping the front behind where it crosses, while it does) or pass ahead of it (a
// gate keeping the rear past it), and the passing gates' rows (the rear past them: stage.hpp).
#pragma once

#include <vector>

#include "nmpc/problem.hpp"
#include "perception/perception.hpp"

namespace ex {

Gate yield_gate(const Crosser& c, const ConstraintOptions& opt);
Gate pass_gate(const Crosser& c, const ConstraintOptions& opt);

// The car, driven as hard as it can from v0 (overtake.hpp: drive), can get its rear past the crosser before it comes.
bool can_pass(const Crosser& c, const Percept& p, double v_max, const VehicleParams& params, const NmpcOptions& nmpc,
              const ConstraintOptions& opt);

// The furthest passing gate to be past at the end of each step (-INF: none).
std::vector<double> pass_profile(const std::vector<Gate>& gates, const std::vector<double>& times);

}  // namespace ex
