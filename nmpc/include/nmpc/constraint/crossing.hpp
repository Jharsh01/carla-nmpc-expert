// Crossers: yield to one (a gate keeping the front behind where it crosses, while it does) or pass ahead of it (a
// gate keeping the rear past it), and the passing gates' rows (the rear past them: stage.hpp).
#pragma once

#include <string>
#include <vector>

#include "nmpc/constraint/candidates.hpp"
#include "nmpc/problem.hpp"
#include "perception/perception.hpp"

namespace ex {

Gate yield_gate(const Crosser& c, const ConstraintOptions& opt);
Gate pass_gate(const Crosser& c, const ConstraintOptions& opt);

// The car, driven as hard as it can from v0 (overtake.hpp: drive), can get its rear past the crosser before it comes.
bool can_pass(const Crosser& c, const Percept& p, double v_max, const VehicleParams& params, const NmpcOptions& nmpc,
              const ConstraintOptions& opt);

// Across the crossers' path between two of them, as PDM-Lite goes (route 10: a left turn across a 10 m/s stream):
// the earliest hold for which the ego, held before where they cross until then and then driven as hard as it can
// (within the bends' speed), is out of each one's way while it is there (its window, cross_buffer either side). Adds
// "cross:<id>" (id: the first it goes ahead of; "cross" if none) with out.cross set, its speed asked for that drive,
// the gates before rolled through as they open. Once the ego cannot stop short of their path, it goes on with the
// gap it chose. `base`: the base problem before its gates for the crossers.
void cross(const Constraints& c, const Percept& p, const Scene& scene, const std::string& last_label,
           const Problem& base, Constraints::Result& out);

// The furthest passing gate to be past at the end of each step (-INF: none).
// and ahead of the chasers' fronts by `chaser_margin`.
std::vector<double> pass_profile(const std::vector<Gate>& gates, const std::vector<Chaser>& chasers,
                                 const std::vector<double>& times, double chaser_margin);

}  // namespace ex
