// What to keep clear of: the movers the optimisation plans with (it has opt.n_movers slots), and their rows (the
// body's three circles outside each mover's clearance circle: stage.hpp).
#pragma once

#include <vector>

#include "nmpc/problem.hpp"

namespace ex {

// One mover at each step: where its centre is, and the clearance (the ego's circle, its own, its growth, the margin).
struct MoverSlot {
    double distance = INF;  // the least clearance left to the plan guessed [m]
    std::vector<double> s, e, R;  // per step [m]
};

// The opt.n_movers movers nearest the plan guessed (s_plan, e_plan at the steps' ends `times`), nearest first.
std::vector<MoverSlot> nearest_movers(const std::vector<Mover>& movers, const std::vector<double>& times,
                                      const std::vector<double>& s_plan, const std::vector<double>& e_plan,
                                      const Body& body, const NmpcOptions& opt);

}  // namespace ex
