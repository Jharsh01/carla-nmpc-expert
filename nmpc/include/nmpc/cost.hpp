// The optimisation's cost: a sum over the steps of weighted terms, each step's weighted by its length. The weights
// are NmpcOptions' (shared_param.hpp). The slack terms price the constraint groups' violation (stage.hpp): all but
// the headway as hard constraints, the headway softer.
#pragma once

#include <functional>
#include <vector>

#include "nmpc/stage.hpp"

namespace ex {

struct CostTerm {
    const char* name;
    double NmpcOptions::*weight;
    std::function<SX(const Stage&)> f;
};

// speed (against the speed asked for), lane (against the line asked for), heading, accel, jerk, rate, lat (lateral
// acceleration), slack, headway; in the order of the weights among the solver's parameters
const std::vector<CostTerm>& cost_terms();

}  // namespace ex
