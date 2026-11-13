// Merging into a stream: at a junction the ego turns into a lane that cars from a side road are coming down
// (Percept::mergers). Waiting for all of them is "base" (a gate before where they come in until the last one seen is
// in, each then a lead). Here is the candidate that goes in between two of them: the earliest gap for which the ego,
// held before the entry until the car ahead of the gap is in and then driven as hard as it can (within the bend's
// speed), keeps clear of the car ahead and stays ahead of the car behind. That car, and those after it, are the
// candidate's chasers: the plan keeps the ego's rear ahead of their fronts.
#pragma once

#include <string>

#include "nmpc/constraint/candidates.hpp"

namespace ex {

// Adds "merge:<id>" (id: the car the ego goes in ahead of) to out, with out.merge set, when a gap is clear; or, once
// the ego has gone in ahead of a car (`last_label`), while that car is still close behind. `base` is the base
// candidate's problem before its gates for the mergers.
void merge(const Constraints& c, const Percept& p, const Scene& scene, const std::string& last_label,
           const Problem& base, Constraints::Result& out);

// The gate before the mergers' entry while they come in, and their leads after: waiting for all of them.
void wait_for_mergers(Problem& pr, const Percept& p, const ConstraintOptions& opt);

}  // namespace ex
