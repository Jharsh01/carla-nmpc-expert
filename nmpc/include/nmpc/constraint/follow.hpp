// Leads: the car ahead in the lane at each step and its speed, the speed asked for kept to a time gap and an
// envelope behind it, and their rows (the front behind the lead; at the end of the horizon room to stop behind it
// were it to brake: stage.hpp). Blockers are waited behind as leads at rest.
#pragma once

#include <vector>

#include "nmpc/problem.hpp"
#include "perception/perception.hpp"

namespace ex {

// The nearest lead's rear (INF: none) and its speed at the end of each step; never nearer than gap_min ahead of the
// front now (a plan that reverses to restore a gap is not one the car can drive).
void lead_profile(const std::vector<Lead>& leads, const std::vector<double>& times, double front_now,
                  const NmpcOptions& opt, std::vector<double>& lead, std::vector<double>& lead_speed);

// The speed asked for v, no more than keeps the time gap to the lead and lets it stop behind it at a_ref.
double behind_lead_speed(double v, double lead_s, double lead_v, double front, const NmpcOptions& opt);

// Wait behind every blocker: a lead at rest 4 m short of it, far enough back to pull out round it.
void stay_behind(Problem& problem, const Percept& p);

}  // namespace ex
