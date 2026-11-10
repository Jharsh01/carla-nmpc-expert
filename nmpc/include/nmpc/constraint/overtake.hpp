// Overtaking: round the blockers through the lane on either side, where the map has one (left_dir, right_dir),
// judged by which way it runs: an oncoming lane must be clear of oncoming traffic until the car is past, seen and
// unseen (an oncoming car at the edge of what the car can see along it); a lane running the car's way must be clear
// of traffic beside or behind that the car would cut off. Once out in that lane and level with the blockers the
// car finishes.
#pragma once

#include <string>
#include <vector>

#include "nmpc/constraint/candidates.hpp"

namespace ex {

// The car driven from v0 as hard as it can, min(a_max, the vehicle's a_drive_max at each speed), up to v_top: how
// far it has gone t from now, and how long it takes to go a distance. Over a pass of several seconds the speed
// changes too much to hold the acceleration at the start's, as a single solve does.
struct Drive {
    double dt = 0.05, v_end = 0.0;
    std::vector<double> d;  // every dt from 0
    double at(double t) const;
    double time_to(double distance) const;
};
Drive drive(double v0, double v_top, double a_max, const VehicleParams& params);

// The candidate going round the blockers [b_lo, b_hi] through the lane on one side, if there is one and it is clear,
// added to out (notes and `committed` too). `lane` and `stop_gates` are the base candidate's.
void overtake(const Constraints& c, const Percept& p, const Scene& scene, bool left, double b_lo, double b_hi,
              const std::string& last_label, const Corridor& lane, const std::vector<Gate>& stop_gates,
              Constraints::Result& out);

}  // namespace ex
