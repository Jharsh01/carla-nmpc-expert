// One step of the horizon as the optimisation's cost and constraint rows see it (CasADi expressions), and the rows
// themselves, each defined beside the manoeuvre it belongs to:
//
//   group      rows                                   defined in
//   corridor   the body's ends within the band        constraint/lane.cpp
//   movers     the body's circles outside each mover  constraint/movers.cpp
//   ahead      the front behind the lead              constraint/follow.cpp
//              and behind the stop gates              constraint/stop.cpp
//   behind     the rear past the passing gates        constraint/crossing.cpp
//   grip       friction circle, speed limit           constraint/grip.cpp
//   headway    room to stop behind the lead           constraint/follow.cpp
//
// Each group is softened by one slack per step (framework.cpp puts them together, in this order). Internal to the
// NMPC: it needs CasADi, which nothing outside nmpc/ does.
#pragma once

#include <array>
#include <casadi/casadi.hpp>
#include <vector>

#include "nmpc/framework.hpp"

namespace ex {

using casadi::SX;
constexpr int NS = 6;  // slacks per step, one per constraint group

struct Stage {
    SX x, u, slack;  // x: the state at the end of the step
    SX v_ref, e_ref, lo, hi, v_max, mu_g;
    std::vector<std::array<SX, 4>> movers;  // s, e_y, clearance radius (ego circle + mover + margin), 1 (0: unused)
    SX lead_s, lead_v, lead_on, cap_s, cap_on, floor_s, floor_on;
    double terminal;  // 1 at the last step, else 0
    const UrbanNMPC* nmpc;
};

inline SX lateral_acceleration(const SX& x, double L) { return casadi::sq(x(IV)) * tan(x(IDELTA)) / L; }

// The rows of each group: every expression <= the group's slack. A switched-off row (on = 0) is the constant -1.
std::vector<SX> corridor_rows(const Stage& g);  // constraint/lane.cpp
std::vector<SX> mover_rows(const Stage& g);  // constraint/movers.cpp
std::vector<SX> lead_rows(const Stage& g);  // constraint/follow.cpp
std::vector<SX> headway_rows(const Stage& g);  // constraint/follow.cpp
std::vector<SX> stop_rows(const Stage& g);  // constraint/stop.cpp
std::vector<SX> pass_rows(const Stage& g);  // constraint/crossing.cpp
std::vector<SX> grip_rows(const Stage& g);  // constraint/grip.cpp

}  // namespace ex
