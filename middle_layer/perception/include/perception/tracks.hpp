// Each actor's predicted trajectory taken into the route's path coordinates (a Track), and its body as circles for
// the NMPC to keep clear of (a Mover each).
#pragma once

#include <utility>
#include <vector>

#include "perception/perception.hpp"

namespace ex {

// Points projected onto the stretch [s_lo, s_hi] of the route around the car. A point beyond either end is clamped
// onto the end with only its sideways offset, so a car straight behind the route's start would look in the lane at
// s = 0: there its distance stands in for the offset.
class Projector {
public:
    Projector(const Path& route, double s_lo, double s_hi) : route(route), s_lo(s_lo), s_hi(s_hi) {}
    std::pair<double, double> operator()(double x, double y) const;  // (s, e_y) [m]

    const Path& route;
    const double s_lo, s_hi;
};

// The actor's trajectory as a track; false if its samples do not line up.
bool make_track(const Actor& a, const Projector& project, Track& out);

// Near the route: not all of it behind the car (its rear at rear0) or beyond the look-ahead from s0.
bool near_route(const Track& tr, double s0, double rear0, double look_ahead);

// The actor's body as circles along its trajectory, added to `into`: three along a car, one for anything shorter
// than 3 m. Parked cars and objects keep `static_margin` and do not grow; moving actors' circles grow with the
// prediction's age (walkers faster).
void add_circles(const Track& tr, const Projector& project, double static_margin, std::vector<Mover>& into);

}  // namespace ex
