// The lanes along the route as the map gives them (LaneSection, from sensors/map.py): which section holds a point,
// whether a body is in the ego lane, which part of the band runs the car's way, and the lanes beside it.
#pragma once

#include <utility>
#include <vector>

#include "perception/perception.hpp"

namespace ex {

// The section at s; beyond the sections, the nearest end's.
const LaneSection& section_at(const std::vector<LaneSection>& lanes, double s);

// A body at offset e, half as wide as ext_e across the route, reaches into the ego lane.
bool in_lane(const LaneSection& l, double e, double ext_e);

// The part of the band [lo, hi] in lanes running the car's way; first >= second if none of it does.
std::pair<double, double> way_band(const LaneSection& l);

// A lane beside the ego lane: how wide it is (0: none, or not to be driven into), the band it takes up across the
// route, and whether it runs against the car.
struct NeighbourLane {
    double width = 0.0;  // [m]
    double lo = 0.0, hi = 0.0;  // [m]
    bool oncoming = false;
    bool exists() const { return width > 0.0; }
};

// left_lane (left_dir.hpp) or right_lane (right_dir.hpp)
NeighbourLane neighbour_lane(const LaneSection& l, bool left);

}  // namespace ex
