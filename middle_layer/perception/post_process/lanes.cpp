#include "perception/lanes.hpp"

#include <algorithm>

#include "perception/left_dir.hpp"
#include "perception/right_dir.hpp"

namespace ex {

const LaneSection& section_at(const std::vector<LaneSection>& lanes, double s) {
    for (const LaneSection& l : lanes)
        if (l.s_from <= s && s < l.s_to) return l;
    return s < lanes.front().s_from ? lanes.front() : lanes.back();
}

const LaneSection& Percept::section(double s) const { return section_at(lanes, s); }

bool in_lane(const LaneSection& l, double e, double ext_e) { return e - ext_e < l.hi && e + ext_e > l.lo; }

std::pair<double, double> way_band(const LaneSection& l) {
    return {std::max(l.lo, l.way_lo), std::min(l.hi, l.way_hi)};
}

NeighbourLane neighbour_lane(const LaneSection& l, bool left) { return left ? left_lane(l) : right_lane(l); }

}  // namespace ex
