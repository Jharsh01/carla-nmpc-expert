// The lane on the left of the ego lane, as the map has it at a section: there or not (and whether it may be driven
// into), how wide, and which way it runs (sensors/map.py reads it beyond the band's leftmost lane).
#pragma once

#include "perception/lanes.hpp"

namespace ex {

NeighbourLane left_lane(const LaneSection& l);

}  // namespace ex
