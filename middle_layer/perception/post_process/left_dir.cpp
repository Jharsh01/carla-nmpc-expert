#include "perception/left_dir.hpp"

namespace ex {

NeighbourLane left_lane(const LaneSection& l) {
    NeighbourLane out;
    out.width = l.left_allowed ? l.left_width : 0.0;
    out.lo = l.hi;
    out.hi = l.hi + out.width;
    out.oncoming = l.left_oncoming;
    return out;
}

}  // namespace ex
