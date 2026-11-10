#include "perception/right_dir.hpp"

namespace ex {

NeighbourLane right_lane(const LaneSection& l) {
    NeighbourLane out;
    out.width = l.right_allowed ? l.right_width : 0.0;
    out.lo = l.lo - out.width;
    out.hi = l.lo;
    out.oncoming = l.right_oncoming;
    return out;
}

}  // namespace ex
