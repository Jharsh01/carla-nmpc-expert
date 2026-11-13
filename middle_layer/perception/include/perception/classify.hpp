// Each track against the ego lane: lead, blocker, crosser or mover; and, whatever its class, traffic coming the
// other way (`oncoming`) and going the car's way (`along`), for the checks that a neighbouring lane is clear.
#pragma once

#include "perception/tracks.hpp"

namespace ex {

// Classify p.tracks, filling p's leads, blockers, crossers, movers (moving, still, stay_out), oncoming, along and
// notes. p's ego (z0, s0, front0, rear0) and lanes must be set.
void classify(Percept& p, const Body& body, const PerceptionOptions& opt, const Projector& project);

}  // namespace ex
