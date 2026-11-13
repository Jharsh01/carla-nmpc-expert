// Stop lines: which to stop at (red and amber lights, stop signs, the route's end) with the traffic rules' state,
// the speed asked for tapered to them, and their rows (the front behind the stop gates, at the last step with room
// to stop: stage.hpp).
//
// A stop sign is released once the car has stood at it for stop_hold; a light turning red (or amber) is decided once:
// driven through if stopping would take more than dilemma_decel (amber: 1.5 a_ref).
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "nmpc/problem.hpp"
#include "perception/perception.hpp"

namespace ex {

class StopRules {
public:
    // The stop gates for the scene's stop lines within the look-ahead, the rules' events added to `notes`.
    std::vector<Gate> gates(const Percept& p, const Scene& scene, double a_ref, const ConstraintOptions& opt,
                            std::vector<std::string>& notes);
    void reset();

private:
    std::map<int, double> stopped_since_;  // stop signs: when the car came to rest at the line
    std::set<int> released_;  // stop signs held at, and lights driven through on amber or in the dilemma zone
    std::set<int> seen_;  // lights already decided on
};

// The nearest gate ahead to stay behind at the end of each step (INF: none); never behind the front now.
std::vector<double> stop_profile(const std::vector<Gate>& gates, const std::vector<double>& times, double front_now,
                                 bool tapered_only = false,
                                 double roll_through = 0.0);

// The speed asked for v, no more than stops at `cap` at a_ref.
double stop_speed(double v, double cap, double front, const NmpcOptions& opt);

}  // namespace ex
