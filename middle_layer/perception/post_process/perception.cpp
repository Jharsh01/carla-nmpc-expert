#include "perception/perception.hpp"

#include "perception/classify.hpp"
#include "perception/tracks.hpp"

namespace ex {

Percept Perception::perceive(const Scene& sc, const Path& route, const Eigen::Matrix<double, NX, 1>& z0) const {
    Percept p;
    p.z0 = z0;
    const double s0 = p.s0 = z0[IS];
    p.v0 = sc.ego.v;
    p.front0 = s0 + 0.5 * body.length - body.cg_to_center;
    p.rear0 = s0 - 0.5 * body.length - body.cg_to_center;
    p.lanes = sc.lanes;
    if (p.lanes.empty()) p.lanes.push_back(LaneSection());

    // the actors in path coordinates, projected onto the stretch of route around the car, those near it kept
    const Projector project(route, s0 - 50.0, s0 + opt.look_ahead + 80.0);
    for (const Actor& a : sc.actors) {
        Track tr;
        if (make_track(a, project, tr) && near_route(tr, s0, p.rear0, opt.look_ahead)) p.tracks.push_back(std::move(tr));
    }
    classify(p, body, opt, project);
    return p;
}

}  // namespace ex
