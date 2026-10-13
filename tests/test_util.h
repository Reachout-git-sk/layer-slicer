#pragma once

#include <map>
#include <memory>
#include <utility>

#include "layerslicer/mesh.h"
#include "layerslicer/slicer.h"

namespace lstest {

inline ls::Layer sliceAt(const ls::Mesh& mesh, double z, ls::SliceOptions opts = {}) {
    ls::Slicer slicer(std::make_shared<const ls::Mesh>(mesh), opts);
    return slicer.sliceLayer(0, z);
}

// Every directed edge must be matched by exactly one opposite edge.
inline bool isClosedManifold(const ls::Mesh& m) {
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (const ls::Triangle& t : m.triangles) {
        for (int k = 0; k < 3; ++k) ++edges[{t[k], t[(k + 1) % 3]}];
    }
    for (const auto& [e, count] : edges) {
        if (count != 1) return false;
        auto it = edges.find({e.second, e.first});
        if (it == edges.end() || it->second != 1) return false;
    }
    return true;
}

inline ls::BBox3 contourBounds(const ls::Contour& c) {
    ls::BBox3 b;
    for (const ls::Vec2& p : c.points) b.expand({p.x, p.y, 0.0});
    return b;
}

}  // namespace lstest
