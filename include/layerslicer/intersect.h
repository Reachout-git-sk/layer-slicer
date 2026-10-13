// Triangle / horizontal plane intersection.
//
// On-plane rule ("symbolic perturbation"): a vertex whose z equals the plane
// height h exactly is classified as ABOVE the plane, i.e. the plane is treated
// as if it sat at h - epsilon for an infinitesimal epsilon. Consequences:
//   * every vertex is strictly below or (weakly) above, so a triangle is cut
//     only if it has at least one vertex with z < h and one with z >= h;
//   * a triangle lying in the plane (all three vertices at h) is "above" and
//     produces nothing; the cross-section is that of the solid just below h;
//   * an edge lying in the plane is reported exactly once, by the triangle on
//     the side that dips below the plane; its neighbour on the upper side
//     produces nothing for that edge;
//   * a triangle touching the plane only at one vertex yields a zero-length
//     segment, which is discarded.
// The result is that every slice is the boundary of a well-defined region and
// closes up without duplicated or dangling segments, with no need to move the
// layer heights.
#pragma once

#include <vector>

#include "layerslicer/mesh.h"
#include "layerslicer/vec.h"

namespace ls {

// Directed segment in the slicing plane. For an outward-oriented mesh the
// material lies to the LEFT of a -> b, so outer boundaries run counter-
// clockwise and hole boundaries clockwise.
struct Segment {
    Vec2 a;
    Vec2 b;
};

// Intersect one triangle with the plane z = h and append at most one segment.
// Returns true if a (non-degenerate) segment was appended.
bool intersectTriangle(const Mesh& mesh, const Triangle& tri, double h, std::vector<Segment>& out);

// Intersect every triangle of the mesh with z = h.
std::vector<Segment> intersectMesh(const Mesh& mesh, double h);

}  // namespace ls
