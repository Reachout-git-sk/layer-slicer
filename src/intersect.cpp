#include "layerslicer/intersect.h"

#include <utility>

namespace ls {

namespace {

// Point where edge (u, v) crosses z = h. The endpoints are put into a
// canonical order (by vertex index) before interpolating so that the two
// triangles sharing an edge compute bit-identical results. A vertex lying
// exactly on the plane is returned as-is rather than interpolated.
Vec2 edgePoint(const Mesh& mesh, std::uint32_t u, std::uint32_t v, double h) {
    if (u > v) std::swap(u, v);
    const Vec3& p = mesh.vertices[u];
    const Vec3& q = mesh.vertices[v];
    if (p.z == h) return {p.x, p.y};
    if (q.z == h) return {q.x, q.y};
    const double t = (h - p.z) / (q.z - p.z);
    return {p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t};
}

}  // namespace

bool intersectTriangle(const Mesh& mesh, const Triangle& tri, double h, std::vector<Segment>& out) {
    bool above[3];
    int nAbove = 0;
    for (int k = 0; k < 3; ++k) {
        above[k] = mesh.vertices[tri[k]].z >= h;  // on-plane counts as above
        nAbove += above[k] ? 1 : 0;
    }
    if (nAbove == 0 || nAbove == 3) return false;

    // Walk the triangle's edges in winding order. Exactly one edge goes from
    // above to below (segment start) and one from below to above (segment end).
    // With counter-clockwise winding seen from outside this leaves the
    // material on the left of the segment.
    Vec2 start{}, end{};
    for (int k = 0; k < 3; ++k) {
        const int k1 = (k + 1) % 3;
        if (above[k] && !above[k1]) start = edgePoint(mesh, tri[k], tri[k1], h);
        if (!above[k] && above[k1]) end = edgePoint(mesh, tri[k], tri[k1], h);
    }
    if (start == end) return false;  // touches the plane in a single point
    out.push_back({start, end});
    return true;
}

std::vector<Segment> intersectMesh(const Mesh& mesh, double h) {
    std::vector<Segment> segments;
    for (const Triangle& tri : mesh.triangles) intersectTriangle(mesh, tri, h, segments);
    return segments;
}

}  // namespace ls
