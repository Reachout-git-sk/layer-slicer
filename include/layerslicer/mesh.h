// Indexed triangle mesh.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "layerslicer/vec.h"

namespace ls {

using Triangle = std::array<std::uint32_t, 3>;   // indices into Mesh::vertices
using TriangleSoupEntry = std::array<Vec3, 3>;  // three explicit corners

// Vertices are shared between triangles (welded on construction from a soup),
// which lets the slicer compute every edge/plane intersection exactly once per
// edge so neighbouring triangles produce bit-identical endpoints.
struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<Triangle> triangles;

    std::size_t triangleCount() const { return triangles.size(); }
    BBox3 bounds() const;

    // Signed volume (positive for a closed, outward-oriented mesh).
    double signedVolume() const;

    // Build an indexed mesh from an unindexed list of triangles. Corners with
    // bit-identical coordinates are welded into one vertex. Triangles that
    // collapse (two or more identical corners) are discarded.
    static Mesh fromSoup(const std::vector<TriangleSoupEntry>& soup);

    // Expand back into an unindexed soup (used by the STL writers).
    std::vector<TriangleSoupEntry> toSoup() const;
};

}  // namespace ls
