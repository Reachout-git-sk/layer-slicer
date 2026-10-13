// Tolerance-based point merging and segment stitching into contours.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "layerslicer/geometry.h"
#include "layerslicer/intersect.h"

namespace ls {

// Integer grid cell of a point quantised with cell size `tolerance`. Two
// endpoints that land in the same cell are treated as the same point.
struct GridKey {
    std::int64_t x = 0;
    std::int64_t y = 0;
    bool operator==(const GridKey& o) const { return x == o.x && y == o.y; }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey& k) const noexcept {
        std::uint64_t h = static_cast<std::uint64_t>(k.x) * 0x9e3779b97f4a7c15ULL;
        h ^= static_cast<std::uint64_t>(k.y) + 0x7f4a7c159e3779b9ULL + (h << 6) + (h >> 2);
        return static_cast<std::size_t>(h ^ (h >> 31));
    }
};

GridKey quantize(Vec2 p, double tolerance);

struct StitchResult {
    std::vector<Contour> contours;
    std::size_t inputSegments = 0;
    std::size_t degenerateSegments = 0;  // zero length after merging
    std::size_t duplicateSegments = 0;   // identical directed segment seen twice
    std::size_t openContours = 0;        // chains that could not be closed
};

// Join directed segments end-to-start into contours.
//
// Endpoints are merged by quantising to a grid of cell size `tolerance`
// (a spatial hash). Each merged point becomes a node; an unordered_map from
// GridKey to node id plus a CSR list of outgoing segments per node lets every
// step of the walk find its successor in O(1), so stitching is O(S) expected
// for S segments. Chains that do not return to their start are kept and
// reported as open contours instead of being dropped. Collinear points (within
// `tolerance`) are removed and each closed contour gets its signed area.
StitchResult stitchSegments(const std::vector<Segment>& segments, double tolerance);

}  // namespace ls
