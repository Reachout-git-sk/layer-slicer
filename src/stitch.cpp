#include "layerslicer/stitch.h"

#include <cmath>

namespace ls {

namespace {

constexpr std::uint32_t kNone = 0xffffffffu;

// True if b can be dropped from a -> b -> c without moving the polyline by
// more than `tol` (b is within tol of segment a-c and lies between them).
bool removable(Vec2 a, Vec2 b, Vec2 c, double tol) {
    const Vec2 d = c - a;
    const double len = length(d);
    if (len <= tol) return distance(a, b) <= tol;
    const Vec2 ab = b - a;
    return std::fabs(cross(d, ab)) / len <= tol && dot(ab, d) > 0.0 && dot(c - b, d) > 0.0;
}

std::vector<Vec2> simplify(const std::vector<Vec2>& pts, bool closed, double tol) {
    std::vector<Vec2> out;
    out.reserve(pts.size());
    for (const Vec2& p : pts) {
        out.push_back(p);
        while (out.size() >= 3 && removable(out[out.size() - 3], out[out.size() - 2], out.back(), tol)) {
            out.erase(out.end() - 2);
        }
    }
    if (closed) {
        // Handle the wrap-around at the seam.
        std::size_t front = 0;
        bool changed = true;
        while (changed && out.size() - front >= 3) {
            changed = false;
            if (removable(out[out.size() - 2], out.back(), out[front], tol)) {
                out.pop_back();
                changed = true;
            } else if (removable(out.back(), out[front], out[front + 1], tol)) {
                ++front;
                changed = true;
            }
        }
        out.erase(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(front));
    }
    return out;
}

}  // namespace

GridKey quantize(Vec2 p, double tolerance) {
    return {static_cast<std::int64_t>(std::llround(p.x / tolerance)),
            static_cast<std::int64_t>(std::llround(p.y / tolerance))};
}

StitchResult stitchSegments(const std::vector<Segment>& segments, double tolerance) {
    StitchResult result;
    result.inputSegments = segments.size();

    // 1. Merge endpoints: spatial hash GridKey -> node id. Open addressing in
    //    flat arrays keeps this to a couple of allocations per layer, which
    //    matters when many threads slice at once (no allocator contention).
    std::size_t capacity = 16;
    while (capacity < segments.size() * 4) capacity <<= 1;
    std::vector<GridKey> slotKey(capacity);
    std::vector<std::uint32_t> slotNode(capacity, kNone);
    const std::size_t mask = capacity - 1;
    std::vector<Vec2> nodePos;
    nodePos.reserve(segments.size() + 1);
    auto node = [&](Vec2 p) {
        const GridKey key = quantize(p, tolerance);
        std::size_t i = GridKeyHash{}(key) & mask;
        while (slotNode[i] != kNone) {
            if (slotKey[i] == key) return slotNode[i];
            i = (i + 1) & mask;
        }
        slotKey[i] = key;
        slotNode[i] = static_cast<std::uint32_t>(nodePos.size());
        nodePos.push_back(p);
        return slotNode[i];
    };

    std::vector<std::uint32_t> from, to;
    from.reserve(segments.size());
    to.reserve(segments.size());
    for (const Segment& s : segments) {
        const std::uint32_t a = node(s.a);
        const std::uint32_t b = node(s.b);
        if (a == b) {
            ++result.degenerateSegments;
            continue;
        }
        from.push_back(a);
        to.push_back(b);
    }

    // 2. Endpoint -> outgoing segments (CSR adjacency), plus in/out degrees.
    const std::size_t nNodes = nodePos.size();
    const std::size_t nSegs = from.size();
    std::vector<std::uint32_t> outStart(nNodes + 1, 0), inDeg(nNodes, 0);
    for (std::size_t i = 0; i < nSegs; ++i) {
        ++outStart[from[i] + 1];
        ++inDeg[to[i]];
    }
    for (std::size_t n = 0; n < nNodes; ++n) outStart[n + 1] += outStart[n];
    std::vector<std::uint32_t> outList(nSegs), cursor(outStart.begin(), outStart.end() - 1);
    {
        std::vector<std::uint32_t> fill(cursor);
        for (std::size_t i = 0; i < nSegs; ++i) outList[fill[from[i]]++] = static_cast<std::uint32_t>(i);
    }
    std::vector<char> used(nSegs, 0);
    // Identical directed segments (e.g. a duplicated triangle) share a start
    // node, so they are found by scanning each node's short outgoing list.
    for (std::size_t n = 0; n < nNodes; ++n) {
        for (std::uint32_t i = outStart[n]; i < outStart[n + 1]; ++i) {
            for (std::uint32_t j = outStart[n]; j < i; ++j) {
                if (!used[outList[j]] && to[outList[j]] == to[outList[i]]) {
                    used[outList[i]] = 1;
                    ++result.duplicateSegments;
                    break;
                }
            }
        }
    }
    auto nextUnused = [&](std::uint32_t n) {
        while (cursor[n] < outStart[n + 1]) {
            const std::uint32_t s = outList[cursor[n]];
            if (!used[s]) return s;
            ++cursor[n];
        }
        return kNone;
    };

    // 3. Walk chains.
    auto walk = [&](std::uint32_t s0) {
        const std::uint32_t startNode = from[s0];
        std::vector<Vec2> pts{nodePos[startNode]};
        bool closed = false;
        std::uint32_t cur = s0;
        std::size_t count = 0;
        for (;;) {
            used[cur] = 1;
            ++count;
            const std::uint32_t n = to[cur];
            if (n == startNode) {
                closed = true;
                break;
            }
            pts.push_back(nodePos[n]);
            cur = nextUnused(n);
            if (cur == kNone) break;
        }
        Contour c;
        c.closed = closed;
        c.points = simplify(pts, closed, tolerance);
        if (closed && c.points.size() < 3) {
            result.degenerateSegments += count;  // zero-area sliver loop
            return;
        }
        if (closed) {
            c.signedArea = signedArea(c.points);
        } else {
            ++result.openContours;
        }
        result.contours.push_back(std::move(c));
    };

    // Open chains first: start where a node has more outgoing than incoming
    // segments, so an open chain is reported whole rather than in pieces.
    for (std::uint32_t n = 0; n < nNodes; ++n) {
        const std::uint32_t outDeg = outStart[n + 1] - outStart[n];
        for (std::uint32_t k = inDeg[n]; k < outDeg; ++k) {
            const std::uint32_t s = nextUnused(n);
            if (s == kNone) break;
            walk(s);
        }
    }
    for (std::uint32_t s = 0; s < nSegs; ++s) {
        if (!used[s]) walk(s);
    }
    return result;
}

}  // namespace ls
