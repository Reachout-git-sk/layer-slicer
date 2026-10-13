#include "layerslicer/paths.h"

#include <algorithm>
#include <cmath>

namespace ls {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Rotation {
    double c, s;
    // Into the hatch frame (rotate by -angle) and back (rotate by +angle).
    Vec2 toLocal(Vec2 p) const { return {p.x * c + p.y * s, -p.x * s + p.y * c}; }
    Vec2 toWorld(Vec2 p) const { return {p.x * c - p.y * s, p.x * s + p.y * c}; }
};

}  // namespace

double infillAngleDeg(const PathConfig& config, std::size_t layerIndex) {
    if (config.anglesDeg.empty()) return 0.0;
    return config.anglesDeg[layerIndex % config.anglesDeg.size()];
}

std::vector<ScanLine> hatchContours(const std::vector<Contour>& contours, double spacing, double angleRad,
                                    double inset, bool serpentine) {
    std::vector<ScanLine> lines;
    if (!(spacing > 0.0)) return lines;
    const Rotation rot{std::cos(angleRad), std::sin(angleRad)};

    // Rotate all closed contour edges into the hatch frame, where scanlines are
    // horizontal: y_k = (k + 0.5) * spacing.
    struct Edge {
        Vec2 p, q;
    };
    std::vector<Edge> edges;
    double ymin = INFINITY, ymax = -INFINITY;
    for (const Contour& c : contours) {
        if (!c.closed || c.points.size() < 3) continue;
        const std::size_t n = c.points.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const Edge e{rot.toLocal(c.points[j]), rot.toLocal(c.points[i])};
            if (e.p.y == e.q.y) continue;  // horizontal edges never cross a half-open scanline test
            ymin = std::min({ymin, e.p.y, e.q.y});
            ymax = std::max({ymax, e.p.y, e.q.y});
            edges.push_back(e);
        }
    }
    if (edges.empty()) return lines;

    const auto kFirst = static_cast<long long>(std::ceil(ymin / spacing - 0.5)) - 1;
    const auto kLast = static_cast<long long>(std::floor(ymax / spacing - 0.5)) + 1;
    auto lineY = [&](long long k) { return (static_cast<double>(k) + 0.5) * spacing; };

    // Bucket edge crossings per scanline: O(E + crossings) instead of O(E * lines).
    // Half-open rule (p.y <= y) != (q.y <= y): a scanline through a vertex counts
    // exactly one of the two edges meeting there, so even-odd pairing stays valid.
    // Crossings are collected as (line, x) pairs in one flat vector and sorted,
    // which avoids one allocation per scanline.
    struct Hit {
        std::size_t line;
        double x;
    };
    std::vector<Hit> hits;
    hits.reserve(edges.size());
    for (const Edge& e : edges) {
        const double lo = std::min(e.p.y, e.q.y);
        const double hi = std::max(e.p.y, e.q.y);
        long long k0 = static_cast<long long>(std::ceil(lo / spacing - 0.5)) - 1;
        long long k1 = static_cast<long long>(std::ceil(hi / spacing - 0.5));
        k0 = std::max(k0, kFirst);
        k1 = std::min(k1, kLast);
        for (long long k = k0; k <= k1; ++k) {
            const double y = lineY(k);
            if ((e.p.y <= y) != (e.q.y <= y)) {
                const double x = e.p.x + (y - e.p.y) * (e.q.x - e.p.x) / (e.q.y - e.p.y);
                hits.push_back({static_cast<std::size_t>(k - kFirst), x});
            }
        }
    }
    std::sort(hits.begin(), hits.end(),
              [](const Hit& a, const Hit& b) { return a.line != b.line ? a.line < b.line : a.x < b.x; });

    bool reverse = false;
    std::vector<ScanLine> row;
    for (std::size_t h = 0; h < hits.size();) {
        std::size_t e = h;
        while (e < hits.size() && hits[e].line == hits[h].line) ++e;
        const double y = lineY(kFirst + static_cast<long long>(hits[h].line));
        row.clear();
        for (std::size_t i = h; i + 1 < e; i += 2) {  // even-odd: inside between pairs
            const double x0 = hits[i].x + inset;
            const double x1 = hits[i + 1].x - inset;
            if (x1 - x0 <= 0.0) continue;
            row.push_back({rot.toWorld({x0, y}), rot.toWorld({x1, y})});
        }
        h = e;
        if (row.empty()) continue;
        if (reverse) {
            std::reverse(row.begin(), row.end());
            for (ScanLine& s : row) std::swap(s.a, s.b);
        }
        lines.insert(lines.end(), row.begin(), row.end());
        if (serpentine) reverse = !reverse;
    }
    return lines;
}

LayerPaths generatePaths(const std::vector<Contour>& contours, const PathConfig& config, std::size_t layerIndex) {
    LayerPaths paths;
    paths.angleDeg = infillAngleDeg(config, layerIndex);

    bool havePos = false;
    Vec2 pos{};
    auto travelTo = [&](Vec2 p) {
        if (havePos) paths.travelLength += distance(pos, p);
        havePos = true;
    };

    if (config.perimeters) {
        for (const Contour& c : contours) {
            if (!c.closed || c.points.size() < 3) continue;
            std::vector<Vec2> loop = c.points;
            loop.push_back(c.points.front());
            travelTo(loop.front());
            paths.perimeterLength += polylineLength(loop, false);
            pos = loop.back();
            paths.perimeters.push_back(std::move(loop));
        }
    }
    if (config.infill) {
        paths.infill = hatchContours(contours, config.hatchSpacing, paths.angleDeg * kPi / 180.0, config.lineInset,
                                     config.serpentine);
        for (const ScanLine& s : paths.infill) {
            travelTo(s.a);
            paths.infillLength += distance(s.a, s.b);
            pos = s.b;
        }
    }
    return paths;
}

}  // namespace ls
