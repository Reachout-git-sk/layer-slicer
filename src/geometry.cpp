#include "layerslicer/geometry.h"

namespace ls {

double Contour::length() const { return polylineLength(points, closed); }

double signedArea(const std::vector<Vec2>& polygon) {
    const std::size_t n = polygon.size();
    if (n < 3) return 0.0;
    double twice = 0.0;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) twice += cross(polygon[j], polygon[i]);
    return 0.5 * twice;
}

double polylineLength(const std::vector<Vec2>& points, bool closed) {
    if (points.size() < 2) return 0.0;
    double len = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) len += distance(points[i - 1], points[i]);
    if (closed) len += distance(points.back(), points.front());
    return len;
}

bool pointInContours(const std::vector<Contour>& contours, Vec2 p) {
    bool inside = false;
    for (const Contour& c : contours) {
        if (!c.closed) continue;
        const std::vector<Vec2>& pts = c.points;
        const std::size_t n = pts.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const Vec2 a = pts[j];
            const Vec2 b = pts[i];
            if ((a.y <= p.y) != (b.y <= p.y)) {
                const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (p.x < x) inside = !inside;
            }
        }
    }
    return inside;
}

double totalArea(const std::vector<Contour>& contours) {
    double a = 0.0;
    for (const Contour& c : contours) {
        if (c.closed) a += c.signedArea;
    }
    return a;
}

}  // namespace ls
