// 2D polygon helpers.
#pragma once

#include <vector>

#include "layerslicer/vec.h"

namespace ls {

// A polyline in a slice. If `closed`, the last point connects back to the
// first (the first point is NOT repeated at the end).
struct Contour {
    std::vector<Vec2> points;
    bool closed = false;
    double signedArea = 0.0;  // shoelace area; > 0 counter-clockwise (outer), < 0 clockwise (hole)

    bool isOuter() const { return closed && signedArea > 0.0; }
    bool isHole() const { return closed && signedArea < 0.0; }
    double length() const;
};

// Shoelace formula. Positive for counter-clockwise polygons.
double signedArea(const std::vector<Vec2>& polygon);

// Length of the polyline, including the closing edge when `closed`.
double polylineLength(const std::vector<Vec2>& points, bool closed);

// Even-odd point-in-polygon test against all closed contours, so a point in
// a hole is "outside". Points exactly on an edge may go either way.
bool pointInContours(const std::vector<Contour>& contours, Vec2 p);

// Sum of signed areas of closed contours (outer minus holes).
double totalArea(const std::vector<Contour>& contours);

}  // namespace ls
