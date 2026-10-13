// Laser-style scan path generation: perimeter passes and line infill.
#pragma once

#include <cstddef>
#include <vector>

#include "layerslicer/geometry.h"

namespace ls {

struct PathConfig {
    double hatchSpacing = 0.5;                // distance between infill lines (model units)
    std::vector<double> anglesDeg{0.0, 90.0};  // infill angle for layer i is anglesDeg[i % size]
    double lineInset = 0.0;                   // shorten each infill line by this much at both ends
    bool serpentine = true;                   // alternate scan direction on successive lines
    bool perimeters = true;
    bool infill = true;
};

struct ScanLine {
    Vec2 a;
    Vec2 b;
};

struct LayerPaths {
    double angleDeg = 0.0;
    std::vector<std::vector<Vec2>> perimeters;  // closed loops, first point repeated at the end
    std::vector<ScanLine> infill;               // in scan order
    double perimeterLength = 0.0;
    double infillLength = 0.0;
    double travelLength = 0.0;  // jumps between consecutive passes (laser off)

    double scanLength() const { return perimeterLength + infillLength; }
};

// Parallel hatch lines at `angleRad`, clipped to the inside of the closed
// contours with the even-odd rule (so holes are left empty). Lines lie on a
// global grid (offset by spacing/2 from the origin in the rotated frame) so
// they do not depend on the contour's position.
std::vector<ScanLine> hatchContours(const std::vector<Contour>& contours, double spacing, double angleRad,
                                    double inset = 0.0, bool serpentine = true);

// Perimeter pass for every closed contour followed by line infill.
LayerPaths generatePaths(const std::vector<Contour>& contours, const PathConfig& config, std::size_t layerIndex);

double infillAngleDeg(const PathConfig& config, std::size_t layerIndex);

}  // namespace ls
