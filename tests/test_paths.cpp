#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "layerslicer/mesh_gen.h"
#include "layerslicer/paths.h"
#include "test_util.h"

using namespace ls;
using lstest::sliceAt;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Sample interior points of every infill line (endpoints lie on the boundary)
// and check each one is inside the region by the even-odd rule.
void expectInfillInside(const std::vector<Contour>& contours, const std::vector<ScanLine>& lines) {
    ASSERT_FALSE(lines.empty());
    for (const ScanLine& s : lines) {
        for (int k = 1; k < 50; ++k) {
            const double t = k / 50.0;
            const Vec2 p = s.a + (s.b - s.a) * t;
            ASSERT_TRUE(pointInContours(contours, p)) << "(" << p.x << ", " << p.y << ")";
        }
    }
}

PathConfig config(double spacing, std::vector<double> angles) {
    PathConfig c;
    c.hatchSpacing = spacing;
    c.anglesDeg = std::move(angles);
    return c;
}

}  // namespace

TEST(Infill, CubeLinesStayInsideAtSeveralAngles) {
    const Layer l = sliceAt(gen::cube(10.0), 5.0);
    for (double deg : {0.0, 17.0, 45.0, 90.0, 135.0}) {
        expectInfillInside(l.contours, hatchContours(l.contours, 0.37, deg * kPi / 180.0));
    }
}

TEST(Infill, HollowBoxLinesAvoidTheHole) {
    const Layer l = sliceAt(gen::hollowBox({0, 0, 0}, {20, 20, 20}, 3.0), 10.0);
    ASSERT_EQ(l.contours.size(), 2u);
    for (double deg : {0.0, 30.0, 45.0, 90.0}) {
        const auto lines = hatchContours(l.contours, 0.5, deg * kPi / 180.0);
        expectInfillInside(l.contours, lines);
        for (const ScanLine& s : lines) {
            for (int k = 1; k < 50; ++k) {
                const Vec2 p = s.a + (s.b - s.a) * (k / 50.0);
                const bool inHole = p.x > 3.0 + 1e-9 && p.x < 17.0 - 1e-9 && p.y > 3.0 + 1e-9 && p.y < 17.0 - 1e-9;
                ASSERT_FALSE(inHole) << "(" << p.x << ", " << p.y << ")";
            }
        }
    }
}

TEST(Infill, TorusLinesAvoidTheHole) {
    const Mesh torus = gen::torus(15.0, 5.0, 96, 48);
    for (double z : {0.0, 2.0, -3.5}) {
        const Layer l = sliceAt(torus, z);
        ASSERT_EQ(l.contours.size(), 2u);
        for (double deg : {0.0, 45.0, 90.0, 135.0}) {
            const auto lines = hatchContours(l.contours, 0.4, deg * kPi / 180.0);
            expectInfillInside(l.contours, lines);
            for (const ScanLine& s : lines) {
                const Vec2 mid = (s.a + s.b) * 0.5;
                EXPECT_GT(length(mid), 9.0);  // never crosses the central opening
            }
        }
    }
}

TEST(Infill, InsetKeepsLinesShorterAndInside) {
    const Layer l = sliceAt(gen::cube(10.0), 5.0);
    const auto lines = hatchContours(l.contours, 0.5, 0.0, 0.25);
    expectInfillInside(l.contours, lines);
    for (const ScanLine& s : lines) {
        EXPECT_NEAR(distance(s.a, s.b), 9.5, 1e-9);
        EXPECT_NEAR(std::min(s.a.x, s.b.x), 0.25, 1e-9);
    }
}

TEST(Infill, SpacingAndCoverageOnSquare) {
    const Layer l = sliceAt(gen::cube(10.0), 5.0);
    const auto lines = hatchContours(l.contours, 0.5, 0.0);
    ASSERT_EQ(lines.size(), 20u);  // y = 0.25, 0.75, ..., 9.75
    std::set<double> ys;
    double total = 0.0;
    for (const ScanLine& s : lines) {
        EXPECT_DOUBLE_EQ(s.a.y, s.b.y);
        ys.insert(s.a.y);
        total += distance(s.a, s.b);
    }
    ASSERT_EQ(ys.size(), 20u);
    double prev = -1.0;
    for (double y : ys) {
        if (prev >= 0.0) {
            EXPECT_NEAR(y - prev, 0.5, 1e-12);
        }
        prev = y;
    }
    EXPECT_NEAR(total, 100.0 / 0.5, 1e-9);  // area / spacing
}

TEST(Infill, SerpentineAlternatesDirection) {
    const Layer l = sliceAt(gen::cube(10.0), 5.0);
    const auto lines = hatchContours(l.contours, 1.0, 0.0, 0.0, true);
    ASSERT_GE(lines.size(), 2u);
    for (std::size_t i = 1; i < lines.size(); ++i) {
        const double d0 = lines[i - 1].b.x - lines[i - 1].a.x;
        const double d1 = lines[i].b.x - lines[i].a.x;
        EXPECT_LT(d0 * d1, 0.0);
    }
}

TEST(Paths, AngleRotatesPerLayer) {
    const PathConfig cfg = config(0.5, {0.0, 45.0, 90.0, 135.0});
    EXPECT_EQ(infillAngleDeg(cfg, 0), 0.0);
    EXPECT_EQ(infillAngleDeg(cfg, 5), 45.0);
    EXPECT_EQ(infillAngleDeg(cfg, 7), 135.0);

    const Layer l = sliceAt(gen::cube(10.0), 5.0);
    const LayerPaths p90 = generatePaths(l.contours, cfg, 2);
    EXPECT_EQ(p90.angleDeg, 90.0);
    for (const ScanLine& s : p90.infill) EXPECT_NEAR(s.a.x, s.b.x, 1e-9);  // vertical lines
    const LayerPaths p45 = generatePaths(l.contours, cfg, 1);
    for (const ScanLine& s : p45.infill) {
        const Vec2 d = s.b - s.a;
        EXPECT_NEAR(std::fabs(d.x), std::fabs(d.y), 1e-9);
    }
}

TEST(Paths, PerimeterTracesEachContour) {
    const Layer l = sliceAt(gen::hollowBox({0, 0, 0}, {20, 20, 20}, 3.0), 10.0);
    const LayerPaths p = generatePaths(l.contours, config(0.5, {0.0}), 0);
    ASSERT_EQ(p.perimeters.size(), 2u);
    for (const auto& loop : p.perimeters) EXPECT_EQ(loop.front(), loop.back());
    EXPECT_NEAR(p.perimeterLength, 80.0 + 56.0, 1e-9);
    EXPECT_GT(p.infillLength, 0.0);
    EXPECT_GT(p.travelLength, 0.0);
    EXPECT_NEAR(p.scanLength(), p.perimeterLength + p.infillLength, 1e-12);
}

TEST(Paths, OpenContoursGetNoInfill) {
    Contour open;
    open.points = {{0, 0}, {10, 0}, {10, 10}};
    open.closed = false;
    const LayerPaths p = generatePaths({open}, config(0.5, {0.0}), 0);
    EXPECT_TRUE(p.perimeters.empty());
    EXPECT_TRUE(p.infill.empty());
}
