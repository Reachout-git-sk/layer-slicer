#include <gtest/gtest.h>

#include <cmath>

#include "layerslicer/intersect.h"
#include "layerslicer/mesh_gen.h"
#include "layerslicer/stitch.h"
#include "test_util.h"

using namespace ls;
using lstest::sliceAt;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectSquare(const Layer& layer, double x0, double side) {
    ASSERT_EQ(layer.contours.size(), 1u) << "z=" << layer.z;
    EXPECT_EQ(layer.openContours, 0u);
    const Contour& c = layer.contours[0];
    EXPECT_TRUE(c.closed);
    EXPECT_TRUE(c.isOuter());
    EXPECT_EQ(c.points.size(), 4u);  // collinear points merged away
    EXPECT_NEAR(c.signedArea, side * side, 1e-9);
    EXPECT_NEAR(c.length(), 4.0 * side, 1e-9);
    const BBox3 b = lstest::contourBounds(c);
    EXPECT_NEAR(b.min.x, x0, 1e-12);
    EXPECT_NEAR(b.min.y, x0, 1e-12);
    EXPECT_NEAR(b.max.x, x0 + side, 1e-12);
    EXPECT_NEAR(b.max.y, x0 + side, 1e-12);
}

void expectAllClosed(const Mesh& mesh, const std::vector<double>& zs) {
    Slicer slicer(std::make_shared<const Mesh>(mesh));
    for (std::size_t i = 0; i < zs.size(); ++i) {
        const Layer l = slicer.sliceLayer(i, zs[i]);
        EXPECT_EQ(l.openContours, 0u) << "z=" << zs[i];
        for (const Contour& c : l.contours) EXPECT_TRUE(c.closed) << "z=" << zs[i];
    }
}

}  // namespace

TEST(Generators, MeshesAreClosedAndOutward) {
    const Mesh meshes[] = {gen::cube(10.0, 3),    gen::uvSphere(5.0, 32, 16), gen::torus(10.0, 3.0, 32, 16),
                           gen::torusKnot(2, 3, 20, 8, 5, 200, 12), gen::hollowBox({0, 0, 0}, {20, 20, 20}, 3),
                           gen::tetrahedron(10, 10, 2, 2), gen::octahedron(1.0), gen::steppedBlock()};
    for (const Mesh& m : meshes) {
        EXPECT_TRUE(lstest::isClosedManifold(m));
        EXPECT_GT(m.signedVolume(), 0.0);
    }
    EXPECT_NEAR(gen::cube(10.0, 3).signedVolume(), 1000.0, 1e-9);
    EXPECT_NEAR(gen::steppedBlock().signedVolume(), 500.0 + 125.0, 1e-9);
    EXPECT_NEAR(gen::hollowBox({0, 0, 0}, {20, 20, 20}, 3).signedVolume(), 8000.0 - 2744.0, 1e-9);
}

TEST(Intersect, SegmentOrientationLeavesMaterialOnLeft) {
    const Mesh cube = gen::cube(10.0);
    const std::vector<Segment> segs = intersectMesh(cube, 5.0);
    ASSERT_FALSE(segs.empty());
    for (const Segment& s : segs) {
        // Left normal of a->b must point toward the cube centre.
        const Vec2 d = s.b - s.a;
        const Vec2 left{-d.y, d.x};
        const Vec2 mid = (s.a + s.b) * 0.5;
        EXPECT_GT(dot(left, Vec2{5, 5} - mid), 0.0);
    }
}

TEST(CubeSlicing, EveryLayerIsOneSquareOfArea100) {
    Slicer slicer(std::make_shared<const Mesh>(gen::cube(10.0)));
    const auto zs = Slicer::uniformHeights(slicer.mesh().bounds(), 0.25);
    ASSERT_EQ(zs.size(), 40u);
    for (std::size_t i = 0; i < zs.size(); ++i) expectSquare(slicer.sliceLayer(i, zs[i]), 0.0, 10.0);
}

TEST(CubeSlicing, ExactlyAtBottomFaceGivesNothingAndNoOpenContours) {
    const Layer l = sliceAt(gen::cube(10.0), 0.0);
    EXPECT_TRUE(l.contours.empty());
    EXPECT_EQ(l.openContours, 0u);
}

TEST(CubeSlicing, ExactlyAtTopFaceGivesClosedSquare) {
    // On-plane vertices count as above, so z = 10 is the section just below the top.
    expectSquare(sliceAt(gen::cube(10.0), 10.0), 0.0, 10.0);
}

TEST(CubeSlicing, AtInteriorVertexHeightsOfSubdividedCube) {
    // With 4 subdivisions there are vertex rings and in-plane edges at z = 2.5, 5, 7.5.
    const Mesh cube = gen::cube(10.0, 4);
    for (double z : {2.5, 5.0, 7.5, 10.0}) {
        const Layer l = sliceAt(cube, z);
        expectSquare(l, 0.0, 10.0);
    }
    EXPECT_TRUE(sliceAt(cube, 0.0).contours.empty());
}

TEST(CubeSlicing, ShiftedBoxGivesSquareAtItsPosition) {
    const Mesh shifted = gen::box({-3, -3, 1}, {7, 7, 11});
    expectSquare(sliceAt(shifted, 6.0), -3.0, 10.0);
    expectSquare(sliceAt(shifted, 11.0), -3.0, 10.0);
    EXPECT_TRUE(sliceAt(shifted, 1.0).contours.empty());
}

TEST(DegenerateSlicing, TetrahedronThroughApexGivesNoBrokenContours) {
    const Mesh tet = gen::tetrahedron(10.0, 10.0, 2.0, 2.0);
    const Layer apex = sliceAt(tet, 10.0);
    EXPECT_TRUE(apex.contours.empty());
    EXPECT_EQ(apex.openContours, 0u);

    const Layer base = sliceAt(tet, 0.0);
    EXPECT_TRUE(base.contours.empty());
    EXPECT_EQ(base.openContours, 0u);

    const Layer mid = sliceAt(tet, 5.0);
    ASSERT_EQ(mid.contours.size(), 1u);
    EXPECT_EQ(mid.contours[0].points.size(), 3u);
    EXPECT_NEAR(mid.contours[0].signedArea, 50.0 * 0.25, 1e-9);
}

TEST(DegenerateSlicing, OctahedronThroughEquatorVertices) {
    const Layer l = sliceAt(gen::octahedron(1.0), 0.0);
    ASSERT_EQ(l.contours.size(), 1u);
    EXPECT_EQ(l.openContours, 0u);
    EXPECT_EQ(l.contours[0].points.size(), 4u);
    EXPECT_NEAR(l.contours[0].signedArea, 2.0, 1e-12);
}

TEST(DegenerateSlicing, HorizontalFaceInPlaneGivesSectionJustBelow) {
    const Mesh m = gen::steppedBlock();
    expectSquare(sliceAt(m, 5.0), 0.0, 10.0);           // annulus face lies in the plane
    expectSquare(sliceAt(m, 5.0 + 1e-9), 2.5, 5.0);     // just above the step
    expectSquare(sliceAt(m, 7.0), 2.5, 5.0);
    expectSquare(sliceAt(m, 10.0), 2.5, 5.0);
    expectSquare(sliceAt(m, 2.0), 0.0, 10.0);
}

TEST(DegenerateSlicing, CavityFacesInPlane) {
    const Mesh m = gen::hollowBox({0, 0, 0}, {20, 20, 20}, 3.0);
    // z = 3: the cavity floor lies in the plane -> section just below has no hole.
    const Layer floor = sliceAt(m, 3.0);
    ASSERT_EQ(floor.contours.size(), 1u);
    EXPECT_NEAR(floor.contours[0].signedArea, 400.0, 1e-9);
    // z = 17: the cavity ceiling lies in the plane -> section just below has the hole.
    const Layer ceil = sliceAt(m, 17.0);
    ASSERT_EQ(ceil.contours.size(), 2u);
    EXPECT_EQ(ceil.openContours, 0u);
    EXPECT_NEAR(totalArea(ceil.contours), 400.0 - 196.0, 1e-9);
}

TEST(HoleClassification, HollowBoxHasOuterAndHole) {
    const Layer l = sliceAt(gen::hollowBox({0, 0, 0}, {20, 20, 20}, 3.0), 10.0);
    ASSERT_EQ(l.contours.size(), 2u);
    int outer = 0, holes = 0;
    for (const Contour& c : l.contours) {
        if (c.isOuter()) {
            ++outer;
            EXPECT_NEAR(c.signedArea, 400.0, 1e-9);
        }
        if (c.isHole()) {
            ++holes;
            EXPECT_NEAR(c.signedArea, -196.0, 1e-9);
        }
    }
    EXPECT_EQ(outer, 1);
    EXPECT_EQ(holes, 1);
}

TEST(HoleClassification, TorusEquatorIsAnnulus) {
    const double R = 15.0, r = 5.0;
    const int nu = 128;
    const Layer l = sliceAt(gen::torus(R, r, nu, 64), 0.0);
    ASSERT_EQ(l.contours.size(), 2u);
    EXPECT_EQ(l.openContours, 0u);
    const Contour& a = l.contours[0];
    const Contour& b = l.contours[1];
    const Contour& outer = a.isOuter() ? a : b;
    const Contour& hole = a.isOuter() ? b : a;
    ASSERT_TRUE(outer.isOuter());
    ASSERT_TRUE(hole.isHole());
    const double polyFactor = std::sin(2 * kPi / nu) / (2 * kPi / nu);  // inscribed polygon
    EXPECT_NEAR(outer.signedArea, kPi * (R + r) * (R + r) * polyFactor, 1e-3);
    EXPECT_NEAR(-hole.signedArea, kPi * (R - r) * (R - r) * polyFactor, 1e-3);
}

TEST(SphereSlicing, AreaMatchesPiTimesRSquaredMinusZSquared) {
    const double r = 10.0;
    const Mesh sphere = gen::uvSphere(r, 256, 128);
    for (double z : {-9.5, -7.0, -4.2, -1.0, 0.0, 0.3, 2.5, 5.0, 8.0, 9.5}) {
        const Layer l = sliceAt(sphere, z);
        ASSERT_EQ(l.contours.size(), 1u) << "z=" << z;
        EXPECT_EQ(l.openContours, 0u);
        EXPECT_TRUE(l.contours[0].isOuter());
        const double expected = kPi * (r * r - z * z);
        EXPECT_NEAR(l.contours[0].signedArea, expected, 2e-3 * expected) << "z=" << z;
    }
}

TEST(SphereSlicing, ExactlyAtEveryVertexRingStaysClosed) {
    const double r = 10.0;
    const int stacks = 48;
    const Mesh sphere = gen::uvSphere(r, 96, stacks);
    std::vector<double> zs;
    for (int i = 1; i < stacks; ++i) zs.push_back(r * std::cos(kPi * i / stacks));  // same formula as generator
    zs.push_back(r);
    zs.push_back(-r);
    expectAllClosed(sphere, zs);
    Slicer slicer(std::make_shared<const Mesh>(sphere));
    for (std::size_t i = 0; i + 2 < zs.size(); ++i) EXPECT_EQ(slicer.sliceLayer(i, zs[i]).contours.size(), 1u);
}

TEST(Robustness, ManyLayersOfCurvedModelsAreClosed) {
    const Mesh knot = gen::torusKnot(2, 3, 20, 8, 5, 400, 24);
    expectAllClosed(knot, Slicer::uniformHeights(knot.bounds(), 0.2));
    const Mesh torus = gen::torus(15, 5, 96, 48);
    expectAllClosed(torus, Slicer::uniformHeights(torus.bounds(), 0.1));
}

TEST(Stitching, ToleranceMergesNearlyCoincidentEndpoints) {
    // A unit square whose segment endpoints are perturbed by 1e-9.
    const std::vector<Segment> segs = {
        {{0, 0}, {1, 0}}, {{1 + 1e-9, 0}, {1, 1}}, {{1, 1 - 1e-9}, {0, 1}}, {{0, 1 + 1e-9}, {1e-9, 0}}};
    const StitchResult strict = stitchSegments(segs, 1e-12);
    EXPECT_EQ(strict.openContours, 4u);  // nothing joins without tolerance
    const StitchResult merged = stitchSegments(segs, 1e-6);
    ASSERT_EQ(merged.contours.size(), 1u);
    EXPECT_TRUE(merged.contours[0].closed);
    EXPECT_NEAR(merged.contours[0].signedArea, 1.0, 1e-6);
}

TEST(Stitching, OpenChainIsReportedNotDropped) {
    // Three sides of a square, given out of order.
    const std::vector<Segment> segs = {{{1, 1}, {0, 1}}, {{0, 0}, {1, 0}}, {{1, 0}, {1, 1}}};
    const StitchResult res = stitchSegments(segs, 1e-6);
    ASSERT_EQ(res.contours.size(), 1u);
    EXPECT_EQ(res.openContours, 1u);
    EXPECT_FALSE(res.contours[0].closed);
    EXPECT_EQ(res.contours[0].points.size(), 4u);  // reported whole, start to end
}

TEST(Stitching, MeshWithMissingTriangleReportsOpenContour) {
    Mesh cube = gen::cube(10.0);
    cube.triangles.erase(cube.triangles.begin());  // punch a hole in one side wall
    std::size_t open = 0;
    Slicer slicer(std::make_shared<const Mesh>(cube));
    for (double z : {1.0, 3.0, 5.0, 7.0, 9.0}) open += slicer.sliceLayer(0, z).openContours;
    EXPECT_GT(open, 0u);
}

TEST(Stitching, DuplicateAndDegenerateSegmentsDropped) {
    const std::vector<Segment> segs = {{{0, 0}, {1, 0}}, {{0, 0}, {1, 0}}, {{1, 0}, {0, 1}},
                                       {{0, 1}, {0, 0}}, {{0.5, 0.5}, {0.5, 0.5}}};
    const StitchResult res = stitchSegments(segs, 1e-6);
    EXPECT_EQ(res.duplicateSegments, 1u);
    EXPECT_EQ(res.degenerateSegments, 1u);
    ASSERT_EQ(res.contours.size(), 1u);
    EXPECT_NEAR(res.contours[0].signedArea, 0.5, 1e-12);
}

TEST(Layers, UniformHeightsAreMidLayer) {
    BBox3 b;
    b.expand({0, 0, 0});
    b.expand({1, 1, 1});
    const auto zs = Slicer::uniformHeights(b, 0.25);
    ASSERT_EQ(zs.size(), 4u);
    EXPECT_DOUBLE_EQ(zs[0], 0.125);
    EXPECT_DOUBLE_EQ(zs[3], 0.875);
}
