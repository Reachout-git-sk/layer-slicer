#include <gtest/gtest.h>

#include <memory>

#include "layerslicer/mesh_gen.h"
#include "layerslicer/slicer.h"

using namespace ls;

namespace {

void expectIdentical(const std::vector<Layer>& a, const std::vector<Layer>& b) {
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Layer& x = a[i];
        const Layer& y = b[i];
        ASSERT_EQ(x.index, y.index);
        ASSERT_EQ(x.z, y.z);
        ASSERT_EQ(x.segmentCount, y.segmentCount);
        ASSERT_EQ(x.openContours, y.openContours);
        ASSERT_EQ(x.contours.size(), y.contours.size()) << "layer " << i;
        for (std::size_t c = 0; c < x.contours.size(); ++c) {
            ASSERT_EQ(x.contours[c].closed, y.contours[c].closed);
            ASSERT_EQ(x.contours[c].signedArea, y.contours[c].signedArea);
            ASSERT_EQ(x.contours[c].points.size(), y.contours[c].points.size());
            for (std::size_t k = 0; k < x.contours[c].points.size(); ++k) {
                ASSERT_EQ(x.contours[c].points[k], y.contours[c].points[k]);
            }
        }
        ASSERT_EQ(x.paths.angleDeg, y.paths.angleDeg);
        ASSERT_EQ(x.paths.infill.size(), y.paths.infill.size());
        for (std::size_t k = 0; k < x.paths.infill.size(); ++k) {
            ASSERT_EQ(x.paths.infill[k].a, y.paths.infill[k].a);
            ASSERT_EQ(x.paths.infill[k].b, y.paths.infill[k].b);
        }
        ASSERT_EQ(x.paths.perimeterLength, y.paths.perimeterLength);
        ASSERT_EQ(x.paths.infillLength, y.paths.infillLength);
        ASSERT_EQ(x.paths.travelLength, y.paths.travelLength);
    }
}

}  // namespace

TEST(ParallelSlicing, IdenticalToSingleThreaded) {
    SliceOptions opts;
    opts.paths.hatchSpacing = 0.5;
    opts.paths.anglesDeg = {0.0, 45.0, 90.0, 135.0};
    const Slicer slicer(std::make_shared<const Mesh>(gen::torusKnot(2, 3, 20, 8, 5, 600, 24)), opts);
    const auto zs = Slicer::uniformHeights(slicer.mesh().bounds(), 0.2);
    ASSERT_GT(zs.size(), 100u);
    const std::vector<Layer> serial = slicer.sliceAll(zs, 1);
    for (std::size_t i = 0; i < serial.size(); ++i) ASSERT_EQ(serial[i].index, i);
    for (unsigned threads : {2u, 3u, 4u, 8u, 0u}) {
        SCOPED_TRACE(threads);
        expectIdentical(serial, slicer.sliceAll(zs, threads));
    }
}

TEST(ParallelSlicing, MoreThreadsThanLayers) {
    const Slicer slicer(std::make_shared<const Mesh>(gen::cube(10.0)));
    const std::vector<double> zs = {2.0, 5.0, 8.0};
    expectIdentical(slicer.sliceAll(zs, 1), slicer.sliceAll(zs, 16));
}

TEST(ParallelSlicing, EmptyHeightList) {
    const Slicer slicer(std::make_shared<const Mesh>(gen::cube(10.0)));
    EXPECT_TRUE(slicer.sliceAll({}, 4).empty());
}

TEST(ParallelSlicing, SlicerSharesImmutableMesh) {
    auto mesh = std::make_shared<const Mesh>(gen::cube(10.0));
    const Slicer a(mesh);
    const Slicer b(mesh);
    EXPECT_EQ(&a.mesh(), &b.mesh());
    EXPECT_EQ(mesh.use_count(), 3);
    EXPECT_THROW(Slicer(nullptr), std::invalid_argument);
}
