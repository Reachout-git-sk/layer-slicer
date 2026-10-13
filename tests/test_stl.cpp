#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>

#include "layerslicer/mesh_gen.h"
#include "layerslicer/stl_io.h"

using namespace ls;

namespace {

Mesh floatRounded(const Mesh& m) {
    Mesh r = m;
    for (Vec3& v : r.vertices) {
        v = {toFloatPrecision(v.x), toFloatPrecision(v.y), toFloatPrecision(v.z)};
    }
    return r;
}

void expectSameMesh(const Mesh& a, const Mesh& b) {
    ASSERT_EQ(a.triangleCount(), b.triangleCount());
    ASSERT_EQ(a.vertices.size(), b.vertices.size());
    for (std::size_t i = 0; i < a.triangleCount(); ++i) {
        for (int k = 0; k < 3; ++k) {
            const Vec3 p = a.vertices[a.triangles[i][k]];
            const Vec3 q = b.vertices[b.triangles[i][k]];
            EXPECT_EQ(p.x, q.x);
            EXPECT_EQ(p.y, q.y);
            EXPECT_EQ(p.z, q.z);
        }
    }
}

const char* kAsciiTriangle =
    "solid test part\n"
    "  facet normal 0 0 1\n"
    "    outer loop\n"
    "      vertex 0 0 0\n"
    "      vertex 1 0 0\n"
    "      vertex 0 1 0\n"
    "    endloop\n"
    "  endfacet\n"
    "endsolid test part\n";

}  // namespace

TEST(StlLoader, AsciiRoundTrip) {
    const Mesh cube = gen::cube(10.0);
    const std::string text = toAsciiStl(cube, "cube");
    EXPECT_EQ(detectStlFormat(text), StlFormat::Ascii);
    const Mesh back = parseStl(text);
    expectSameMesh(cube, back);
    EXPECT_EQ(back.vertices.size(), 8u);  // shared corners are welded
    EXPECT_NEAR(back.signedVolume(), 1000.0, 1e-9);
}

TEST(StlLoader, BinaryRoundTrip) {
    const Mesh sphere = floatRounded(gen::uvSphere(10.0, 48, 24));
    const std::string data = toBinaryStl(sphere);
    EXPECT_EQ(data.size(), 84u + 50u * sphere.triangleCount());
    EXPECT_EQ(detectStlFormat(data), StlFormat::Binary);
    expectSameMesh(sphere, parseStl(data));
}

TEST(StlLoader, BinaryWithSolidHeaderIsDetectedAsBinary) {
    const Mesh cube = gen::cube(2.0);
    const std::string data = toBinaryStl(cube, "solid cube exported by some CAD tool");
    ASSERT_EQ(data.compare(0, 5, "solid"), 0);
    EXPECT_EQ(detectStlFormat(data), StlFormat::Binary);
    const Mesh back = parseStl(data);
    expectSameMesh(cube, back);
}

TEST(StlLoader, FileRoundTripBothFormats) {
    const auto dir = std::filesystem::temp_directory_path() / "layer_slicer_test_stl";
    std::filesystem::create_directories(dir);
    const Mesh torus = floatRounded(gen::torus(5.0, 2.0, 24, 12));
    saveStlBinary(torus, dir / "t_bin.stl");
    saveStlAscii(torus, dir / "t_ascii.stl");
    expectSameMesh(torus, loadStl(dir / "t_bin.stl"));
    expectSameMesh(torus, loadStl(dir / "t_ascii.stl"));
    std::filesystem::remove_all(dir);
}

TEST(StlLoader, AsciiToleratesCrLfExponentsAndSigns) {
    const std::string text =
        "solid  name with spaces\r\n facet normal 0 0 1\r\n outer loop\r\n"
        "vertex 0 0 0\r\nvertex 1e1 0 +0\r\nvertex 0 -2.5E+0 0\r\n endloop\r\n endfacet\r\nendsolid\r\n";
    const Mesh m = parseStl(text);
    ASSERT_EQ(m.triangleCount(), 1u);
    EXPECT_EQ(m.vertices[1].x, 10.0);
    EXPECT_EQ(m.vertices[2].y, -2.5);
}

TEST(StlLoader, EmptyModels) {
    EXPECT_EQ(parseStl("solid empty\nendsolid empty\n").triangleCount(), 0u);
    std::string bin(84, '\0');
    EXPECT_EQ(detectStlFormat(bin), StlFormat::Binary);
    EXPECT_EQ(parseStl(bin).triangleCount(), 0u);
}

TEST(StlLoader, ParsesSimpleAscii) {
    const Mesh m = parseStl(kAsciiTriangle);
    EXPECT_EQ(m.triangleCount(), 1u);
    EXPECT_EQ(m.vertices.size(), 3u);
}

TEST(StlLoaderMalformed, EmptyBuffer) { EXPECT_THROW(parseStl(""), StlError); }

TEST(StlLoaderMalformed, TooSmallForBinary) { EXPECT_THROW(parseStl(std::string(40, 'x')), StlError); }

TEST(StlLoaderMalformed, TruncatedBinary) {
    std::string data = toBinaryStl(gen::cube(1.0), "no solid here");
    data.resize(data.size() - 7);
    EXPECT_THROW(parseStl(data), StlError);
}

TEST(StlLoaderMalformed, BinaryCountMismatch) {
    std::string data = toBinaryStl(gen::cube(1.0), "header");
    data[80] = 13;  // claims 13 triangles, has 12
    EXPECT_THROW(parseStl(data), StlError);
}

TEST(StlLoaderMalformed, BinaryNonFiniteCoordinate) {
    std::string data = toBinaryStl(gen::cube(1.0), "header");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(&data[84 + 12], &nan, sizeof nan);  // first vertex x of first facet
    EXPECT_THROW(parseStl(data), StlError);
}

TEST(StlLoaderMalformed, AsciiMissingEndsolid) {
    std::string text = kAsciiTriangle;
    text.resize(text.find("endsolid"));
    EXPECT_THROW(parseStl(text), StlError);
}

TEST(StlLoaderMalformed, AsciiBadNumber) {
    std::string text = kAsciiTriangle;
    text.replace(text.find("vertex 1 0 0"), 12, "vertex 1 zz 0");
    EXPECT_THROW(parseStl(text), StlError);
}

TEST(StlLoaderMalformed, AsciiTwoVertexLoop) {
    const std::string text =
        "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid t\n";
    EXPECT_THROW(parseStl(text), StlError);
}

TEST(StlLoaderMalformed, AsciiTruncatedMidFacet) {
    std::string text = kAsciiTriangle;
    text.resize(text.find("vertex 0 1 0") + 9);
    EXPECT_THROW(parseStl(text), StlError);
}

TEST(StlLoaderMalformed, ErrorMessageHasLineNumber) {
    std::string text = kAsciiTriangle;
    text.replace(text.find("endloop"), 7, "endlop");
    try {
        parseStl(text);
        FAIL() << "expected StlError";
    } catch (const StlError& e) {
        EXPECT_NE(std::string(e.what()).find("line 7"), std::string::npos) << e.what();
    }
}

TEST(StlLoaderMalformed, MissingFile) {
    EXPECT_THROW(loadStl("this/file/does/not/exist.stl"), StlError);
}
