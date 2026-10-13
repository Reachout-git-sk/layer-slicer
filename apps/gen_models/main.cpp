// Writes the demo/benchmark models into a directory (default: models/).
#include <cstdio>
#include <filesystem>
#include <string>

#include "layerslicer/mesh_gen.h"
#include "layerslicer/stl_io.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1]) : fs::path("models");
    try {
        fs::create_directories(dir);
        auto report = [&](const char* name, const ls::Mesh& m) {
            std::printf("%-16s %8zu triangles  %8zu vertices  volume %.3f\n", name, m.triangleCount(),
                        m.vertices.size(), m.signedVolume());
        };

        const ls::Mesh cube = ls::gen::cube(10.0);
        ls::saveStlAscii(cube, dir / "cube.stl", "cube_10mm");
        report("cube.stl", cube);

        const ls::Mesh sphere = ls::gen::uvSphere(10.0, 96, 48, {0, 0, 10});
        ls::saveStlBinary(sphere, dir / "sphere.stl", "sphere r=10");
        report("sphere.stl", sphere);

        const ls::Mesh torus = ls::gen::torus(15.0, 5.0, 96, 48);
        ls::saveStlBinary(torus, dir / "torus.stl", "torus R=15 r=5");
        report("torus.stl", torus);

        const ls::Mesh hollow = ls::gen::hollowBox({0, 0, 0}, {20, 20, 20}, 3.0);
        ls::saveStlBinary(hollow, dir / "hollow_box.stl", "hollow box 20mm wall 3mm");
        report("hollow_box.stl", hollow);

        const ls::Mesh knot = ls::gen::torusKnot(2, 3, 20.0, 8.0, 5.0, 1600, 48);
        ls::saveStlBinary(knot, dir / "torus_knot.stl", "(2,3) torus knot, benchmark model");
        report("torus_knot.stl", knot);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "gen_models: %s\n", e.what());
        return 1;
    }
    return 0;
}
