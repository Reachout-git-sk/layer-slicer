// slicer_cli: slice an STL file, print per-layer statistics and timing, or
// benchmark the parallel slicer with 1, 2, 4 and 8 threads.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "layerslicer/slicer.h"
#include "layerslicer/stl_io.h"

namespace {

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

void usage() {
    std::puts(
        "usage: slicer_cli <model.stl> [options]\n"
        "  --layer-height <h>   layer thickness (default 0.2)\n"
        "  --spacing <s>        infill hatch spacing (default 0.5)\n"
        "  --angles <a,b,...>   infill angles in degrees, cycled per layer (default 0,90)\n"
        "  --threads <n>        worker threads, 0 = hardware concurrency (default 0)\n"
        "  --no-paths           only build contours\n"
        "  --quiet              skip the per-layer table\n"
        "  --bench              time 1, 2, 4 and 8 threads\n"
        "  --repeat <n>         runs per thread count in --bench mode (default 3, median reported)\n");
}

std::vector<double> parseList(const std::string& s) {
    std::vector<double> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(std::stod(item));
    return out;
}

double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }
    std::string path;
    double layerHeight = 0.2;
    unsigned threads = 0;
    bool quiet = false, bench = false;
    int repeat = 3;
    ls::SliceOptions opts;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string a = argv[i];
            auto value = [&]() -> std::string {
                if (i + 1 >= argc) throw std::invalid_argument("missing value for " + a);
                return argv[++i];
            };
            if (a == "--layer-height") layerHeight = std::stod(value());
            else if (a == "--spacing") opts.paths.hatchSpacing = std::stod(value());
            else if (a == "--angles") opts.paths.anglesDeg = parseList(value());
            else if (a == "--threads") threads = static_cast<unsigned>(std::stoul(value()));
            else if (a == "--repeat") repeat = std::max(1, std::stoi(value()));
            else if (a == "--no-paths") opts.generatePaths = false;
            else if (a == "--quiet") quiet = true;
            else if (a == "--bench") bench = true;
            else if (a == "-h" || a == "--help") {
                usage();
                return 0;
            } else if (!a.empty() && a[0] == '-') throw std::invalid_argument("unknown option " + a);
            else path = a;
        }
        if (path.empty()) throw std::invalid_argument("no input file");
        if (!(layerHeight > 0.0)) throw std::invalid_argument("--layer-height must be positive");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "slicer_cli: %s\n", e.what());
        usage();
        return 2;
    }

    try {
        auto t0 = Clock::now();
        auto mesh = std::make_shared<const ls::Mesh>(ls::loadStl(path));
        const double loadMs = msSince(t0);
        const ls::BBox3 box = mesh->bounds();
        const ls::Slicer slicer(mesh, opts);
        const std::vector<double> zs = ls::Slicer::uniformHeights(box, layerHeight);

        std::printf("model      %s\n", path.c_str());
        std::printf("triangles  %zu   vertices %zu   load %.1f ms\n", mesh->triangleCount(), mesh->vertices.size(),
                    loadMs);
        std::printf("bounds     [%.3f, %.3f] x [%.3f, %.3f] x [%.3f, %.3f]\n", box.min.x, box.max.x, box.min.y,
                    box.max.y, box.min.z, box.max.z);
        std::printf("layers     %zu  (height %.3f, tolerance %.3g)\n", zs.size(), layerHeight, slicer.tolerance());
        std::printf("hardware   %u threads\n", std::thread::hardware_concurrency());

        if (bench) {
            // Rounds are interleaved (1, 2, 4, 8, 1, 2, 4, 8, ...) after one
            // warm-up pass so that background load drifts affect every thread
            // count alike; the median per thread count is reported.
            const unsigned counts[] = {1u, 2u, 4u, 8u};
            std::vector<std::vector<double>> times(4);
            if (slicer.sliceAll(zs, 0).size() != zs.size()) return 1;  // warm-up
            for (int r = 0; r < repeat; ++r) {
                for (std::size_t c = 0; c < 4; ++c) {
                    const auto t = Clock::now();
                    const auto layers = slicer.sliceAll(zs, counts[c]);
                    times[c].push_back(msSince(t));
                    if (layers.size() != zs.size()) return 1;
                }
            }
            std::printf("\nbenchmark: full pipeline (intersect + stitch + paths), %d interleaved round(s), median\n",
                        repeat);
            std::printf("%8s %12s %12s %12s %9s\n", "threads", "median ms", "min ms", "max ms", "speedup");
            const double base = median(times[0]);
            for (std::size_t c = 0; c < 4; ++c) {
                const double med = median(times[c]);
                std::printf("%8u %12.1f %12.1f %12.1f %8.2fx\n", counts[c], med,
                            *std::min_element(times[c].begin(), times[c].end()),
                            *std::max_element(times[c].begin(), times[c].end()), base / med);
            }
            return 0;
        }

        t0 = Clock::now();
        const std::vector<ls::Layer> layers = slicer.sliceAll(zs, threads);
        const double sliceMs = msSince(t0);

        if (!quiet) {
            std::printf("\n%6s %9s %5s %5s %5s %7s %11s %11s %11s %6s\n", "layer", "z", "outer", "holes", "open",
                        "segs", "perimeter", "infill", "travel", "angle");
        }
        std::size_t contours = 0, open = 0, segs = 0, lines = 0;
        double scan = 0.0, travel = 0.0;
        for (const ls::Layer& l : layers) {
            std::size_t outer = 0, holes = 0;
            for (const ls::Contour& c : l.contours) {
                outer += c.isOuter() ? 1 : 0;
                holes += c.isHole() ? 1 : 0;
            }
            contours += l.contours.size();
            open += l.openContours;
            segs += l.segmentCount;
            lines += l.paths.infill.size();
            scan += l.paths.scanLength();
            travel += l.paths.travelLength;
            if (!quiet) {
                std::printf("%6zu %9.4f %5zu %5zu %5zu %7zu %11.2f %11.2f %11.2f %6.1f\n", l.index, l.z, outer, holes,
                            l.openContours, l.segmentCount, l.paths.perimeterLength, l.paths.infillLength,
                            l.paths.travelLength, l.paths.angleDeg);
            }
        }
        const unsigned used = threads == 0 ? std::max(1u, std::thread::hardware_concurrency()) : threads;
        std::printf("\ntotal      %zu layers, %zu contours (%zu open), %zu segments, %zu infill lines\n", layers.size(),
                    contours, open, segs, lines);
        std::printf("paths      scan %.1f, travel %.1f (model units)\n", scan, travel);
        std::printf("time       slice %.1f ms with %u thread(s)\n", sliceMs, used);
        return open == 0 ? 0 : 3;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "slicer_cli: %s\n", e.what());
        return 1;
    }
}
