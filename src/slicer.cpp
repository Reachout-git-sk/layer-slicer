#include "layerslicer/slicer.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <numeric>
#include <stdexcept>
#include <thread>

#include "layerslicer/intersect.h"
#include "layerslicer/stitch.h"

namespace ls {

Slicer::Slicer(std::shared_ptr<const Mesh> mesh, SliceOptions options)
    : mesh_(std::move(mesh)), options_(std::move(options)) {
    if (!mesh_) throw std::invalid_argument("Slicer: null mesh");
    const double diag = mesh_->bounds().diagonal();
    tolerance_ = std::max(options_.relativeTolerance * diag, 1e-12);

    const std::size_t n = mesh_->triangleCount();
    std::vector<double> lo(n), hi(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Triangle& t = mesh_->triangles[i];
        const double a = mesh_->vertices[t[0]].z, b = mesh_->vertices[t[1]].z, c = mesh_->vertices[t[2]].z;
        lo[i] = std::min({a, b, c});
        hi[i] = std::max({a, b, c});
    }
    order_.resize(n);
    std::iota(order_.begin(), order_.end(), 0u);
    std::sort(order_.begin(), order_.end(), [&](std::uint32_t x, std::uint32_t y) { return lo[x] < lo[y]; });
    zmin_.resize(n);
    zmax_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        zmin_[i] = lo[order_[i]];
        zmax_[i] = hi[order_[i]];
    }
}

Layer Slicer::sliceLayer(std::size_t index, double z) const {
    Layer layer;
    layer.index = index;
    layer.z = z;

    // A triangle is cut iff zmin < z <= zmax (on-plane vertices count as above).
    std::vector<Segment> segments;
    const std::size_t end =
        static_cast<std::size_t>(std::lower_bound(zmin_.begin(), zmin_.end(), z) - zmin_.begin());
    for (std::size_t i = 0; i < end; ++i) {
        if (zmax_[i] >= z) intersectTriangle(*mesh_, mesh_->triangles[order_[i]], z, segments);
    }
    layer.segmentCount = segments.size();

    StitchResult st = stitchSegments(segments, tolerance_);
    layer.degenerateSegments = st.degenerateSegments + st.duplicateSegments;
    layer.openContours = st.openContours;
    layer.contours = std::move(st.contours);

    if (options_.generatePaths) layer.paths = generatePaths(layer.contours, options_.paths, index);
    return layer;
}

std::vector<Layer> Slicer::sliceAll(const std::vector<double>& heights, unsigned threads) const {
    const std::size_t n = heights.size();
    std::vector<Layer> results(n);  // pre-allocated: one slot per layer
    if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
    const std::size_t workers = std::min<std::size_t>(threads, std::max<std::size_t>(n, 1));

    if (workers <= 1) {
        for (std::size_t i = 0; i < n; ++i) results[i] = sliceLayer(i, heights[i]);
        return results;
    }

    std::vector<std::exception_ptr> errors(workers);  // one slot per worker
    std::vector<std::thread> pool;
    pool.reserve(workers);
    for (std::size_t t = 0; t < workers; ++t) {
        pool.emplace_back([this, t, workers, n, &heights, &results, &errors] {
            try {
                for (std::size_t i = t; i < n; i += workers) results[i] = sliceLayer(i, heights[i]);
            } catch (...) {
                errors[t] = std::current_exception();
            }
        });
    }
    for (std::thread& th : pool) th.join();
    for (const std::exception_ptr& e : errors) {
        if (e) std::rethrow_exception(e);
    }
    return results;
}

std::vector<double> Slicer::uniformHeights(const BBox3& bounds, double layerHeight) {
    std::vector<double> zs;
    if (!bounds.valid || !(layerHeight > 0.0)) return zs;
    const double height = bounds.max.z - bounds.min.z;
    const auto count = static_cast<std::size_t>(std::max(1.0, std::ceil(height / layerHeight - 1e-9)));
    zs.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double z = bounds.min.z + (static_cast<double>(i) + 0.5) * layerHeight;
        if (z >= bounds.max.z) break;
        zs.push_back(z);
    }
    return zs;
}

}  // namespace ls
