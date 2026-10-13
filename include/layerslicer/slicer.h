// Layer slicing pipeline: intersect -> merge/stitch -> scan paths, in parallel.
#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "layerslicer/geometry.h"
#include "layerslicer/mesh.h"
#include "layerslicer/paths.h"

namespace ls {

struct SliceOptions {
    // Endpoint merge tolerance relative to the bounding-box diagonal.
    double relativeTolerance = 1e-6;
    bool generatePaths = true;
    PathConfig paths;
};

struct Layer {
    std::size_t index = 0;
    double z = 0.0;
    std::vector<Contour> contours;
    std::size_t segmentCount = 0;        // raw triangle/plane segments
    std::size_t degenerateSegments = 0;  // dropped as zero length after merging
    std::size_t openContours = 0;        // contours that failed to close (bad mesh)
    LayerPaths paths;

    std::size_t closedContours() const { return contours.size() - openContours; }
};

class Slicer {
public:
    explicit Slicer(std::shared_ptr<const Mesh> mesh, SliceOptions options = {});

    const Mesh& mesh() const { return *mesh_; }
    const std::shared_ptr<const Mesh>& meshPtr() const { return mesh_; }
    const SliceOptions& options() const { return options_; }
    double tolerance() const { return tolerance_; }

    // Slice a single plane z = h (index selects the infill angle).
    Layer sliceLayer(std::size_t index, double z) const;

    // Slice all heights using `threads` worker threads (0 = hardware
    // concurrency). Layers are distributed cyclically (thread t takes layers
    // t, t+N, t+2N, ...) which balances uneven per-layer cost. Every worker
    // writes only into its own pre-allocated slots of the result vector and
    // reads only immutable shared data, so no locks are needed. The result is
    // identical for any thread count.
    std::vector<Layer> sliceAll(const std::vector<double>& heights, unsigned threads = 0) const;

    // Heights at the middle of each layer of thickness `layerHeight`,
    // from the bottom of the mesh to its top.
    static std::vector<double> uniformHeights(const BBox3& bounds, double layerHeight);

private:
    std::shared_ptr<const Mesh> mesh_;
    SliceOptions options_;
    double tolerance_ = 1e-9;
    // Triangle ids sorted by their minimum z, with matching min/max z arrays,
    // so a layer only visits triangles whose z range can reach it.
    std::vector<std::uint32_t> order_;
    std::vector<double> zmin_;
    std::vector<double> zmax_;
};

}  // namespace ls
