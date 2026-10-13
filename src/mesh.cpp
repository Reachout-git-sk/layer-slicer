#include "layerslicer/mesh.h"

#include <cstring>
#include <functional>
#include <unordered_map>

namespace ls {

namespace {

struct Vec3Hash {
    std::size_t operator()(const Vec3& v) const noexcept {
        // Hash the raw bit patterns; normalise -0.0 to 0.0 so they weld.
        auto bits = [](double d) {
            if (d == 0.0) d = 0.0;
            std::uint64_t u = 0;
            std::memcpy(&u, &d, sizeof u);
            return u;
        };
        std::size_t h = std::hash<std::uint64_t>{}(bits(v.x));
        h ^= std::hash<std::uint64_t>{}(bits(v.y)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        h ^= std::hash<std::uint64_t>{}(bits(v.z)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return h;
    }
};

}  // namespace

BBox3 Mesh::bounds() const {
    BBox3 box;
    for (const Vec3& v : vertices) box.expand(v);
    return box;
}

double Mesh::signedVolume() const {
    double vol = 0.0;
    for (const Triangle& t : triangles) {
        const Vec3& a = vertices[t[0]];
        const Vec3& b = vertices[t[1]];
        const Vec3& c = vertices[t[2]];
        vol += dot(a, cross(b, c));
    }
    return vol / 6.0;
}

Mesh Mesh::fromSoup(const std::vector<TriangleSoupEntry>& soup) {
    Mesh mesh;
    mesh.triangles.reserve(soup.size());
    std::unordered_map<Vec3, std::uint32_t, Vec3Hash> index;
    index.reserve(soup.size());

    auto vertexId = [&](Vec3 p) {
        if (p.x == 0.0) p.x = 0.0;  // fold -0.0
        if (p.y == 0.0) p.y = 0.0;
        if (p.z == 0.0) p.z = 0.0;
        auto [it, inserted] = index.try_emplace(p, static_cast<std::uint32_t>(mesh.vertices.size()));
        if (inserted) mesh.vertices.push_back(p);
        return it->second;
    };

    for (const TriangleSoupEntry& tri : soup) {
        const Triangle t{vertexId(tri[0]), vertexId(tri[1]), vertexId(tri[2])};
        if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) continue;
        mesh.triangles.push_back(t);
    }
    return mesh;
}

std::vector<TriangleSoupEntry> Mesh::toSoup() const {
    std::vector<TriangleSoupEntry> soup;
    soup.reserve(triangles.size());
    for (const Triangle& t : triangles) {
        soup.push_back({vertices[t[0]], vertices[t[1]], vertices[t[2]]});
    }
    return soup;
}

}  // namespace ls
