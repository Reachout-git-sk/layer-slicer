#include "layerslicer/mesh_gen.h"

#include <cmath>
#include <utility>
#include <vector>

namespace ls::gen {

namespace {

constexpr double kPi = 3.14159265358979323846;

void addQuad(Mesh& m, std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d) {
    m.triangles.push_back({a, b, c});
    m.triangles.push_back({a, c, d});
}

std::uint32_t addVertex(Mesh& m, Vec3 p) {
    m.vertices.push_back(p);
    return static_cast<std::uint32_t>(m.vertices.size() - 1);
}

void flip(Mesh& m) {
    for (Triangle& t : m.triangles) std::swap(t[1], t[2]);
}

void append(Mesh& dst, const Mesh& src) {
    const auto base = static_cast<std::uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (const Triangle& t : src.triangles) dst.triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
}

// Square/convex polygon helpers for steppedBlock (polygon CCW seen from +z).
void addSides(Mesh& m, const std::vector<std::uint32_t>& lower, const std::vector<std::uint32_t>& upper) {
    const std::size_t n = lower.size();
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        addQuad(m, lower[i], lower[j], upper[j], upper[i]);
    }
}

}  // namespace

Mesh box(Vec3 lo, Vec3 hi, int subdiv) {
    const int n = subdiv < 1 ? 1 : subdiv;
    // Lattice coordinates are computed by one formula per axis so that the
    // shared edges of neighbouring faces produce bit-identical points, which
    // Mesh::fromSoup then welds.
    auto coord = [&](int axis, int i) {
        const double a = axis == 0 ? lo.x : axis == 1 ? lo.y : lo.z;
        const double b = axis == 0 ? hi.x : axis == 1 ? hi.y : hi.z;
        return i == n ? b : a + (b - a) * static_cast<double>(i) / n;
    };
    auto point = [&](int ix, int iy, int iz) { return Vec3{coord(0, ix), coord(1, iy), coord(2, iz)}; };

    std::vector<TriangleSoupEntry> soup;
    // For fixed axis a, (b, c) = the next two axes cyclically, so e_b x e_c = +e_a.
    for (int a = 0; a < 3; ++a) {
        const int b = (a + 1) % 3, c = (a + 2) % 3;
        for (int side = 0; side < 2; ++side) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    auto corner = [&](int di, int dj) {
                        int idx[3];
                        idx[a] = side * n;
                        idx[b] = i + di;
                        idx[c] = j + dj;
                        return point(idx[0], idx[1], idx[2]);
                    };
                    Vec3 q[4] = {corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)};
                    if (side == 0) std::swap(q[1], q[3]);  // outward normal is -e_a
                    soup.push_back({q[0], q[1], q[2]});
                    soup.push_back({q[0], q[2], q[3]});
                }
            }
        }
    }
    return Mesh::fromSoup(soup);
}

Mesh cube(double side, int subdiv) { return box({0, 0, 0}, {side, side, side}, subdiv); }

Mesh uvSphere(double radius, int slices, int stacks, Vec3 center) {
    Mesh m;
    const std::uint32_t north = addVertex(m, center + Vec3{0, 0, radius});
    // rings[i][j] for i = 1 .. stacks-1
    std::vector<std::vector<std::uint32_t>> rings(static_cast<std::size_t>(stacks));
    for (int i = 1; i < stacks; ++i) {
        const double theta = kPi * i / stacks;
        for (int j = 0; j < slices; ++j) {
            const double phi = 2.0 * kPi * j / slices;
            rings[static_cast<std::size_t>(i)].push_back(addVertex(
                m, center + Vec3{radius * std::sin(theta) * std::cos(phi), radius * std::sin(theta) * std::sin(phi),
                                 radius * std::cos(theta)}));
        }
    }
    const std::uint32_t south = addVertex(m, center - Vec3{0, 0, radius});
    auto ring = [&](int i, int j) { return rings[static_cast<std::size_t>(i)][static_cast<std::size_t>(j % slices)]; };
    for (int j = 0; j < slices; ++j) {
        m.triangles.push_back({north, ring(1, j), ring(1, j + 1)});
        m.triangles.push_back({south, ring(stacks - 1, j + 1), ring(stacks - 1, j)});
    }
    for (int i = 1; i + 1 < stacks; ++i) {
        for (int j = 0; j < slices; ++j) addQuad(m, ring(i, j), ring(i + 1, j), ring(i + 1, j + 1), ring(i, j + 1));
    }
    return m;
}

Mesh torus(double R, double r, int nu, int nv) {
    Mesh m;
    for (int i = 0; i < nu; ++i) {
        const double u = 2.0 * kPi * i / nu;
        for (int j = 0; j < nv; ++j) {
            const double v = 2.0 * kPi * j / nv;
            addVertex(m, {(R + r * std::cos(v)) * std::cos(u), (R + r * std::cos(v)) * std::sin(u), r * std::sin(v)});
        }
    }
    auto id = [&](int i, int j) { return static_cast<std::uint32_t>((i % nu) * nv + (j % nv)); };
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) addQuad(m, id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
    }
    return m;
}

Mesh torusKnot(int p, int q, double R, double r, double tube, int nu, int nv) {
    Mesh m;
    auto curve = [&](double t) {
        const double rr = R + r * std::cos(q * t);
        return Vec3{rr * std::cos(p * t), rr * std::sin(p * t), r * std::sin(q * t)};
    };
    auto tangent = [&](double t) {
        const double rr = R + r * std::cos(q * t);
        const double drr = -r * q * std::sin(q * t);
        return normalized(Vec3{drr * std::cos(p * t) - rr * p * std::sin(p * t),
                               drr * std::sin(p * t) + rr * p * std::cos(p * t), r * q * std::cos(q * t)});
    };
    for (int i = 0; i < nu; ++i) {
        const double t = 2.0 * kPi * i / nu;
        const Vec3 c = curve(t);
        const Vec3 T = tangent(t);
        const Vec3 N = normalized(cross(T, Vec3{0, 0, 1}));  // the knot never runs vertically
        const Vec3 B = cross(N, T);
        for (int j = 0; j < nv; ++j) {
            const double v = 2.0 * kPi * j / nv;
            addVertex(m, c + N * (tube * std::cos(v)) + B * (tube * std::sin(v)));
        }
    }
    auto id = [&](int i, int j) { return static_cast<std::uint32_t>((i % nu) * nv + (j % nv)); };
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) addQuad(m, id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
    }
    if (m.signedVolume() < 0.0) flip(m);
    return m;
}

Mesh hollowBox(Vec3 lo, Vec3 hi, double wall) {
    Mesh m = box(lo, hi);
    Mesh inner = box(lo + Vec3{wall, wall, wall}, hi - Vec3{wall, wall, wall});
    flip(inner);
    append(m, inner);
    return m;
}

Mesh tetrahedron(double s, double h, double ax, double ay) {
    Mesh m;
    const auto a = addVertex(m, {0, 0, 0});
    const auto b = addVertex(m, {s, 0, 0});
    const auto c = addVertex(m, {0, s, 0});
    const auto d = addVertex(m, {ax, ay, h});
    m.triangles = {{a, c, b}, {a, b, d}, {b, c, d}, {c, a, d}};
    return m;
}

Mesh octahedron(double r) {
    Mesh m;
    std::uint32_t eq[4];
    eq[0] = addVertex(m, {r, 0, 0});
    eq[1] = addVertex(m, {0, r, 0});
    eq[2] = addVertex(m, {-r, 0, 0});
    eq[3] = addVertex(m, {0, -r, 0});
    const auto top = addVertex(m, {0, 0, r});
    const auto bottom = addVertex(m, {0, 0, -r});
    for (int k = 0; k < 4; ++k) {
        const auto e0 = eq[k], e1 = eq[(k + 1) % 4];
        m.triangles.push_back({e0, e1, top});
        m.triangles.push_back({e1, e0, bottom});
    }
    return m;
}

Mesh steppedBlock() {
    Mesh m;
    auto square = [&](double x0, double x1, double z) {
        return std::vector<std::uint32_t>{addVertex(m, {x0, x0, z}), addVertex(m, {x1, x0, z}),
                                          addVertex(m, {x1, x1, z}), addVertex(m, {x0, x1, z})};
    };
    const auto base = square(0, 10, 0);
    const auto outerMid = square(0, 10, 5);
    const auto innerMid = square(2.5, 7.5, 5);
    const auto top = square(2.5, 7.5, 10);
    addQuad(m, base[0], base[3], base[2], base[1]);  // bottom, facing -z
    addSides(m, base, outerMid);
    for (int k = 0; k < 4; ++k) {  // annulus at z = 5, facing +z
        const int k1 = (k + 1) % 4;
        addQuad(m, outerMid[k], outerMid[k1], innerMid[k1], innerMid[k]);
    }
    addSides(m, innerMid, top);
    addQuad(m, top[0], top[1], top[2], top[3]);  // top, facing +z
    return m;
}

}  // namespace ls::gen
