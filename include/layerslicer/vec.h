// Small value types used throughout the slicer.
#pragma once

#include <cmath>

namespace ls {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline bool operator==(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Vec2 a, Vec2 b) { return !(a == b); }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline double distance(Vec2 a, Vec2 b) { return length(b - a); }

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline bool operator==(Vec3 a, Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline bool operator!=(Vec3 a, Vec3 b) { return !(a == b); }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(Vec3 a) {
    const double l = length(a);
    return l > 0.0 ? a * (1.0 / l) : Vec3{};
}

struct BBox3 {
    Vec3 min{};
    Vec3 max{};
    bool valid = false;

    void expand(Vec3 p) {
        if (!valid) {
            min = max = p;
            valid = true;
            return;
        }
        min = {std::fmin(min.x, p.x), std::fmin(min.y, p.y), std::fmin(min.z, p.z)};
        max = {std::fmax(max.x, p.x), std::fmax(max.y, p.y), std::fmax(max.z, p.z)};
    }
    Vec3 size() const { return valid ? max - min : Vec3{}; }
    double diagonal() const { return length(size()); }
};

}  // namespace ls
