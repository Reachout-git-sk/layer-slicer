// Procedural closed meshes for tests, demos and benchmarks.
// All meshes are closed and oriented with outward-facing (counter-clockwise)
// triangles, so Mesh::signedVolume() is positive.
#pragma once

#include "layerslicer/mesh.h"

namespace ls::gen {

// Axis-aligned box; each face split into subdiv x subdiv quads.
Mesh box(Vec3 min, Vec3 max, int subdiv = 1);

// Cube of side `side` with one corner at the origin.
Mesh cube(double side, int subdiv = 1);

// UV sphere centred at `center` (stacks along z, single vertex at each pole).
Mesh uvSphere(double radius, int slices, int stacks, Vec3 center = {});

// Torus around the z axis: ring radius R, tube radius r.
Mesh torus(double R, double r, int ringSegments, int tubeSegments);

// (p, q) torus knot swept with a circular tube. With p = 2, q = 3,
// R = 20, r = 8, tube = 5 the strands never touch.
Mesh torusKnot(int p, int q, double R, double r, double tube, int curveSegments, int tubeSegments);

// Box with a closed internal cavity (inner surface inverted).
Mesh hollowBox(Vec3 min, Vec3 max, double wall);

// Tetrahedron with base (0,0,0), (s,0,0), (0,s,0) and apex (apexX, apexY, h).
Mesh tetrahedron(double s, double h, double apexX, double apexY);

// Regular octahedron with vertices at distance r on the axes.
Mesh octahedron(double r);

// 10 x 10 x 5 slab with a 5 x 5 x 5 block centred on top: a horizontal
// annular face at z = 5 lies in the plane when slicing at z = 5.
Mesh steppedBlock();

}  // namespace ls::gen
