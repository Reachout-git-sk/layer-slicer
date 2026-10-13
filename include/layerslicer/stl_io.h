// STL reading and writing (binary and ASCII).
#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

#include "layerslicer/mesh.h"

namespace ls {

class StlError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

enum class StlFormat { Ascii, Binary };

// Round a coordinate to the nearest 32-bit float (the precision STL stores).
// The volatile store keeps the rounding from being folded away: GCC 13.1's SLP
// vectoriser was observed to drop double->float->double round trips at -O2.
inline double toFloatPrecision(double d) {
    volatile float f = static_cast<float>(d);
    return f;
}

// Decide whether a buffer holds a binary or ASCII STL.
//
// A binary file is 80 header bytes + uint32 triangle count + 50 bytes per
// triangle. If the buffer size is exactly 84 + 50*n it is treated as binary,
// even when the header happens to begin with "solid" (many exporters do
// this). Otherwise a buffer starting with "solid" is ASCII. Anything else is
// rejected with StlError.
StlFormat detectStlFormat(std::string_view data);

// Parse an in-memory STL buffer. Throws StlError on malformed input.
Mesh parseStl(std::string_view data);

// Load an STL file from disk. Throws StlError on I/O or format errors.
Mesh loadStl(const std::filesystem::path& path);

// Serialise to STL. Coordinates are written as 32-bit floats (binary) or as
// floats with 9 significant digits (ASCII). Loading reads both as 32-bit
// floats, so a mesh round-trips exactly at float precision.
std::string toBinaryStl(const Mesh& mesh, std::string_view header = "layer-slicer binary STL");
std::string toAsciiStl(const Mesh& mesh, std::string_view solidName = "layer_slicer");

void saveStlBinary(const Mesh& mesh, const std::filesystem::path& path,
                   std::string_view header = "layer-slicer binary STL");
void saveStlAscii(const Mesh& mesh, const std::filesystem::path& path,
                  std::string_view solidName = "layer_slicer");

}  // namespace ls
