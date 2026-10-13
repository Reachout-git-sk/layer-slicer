#include "layerslicer/stl_io.h"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>

namespace ls {

namespace {

constexpr std::size_t kHeaderSize = 80;
constexpr std::size_t kBinaryPrefix = 84;
constexpr std::size_t kFacetSize = 50;

bool isSpace(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v';
}

std::uint32_t readU32(const char* p) {
    // STL is little-endian; assemble byte-wise so host endianness does not matter.
    const auto* b = reinterpret_cast<const unsigned char*>(p);
    return static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8) |
           (static_cast<std::uint32_t>(b[2]) << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
}

float readF32(const char* p) {
    const std::uint32_t u = readU32(p);
    float f = 0.0f;
    std::memcpy(&f, &u, sizeof f);
    return f;
}

void writeU32(std::string& out, std::uint32_t u) {
    out.push_back(static_cast<char>(u & 0xff));
    out.push_back(static_cast<char>((u >> 8) & 0xff));
    out.push_back(static_cast<char>((u >> 16) & 0xff));
    out.push_back(static_cast<char>((u >> 24) & 0xff));
}

void writeF32(std::string& out, double d) {
    const float f = static_cast<float>(d);
    std::uint32_t u = 0;
    std::memcpy(&u, &f, sizeof u);
    writeU32(out, u);
}

bool startsWithSolid(std::string_view data) {
    std::size_t i = 0;
    while (i < data.size() && isSpace(data[i])) ++i;
    return data.substr(i, 5) == "solid";
}

Mesh parseBinary(std::string_view data) {
    const std::uint32_t n = readU32(data.data() + kHeaderSize);
    std::vector<TriangleSoupEntry> soup;
    soup.reserve(n);
    const char* p = data.data() + kBinaryPrefix;
    for (std::uint32_t i = 0; i < n; ++i, p += kFacetSize) {
        TriangleSoupEntry tri;
        for (int v = 0; v < 3; ++v) {
            const char* q = p + 12 + v * 12;  // skip the 12-byte facet normal
            const double x = readF32(q), y = readF32(q + 4), z = readF32(q + 8);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                throw StlError("binary STL: non-finite vertex coordinate in triangle " +
                               std::to_string(i));
            }
            tri[v] = {x, y, z};
        }
        soup.push_back(tri);
    }
    return Mesh::fromSoup(soup);
}

// Whitespace tokenizer for ASCII STL that tracks line numbers for messages.
class Tokenizer {
public:
    explicit Tokenizer(std::string_view data) : data_(data) {}

    bool next(std::string_view& tok) {
        while (pos_ < data_.size() && isSpace(data_[pos_])) {
            if (data_[pos_] == '\n') ++line_;
            ++pos_;
        }
        if (pos_ >= data_.size()) return false;
        const std::size_t start = pos_;
        while (pos_ < data_.size() && !isSpace(data_[pos_])) ++pos_;
        tok = data_.substr(start, pos_ - start);
        return true;
    }

    void skipLine() {
        while (pos_ < data_.size() && data_[pos_] != '\n') ++pos_;
    }

    [[noreturn]] void fail(const std::string& what) const {
        throw StlError("ASCII STL line " + std::to_string(line_) + ": " + what);
    }

    void expect(std::string_view word) {
        std::string_view tok;
        if (!next(tok)) fail("unexpected end of file, expected '" + std::string(word) + "'");
        if (tok != word) fail("expected '" + std::string(word) + "', got '" + std::string(tok) + "'");
    }

    double number() {
        std::string_view tok;
        if (!next(tok)) fail("unexpected end of file, expected a number");
        double value = 0.0;
        const char* first = tok.data();
        const char* last = tok.data() + tok.size();
        if (first != last && *first == '+') ++first;  // from_chars rejects a leading '+'
        // from_chars is locale-independent, unlike strtod (Qt may set a locale).
        const auto res = std::from_chars(first, last, value);
        if (res.ec != std::errc() || res.ptr != last || !std::isfinite(value)) {
            fail("invalid number '" + std::string(tok) + "'");
        }
        return value;
    }

private:
    std::string_view data_;
    std::size_t pos_ = 0;
    std::size_t line_ = 1;
};

Mesh parseAscii(std::string_view data) {
    Tokenizer tk(data);
    tk.expect("solid");
    tk.skipLine();  // the solid name may contain arbitrary words

    std::vector<TriangleSoupEntry> soup;
    std::string_view tok;
    bool ended = false;
    while (tk.next(tok)) {
        if (tok == "endsolid") {
            ended = true;
            break;
        }
        if (tok != "facet") {
            tk.fail("expected 'facet' or 'endsolid', got '" + std::string(tok) + "'");
        }
        tk.expect("normal");
        tk.number();
        tk.number();
        tk.number();
        tk.expect("outer");
        tk.expect("loop");
        TriangleSoupEntry tri;
        for (int v = 0; v < 3; ++v) {
            tk.expect("vertex");
            const double x = tk.number();
            const double y = tk.number();
            const double z = tk.number();
            tri[v] = {x, y, z};
        }
        tk.expect("endloop");
        tk.expect("endfacet");
        soup.push_back(tri);
    }
    if (!ended) tk.fail("missing 'endsolid'");
    return Mesh::fromSoup(soup);
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw StlError("cannot open '" + path.string() + "'");
    std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (in.bad()) throw StlError("error reading '" + path.string() + "'");
    return data;
}

void writeFile(const std::filesystem::path& path, const std::string& data) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) throw StlError("cannot create '" + path.string() + "'");
    out.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!out) throw StlError("error writing '" + path.string() + "'");
}

Vec3 facetNormal(const TriangleSoupEntry& t) {
    return normalized(cross(t[1] - t[0], t[2] - t[0]));
}

}  // namespace

StlFormat detectStlFormat(std::string_view data) {
    if (data.size() >= kBinaryPrefix) {
        const std::uint64_t n = readU32(data.data() + kHeaderSize);
        if (kBinaryPrefix + kFacetSize * n == data.size()) return StlFormat::Binary;
    }
    if (startsWithSolid(data)) return StlFormat::Ascii;
    if (data.size() < kBinaryPrefix) {
        throw StlError("file too small to be a binary STL (" + std::to_string(data.size()) +
                       " bytes)");
    }
    const std::uint64_t n = readU32(data.data() + kHeaderSize);
    throw StlError("binary STL size mismatch: header declares " + std::to_string(n) +
                   " triangles (" + std::to_string(kBinaryPrefix + kFacetSize * n) +
                   " bytes) but file has " + std::to_string(data.size()) + " bytes");
}

Mesh parseStl(std::string_view data) {
    return detectStlFormat(data) == StlFormat::Binary ? parseBinary(data) : parseAscii(data);
}

Mesh loadStl(const std::filesystem::path& path) { return parseStl(readFile(path)); }

std::string toBinaryStl(const Mesh& mesh, std::string_view header) {
    std::string out;
    out.reserve(kBinaryPrefix + kFacetSize * mesh.triangleCount());
    std::string head(header.substr(0, kHeaderSize));
    head.resize(kHeaderSize, '\0');
    out += head;
    writeU32(out, static_cast<std::uint32_t>(mesh.triangleCount()));
    for (const TriangleSoupEntry& t : mesh.toSoup()) {
        const Vec3 n = facetNormal(t);
        writeF32(out, n.x);
        writeF32(out, n.y);
        writeF32(out, n.z);
        for (const Vec3& v : t) {
            writeF32(out, v.x);
            writeF32(out, v.y);
            writeF32(out, v.z);
        }
        out.push_back('\0');
        out.push_back('\0');
    }
    return out;
}

std::string toAsciiStl(const Mesh& mesh, std::string_view solidName) {
    std::string out;
    out.reserve(256 * mesh.triangleCount() + 64);
    out += "solid ";
    out += solidName;
    out += '\n';
    char buf[160];
    auto vec = [&](const char* prefix, Vec3 v) {
        std::snprintf(buf, sizeof buf, "%s %.9g %.9g %.9g\n", prefix,
                      static_cast<double>(static_cast<float>(v.x)),
                      static_cast<double>(static_cast<float>(v.y)),
                      static_cast<double>(static_cast<float>(v.z)));
        out += buf;
    };
    for (const TriangleSoupEntry& t : mesh.toSoup()) {
        vec("  facet normal", facetNormal(t));
        out += "    outer loop\n";
        for (const Vec3& v : t) vec("      vertex", v);
        out += "    endloop\n  endfacet\n";
    }
    out += "endsolid ";
    out += solidName;
    out += '\n';
    return out;
}

void saveStlBinary(const Mesh& mesh, const std::filesystem::path& path, std::string_view header) {
    writeFile(path, toBinaryStl(mesh, header));
}

void saveStlAscii(const Mesh& mesh, const std::filesystem::path& path, std::string_view solidName) {
    writeFile(path, toAsciiStl(mesh, solidName));
}

}  // namespace ls
