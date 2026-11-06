// Minimal FIPS 180-4 SHA-256 used for trust bindings and descriptor
// caches.

#ifndef ENGINE_KERNEL_SHA256_HPP
#define ENGINE_KERNEL_SHA256_HPP

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace engine
{

// Streaming hasher: update() any number of times, finish() once. Not
// thread-safe; one instance per hash.
class Sha256 final
{
public:
    // 32-byte digest / 64-character hex form.
    static constexpr std::size_t digestSize    = 32;
    static constexpr std::size_t hexDigestSize = digestSize * 2;

    Sha256();

    void update(const void *data, std::size_t size);

    void finish(std::uint8_t outDigest[digestSize]);

    // One-shot in-memory hash as lowercase hex.
    static std::string hexDigest(const void *data, std::size_t size);

    // Streams the file in 64 KiB chunks; returns an empty string and fills
    // error when the file cannot be opened or read.
    static std::string fileHexDigest(const std::filesystem::path &path, std::string &error);

private:
    // state: 8 chaining words; buffer: the 64-byte block under construction;
    // totalBytes: bytes hashed so far (for the length footer).
    std::uint32_t state[8];
    std::uint8_t  buffer[64];
    std::size_t   bufferedBytes;
    std::uint64_t totalBytes;
};

// 64 hex characters (either case)?
bool isValidSha256HexDigest(const std::string &text);

// Case-insensitive comparison for config-provided digests.
bool sha256HexDigestsEqual(const std::string &left, const std::string &right);

} // namespace engine

#endif
