#ifndef ENGINE_KERNEL_SHA256_HPP
#define ENGINE_KERNEL_SHA256_HPP

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace engine
{

// Self-contained SHA-256 (FIPS 180-4). The kernel uses it to bind trust
// decisions to component content: a trust entry may carry the digest of the
// binary or scripts it approves, and changed content forces a new decision.
class Sha256 final
{
public:
    static constexpr std::size_t digestSize    = 32;
    static constexpr std::size_t hexDigestSize = digestSize * 2;

    Sha256();

    void update(const void *data, std::size_t size);

    void finish(std::uint8_t outDigest[digestSize]);

    // Lowercase hexadecimal digest of a buffer ("abc" ->
    // ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad).
    static std::string hexDigest(const void *data, std::size_t size);

    // Lowercase hexadecimal digest of a file's bytes; returns an empty string
    // when the file cannot be read (the error string explains why).
    static std::string fileHexDigest(
        const std::filesystem::path &path,
        std::string                 &error
    );

private:
    std::uint32_t state[8];
    std::uint8_t  buffer[64];
    std::size_t   bufferedBytes;
    std::uint64_t totalBytes;
};

// True when 'text' is 64 hexadecimal characters (either case); comparison of
// digests is case-insensitive.
bool isValidSha256HexDigest(const std::string &text);

bool sha256HexDigestsEqual(const std::string &left, const std::string &right);

} // namespace engine

#endif
