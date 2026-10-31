#include <engine/kernel/sha256.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace engine
{

namespace
{

constexpr std::array<std::uint32_t, 64> roundConstants = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

std::uint32_t rotateRight(std::uint32_t value, unsigned bits)
{
    return (value >> bits) | (value << (32 - bits));
}

void sha256CompressBlock(std::uint32_t state[8], const std::uint8_t block[64])
{
    std::array<std::uint32_t, 64> schedule{};
    for (std::size_t index = 0; index < 16; ++index)
    {
        schedule[index] = (static_cast<std::uint32_t>(block[index * 4]) << 24)
                        | (static_cast<std::uint32_t>(block[index * 4 + 1]) << 16)
                        | (static_cast<std::uint32_t>(block[index * 4 + 2]) << 8)
                        | static_cast<std::uint32_t>(block[index * 4 + 3]);
    }
    for (std::size_t index = 16; index < 64; ++index)
    {
        const std::uint32_t s0 = rotateRight(schedule[index - 15], 7) ^ rotateRight(schedule[index - 15], 18)
                               ^ (schedule[index - 15] >> 3);
        const std::uint32_t s1 = rotateRight(schedule[index - 2], 17) ^ rotateRight(schedule[index - 2], 19)
                               ^ (schedule[index - 2] >> 10);
        schedule[index]        = schedule[index - 16] + s0 + schedule[index - 7] + s1;
    }

    std::uint32_t a = state[0];
    std::uint32_t b = state[1];
    std::uint32_t c = state[2];
    std::uint32_t d = state[3];
    std::uint32_t e = state[4];
    std::uint32_t f = state[5];
    std::uint32_t g = state[6];
    std::uint32_t h = state[7];

    for (std::size_t index = 0; index < 64; ++index)
    {
        const std::uint32_t s1  = rotateRight(e, 6) ^ rotateRight(e, 11) ^ rotateRight(e, 25);
        const std::uint32_t ch  = (e & f) ^ (~e & g);
        const std::uint32_t t1  = h + s1 + ch + roundConstants[index] + schedule[index];
        const std::uint32_t s0  = rotateRight(a, 2) ^ rotateRight(a, 13) ^ rotateRight(a, 22);
        const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t t2  = s0 + maj;

        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
}

char hexDigit(const std::uint8_t value)
{
    static const char digits[] = "0123456789abcdef";
    return digits[value & 0x0Fu];
}

} // namespace

Sha256::Sha256()
{
    state[0] = 0x6a09e667u;
    state[1] = 0xbb67ae85u;
    state[2] = 0x3c6ef372u;
    state[3] = 0xa54ff53au;
    state[4] = 0x510e527fu;
    state[5] = 0x9b05688cu;
    state[6] = 0x1f83d9abu;
    state[7] = 0x5be0cd19u;
    bufferedBytes = 0;
    totalBytes    = 0;
}

void Sha256::update(const void *data, std::size_t size)
{
    const auto *bytes = static_cast<const std::uint8_t *>(data);
    totalBytes += static_cast<std::uint64_t>(size);

    if (bufferedBytes > 0)
    {
        const std::size_t take = std::min(size, std::size_t{64} - bufferedBytes);
        std::memcpy(buffer + bufferedBytes, bytes, take);
        bufferedBytes += take;
        bytes += take;
        size -= take;
        if (bufferedBytes == 64)
        {
            sha256CompressBlock(state, buffer);
            bufferedBytes = 0;
        }
    }
    while (size >= 64)
    {
        sha256CompressBlock(state, bytes);
        bytes += 64;
        size -= 64;
    }
    if (size > 0)
    {
        std::memcpy(buffer, bytes, size);
        bufferedBytes = size;
    }
}

void Sha256::finish(std::uint8_t outDigest[digestSize])
{
    const std::uint64_t bitLength = totalBytes * 8;

    const std::uint8_t padding = 0x80u;
    update(&padding, 1);
    const std::uint8_t zero = 0x00u;
    while (bufferedBytes != 56)
    {
        update(&zero, 1);
    }
    const std::array<std::uint8_t, 8> lengthBytes = {
        static_cast<std::uint8_t>((bitLength >> 56) & 0xFFu),
        static_cast<std::uint8_t>((bitLength >> 48) & 0xFFu),
        static_cast<std::uint8_t>((bitLength >> 40) & 0xFFu),
        static_cast<std::uint8_t>((bitLength >> 32) & 0xFFu),
        static_cast<std::uint8_t>((bitLength >> 24) & 0xFFu),
        static_cast<std::uint8_t>((bitLength >> 16) & 0xFFu),
        static_cast<std::uint8_t>((bitLength >> 8) & 0xFFu),
        static_cast<std::uint8_t>(bitLength & 0xFFu),
    };
    // Feed the length directly through the compression path: update() would
    // count it into totalBytes, which no longer matters here, but keep the
    // buffer bookkeeping exact by compressing manually.
    std::memcpy(buffer + 56, lengthBytes.data(), 8);
    sha256CompressBlock(state, buffer);
    bufferedBytes = 0;

    for (std::size_t index = 0; index < 8; ++index)
    {
        outDigest[index * 4]     = static_cast<std::uint8_t>((state[index] >> 24) & 0xFFu);
        outDigest[index * 4 + 1] = static_cast<std::uint8_t>((state[index] >> 16) & 0xFFu);
        outDigest[index * 4 + 2] = static_cast<std::uint8_t>((state[index] >> 8) & 0xFFu);
        outDigest[index * 4 + 3] = static_cast<std::uint8_t>(state[index] & 0xFFu);
    }
}

std::string Sha256::hexDigest(const void *data, std::size_t size)
{
    Sha256         hasher;
    std::uint8_t   digest[digestSize];
    hasher.update(data, size);
    hasher.finish(digest);

    std::string hex;
    hex.reserve(hexDigestSize);
    for (const std::uint8_t byte : digest)
    {
        hex.push_back(hexDigit(static_cast<std::uint8_t>(byte >> 4)));
        hex.push_back(hexDigit(byte));
    }
    return hex;
}

std::string Sha256::fileHexDigest(
    const std::filesystem::path &path,
    std::string                 &error
)
{
    error.clear();
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if (!stream.is_open())
    {
        error = "cannot open '" + path.string() + "'";
        return {};
    }
    Sha256       hasher;
    std::uint8_t digest[digestSize];
    char         chunk[65536];
    while (stream.good())
    {
        stream.read(chunk, static_cast<std::streamsize>(sizeof(chunk)));
        const std::size_t read = static_cast<std::size_t>(stream.gcount());
        if (read > 0)
        {
            hasher.update(chunk, read);
        }
        if (stream.fail() && !stream.eof())
        {
            error = "cannot read '" + path.string() + "'";
            return {};
        }
    }
    hasher.finish(digest);

    std::string hex;
    hex.reserve(hexDigestSize);
    for (const std::uint8_t byte : digest)
    {
        hex.push_back(hexDigit(static_cast<std::uint8_t>(byte >> 4)));
        hex.push_back(hexDigit(byte));
    }
    return hex;
}

bool isValidSha256HexDigest(const std::string &text)
{
    if (text.size() != Sha256::hexDigestSize)
    {
        return false;
    }
    for (const char character : text)
    {
        const bool allowed = (character >= '0' && character <= '9') || (character >= 'a' && character <= 'f')
                          || (character >= 'A' && character <= 'F');
        if (!allowed)
        {
            return false;
        }
    }
    return true;
}

bool sha256HexDigestsEqual(const std::string &left, const std::string &right)
{
    if (left.size() != right.size())
    {
        return false;
    }
    for (std::size_t index = 0; index < left.size(); ++index)
    {
        const char leftChar  = static_cast<char>(left[index] >= 'A' && left[index] <= 'F' ? left[index] - 'A' + 'a'
                                                                                          : left[index]);
        const char rightChar = static_cast<char>(right[index] >= 'A' && right[index] <= 'F' ? right[index] - 'A' + 'a'
                                                                                           : right[index]);
        if (leftChar != rightChar)
        {
            return false;
        }
    }
    return true;
}

} // namespace engine
