// Shared PNG writers for packed RGB and RGBA image buffers.
//
// The helpers preserve the caller's row stride and can adapt RGBA row order
// and red/blue channel order without depending on a renderer or window API.
// The encoder is self-contained (no external compression dependency): the
// IDAT payload is a zlib stream wrapping a single fixed-Huffman deflate
// block whose LZ77 matches come from hash-chain search over Paeth-filtered
// scanlines, so screenshots stay compact without paying for a compression
// library the engine does not otherwise link.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

namespace engine::util
{

namespace png_detail
{

// CRC-32 (IEEE 802.3 polynomial) over the PNG chunk type and data.
inline std::uint32_t crc32Update(std::uint32_t crc, const std::uint8_t *data, std::size_t size)
{
    static std::uint32_t table[256];
    static bool tableReady = false;
    if (!tableReady)
    {
        for (std::uint32_t index = 0; index < 256; ++index)
        {
            std::uint32_t value = index;
            for (int bit = 0; bit < 8; ++bit)
            {
                value = (value & 1u) != 0u ? 0xEDB88320u ^ (value >> 1) : value >> 1;
            }
            table[index] = value;
        }
        tableReady = true;
    }
    crc ^= 0xFFFFFFFFu;
    for (std::size_t index = 0; index < size; ++index)
    {
        crc = table[(crc ^ data[index]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

inline void appendBigEndian32(std::vector<std::uint8_t> &bytes, std::uint32_t value)
{
    bytes.push_back(static_cast<std::uint8_t>(value >> 24));
    bytes.push_back(static_cast<std::uint8_t>(value >> 16));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8));
    bytes.push_back(static_cast<std::uint8_t>(value));
}

// Appends one PNG chunk (length, type, data, CRC). type is four ASCII bytes.
inline void appendChunk(std::vector<std::uint8_t> &png, const char type[4], const std::vector<std::uint8_t> &data)
{
    appendBigEndian32(png, static_cast<std::uint32_t>(data.size()));
    std::vector<std::uint8_t> crcInput;
    crcInput.reserve(4u + data.size());
    for (int index = 0; index < 4; ++index)
    {
        crcInput.push_back(static_cast<std::uint8_t>(type[index]));
    }
    crcInput.insert(crcInput.end(), data.begin(), data.end());
    png.insert(png.end(), crcInput.begin(), crcInput.end());
    appendBigEndian32(png, crc32Update(0u, crcInput.data(), crcInput.size()));
}

// LSB-first bit writer with MSB-first support for Huffman codes (RFC 1951
// packs Huffman codes most-significant-bit first into the bitstream).
struct BitWriter
{
    std::vector<std::uint8_t> &out;
    std::uint32_t bitBuffer = 0;
    std::uint32_t bitCount  = 0;

    explicit BitWriter(std::vector<std::uint8_t> &output)
        : out(output)
    {
    }

    void write(std::uint32_t value, std::uint32_t bits)
    {
        bitBuffer |= value << bitCount;
        bitCount += bits;
        while (bitCount >= 8u)
        {
            out.push_back(static_cast<std::uint8_t>(bitBuffer & 0xFFu));
            bitBuffer >>= 8;
            bitCount -= 8u;
        }
    }

    void writeCode(std::uint32_t code, std::uint32_t bits)
    {
        std::uint32_t reversed = 0;
        for (std::uint32_t bit = 0; bit < bits; ++bit)
        {
            reversed |= ((code >> bit) & 1u) << (bits - 1u - bit);
        }
        write(reversed, bits);
    }

    void flush()
    {
        if (bitCount > 0u)
        {
            out.push_back(static_cast<std::uint8_t>(bitBuffer & 0xFFu));
            bitBuffer = 0;
            bitCount  = 0;
        }
    }
};

// Fixed-Huffman (BTYPE 01) literal/length code per RFC 1951 section 3.2.5.
inline void fixedLiteralCode(std::uint32_t symbol, std::uint32_t &code, std::uint32_t &bits)
{
    if (symbol <= 143u)
    {
        code = 0x30u + symbol;
        bits = 8u;
    }
    else if (symbol <= 255u)
    {
        code = 0x190u + (symbol - 144u);
        bits = 9u;
    }
    else if (symbol <= 279u)
    {
        code = symbol - 256u;
        bits = 7u;
    }
    else
    {
        code = 0xC0u + (symbol - 280u);
        bits = 8u;
    }
}

// Length (257..285) and distance (0..29) code tables per RFC 1951 section
// 3.2.5: base value and extra-bit count per symbol.
struct MatchCode
{
    std::uint32_t base  = 0;
    std::uint32_t extra = 0;
};

inline const MatchCode kLengthCodes[29] = {
    {3, 0},   {4, 0},   {5, 0},    {6, 0},    {7, 0},    {8, 0},   {9, 0},   {10, 0},  {11, 1},
    {13, 1},  {15, 1},  {17, 1},   {19, 2},   {23, 2},   {27, 2},  {31, 2},  {35, 3},  {43, 3},
    {51, 3},  {59, 3},  {67, 4},   {83, 4},   {99, 4},   {115, 4}, {131, 5}, {163, 5}, {195, 5},
    {227, 5}, {258, 0},
};

inline const MatchCode kDistanceCodes[30] = {
    {1, 0},     {2, 0},     {3, 0},     {4, 0},     {5, 1},     {7, 1},     {9, 2},     {13, 2},
    {17, 3},    {25, 3},    {33, 4},    {49, 4},    {65, 5},    {97, 5},    {129, 6},   {193, 6},
    {257, 7},   {385, 7},   {513, 8},   {769, 8},   {1025, 9},  {1537, 9},  {2049, 10}, {3073, 10},
    {4097, 11}, {6145, 11}, {8193, 12}, {12289, 12}, {16385, 13}, {24577, 13},
};

inline void emitLiteral(BitWriter &writer, std::uint32_t symbol)
{
    std::uint32_t code = 0;
    std::uint32_t bits = 0;
    fixedLiteralCode(symbol, code, bits);
    writer.writeCode(code, bits);
}

inline void emitEndOfBlock(BitWriter &writer)
{
    writer.writeCode(0u, 7u); // symbol 256 under the fixed code
}

inline void emitMatch(BitWriter &writer, std::uint32_t length, std::uint32_t distance)
{
    // Walk the length-code table from the shortest entry; the walk stops at
    // the LAST entry (base 258, no extra bits), which is the correct encoding
    // for length 258 itself rather than the 227+31 encoding of code 284.
    std::uint32_t lengthSymbol = 0u;
    while (lengthSymbol < 28u && kLengthCodes[lengthSymbol + 1u].base <= length)
    {
        ++lengthSymbol;
    }
    const MatchCode &lengthEntry = kLengthCodes[lengthSymbol];
    std::uint32_t code = 0;
    std::uint32_t bits = 0;
    fixedLiteralCode(257u + lengthSymbol, code, bits);
    writer.writeCode(code, bits);
    if (lengthEntry.extra != 0u)
    {
        writer.write(length - lengthEntry.base, lengthEntry.extra);
    }
    std::uint32_t distanceSymbol = 29u;
    while (kDistanceCodes[distanceSymbol].base > distance)
    {
        --distanceSymbol;
    }
    const MatchCode &distanceEntry = kDistanceCodes[distanceSymbol];
    writer.writeCode(distanceSymbol, 5u);
    if (distanceEntry.extra != 0u)
    {
        writer.write(distance - distanceEntry.base, distanceEntry.extra);
    }
}

// Greedy LZ77 over the filtered scanlines: 3-byte hash head table plus a
// prev-link chain bounded by the 32 KiB deflate window. Chain entries carry
// absolute positions; the newest-first ordering means the first entry older
// than the window ends the walk. Chain depth and the early-out "nice
// length" keep export latency bounded without hurting the match quality the
// Paeth-filtered rows offer.
struct MatchFinder
{
    static constexpr std::uint32_t kWindowSize  = 32768u;
    static constexpr std::uint32_t kMinMatch    = 3u;
    static constexpr std::uint32_t kMaxMatch    = 258u;
    static constexpr std::uint32_t kHashBits    = 15u;
    static constexpr std::uint32_t kHashSize    = 1u << kHashBits;
    static constexpr std::uint32_t kChainDepth  = 64u;
    static constexpr std::uint32_t kNiceLength  = 128u;
    static constexpr std::uint32_t kEmpty       = ~std::uint32_t(0);

    const std::uint8_t *data = nullptr;
    std::size_t         size = 0;
    std::vector<std::uint32_t> head; // hash slot -> newest absolute position
    std::vector<std::uint32_t> prev; // window slot -> previous absolute position

    explicit MatchFinder(const std::uint8_t *bytes, std::size_t byteCount)
        : data(bytes)
        , size(byteCount)
        , head(kHashSize, kEmpty)
    {
        prev.resize(byteCount < kWindowSize ? byteCount : kWindowSize, kEmpty);
    }

    static std::uint32_t hash(const std::uint8_t *bytes)
    {
        const std::uint32_t value = (static_cast<std::uint32_t>(bytes[0]) << 16)
            | (static_cast<std::uint32_t>(bytes[1]) << 8) | static_cast<std::uint32_t>(bytes[2]);
        return (value * 2654435761u) >> (32u - kHashBits);
    }

    void insert(std::size_t position)
    {
        if (position + kMinMatch > size)
        {
            return;
        }
        const std::uint32_t slot      = hash(data + position);
        const std::uint32_t slotIndex = static_cast<std::uint32_t>(position % kWindowSize);
        prev[slotIndex] = head[slot];
        head[slot]      = static_cast<std::uint32_t>(position);
    }

    // Longest match at `position` against inserted positions before it;
    // returns 0 when no match of at least kMinMatch exists.
    std::uint32_t longestMatch(std::size_t position, std::size_t &outMatchPosition) const
    {
        if (position + kMinMatch > size)
        {
            return 0u;
        }
        std::uint32_t candidate = head[hash(data + position)];
        const std::size_t maximum = size - position < kMaxMatch ? size - position : kMaxMatch;
        std::uint32_t bestLength   = 0;
        std::size_t   bestPosition = 0;
        for (std::uint32_t chain = 0; chain < kChainDepth && candidate != kEmpty; ++chain)
        {
            const std::size_t matchPosition = candidate;
            if (matchPosition >= position || position - matchPosition > kWindowSize)
            {
                break; // newest-first chain: everything after is older still
            }
            std::uint32_t length = 0;
            while (length < maximum && data[matchPosition + length] == data[position + length])
            {
                ++length;
            }
            if (length > bestLength)
            {
                bestLength   = length;
                bestPosition = matchPosition;
                if (length >= kNiceLength)
                {
                    break;
                }
            }
            candidate = prev[matchPosition % kWindowSize];
        }
        if (bestLength >= kMinMatch)
        {
            outMatchPosition = bestPosition;
            return bestLength;
        }
        return 0u;
    }
};

// Deflates `input` as a single fixed-Huffman block (RFC 1951) and wraps it
// in a zlib stream (RFC 1950): header, deflate payload, Adler-32.
inline void zlibDeflate(const std::uint8_t *input, std::size_t size, std::vector<std::uint8_t> &out)
{
    out.push_back(0x78u); // CMF: deflate, 32 KiB window
    out.push_back(0x01u); // FLG: fastest level, no preset dictionary
    BitWriter writer(out);
    writer.write(1u, 1u); // BFINAL
    writer.write(1u, 2u); // BTYPE 01: fixed Huffman
    MatchFinder finder(input, size);
    std::size_t position = 0;
    while (position < size)
    {
        std::size_t matchPosition = 0;
        const std::uint32_t length = finder.longestMatch(position, matchPosition);
        if (length != 0u)
        {
            emitMatch(writer, length, static_cast<std::uint32_t>(position - matchPosition));
            for (std::uint32_t inserted = 0; inserted < length; ++inserted)
            {
                finder.insert(position + inserted);
            }
            position += length;
        }
        else
        {
            emitLiteral(writer, input[position]);
            finder.insert(position);
            ++position;
        }
    }
    emitEndOfBlock(writer);
    writer.flush();
    // Adler-32 of the UNGZIPPED payload (the raw input, per RFC 1950).
    std::uint32_t s1 = 1u;
    std::uint32_t s2 = 0u;
    for (std::size_t index = 0; index < size; ++index)
    {
        s1 = (s1 + input[index]) % 65521u;
        s2 = (s2 + s1) % 65521u;
    }
    appendBigEndian32(out, (s2 << 16) | s1);
}

// Paeth predictor (PNG specification section 9.4).
inline std::uint8_t paethPredictor(int a, int b, int c)
{
    const int p  = a + b - c;
    const int pa = p > a ? p - a : a - p;
    const int pb = p > b ? p - b : b - p;
    const int pc = p > c ? p - c : c - p;
    if (pa <= pb && pa <= pc)
    {
        return static_cast<std::uint8_t>(a);
    }
    if (pb <= pc)
    {
        return static_cast<std::uint8_t>(b);
    }
    return static_cast<std::uint8_t>(c);
}

// Builds the Paeth-filtered scanline stream (filter byte 4 per row) from an
// interleaved RGB image. rowAt yields the row's first RGB byte triple. PNG
// predictors always read the RAW scanline bytes (the row being filtered and
// the raw row above it), never the filtered output.
template <typename RowAt>
inline void buildFilteredStream(std::uint32_t width, std::uint32_t height, std::uint32_t bytesPerPixel, RowAt &&rowAt,
    std::vector<std::uint8_t> &filtered)
{
    const std::size_t rowBytes = static_cast<std::size_t>(width) * bytesPerPixel;
    filtered.reserve((rowBytes + 1u) * height);
    std::vector<std::uint8_t> previous(rowBytes, 0u); // raw bytes of the row above
    std::vector<std::uint8_t> raw(rowBytes, 0u);      // raw bytes of this row
    std::vector<std::uint8_t> current(rowBytes, 0u);  // filtered output of this row
    for (std::uint32_t row = 0; row < height; ++row)
    {
        const std::uint8_t *source = rowAt(row);
        std::memcpy(raw.data(), source, rowBytes);
        for (std::size_t byte = 0; byte < rowBytes; ++byte)
        {
            const int left   = byte >= bytesPerPixel ? raw[byte - bytesPerPixel] : 0;
            const int up     = row != 0u ? previous[byte] : 0;
            const int upLeft = (row != 0u && byte >= bytesPerPixel) ? previous[byte - bytesPerPixel] : 0;
            current[byte] = static_cast<std::uint8_t>(raw[byte] - paethPredictor(left, up, upLeft));
        }
        filtered.push_back(4u); // Paeth filter byte
        filtered.insert(filtered.end(), current.begin(), current.end());
        previous.swap(raw);
    }
}

} // namespace png_detail

// Writes a truecolor (8-bit RGB, color type 2) PNG. rgb holds packed RGB
// rows with the caller's stride; returns false on invalid arguments or I/O
// failure (matching the PPM helpers' contract).
inline bool writePngRgb(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgb,
    std::size_t sourceStrideBytes)
{
    if (filePath == nullptr || rgb == nullptr || width == 0 || height == 0
        || static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / 3)
    {
        return false;
    }
    const std::size_t rowBytes = static_cast<std::size_t>(width) * 3;
    if (sourceStrideBytes < rowBytes
        || (height > 1
            && sourceStrideBytes > (std::numeric_limits<std::size_t>::max() - rowBytes) / (height - 1)))
    {
        return false;
    }

    using namespace png_detail;
    std::vector<std::uint8_t> png;
    static const std::uint8_t signature[8] = {137u, 80u, 78u, 71u, 13u, 10u, 26u, 10u};
    png.insert(png.end(), signature, signature + 8u);

    std::vector<std::uint8_t> header;
    appendBigEndian32(header, width);
    appendBigEndian32(header, height);
    header.push_back(8u);  // bit depth
    header.push_back(2u);  // color type: truecolor RGB
    header.push_back(0u);  // deflate
    header.push_back(0u);  // adaptive filtering
    header.push_back(0u);  // no interlace
    appendChunk(png, "IHDR", header);

    const auto rowAt = [rgb, sourceStrideBytes](std::uint32_t row) -> const std::uint8_t *
    { return rgb + static_cast<std::size_t>(row) * sourceStrideBytes; };
    std::vector<std::uint8_t> filtered;
    buildFilteredStream(width, height, 3u, rowAt, filtered);
    std::vector<std::uint8_t> idat;
    zlibDeflate(filtered.data(), filtered.size(), idat);
    appendChunk(png, "IDAT", idat);

    const std::vector<std::uint8_t> empty;
    appendChunk(png, "IEND", empty);

    std::FILE *file = std::fopen(filePath, "wb");
    if (file == nullptr)
    {
        return false;
    }
    const bool success = std::fwrite(png.data(), 1, png.size(), file) == png.size();
    if (std::fclose(file) != 0)
    {
        return false;
    }
    return success;
}

// Writes a truecolor PNG from an RGBA8 raster (color type 2: the alpha
// channel is dropped, matching the PPM helper). flipVertically adapts the
// source row order, swapRedBlue the channel order.
inline bool writePngRgba(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgba,
    std::size_t sourceStrideBytes, bool flipVertically, bool swapRedBlue)
{
    if (filePath == nullptr || rgba == nullptr || width == 0 || height == 0
        || static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / 4)
    {
        return false;
    }
    const std::size_t sourceRowBytes = static_cast<std::size_t>(width) * 4;
    if (sourceStrideBytes < sourceRowBytes
        || (height > 1
            && sourceStrideBytes > (std::numeric_limits<std::size_t>::max() - sourceRowBytes) / (height - 1))
        || static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / 3)
    {
        return false;
    }

    const std::size_t packedRowBytes = static_cast<std::size_t>(width) * 3;
    std::vector<std::uint8_t> packed(packedRowBytes * height);
    for (std::uint32_t row = 0; row < height; ++row)
    {
        const std::uint32_t sourceRow = flipVertically ? height - 1u - row : row;
        const std::uint8_t *source    = rgba + static_cast<std::size_t>(sourceRow) * sourceStrideBytes;
        std::uint8_t       *target    = packed.data() + static_cast<std::size_t>(row) * packedRowBytes;
        for (std::uint32_t column = 0; column < width; ++column)
        {
            const std::size_t sourceOffset = static_cast<std::size_t>(column) * 4;
            const std::size_t targetOffset = static_cast<std::size_t>(column) * 3;
            target[targetOffset + 0] = source[sourceOffset + (swapRedBlue ? 2 : 0)];
            target[targetOffset + 1] = source[sourceOffset + 1];
            target[targetOffset + 2] = source[sourceOffset + (swapRedBlue ? 0 : 2)];
        }
    }
    return writePngRgb(filePath, width, height, packed.data(), packedRowBytes);
}

} // namespace engine::util
