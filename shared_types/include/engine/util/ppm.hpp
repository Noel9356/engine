// Shared binary PPM writers for packed RGB and RGBA image buffers.
//
// The helpers preserve the caller's row stride and can adapt RGBA row order
// and red/blue channel order without depending on a renderer or window API.

#ifndef ENGINE_UTIL_PPM_HPP
#define ENGINE_UTIL_PPM_HPP

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

namespace engine::util
{

inline bool writePpmRgb(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgb,
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

    std::FILE *file = std::fopen(filePath, "wb");
    if (file == nullptr)
    {
        return false;
    }
    bool success = std::fprintf(file, "P6\n%u %u\n255\n", width, height) >= 0;
    for (std::uint32_t row = 0; success && row < height; ++row)
    {
        const std::uint8_t *source = rgb + static_cast<std::size_t>(row) * sourceStrideBytes;
        success = std::fwrite(source, 1, rowBytes, file) == rowBytes;
    }
    if (std::fclose(file) != 0)
    {
        success = false;
    }
    return success;
}

inline bool writePpmRgba(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgba,
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

    const std::size_t outputRowBytes = static_cast<std::size_t>(width) * 3;
    std::vector<std::uint8_t> outputRow(outputRowBytes);
    std::FILE *file = std::fopen(filePath, "wb");
    if (file == nullptr)
    {
        return false;
    }
    bool success = std::fprintf(file, "P6\n%u %u\n255\n", width, height) >= 0;
    for (std::uint32_t row = 0; success && row < height; ++row)
    {
        const std::uint32_t sourceRow = flipVertically ? height - 1 - row : row;
        const std::uint8_t *source = rgba + static_cast<std::size_t>(sourceRow) * sourceStrideBytes;
        for (std::uint32_t column = 0; column < width; ++column)
        {
            const std::size_t sourceOffset = static_cast<std::size_t>(column) * 4;
            const std::size_t outputOffset = static_cast<std::size_t>(column) * 3;
            outputRow[outputOffset + 0] = source[sourceOffset + (swapRedBlue ? 2 : 0)];
            outputRow[outputOffset + 1] = source[sourceOffset + 1];
            outputRow[outputOffset + 2] = source[sourceOffset + (swapRedBlue ? 0 : 2)];
        }
        success = std::fwrite(outputRow.data(), 1, outputRow.size(), file) == outputRow.size();
    }
    if (std::fclose(file) != 0)
    {
        success = false;
    }
    return success;
}

} // namespace engine::util

#endif
