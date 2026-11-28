// KTX texture loading implementation (see ktx_texture_loader.hpp for the
// contract). Uses libktx for container parsing, supercompression
// (zstd/BasisLZ) inflation and Basis transcoding, and bcdec for
// software-decoding block data (the level-0 mean and the headless
// backend's RGBA expansion).
//
// Everything here runs once per texture at creation, on the main thread,
// matching createTextureFromFile's existing contract.

#include "ktx_texture_loader.hpp"

#define BCDEC_IMPLEMENTATION
#include <bcdec.h>

#include <ktx.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>
#include <new>
#include <utility>

namespace engine
{
namespace renderer
{
namespace
{

// KTX 1 identifier: 0xAB 'K' 'T' 'X' ' ' '1' '1' 0xBB 0x0D 0x0A 0x1A 0x0A.
constexpr std::uint8_t ktx1Identifier[12] = {0xAB, 'K', 'T', 'X', ' ', '1', '1', 0xBB, 0x0D, 0x0A, 0x1A, 0x0A};

// KTX 2 identifier: 0xAB 'K' 'T' 'X' ' ' '2' '0' 0xBB 0x0D 0x0A 0x1A 0x0A.
constexpr std::uint8_t ktx2Identifier[12] = {0xAB, 'K', 'T', 'X', ' ', '2', '0', 0xBB, 0x0D, 0x0A, 0x1A, 0x0A};

// Block count per axis for a level of the given extent (round up: partial
// blocks at the edge are padded inside the file). All supported formats
// cover 4x4 texels per block.
constexpr std::uint32_t blockExtent = 4;

std::uint32_t blockCount(std::uint32_t extent)
{
    return (extent + blockExtent - 1u) / blockExtent;
}

// Byte count of one block of each supported format.
std::size_t blockBytes(BlockTextureFormat format)
{
    switch (format)
    {
    case BlockTextureFormat::Bc1Rgb:
    case BlockTextureFormat::Bc4R:
        return 8;
    case BlockTextureFormat::Bc3Rgba:
    case BlockTextureFormat::Bc5Rg:
    case BlockTextureFormat::Bc7Rgba:
        return 16;
    }
    return 16;
}

// Decodes one block into a 4x4 RGBA tile with bcdec. BC1/BC3/BC7 decode to
// RGBA directly; BC4 produces one channel per texel (replicated to RGB,
// opaque alpha, matching the R8 expansion prepareTextureData documents)
// and BC5 produces two (R,G with blue zero and opaque alpha, matching the
// RG8 expansion). The RG expansions keep the downstream RGB samplers fed
// with the same bytes the uncompressed path would have produced.
void decodeBlock(const std::uint8_t *block, BlockTextureFormat format, std::uint8_t *outTile /* 4*4*4 */)
{
    switch (format)
    {
    case BlockTextureFormat::Bc1Rgb:
        bcdec_bc1(block, outTile, blockExtent * 4);
        break;
    case BlockTextureFormat::Bc3Rgba:
        bcdec_bc3(block, outTile, blockExtent * 4);
        break;
    case BlockTextureFormat::Bc7Rgba:
        bcdec_bc7(block, outTile, blockExtent * 4);
        break;
    case BlockTextureFormat::Bc4R:
    {
        std::uint8_t red[blockExtent * blockExtent];
        bcdec_bc4(block, red, blockExtent);
        for (std::size_t texel = 0; texel < blockExtent * blockExtent; ++texel)
        {
            outTile[texel * 4 + 0] = red[texel];
            outTile[texel * 4 + 1] = red[texel];
            outTile[texel * 4 + 2] = red[texel];
            outTile[texel * 4 + 3] = 255;
        }
        break;
    }
    case BlockTextureFormat::Bc5Rg:
    {
        std::uint8_t redGreen[blockExtent * blockExtent * 2];
        bcdec_bc5(block, redGreen, blockExtent * 2);
        for (std::size_t texel = 0; texel < blockExtent * blockExtent; ++texel)
        {
            outTile[texel * 4 + 0] = redGreen[texel * 2 + 0];
            outTile[texel * 4 + 1] = redGreen[texel * 2 + 1];
            outTile[texel * 4 + 2] = 0;
            outTile[texel * 4 + 3] = 255;
        }
        break;
    }
    }
}

// Decodes one full level of BC blocks into tightly packed RGBA8 (the
// loader's level-0 mean and the headless backend's block-texture creation
// share it).

// Mean linear RGB over the decoded level-0 bytes (identical formula to the
// createTextureImpl mean: bytes * 1/255 per channel, no gamma).
std::array<float, 3> rgbaMean(const std::vector<std::uint8_t> &rgba)
{
    std::array<float, 3> mean{0.0f, 0.0f, 0.0f};
    if (rgba.empty() || rgba.size() % 4 != 0)
    {
        return mean;
    }
    const std::size_t texels = rgba.size() / 4;
    double sum[3]{0.0, 0.0, 0.0};
    for (std::size_t texel = 0; texel < texels; ++texel)
    {
        sum[0] += rgba[texel * 4 + 0];
        sum[1] += rgba[texel * 4 + 1];
        sum[2] += rgba[texel * 4 + 2];
    }
    const double texelInv = 1.0 / (static_cast<double>(texels) * 255.0);
    for (int channel = 0; channel < 3; ++channel)
    {
        mean[static_cast<std::size_t>(channel)] = static_cast<float>(sum[channel] * texelInv);
    }
    return mean;
}

// The VkFormat values libktx leaves on a raw (non-Basis) KTX2 that this
// loader consumes; anything else is rejected with a clear reason.
// Kept as plain integers so this file stays Vulkan-free.
constexpr std::uint32_t vkFormatBc1RgbUnorm     = 131u;
constexpr std::uint32_t vkFormatBc1RgbSrgb     = 132u;
constexpr std::uint32_t vkFormatBc3Unorm       = 133u;
constexpr std::uint32_t vkFormatBc3Srgb        = 134u;
constexpr std::uint32_t vkFormatBc4Unorm       = 135u;
constexpr std::uint32_t vkFormatBc4Snorm        = 136u;
constexpr std::uint32_t vkFormatBc5Unorm       = 137u;
constexpr std::uint32_t vkFormatBc5Snorm        = 138u;
constexpr std::uint32_t vkFormatBc7Unorm       = 145u;
constexpr std::uint32_t vkFormatBc7Srgb        = 146u;
constexpr std::uint32_t vkFormatR8Unorm        = 9u;
constexpr std::uint32_t vkFormatR8G8Unorm      = 16u;
constexpr std::uint32_t vkFormatR8G8B8A8Unorm  = 37u;
constexpr std::uint32_t vkFormatR8G8B8A8Srgb   = 43u;
constexpr std::uint32_t vkFormatR8Srgb         = 11u;
constexpr std::uint32_t vkFormatR8G8Srgb       = 23u;
constexpr std::uint32_t vkFormatBc6HUfloat      = 143u;
constexpr std::uint32_t vkFormatBc6HSfloat      = 144u;
constexpr std::uint32_t vkFormatBc2Unorm       = 109u;
constexpr std::uint32_t vkFormatBc2Srgb        = 110u;

// Maps a raw vkFormat to the loader's block formats; nullptr when the
// format is not one this loader handles.
const char *blockFormatForVkFormat(std::uint32_t vkFormat, BlockTextureFormat &outFormat)
{
    switch (vkFormat)
    {
    case vkFormatBc1RgbUnorm:
    case vkFormatBc1RgbSrgb:
        outFormat = BlockTextureFormat::Bc1Rgb;
        return nullptr;
    case vkFormatBc3Unorm:
    case vkFormatBc3Srgb:
        outFormat = BlockTextureFormat::Bc3Rgba;
        return nullptr;
    case vkFormatBc4Unorm:
    case vkFormatBc4Snorm:
        outFormat = BlockTextureFormat::Bc4R;
        return nullptr;
    case vkFormatBc5Unorm:
    case vkFormatBc5Snorm:
        outFormat = BlockTextureFormat::Bc5Rg;
        return nullptr;
    case vkFormatBc7Unorm:
    case vkFormatBc7Srgb:
        outFormat = BlockTextureFormat::Bc7Rgba;
        return nullptr;
    default:
        break;
    }
    return "the file's vkFormat is not a block format this renderer consumes";
}

// Maps a transcode target back to the loader's block format enum.
BlockTextureFormat blockFormatForTranscodeTarget(ktx_transcode_fmt_e target)
{
    switch (target)
    {
    case KTX_TTF_BC1_RGB:
        return BlockTextureFormat::Bc1Rgb;
    case KTX_TTF_BC3_RGBA:
        return BlockTextureFormat::Bc3Rgba;
    case KTX_TTF_BC4_R:
        return BlockTextureFormat::Bc4R;
    case KTX_TTF_BC5_RG:
        return BlockTextureFormat::Bc5Rg;
    case KTX_TTF_BC7_RGBA:
        return BlockTextureFormat::Bc7Rgba;
    default:
        break;
    }
    return BlockTextureFormat::Bc7Rgba;
}

// Picks the Basis transcode target for the backend's supported set and
// the texture's component count. Channels: 3/4 (colour or colour+alpha)
// map to the best available of BC7, BC3, BC1; 2 (X+Y normal maps) maps to
// BC5; 1 maps to BC4. Falls back to RGBA32 when the needed target is not
// supported, so every conforming file renders on every machine.
ktx_transcode_fmt_e chooseTranscodeTarget(const BlockTextureFormatSupport &support, std::uint32_t components,
    bool &outBlockCompressed)
{
    outBlockCompressed = true;
    if (components <= 1)
    {
        if (support.bc4)
            return KTX_TTF_BC4_R;
    }
    else if (components == 2)
    {
        if (support.bc5)
            return KTX_TTF_BC5_RG;
    }
    else
    {
        if (support.bc7)
            return KTX_TTF_BC7_RGBA;
        if (support.bc3)
            return KTX_TTF_BC3_RGBA;
        if (support.bc1)
            return KTX_TTF_BC1_RGB;
    }
    outBlockCompressed = false;
    return KTX_TTF_RGBA32;
}

// True when the KTX1 file's glInternalFormat is one this loader consumes
// (checked via libktx's ktxTexture1 -> vkFormat translation happening
// inside ktxTexture_CreateFromNamedFile; a KTX1 that cannot be mapped is
// rejected right after load).
bool acceptedTextureShape(const ktxTexture *texture)
{
    return texture->numDimensions == 2 && texture->numFaces == 1 && !texture->isArray && texture->numLayers <= 1
        && texture->baseDepth <= 1;
}

// Copies every level's block payload out of a (transcoded or raw) texture
// into the destination buffer with per-level sizes. Returns false on
// sizes the loader refuses (a level larger than the base level, or a total
// that overflows the size_t address space).
bool gatherBlockLevels(ktxTexture2 &texture, BlockTextureFormat format, std::vector<std::uint8_t> &outBlocks,
    std::vector<std::uint64_t> &outLevelBytes)
{
    outBlocks.clear();
    outLevelBytes.clear();
    outLevelBytes.reserve(texture.numLevels);
    std::size_t totalBytes = 0;
    for (std::uint32_t level = 0; level < texture.numLevels; ++level)
    {
        ktx_size_t offset = 0;
        if (ktxTexture_GetImageOffset(reinterpret_cast<ktxTexture *>(&texture), level, 0, 0, &offset) != KTX_SUCCESS)
        {
            return false;
        }
        const std::uint32_t levelWidth  = std::max(1u, texture.baseWidth >> level);
        const std::uint32_t levelHeight = std::max(1u, texture.baseHeight >> level);
        const std::size_t   levelSize
            = static_cast<std::size_t>(blockCount(levelWidth)) * blockCount(levelHeight) * blockBytes(format);
        if (levelSize == 0 || totalBytes + levelSize < totalBytes)
        {
            return false;
        }
        totalBytes += levelSize;
        outLevelBytes.push_back(levelSize);
    }
    outBlocks.resize(totalBytes);
    std::size_t writeOffset = 0;
    for (std::uint32_t level = 0; level < texture.numLevels; ++level)
    {
        ktx_size_t offset = 0;
        if (ktxTexture_GetImageOffset(reinterpret_cast<ktxTexture *>(&texture), level, 0, 0, &offset) != KTX_SUCCESS)
        {
            return false;
        }
        std::memcpy(outBlocks.data() + writeOffset, ktxTexture_GetData(reinterpret_cast<ktxTexture *>(&texture)) + offset,
            static_cast<std::size_t>(outLevelBytes[level]));
        writeOffset += static_cast<std::size_t>(outLevelBytes[level]);
    }
    return true;
}

// RGBA path for an uncompressed (or RGBA32-transcoded) texture: copies the
// level-0 plane as canonical RGBA8. Returns false when the texture's data
// does not describe a single RGBA8 plane (unexpected after the checks
// above).
bool gatherRgbaLevel0(ktxTexture2 &texture, std::vector<std::uint8_t> &outRgba)
{
    ktx_size_t offset = 0;
    if (ktxTexture_GetImageOffset(reinterpret_cast<ktxTexture *>(&texture), 0, 0, 0, &offset) != KTX_SUCCESS)
    {
        return false;
    }
    const std::size_t pixels = static_cast<std::size_t>(texture.baseWidth) * texture.baseHeight;
    outRgba.resize(pixels * 4);
    std::memcpy(outRgba.data(), ktxTexture_GetData(reinterpret_cast<ktxTexture *>(&texture)) + offset, pixels * 4);
    return true;
}

} // namespace

void decodeBlockLevelToRgba(
    const std::uint8_t *blocks, BlockTextureFormat format, std::uint32_t width, std::uint32_t height,
    std::vector<std::uint8_t> &out)
{
    out.assign(static_cast<std::size_t>(width) * height * 4, 0);
    const std::uint32_t blocksX = blockCount(width);
    const std::uint32_t blocksY = blockCount(height);
    const std::size_t   stride  = static_cast<std::size_t>(width) * 4;
    const std::uint8_t *source  = blocks;
    std::uint8_t        tile[blockExtent * blockExtent * 4];
    for (std::uint32_t by = 0; by < blocksY; ++by)
    {
        for (std::uint32_t bx = 0; bx < blocksX; ++bx)
        {
            std::memset(tile, 0, sizeof(tile));
            decodeBlock(source, format, tile);
            for (std::uint32_t y = 0; y < blockExtent; ++y)
            {
                const std::uint32_t destinationY = by * blockExtent + y;
                if (destinationY >= height)
                {
                    continue;
                }
                for (std::uint32_t x = 0; x < blockExtent; ++x)
                {
                    const std::uint32_t destinationX = bx * blockExtent + x;
                    if (destinationX >= width)
                    {
                        continue;
                    }
                    const std::size_t texel = static_cast<std::size_t>(y) * blockExtent + x;
                    std::memcpy(out.data() + static_cast<std::size_t>(destinationY) * stride
                            + static_cast<std::size_t>(destinationX) * 4,
                        tile + texel * 4,
                        4);
                }
            }
            source += blockBytes(format);
        }
    }
}

bool isKtxFile(const char *filePath)
{
    if (filePath == nullptr)
    {
        return false;
    }
    std::FILE *file = std::fopen(filePath, "rb");
    if (file == nullptr)
    {
        return false;
    }
    std::uint8_t identifier[12] = {};
    const std::size_t read = std::fread(identifier, 1, sizeof(identifier), file);
    std::fclose(file);
    if (read != sizeof(identifier))
    {
        return false;
    }
    // Both identifiers are twelve bytes, including the 0xBB byte that
    // follows the version; a full comparison keeps detection exact.
    return std::memcmp(identifier, ktx1Identifier, 12) == 0 || std::memcmp(identifier, ktx2Identifier, 12) == 0;
}

bool loadKtxTexture(const char *filePath, const BlockTextureFormatSupport &support, TextureFilter filter,
    TextureAddress address, KtxLoadedTexture &out, RenderError *error, std::string &failureReason)
{
    out = KtxLoadedTexture{};
    failureReason.clear();
    if (filePath == nullptr)
    {
        failureReason = "no file path";
        reportError(error, RenderError::InvalidTexture);
        return false;
    }

    ktxTexture *texture = nullptr;
    const ktxResult loadResult
        = ktxTexture_CreateFromNamedFile(filePath, KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT, &texture);
    if (loadResult != KTX_SUCCESS || texture == nullptr)
    {
        failureReason = "libktx could not load the KTX file (" + std::string(ktxErrorString(loadResult)) + ")";
        reportError(error, RenderError::InvalidTexture);
        return false;
    }
    struct TextureGuard
    {
        ktxTexture *texture;
        ~TextureGuard()
        {
            ktxTexture_Destroy(texture);
        }
    } guard{texture};
    ktxTexture2 *texture2 = texture->classId == ktxTexture2_c ? reinterpret_cast<ktxTexture2 *>(texture) : nullptr;
    if (texture2 == nullptr)
    {
        // KTX 1 files are consumed through their RGBA8-equivalent planes
        // only when libktx mapped them to an RGBA8 VkFormat; otherwise the
        // KTX1 glInternalFormat space (GL-only formats, cubemaps, arrays)
        // is outside this loader's contract.
        const std::uint32_t vkFormat = 0;
        const char *const   reason = "KTX 1 files are not consumed by this renderer; re-encode the texture as KTX 2";
        (void)vkFormat;
        failureReason = reason;
        reportError(error, RenderError::InvalidTexture);
        return false;
    }

    if (!acceptedTextureShape(texture))
    {
        failureReason = "only 2D single-face non-array KTX textures are consumed";
        reportError(error, RenderError::InvalidTexture);
        return false;
    }
    if (texture2->baseWidth == 0 || texture2->baseHeight == 0
        || texture2->baseWidth > maximumKtxTextureSize || texture2->baseHeight > maximumKtxTextureSize)
    {
        failureReason = "the texture's dimensions are zero or above the accepted "
                        + std::to_string(maximumKtxTextureSize) + " texel limit";
        reportError(error, RenderError::InvalidTexture);
        return false;
    }

    try
    {
        if (ktxTexture2_NeedsTranscoding(texture2))
        {
            // BasisLZ/ETC1S or UASTC payload: transcode once, here, into
            // the best target the backend samples natively. The component
            // count is pinned to 4: for Basis-compressed files
            // ktxTexture2_GetComponentInfo reports the TEXEL BLOCK layout
            // (one sample of the block's byte length), not the color
            // channels, so its answer here is "1 component, 16 bytes" for
            // every UASTC RGBA file - reading it as a channel count picked
            // the single-channel BC4 target for full-color scans (the RGB
            // collapsed to gray, the alpha - the vegetation atlases' whole
            // cutout mask - flattened to opaque). The transcode targets all
            // carry RGBA; a source without an alpha plane simply transcodes
            // to opaque alpha, so 4 is lossless for every Basis file.
            std::uint32_t components = 4;
            bool                  blockCompressed = false;
            ktx_transcode_fmt_e target = chooseTranscodeTarget(support, components, blockCompressed);
            const ktxResult transcodeResult = ktxTexture2_TranscodeBasis(texture2, target, 0);
            if (transcodeResult != KTX_SUCCESS)
            {
                failureReason
                    = "libktx could not transcode the Basis payload (" + std::string(ktxErrorString(transcodeResult)) + ")";
                reportError(error, RenderError::InvalidTexture);
                return false;
            }
            if (!blockCompressed)
            {
                if (!gatherRgbaLevel0(*texture2, out.rgba))
                {
                    failureReason = "the RGBA32-transcoded payload could not be read";
                    reportError(error, RenderError::InvalidTexture);
                    return false;
                }
                out.storage     = KtxLoadedTexture::Storage::Rgba;
                out.description = TextureDescription{
                    .width   = texture2->baseWidth,
                    .height  = texture2->baseHeight,
                    .pixels  = out.rgba.data(),
                    .filter  = filter,
                    .address = address,
                };
                out.mean = rgbaMean(out.rgba);
                reportError(error, RenderError::None);
                return true;
            }
            out.storage  = KtxLoadedTexture::Storage::Blocks;
            out.format   = blockFormatForTranscodeTarget(target);
            out.width    = texture2->baseWidth;
            out.height   = texture2->baseHeight;
            out.levelCount = texture2->numLevels;
            if (!gatherBlockLevels(*texture2, out.format, out.blocks, out.levelBytes))
            {
                failureReason = "the transcoded block levels could not be gathered";
                reportError(error, RenderError::InvalidTexture);
                return false;
            }
        }
        else
        {
            // Raw payload: consume block formats natively (the Vulkan
            // backend uploads them as-is), and everything else through the
            // RGBA path when the plane is RGBA8.
            const std::uint32_t vkFormat = texture2->vkFormat;
            BlockTextureFormat blockFormat = BlockTextureFormat::Bc7Rgba;
            const char *const  blockReason = blockFormatForVkFormat(vkFormat, blockFormat);
            if (blockReason == nullptr)
            {
                out.storage   = KtxLoadedTexture::Storage::Blocks;
                out.format    = blockFormat;
                out.width     = texture2->baseWidth;
                out.height    = texture2->baseHeight;
                out.levelCount = texture2->numLevels;
                if (!gatherBlockLevels(*texture2, blockFormat, out.blocks, out.levelBytes))
                {
                    failureReason = "the raw block levels could not be gathered";
                    reportError(error, RenderError::InvalidTexture);
                    return false;
                }
            }
            else if (vkFormat == vkFormatR8G8B8A8Unorm || vkFormat == vkFormatR8G8B8A8Srgb)
            {
                if (!gatherRgbaLevel0(*texture2, out.rgba))
                {
                    failureReason = "the uncompressed RGBA plane could not be read";
                    reportError(error, RenderError::InvalidTexture);
                    return false;
                }
                out.storage     = KtxLoadedTexture::Storage::Rgba;
                out.description = TextureDescription{
                    .width   = texture2->baseWidth,
                    .height  = texture2->baseHeight,
                    .pixels  = out.rgba.data(),
                    .filter  = filter,
                    .address = address,
                };
                out.mean = rgbaMean(out.rgba);
                reportError(error, RenderError::None);
                return true;
            }
            else
            {
                failureReason = blockReason;
                reportError(error, RenderError::InvalidTexture);
                return false;
            }
        }
    }
    catch (const std::bad_alloc &)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }

    // Block path: level-0 mean through a software decode of the base level.
    if (out.storage == KtxLoadedTexture::Storage::Blocks)
    {
        std::vector<std::uint8_t> level0;
        decodeBlockLevelToRgba(out.blocks.data(), out.format, out.width, out.height, level0);
        out.mean = rgbaMean(level0);
    }
    reportError(error, RenderError::None);
    return true;
}

} // namespace renderer
} // namespace engine
