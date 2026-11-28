// KTX texture loading for createTextureFromFile: detects KTX 1/2 containers
// by their identifier, loads them through libktx, and produces either a
// canonical RGBA8 TextureDescription (the same shape the stb_image path
// feeds) or a block-compressed description that keeps the GPU-native
// payload (BC1/BC3/BC4/BC5/BC7 blocks plus the file's own mip chain) so the
// Vulkan backend uploads the blocks directly and the headless backend
// software-decodes them once at creation.
//
// Basis-encoded KTX2 (BasisLZ/ETC1S and UASTC, optionally zstd- or
// BasisLZ-supercompressed) is transcoded at load time to the best block
// format the consuming backend reports; raw block-compressed and
// uncompressed KTX files are consumed as-is. The mean color for
// textureMean is decoded from the level-0 blocks (or read straight from
// the level-0 pixels on the RGBA path) so it stays byte-faithful to what
// the sampler will read, exactly like the JPG path's mean.
//
// Design note on color: the engine's texture pipeline is byte-linear -
// createTexture feeds RGBA8 into an UNORM image and the shaders treat the
// sampled values as linear - so block payloads are uploaded through UNORM
// format variants to reproduce the same samples. The KTX2 DFD's sRGB
// transfer flag is honoured only for the record's metadata; no _SRGB
// VkFormat is requested, matching the RGBA8 path's behaviour byte for byte.

#pragma once

#include "render_shared.hpp"

#include <engine/renderer/capabilities.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace engine
{
namespace renderer
{

// BlockTextureFormat and BlockTextureFormatSupport are declared in
// render_shared.hpp (both backends and the module core need them without
// pulling this loader header in).

// A loaded KTX file ready for backend consumption. Exactly one storage kind
// is populated:
//  - RGBA storage: a canonical TextureDescription whose pixels point at
//    the decoded RGBA8 copy owned by this object (row stride width*4,
//    top-left origin). The headless backend consumes this directly and the
//    Vulkan backend uploads it as an uncompressed RGBA8 image - identical
//    to the stb_image path's data.
//  - Block storage: the raw block bytes of every mip level concatenated
//    with per-level offsets, the level dimensions, and the block format.
//    The backends upload (Vulkan) or decode (headless) these themselves.
struct KtxLoadedTexture
{
    enum class Storage
    {
        Rgba,
        Blocks
    };

    Storage storage = Storage::Rgba;

    // RGBA storage (Storage::Rgba): pixels stay valid while this object
    // lives; description points into them.
    std::vector<std::uint8_t> rgba;
    TextureDescription        description{};

    // Block storage (Storage::Blocks): all levels concatenated; levelBytes
    // gives each level's byte count in level order.
    std::vector<std::uint8_t> blocks;
    std::vector<std::uint64_t> levelBytes;
    BlockTextureFormat        format = BlockTextureFormat::Bc7Rgba;
    std::uint32_t             width  = 0;
    std::uint32_t             height = 0;
    std::uint32_t             levelCount = 0;

    // Mean linear RGB of the level-0 payload as the sampler will read it
    // (bytes scaled by 1/255, no gamma), matching the JPG path's contract.
    std::array<float, 3> mean{0.0f, 0.0f, 0.0f};
};

// True when the file starts with a KTX 1 or KTX 2 container identifier.
// Only the first bytes are examined, so a false positive is impossible
// for the image formats stb_image decodes (none of them begins with these
// bytes; JPG starts 0xFFD8, PNG 0x89 'P' 'N' 'G').
bool isKtxFile(const char *filePath);

// Loads and prepares a KTX texture. support tells the loader which block
// formats the backend can consume; when none are supported the loader
// always yields RGBA storage (Basis files transcode to RGBA32, raw block
// files are software-decoded, uncompressed files pass through). filter and
// address flow into the produced descriptions exactly like the stb_image
// path's equivalents. Returns false with error/failureReason set on any
// rejection (unsupported KTX variant, oversized dimensions, decode
// failure); dimensions are capped at maximumKtxTextureSize so a hostile
// file cannot allocate unbounded pixel storage.
bool loadKtxTexture(const char *filePath, const BlockTextureFormatSupport &support, TextureFilter filter,
    TextureAddress address, KtxLoadedTexture &out, RenderError *error, std::string &failureReason);

// Largest KTX width/height accepted by loadKtxTexture (matches the Vulkan
// spec's minimum guaranteed 2D image limit; every conforming driver can
// allocate at least this, and larger textures would also be a
// decompression-bomb vector through the RGBA fallback).
inline constexpr std::uint32_t maximumKtxTextureSize = 16384u;

// Software-decodes one full level of BC blocks (any supported format) into
// tightly packed RGBA8, filling partial edge blocks' padding with zeros.
// Shared by the loader (level-0 mean) and the headless backend (block
// texture creation).
void decodeBlockLevelToRgba(
    const std::uint8_t *blocks, BlockTextureFormat format, std::uint32_t width, std::uint32_t height,
    std::vector<std::uint8_t> &outRgba);

} // namespace renderer
} // namespace engine
