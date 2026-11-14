// previews.cpp - runtime part thumbnails and the capture-side preview sheets.
//
// generatePreviewTextures renders every registered part once at a small
// viewport during startup: neutral background, the part framed alone, the
// frame read back (headless surface lock, or the Vulkan frame export through
// a temporary PPM), and the pixels uploaded as a small texture. No
// pre-generated thumbnail files ship with the demo - what the panel shows is
// always the real geometry through the real renderer.
//
// renderPartPreviews writes the same standalone frames as PPM files for the
// capture-side sample sheets (--part-previews).

#include "../character_screen_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace demo
{
using namespace screen_detail;

namespace
{

// The thumbnail render size; the panel draws them at half that, so the
// linear filter downsamples a supersampled image.
constexpr std::uint32_t kThumbnailPixels = 128;

// The neutral picker background (a calm dark slate).
constexpr float kPreviewClearColor[4] = {0.145f, 0.155f, 0.175f, 1.0f};

// One P6 (binary) PPM frame as RGBA.
struct ParsedPpm
{
    std::vector<std::uint8_t> rgba;
    std::uint32_t             width  = 0;
    std::uint32_t             height = 0;
};

bool parsePpm(const std::string &path, ParsedPpm &out)
{
    std::FILE *file = std::fopen(path.c_str(), "rb");
    if (file == nullptr)
    {
        return false;
    }
    char       magic[3] = {};
    unsigned   width    = 0;
    unsigned   height   = 0;
    unsigned   maximum  = 0;
    const int  fields   = std::fscanf(file, "%2s %u %u %u", magic, &width, &height, &maximum);
    const bool headerOk = fields == 4 && std::strcmp(magic, "P6") == 0 && maximum == 255 && width > 0 && height > 0
                       && width < 16384 && height < 16384;
    if (!headerOk)
    {
        std::fclose(file);
        return false;
    }
    // Exactly one whitespace byte separates the header from the binary data.
    int separator = std::fgetc(file);
    if (separator != ' ' && separator != '\n' && separator != '\r' && separator != '\t')
    {
        std::fclose(file);
        return false;
    }
    out.rgba.resize(static_cast<std::size_t>(width) * height * 4);
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(width) * height * 3);
    const std::size_t         read = std::fread(rgb.data(), 1, rgb.size(), file);
    std::fclose(file);
    if (read != rgb.size())
    {
        return false;
    }
    for (std::size_t index = 0; index < static_cast<std::size_t>(width) * height; ++index)
    {
        out.rgba[index * 4 + 0] = rgb[index * 3 + 0];
        out.rgba[index * 4 + 1] = rgb[index * 3 + 1];
        out.rgba[index * 4 + 2] = rgb[index * 3 + 2];
        out.rgba[index * 4 + 3] = 255u;
    }
    out.width  = width;
    out.height = height;
    return true;
}

// The thumbnail occupies one corner of the exported frame (the small
// viewport lands in a backend-dependent corner); pick the corner that holds
// the rendered content by comparing each candidate against the clear colour.
std::uint32_t cornerContentScore(const ParsedPpm &frame, std::uint32_t x0, std::uint32_t y0)
{
    std::uint32_t       score  = 0;
    const std::uint32_t limitX = std::min(kThumbnailPixels, frame.width);
    const std::uint32_t limitY = std::min(kThumbnailPixels, frame.height);
    for (std::uint32_t y = 0; y < limitY; y += 4)
    {
        for (std::uint32_t x = 0; x < limitX; x += 4)
        {
            const std::size_t index = (static_cast<std::size_t>(y0 + y) * frame.width + x0 + x) * 4;
            const int         dr    = std::abs(static_cast<int>(frame.rgba[index]) - 37);
            const int         dg    = std::abs(static_cast<int>(frame.rgba[index + 1]) - 40);
            const int         db    = std::abs(static_cast<int>(frame.rgba[index + 2]) - 45);
            if (dr + dg + db > 24)
            {
                ++score;
            }
        }
    }
    return score;
}

} // namespace

bool CharacterScreen::generatePreviewTextures(const std::function<bool(const std::string &)> &exportPpm)
{
    if (!created_ || render_ == nullptr || window_ == nullptr)
    {
        return false;
    }
    if (content_.hasPreviewTextures())
    {
        return true;
    }

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    window_->getFramebufferSize(window_->self, windowHandle_, &framebufferWidth, &framebufferHeight);

    const std::vector<std::string> slots
        = {"head", "neck", "ears", "tail", "wings", "torso", "pelvis", "arms", "legs", "garment", "quadruped"};
    std::uint32_t               generated    = 0;
    const std::filesystem::path temporaryPpm = std::filesystem::temp_directory_path() / "engine_part_thumbnail.ppm";

    // The export fallback (Vulkan surface) rounds through a temporary PPM.
    const auto readFrame = [&](std::vector<std::uint8_t> &outRgba, std::uint32_t &outWidth, std::uint32_t &outHeight)
    {
        // Headless surfaces map directly.
        engine::window::SurfacePixels pixels;
        if (window_->lockSurfacePixels != nullptr && window_->lockSurfacePixels(window_->self, windowHandle_, &pixels))
        {
            outWidth  = std::min(kThumbnailPixels, pixels.width);
            outHeight = std::min(kThumbnailPixels, pixels.height);
            outRgba.resize(static_cast<std::size_t>(outWidth) * outHeight * 4);
            for (std::uint32_t y = 0; y < outHeight; ++y)
            {
                const std::uint8_t *row = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
                std::memcpy(outRgba.data() + static_cast<std::size_t>(y) * outWidth * 4,
                    row,
                    static_cast<std::size_t>(outWidth) * 4);
            }
            window_->unlockSurfacePixels(window_->self, windowHandle_);
            return true;
        }
        if (!exportPpm || !exportPpm(temporaryPpm.string()))
        {
            return false;
        }
        ParsedPpm frame;
        if (!parsePpm(temporaryPpm.string(), frame))
        {
            return false;
        }
        // Locate the corner that carries the rendered viewport.
        std::uint32_t       bestX     = 0;
        std::uint32_t       bestY     = 0;
        std::uint32_t       bestScore = 0;
        const std::uint32_t cornerY   = frame.height > kThumbnailPixels ? frame.height - kThumbnailPixels : 0u;
        const std::uint32_t cornerX   = frame.width > kThumbnailPixels ? frame.width - kThumbnailPixels : 0u;
        for (const std::uint32_t y0 : {0u, cornerY})
        {
            for (const std::uint32_t x0 : {0u, cornerX})
            {
                const std::uint32_t score = cornerContentScore(frame, x0, y0);
                if (score > bestScore)
                {
                    bestScore = score;
                    bestX     = x0;
                    bestY     = y0;
                }
            }
        }
        if (bestScore == 0)
        {
            bestX = 0;
            bestY = 0;
        }
        outWidth  = std::min(kThumbnailPixels, frame.width);
        outHeight = std::min(kThumbnailPixels, frame.height);
        outRgba.resize(static_cast<std::size_t>(outWidth) * outHeight * 4);
        for (std::uint32_t y = 0; y < outHeight; ++y)
        {
            const std::size_t source = (static_cast<std::size_t>(bestY + y) * frame.width + bestX) * 4;
            std::memcpy(outRgba.data() + static_cast<std::size_t>(y) * outWidth * 4,
                frame.rgba.data() + source,
                static_cast<std::size_t>(outWidth) * 4);
        }
        std::filesystem::remove(temporaryPpm);
        return true;
    };

    render_->setViewport(render_->self, rendererHandle_, kThumbnailPixels, kThumbnailPixels);

    for (const std::string &slot : slots)
    {
        for (const std::string &partId : content_.partsInSlot(slot))
        {
            const PartRenderInfo *info = content_.renderInfoForPart(partId);
            if (info == nullptr)
            {
                continue;
            }
            const MeshHandle mesh = content_.meshForGeometry(info->geometryId);
            if (mesh == engine::renderer::invalidMesh)
            {
                continue;
            }
            float boundsCenter[3] = {0.0f, 0.0f, 0.0f};
            float boundsRadius    = 0.3f;
            if (!content_.geometryBounds(info->geometryId, boundsCenter, &boundsRadius))
            {
                continue;
            }
            // Neutral three-quarter framing; the bounding sphere fills the
            // tile. The orbit target is the part's OWN centre - parts sit
            // off the origin (a head's geometry rides above its neck-ring
            // origin, a tail behind its root ring), so targeting (0, y, 0)
            // pushed every part out of its tile and clipped the cells.
            const float targetY  = boundsCenter[1];
            const float distance = std::clamp(boundsRadius * 1.85f, 0.35f, 3.6f);
            configureCamera(28.0f, 16.0f, distance, targetY);
            const float orbitTarget[3] = {boundsCenter[0], boundsCenter[1], boundsCenter[2]};
            camera_->setOrbitTarget(camera_->self, cameraHandle_, orbitTarget);
            // The thumbnail viewport is square; the view state must match it
            // (the framebuffer is letterboxed 16:9, which squeezed every
            // thumbnail horizontally when it drove the projection).
            engine::camera::ViewState previewView;
            camera_->viewState(camera_->self, cameraHandle_, kThumbnailPixels, kThumbnailPixels, &previewView);
            engine::renderer::CameraMatrices previewMatrices;
            std::memcpy(previewMatrices.view, previewView.view, sizeof(previewMatrices.view));
            std::memcpy(previewMatrices.projection, previewView.projection, sizeof(previewMatrices.projection));
            std::memcpy(previewMatrices.position, previewView.position, sizeof(previewMatrices.position));
            render_->setCamera(render_->self, rendererHandle_, &previewMatrices);
            const engine::renderer::FrameParameters frame{
                .clearColor
                = {kPreviewClearColor[0], kPreviewClearColor[1], kPreviewClearColor[2], kPreviewClearColor[3]}};
            render_->beginFrame(render_->self, rendererHandle_, &frame);
            const float identity[16]
                = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            engine::renderer::MeshMaterial material = info->material;
            render_->drawMesh(render_->self, rendererHandle_, mesh, identity, &material);
            render_->endFrame(render_->self, rendererHandle_);

            std::vector<std::uint8_t> rgba;
            std::uint32_t             width  = 0;
            std::uint32_t             height = 0;
            if (!readFrame(rgba, width, height) || width == 0 || height == 0)
            {
                continue;
            }
            const engine::renderer::TextureDescription textureDescription{
                .width          = width,
                .height         = height,
                .pixels         = rgba.data(),
                .filter         = engine::renderer::TextureFilter::Linear,
                .address        = engine::renderer::TextureAddress::ClampEdge,
                .pixelFormat    = engine::renderer::TexturePixelFormat::RGBA8,
                .origin         = engine::renderer::TextureOrigin::TopLeft,
                .rowStrideBytes = width * 4,
            };
            engine::renderer::RenderError         renderError = engine::renderer::RenderError::None;
            const engine::renderer::TextureHandle texture
                = render_->createTexture(render_->self, rendererHandle_, &textureDescription, &renderError);
            if (texture != engine::renderer::invalidTexture)
            {
                content_.setPreviewTexture(partId, texture);
                ++generated;
            }
        }
    }

    // Restore the full viewport and the studio framing (the caller re-applies
    // any explicit CLI framing afterwards).
    render_->setViewport(render_->self, rendererHandle_, framebufferWidth, framebufferHeight);
    configureCamera(52.0f, 10.0f, 2.6f, 1.0f);
    std::printf("part-previews: %u runtime thumbnails generated\n", generated);
    return generated > 0;
}

bool CharacterScreen::renderPartPreviews(
    const std::string &previewDirectory, const std::function<bool(const std::string &)> &exportPpm)
{
    // Render one standalone frame per registered part: neutral three-quarter
    // camera framed on the part's bounds, and a PPM surface export per part
    // (the capture host turns the files into sample sheets).
    if (!created_ || character_ == nullptr)
    {
        return false;
    }
    std::printf("part-previews: rendering part thumbnails into %s\n", previewDirectory.c_str());
    // The quadruped bodies and the internal _taur tails ride their own
    // preview-only slots: the panel never lists them, but the capture-side
    // sample sheets review every part asset.
    const std::vector<std::string> slots
        = {"head",
            "neck",
            "ears",
            "tail",
            "tail_taur",
            "wings",
            "torso",
            "pelvis",
            "arms",
            "legs",
            "garment",
            "quadruped"};
    std::uint32_t rendered = 0;
    for (const std::string &slot : slots)
    {
        for (const std::string &partId : content_.partsInSlot(slot))
        {
            const PartRenderInfo *info = content_.renderInfoForPart(partId);
            if (info == nullptr)
            {
                continue;
            }
            const MeshHandle mesh = content_.meshForGeometry(info->geometryId);
            if (mesh == engine::renderer::invalidMesh)
            {
                continue;
            }
            // Frame the part's own geometry bounds. The orbit target is the
            // part's centre in every axis (parts sit off the origin), and the
            // view state mirrors the export viewport so the projection aspect
            // matches the pixels actually written.
            float boundsCenter[3] = {0.0f, 0.0f, 0.0f};
            float boundsRadius    = 0.3f;
            if (!content_.geometryBounds(info->geometryId, boundsCenter, &boundsRadius))
            {
                continue;
            }
            const float targetY = boundsCenter[1];
            // Tight framing: the bounding sphere fills most of the tile.
            const float distance = std::clamp(boundsRadius * 1.85f, 0.35f, 3.6f);
            configureCamera(38.0f, 10.0f, distance, targetY);
            const float orbitTarget[3] = {boundsCenter[0], boundsCenter[1], boundsCenter[2]};
            camera_->setOrbitTarget(camera_->self, cameraHandle_, orbitTarget);
            // Push the camera's view state into the renderer (configureCamera
            // only sets the camera; the renderer needs the matrices).
            std::uint32_t previewWidth  = 0;
            std::uint32_t previewHeight = 0;
            window_->getFramebufferSize(window_->self, windowHandle_, &previewWidth, &previewHeight);
            engine::camera::ViewState previewView;
            camera_->viewState(camera_->self, cameraHandle_, previewWidth, previewHeight, &previewView);
            engine::renderer::CameraMatrices previewMatrices;
            std::memcpy(previewMatrices.view, previewView.view, sizeof(previewMatrices.view));
            std::memcpy(previewMatrices.projection, previewView.projection, sizeof(previewMatrices.projection));
            std::memcpy(previewMatrices.position, previewView.position, sizeof(previewMatrices.position));
            render_->setCamera(render_->self, rendererHandle_, &previewMatrices);
            const engine::renderer::FrameParameters frame{
                .clearColor
                = {kPreviewClearColor[0], kPreviewClearColor[1], kPreviewClearColor[2], kPreviewClearColor[3]}};
            render_->beginFrame(render_->self, rendererHandle_, &frame);
            const float identity[16]
                = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            engine::renderer::MeshMaterial material = info->material;
            render_->drawMesh(render_->self, rendererHandle_, mesh, identity, &material);
            render_->endFrame(render_->self, rendererHandle_);
            const std::string ppmPath = previewDirectory + "/" + partId + ".ppm";
            if (exportPpm(ppmPath))
            {
                ++rendered;
            }
        }
    }
    std::printf("part-previews: %u thumbnails rendered\n", rendered);
    return rendered > 0;
}

} // namespace demo
