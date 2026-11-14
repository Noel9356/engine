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
constexpr float kPreviewClearColor[4] = {0.36f, 0.38f, 0.42f, 1.0f};

// Part tiles render with an orthographic projection: the tile frames the
// part's own bounding box at a scale that reads, independent of the part's
// distance from the lens. Perspective framing clipped small parts (ears,
// eyes) into the camera's near plane and rendered empty tiles, and pushed
// flat parts (fins, blades) into tiny silhouettes because the framing
// radius came from the box diagonal instead of the projected silhouette.
// The eye parks a fixed distance away and the frame height solves from the
// projected box corners, so every part fills its tile the same way.
constexpr float kPreviewEyeDistance = 3.0f;
constexpr float kPreviewOrthoNearZ  = 0.1f;
constexpr float kPreviewOrthoFarZ   = 10.0f;

// Orthographic frame height (world units mapped to the viewport height)
// that shows an axis-aligned box (half-extents around the orbit target)
// from a yaw/pitch three-quarter view so the widest projected corner
// reaches `fill` of the viewport height (and the same fraction of width
// at `aspect`).
float orthoFrameHeight(const float halfExtents[3], float yawDegrees, float pitchDegrees, float fill, float aspect)
{
    const float     yaw     = glm::radians(yawDegrees);
    const float     pitch   = glm::radians(pitchDegrees);
    const glm::vec3 toEye(std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch));
    const glm::vec3 forward = -toEye;
    const glm::vec3 right   = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    const glm::vec3 up      = glm::normalize(glm::cross(right, forward));
    float           widest  = 0.0f;
    for (int cornerIndex = 0; cornerIndex < 8; ++cornerIndex)
    {
        const glm::vec3 corner(halfExtents[0] * ((cornerIndex & 1) != 0 ? 1.0f : -1.0f),
            halfExtents[1] * ((cornerIndex & 2) != 0 ? 1.0f : -1.0f),
            halfExtents[2] * ((cornerIndex & 4) != 0 ? 1.0f : -1.0f));
        const float across = std::abs(glm::dot(corner, right));
        const float tall   = std::abs(glm::dot(corner, up));
        widest = std::max(widest, std::max(tall / fill, across / (fill * aspect)));
    }
    return widest * 2.0f + 1.0e-3f;
}

// Reads the camera's live perspective parameters back from its projection
// matrix (column-major: M[2][2] = -(f+n)/(f-n), M[2][3] = 2fn/(n-f),
// M[3][2] = -1) so the post-preview restore uses the camera's real values
// instead of assumptions. Returns false when the matrix is not perspective.
bool extractPerspective(const float projection[16], float &outFovYDegrees, float &outNearZ, float &outFarZ)
{
    const float depthScale  = projection[10];
    const float depthOffset = projection[14];
    if (!std::isfinite(depthScale) || !std::isfinite(depthOffset)
        || std::abs(projection[11] + 1.0f) > 1.0e-4f || std::abs(depthScale + 1.0f) < 1.0e-4f)
    {
        return false;
    }
    const float nearZ = depthOffset / (depthScale - 1.0f);
    const float farZ  = nearZ * (depthScale - 1.0f) / (depthScale + 1.0f);
    const float yScale = projection[5];
    if (!std::isfinite(nearZ) || !std::isfinite(farZ) || nearZ <= 0.0f || farZ <= nearZ || !std::isfinite(yScale)
        || yScale <= 0.0f)
    {
        return false;
    }
    outFovYDegrees = glm::degrees(2.0f * std::atan(1.0f / yScale));
    outNearZ       = nearZ;
    outFarZ        = farZ;
    return true;
}

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
            const int         dr    = std::abs(static_cast<int>(frame.rgba[index]) - 92);
            const int         dg    = std::abs(static_cast<int>(frame.rgba[index + 1]) - 97);
            const int         db    = std::abs(static_cast<int>(frame.rgba[index + 2]) - 107);
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

    // The studio camera's live perspective parameters, read back from the
    // cached projection so the post-preview restore returns the camera to
    // exactly the state the scene was using.
    engine::camera::ViewState studioView;
    camera_->viewState(camera_->self, cameraHandle_, framebufferWidth, framebufferHeight, &studioView);
    float studioFovYDegrees = 55.0f;
    float studioNearZ       = 0.25f;
    float studioFarZ        = 2000.0f;
    extractPerspective(studioView.projection, studioFovYDegrees, studioNearZ, studioFarZ);

    // The part thumbnail renderer, factored once: frames one mesh alone
    // (bounds from the given geometry) and reads the frame back into a
    // texture stored under the target key.
    const auto renderThumbnail
        = [&](const MeshHandle mesh, const engine::renderer::MeshMaterial &material, const std::string &boundsKey,
              const std::string &textureKey) -> bool
    {
        float boundsCenter[3] = {0.0f, 0.0f, 0.0f};
        float boundsRadius    = 0.3f;
        if (!content_.geometryBounds(boundsKey, boundsCenter, &boundsRadius))
        {
            return false;
        }
        // Orthographic framing on the part's OWN box: the corner projection
        // above sets the frame height so the tile fills evenly for every
        // part size, and the parked eye keeps the whole part inside the
        // ortho near/far band (perspective framing clipped small parts into
        // the camera's near plane and rendered empty tiles).
        float boundsMin[3]   = {0.0f, 0.0f, 0.0f};
        float boundsMax[3]   = {0.0f, 0.0f, 0.0f};
        float halfExtents[3] = {boundsRadius * 0.5f, boundsRadius * 0.5f, boundsRadius * 0.5f};
        if (content_.geometryExtents(boundsKey, boundsMin, boundsMax))
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                halfExtents[axis] = (boundsMax[axis] - boundsMin[axis]) * 0.5f;
            }
        }
        const float targetY     = boundsCenter[1];
        const float frameHeight = orthoFrameHeight(halfExtents, 30.0f, 14.0f, 0.85f, 1.0f);
        camera_->setProjection(camera_->self, cameraHandle_, engine::camera::ProjectionType::Orthographic, frameHeight,
            kPreviewOrthoNearZ, kPreviewOrthoFarZ);
        configureCamera(30.0f, 14.0f, kPreviewEyeDistance, targetY);
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
        render_->drawMesh(render_->self, rendererHandle_, mesh, identity, &material);
        render_->endFrame(render_->self, rendererHandle_);

        std::vector<std::uint8_t> rgba;
        std::uint32_t             width  = 0;
        std::uint32_t             height = 0;
        if (!readFrame(rgba, width, height) || width == 0 || height == 0)
        {
            return false;
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
        if (texture == engine::renderer::invalidTexture)
        {
            return false;
        }
        content_.setPreviewTexture(textureKey, texture);
        return true;
    };

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
            if (renderThumbnail(mesh, info->material, info->geometryId, info->geometryId))
            {
                content_.setPreviewTexture(partId, content_.previewTextureForPart(info->geometryId));
                ++generated;
            }
        }
    }

    // The coat swatches and marking modes preview as the real torso repaint:
    // each card is the bare thoracic mesh re-derived under that coat/marking
    // combination and rendered through the same thumbnail path. The swatches
    // use the current head species' match-head palette (the preview tells the
    // truth about the one-pelt read).
    {
        DeformationInputs liveInputs;
        gatherDeformationInputs(liveInputs);
        const PartRenderInfo *torsoInfo = content_.renderInfoForPart("torso_thoracic_bare");
        if (torsoInfo != nullptr)
        {
            for (int coatIndex = 0; coatIndex < 8; ++coatIndex)
            {
                DeformationInputs inputs = liveInputs;
                inputs.furryBody          = true;
                inputs.coatIndex          = coatIndex;
                inputs.markingMode        = 0;
                inputs.muzzleWeight       = 0.0f;
                inputs.eyesWidthWeight    = 0.0f;
                inputs.neckThicknessWeight = 0.0f;
                inputs.neckHeadSizeWeight = 0.0f;
                char key[24];
                std::snprintf(key, sizeof(key), "coat_%d", coatIndex);
                const MeshHandle mesh = content_.repaintPreviewMesh("torso_thoracic_bare", inputs);
                if (mesh != engine::renderer::invalidMesh
                    && renderThumbnail(mesh, torsoInfo->material, "torso_thoracic_bare", key))
                {
                    ++generated;
                }
                render_->destroyMesh(render_->self, rendererHandle_, mesh);
            }
            for (int markingMode = 0; markingMode < 6; ++markingMode)
            {
                DeformationInputs inputs = liveInputs;
                inputs.furryBody          = true;
                inputs.coatIndex          = 0;
                inputs.markingMode        = markingMode;
                inputs.muzzleWeight       = 0.0f;
                inputs.eyesWidthWeight    = 0.0f;
                inputs.neckThicknessWeight = 0.0f;
                inputs.neckHeadSizeWeight = 0.0f;
                char key[28];
                std::snprintf(key, sizeof(key), "marking_%d", markingMode);
                const MeshHandle mesh = content_.repaintPreviewMesh("torso_thoracic_bare", inputs);
                if (mesh != engine::renderer::invalidMesh
                    && renderThumbnail(mesh, torsoInfo->material, "torso_thoracic_bare", key))
                {
                    ++generated;
                }
                render_->destroyMesh(render_->self, rendererHandle_, mesh);
            }
        }
    }

    // Restore the full viewport and the studio framing (the caller re-applies
    // any explicit CLI framing afterwards).
    render_->setViewport(render_->self, rendererHandle_, framebufferWidth, framebufferHeight);
    camera_->setProjection(camera_->self, cameraHandle_, engine::camera::ProjectionType::Perspective, studioFovYDegrees,
        studioNearZ, studioFarZ);
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
    // The studio camera's live perspective parameters, read back from the
    // cached projection so the post-preview restore returns the camera to
    // exactly the state the scene was using.
    std::uint32_t previewWidthFull  = 0;
    std::uint32_t previewHeightFull = 0;
    window_->getFramebufferSize(window_->self, windowHandle_, &previewWidthFull, &previewHeightFull);
    engine::camera::ViewState studioView;
    camera_->viewState(camera_->self, cameraHandle_, previewWidthFull, previewHeightFull, &studioView);
    float studioFovYDegrees = 55.0f;
    float studioNearZ       = 0.25f;
    float studioFarZ        = 2000.0f;
    extractPerspective(studioView.projection, studioFovYDegrees, studioNearZ, studioFarZ);
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
            // Push the camera's view state into the renderer (configureCamera
            // only sets the camera; the renderer needs the matrices).
            std::uint32_t previewWidth  = 0;
            std::uint32_t previewHeight = 0;
            window_->getFramebufferSize(window_->self, windowHandle_, &previewWidth, &previewHeight);
            const float targetY = boundsCenter[1];
            // Orthographic framing on the part's OWN box: the corner
            // projection above sets the frame height so the tile fills
            // evenly for every part size, and the parked eye keeps the
            // whole part inside the ortho near/far band (perspective
            // framing clipped small parts into the camera's near plane and
            // rendered empty tiles).
            float boundsMin[3]   = {0.0f, 0.0f, 0.0f};
            float boundsMax[3]   = {0.0f, 0.0f, 0.0f};
            float halfExtents[3] = {boundsRadius * 0.5f, boundsRadius * 0.5f, boundsRadius * 0.5f};
            if (content_.geometryExtents(info->geometryId, boundsMin, boundsMax))
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    halfExtents[axis] = (boundsMax[axis] - boundsMin[axis]) * 0.5f;
                }
            }
            const float frameHeight = orthoFrameHeight(halfExtents, 38.0f, 10.0f, 0.82f,
                previewHeight != 0 ? static_cast<float>(previewWidth) / static_cast<float>(previewHeight) : 1.0f);
            camera_->setProjection(camera_->self, cameraHandle_, engine::camera::ProjectionType::Orthographic,
                frameHeight, kPreviewOrthoNearZ, kPreviewOrthoFarZ);
            configureCamera(38.0f, 10.0f, kPreviewEyeDistance, targetY);
            const float orbitTarget[3] = {boundsCenter[0], boundsCenter[1], boundsCenter[2]};
            camera_->setOrbitTarget(camera_->self, cameraHandle_, orbitTarget);
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
    // Restore the studio camera (projection type, scale, near/far band, and
    // the framing the caller re-applies any explicit CLI values onto).
    camera_->setProjection(camera_->self, cameraHandle_, engine::camera::ProjectionType::Perspective, studioFovYDegrees,
        studioNearZ, studioFarZ);
    configureCamera(52.0f, 10.0f, 2.6f, 1.0f);
    std::printf("part-previews: %u thumbnails rendered\n", rendered);
    return rendered > 0;
}

} // namespace demo
