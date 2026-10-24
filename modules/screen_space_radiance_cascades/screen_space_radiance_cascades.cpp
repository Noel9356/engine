#include "screen_space_radiance_cascades.hpp"

#include "lighting_capability.hpp"
#include "light_uniform.hpp"
#include "render_interface.hpp"
#include "renderer.hpp"
#include "scene_description.hpp"
#include "vulkan_utilities.hpp"

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace engine
{

namespace
{

constexpr std::string_view moduleContext = "screen_space_radiance_cascades module";

// Technique settings defaults. The Settings struct's member initializers and
// the [lighting.screen_space_radiance_cascades] fallbacks state the same numbers through
// these constants: the struct defaults previously drifted from the config
// fallbacks (probeStride 8 vs 4, baseRays 64 vs 32, losSteps 16 vs 24) -
// unreachable but a silent trap for the first direct construction. The
// values here are the ones the configuration layer has always applied.
constexpr std::uint32_t defaultCascades            = 5;
constexpr float         defaultIntervalBase        = 0.5f;
constexpr float         defaultIntervalFactor      = 3.0f;
constexpr std::uint32_t defaultProbeStride         = 4;
constexpr std::uint32_t defaultBaseRays            = 32;
constexpr std::uint32_t defaultLosSteps            = 24;
constexpr std::uint32_t defaultMarchSteps          = 48;
constexpr std::uint32_t defaultShadowSteps         = 16;
constexpr std::uint32_t defaultSpatialFilterRadius = 5;

// Angular mapping of the cascade field's directional domain. Every mode
// stores one image layer per direction (the march/compose stages are
// mapping-agnostic - they read the precomputed direction table), the modes
// differ in WHICH directions cover the sphere and how the gather integrates
// them into irradiance:
//
// - octahedron: the direction set of an octahedral equal-basis map texel
//   grid (N x N texels over the sphere, ~base_rays directions). The gather
//   integrates the field with each texel's exact solid angle. DEFAULT -
//   even texel areas keep the quadrature free of the equirect schedule's
//   pole clustering, at the same direction budget.
// - cubemap:    six face grids (6 * F^2 directions, ~base_rays). The gather
//   integrates with each cube texel's exact solid angle.
// - spherical_harmonic: the march runs on the octahedral direction set, but
//   the gather PROJECTS the composed field onto the real spherical harmonics
//   through band 2 and evaluates the irradiance from the coefficients
//   (the Ramamoorthi-Hanrahan truncated cosine-lobe convolution). The field
//   is band-limited: smoother than any texel quadrature, which reads as
//   softly blurred GI at the same march cost.
enum class CascadeAngularMapping : std::uint32_t
{
    Octahedron        = 0,
    Cubemap           = 1,
    SphericalHarmonic = 2,
};

// The [lighting.screen_space_radiance_cascades].mapping spellings.
constexpr const char *defaultMappingName = "octahedron";

bool parseMappingName(
    const std::string &name, CascadeAngularMapping &outMapping
)
{
    if (name == "octahedron")
    {
        outMapping = CascadeAngularMapping::Octahedron;
        return true;
    }
    if (name == "cubemap")
    {
        outMapping = CascadeAngularMapping::Cubemap;
        return true;
    }
    if (name == "spherical_harmonic")
    {
        outMapping = CascadeAngularMapping::SphericalHarmonic;
        return true;
    }
    return false;
}

// Light transport bounces inside one frame: bounce 0 relights march hits
// with the direct lights only; bounce b > 0 adds the previous bounce's
// gathered transport at the hit point (the march samples the gather image of
// bounce b-1 through the hit's screen position). 1 = the historical single
// bounce; every extra bounce re-runs the whole cascade sequence, so the cost
// is linear in bounces.
constexpr std::uint32_t defaultBounces = 1;

// Direction table ceiling: Vulkan only guarantees maxImageArrayLayers >= 256,
// and every cascade image carries one layer per direction. The octahedron
// grid stays within that (16 x 16 = 256); the cubemap grid is clamped to six
// texels per face (6 * 36 = 216). The octahedron floor of 3 texels per axis
// keeps every texel's corner quad free of antipodal vertex pairs (with 2 the
// corner texels span both poles, where the spherical-triangle solid angle
// formula degenerates); the cubemap texel formula is exact at any size.
constexpr std::uint32_t maxOctahedronTexelsPerAxis   = 16;
constexpr std::uint32_t minOctahedronTexelsPerAxis   = 3;
constexpr std::uint32_t maxCubemapFaceTexels         = 6;
constexpr std::uint32_t minCubemapFaceTexels         = 1;

// Push-constant blocks as the .comp shaders see them (file scope so the
// pipeline layouts size their ranges with sizeof; the static_asserts pin the
// shader-side byte counts).
struct MarchPush
{
    std::array<float, 4>         interval;
    std::array<std::uint32_t, 4> flags;
    std::array<std::uint32_t, 4> grid;
    std::array<std::uint32_t, 4> extentStride;
};

struct ComposePush
{
    std::array<float, 4>         coarsestFlag; // w 1 when this is the coarsest cascade
    std::array<std::uint32_t, 4> fineGrid;
    std::array<std::uint32_t, 4> coarseGrid;
};

struct GatherPush
{
    std::array<std::uint32_t, 4> counts;
    std::array<float, 4>         schedule;
    std::array<std::uint32_t, 4> dirs;
};

struct FilterPush
{
    std::array<std::int32_t, 4> params;
    std::array<float, 4>        cameraPlanes;
};

static_assert(sizeof(MarchPush) == 64, "rc_march.comp expects a 64-byte push block");
static_assert(sizeof(ComposePush) == 48, "rc_compose.comp expects a 48-byte push block");
static_assert(sizeof(GatherPush) == 48, "rc_gather.comp expects a 48-byte push block");
static_assert(sizeof(FilterPush) == 32, "rc_denoise.comp expects a 32-byte push block");

// The spatial filter's fixed edge-awareness weights (rc_denoise.comp reads
// params.y/z): the normal power and the depth-band percentage of the
// camera depth range.
constexpr std::int32_t filterNormalPower      = 8;
constexpr std::int32_t filterDepthBandPercent = 4;

// ---- direction tables ----------------------------------------------------------------
//
// One vec4 per direction: xyz the unit direction, w the texel's exact solid
// angle (the gather's quadrature weight). Built once per rebuild on the CPU
// and uploaded to the storage buffer bound at shared set 0 / binding 7
// (cascade_common.glsl); layer order matches the march image layers.

// Solid angle of the spherical triangle spanned by three unit vectors
// (Van Oosterom & Strackee): tan(Omega/2) = |a . (b x c)| / (1 + a.b + b.c + c.a).
float sphericalTriangleSolidAngle(
    const std::array<float, 3> &a, const std::array<float, 3> &b, const std::array<float, 3> &c
)
{
    const float crossBC[3] = {
        b[1] * c[2] - b[2] * c[1],
        b[2] * c[0] - b[0] * c[2],
        b[0] * c[1] - b[1] * c[0],
    };
    const float numerator   = std::abs(a[0] * crossBC[0] + a[1] * crossBC[1] + a[2] * crossBC[2]);
    const float denominator = 1.0f + (a[0] * b[0] + a[1] * b[1] + a[2] * b[2])
                            + (b[0] * c[0] + b[1] * c[1] + b[2] * c[2])
                            + (c[0] * a[0] + c[1] * a[1] + c[2] * a[2]);
    return 2.0f * std::atan2(numerator, denominator);
}

// Octahedral decode: the square [-1,1]^2 folds onto the octahedron (and
// normalized, onto the sphere). Texel centers land strictly inside the
// square; corners may sit on the fold, where the decode is continuous.
std::array<float, 3> octahedronDecode(
    float u, float v
)
{
    float x = u;
    float y = v;
    float z = 1.0f - std::abs(u) - std::abs(v);
    if (z < 0.0f)
    {
        // The lower pyramid: fold the corner triangles back up.
        const float signX = u >= 0.0f ? 1.0f : -1.0f;
        const float signY = v >= 0.0f ? 1.0f : -1.0f;
        x = signX * (1.0f - std::abs(v));
        y = signY * (1.0f - std::abs(u));
    }
    const float length = std::sqrt(x * x + y * y + z * z);
    return {x / length, y / length, z / length};
}

// Octahedron mapping: texelsPerAxis^2 directions, one per texel center of
// the octahedral square. Each texel's solid angle is the exact area of its
// spherical quadrilateral (two spherical triangles over the decoded
// corners) - the octahedral map is only approximately equal-area, so the
// quadrature weights must not be assumed uniform.
std::vector<std::array<float, 4>> buildOctahedronDirections(
    std::uint32_t texelsPerAxis
)
{
    const float                          inverseAxis = 1.0f / static_cast<float>(texelsPerAxis);
    std::vector<std::array<float, 4>>    directions;
    directions.reserve(static_cast<std::size_t>(texelsPerAxis) * texelsPerAxis);
    for (std::uint32_t texelY = 0; texelY < texelsPerAxis; ++texelY)
    {
        for (std::uint32_t texelX = 0; texelX < texelsPerAxis; ++texelX)
        {
            // Texel center direction.
            const float u = (static_cast<float>(texelX) + 0.5f) * 2.0f * inverseAxis - 1.0f;
            const float v = (static_cast<float>(texelY) + 0.5f) * 2.0f * inverseAxis - 1.0f;
            const std::array<float, 3> center = octahedronDecode(u, v);
            // Corner grid over the square, mapped exactly like the decode.
            const std::array<float, 3> corner00 = octahedronDecode(
                static_cast<float>(texelX) * 2.0f * inverseAxis - 1.0f,
                static_cast<float>(texelY) * 2.0f * inverseAxis - 1.0f);
            const std::array<float, 3> corner10 = octahedronDecode(
                static_cast<float>(texelX + 1) * 2.0f * inverseAxis - 1.0f,
                static_cast<float>(texelY) * 2.0f * inverseAxis - 1.0f);
            const std::array<float, 3> corner11 = octahedronDecode(
                static_cast<float>(texelX + 1) * 2.0f * inverseAxis - 1.0f,
                static_cast<float>(texelY + 1) * 2.0f * inverseAxis - 1.0f);
            const std::array<float, 3> corner01 = octahedronDecode(
                static_cast<float>(texelX) * 2.0f * inverseAxis - 1.0f,
                static_cast<float>(texelY + 1) * 2.0f * inverseAxis - 1.0f);
            const float solidAngle = sphericalTriangleSolidAngle(corner00, corner10, corner11)
                                   + sphericalTriangleSolidAngle(corner00, corner11, corner01);
            directions.push_back({center[0], center[1], center[2], solidAngle});
        }
    }
    return directions;
}

// Cubemap mapping: six faces of faceSize^2 texels. A face texel's solid
// angle is the exact solid angle of its square on the face plane z = 1
// (the closed form of the 1/(1+x^2+y^2)^(3/2) integral over the rectangle).
std::vector<std::array<float, 4>> buildCubemapDirections(
    std::uint32_t faceSize
)
{
    const float inverseFace = 1.0f / static_cast<float>(faceSize);
    // Face axes: (forward, uBasis, vBasis) per cube face.
    const std::array<std::array<float, 3>, 6> faceForward = {{
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
    }};
    const std::array<std::array<float, 3>, 6> faceUBasis = {{
        {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
    }};
    const std::array<std::array<float, 3>, 6> faceVBasis = {{
        {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
    }};
    std::vector<std::array<float, 4>> directions;
    directions.reserve(static_cast<std::size_t>(6) * faceSize * faceSize);
    for (std::uint32_t face = 0; face < 6; ++face)
    {
        for (std::uint32_t texelY = 0; texelY < faceSize; ++texelY)
        {
            for (std::uint32_t texelX = 0; texelX < faceSize; ++texelX)
            {
                const float u = (static_cast<float>(texelX) + 0.5f) * 2.0f * inverseFace - 1.0f;
                const float v = (static_cast<float>(texelY) + 0.5f) * 2.0f * inverseFace - 1.0f;
                const std::array<float, 3> &forward = faceForward[face];
                const std::array<float, 3> &uBasis  = faceUBasis[face];
                const std::array<float, 3> &vBasis  = faceVBasis[face];
                const std::array<float, 3>  direction = {
                    forward[0] + u * uBasis[0] + v * vBasis[0],
                    forward[1] + u * uBasis[1] + v * vBasis[1],
                    forward[2] + u * uBasis[2] + v * vBasis[2],
                };
                const float length = std::sqrt(
                    direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]
                );
                // Exact texel solid angle: atan2 terms over the rectangle's
                // corners on the unit face plane.
                const float u0 = static_cast<float>(texelX) * 2.0f * inverseFace - 1.0f;
                const float u1 = static_cast<float>(texelX + 1) * 2.0f * inverseFace - 1.0f;
                const float v0 = static_cast<float>(texelY) * 2.0f * inverseFace - 1.0f;
                const float v1 = static_cast<float>(texelY + 1) * 2.0f * inverseFace - 1.0f;
                const auto  cornerTerm = [](float a, float b)
                {
                    return std::atan2(a * b, std::sqrt(1.0f + a * a + b * b));
                };
                const float solidAngle = cornerTerm(u1, v1) - cornerTerm(u0, v1)
                                       - cornerTerm(u1, v0) + cornerTerm(u0, v0);
                directions.push_back({
                    direction[0] / length, direction[1] / length, direction[2] / length, solidAngle
                });
            }
        }
    }
    return directions;
}

// Direction count and grid resolution of one mapping at the configured ray
// budget: every mode spends approximately base_rays directions. Returns the
// ready-to-upload direction table.
std::vector<std::array<float, 4>> buildDirectionTable(
    CascadeAngularMapping mapping, std::uint32_t baseRays
)
{
    switch (mapping)
    {
    case CascadeAngularMapping::Cubemap:
    {
        const auto faceSize = std::clamp<std::uint32_t>(
            static_cast<std::uint32_t>(std::lround(std::sqrt(static_cast<double>(baseRays) / 6.0))),
            minCubemapFaceTexels,
            maxCubemapFaceTexels
        );
        return buildCubemapDirections(faceSize);
    }
    case CascadeAngularMapping::Octahedron:
    case CascadeAngularMapping::SphericalHarmonic:
    default:
    {
        const auto texelsPerAxis = std::clamp<std::uint32_t>(
            static_cast<std::uint32_t>(std::lround(std::sqrt(static_cast<double>(baseRays)))),
            minOctahedronTexelsPerAxis,
            maxOctahedronTexelsPerAxis
        );
        return buildOctahedronDirections(texelsPerAxis);
    }
    }
}

struct CascadeImage
{
    std::unique_ptr<vk::raii::Image>        image;
    std::unique_ptr<vk::raii::DeviceMemory> memory;
    std::unique_ptr<vk::raii::ImageView>    view;
};

} // namespace

// The screen-space radiance cascades pass: march (per cascade interval), compose
// (front-to-back interval chaining, coarsest first), gather (leak-proof
// visibility weighted probe blend). See the .comp files for the technique.
class ScreenSpaceRadianceCascadesPass final : public LightingTechniquePass
{
public:
    struct Settings
    {
        std::uint32_t cascades       = defaultCascades;
        float         intervalBase   = defaultIntervalBase;
        float         intervalFactor = defaultIntervalFactor;
        // Cost note: the march dispatches (extent/stride)^2 * rays * steps
        // scene-SDF evaluations per frame. Stride 2 at 720p meant 29 million
        // march invocations - multi-second frames and a starved message pump
        // ("not responding"); the configured stride (demo: 8+) trades march
        // cost against smoothness, which the LOS-guarded gather and the
        // spatial filter carry.
        std::uint32_t probeStride      = defaultProbeStride;
        std::uint32_t baseRays         = defaultBaseRays;
        bool          gatherVisibility = true;
        std::uint32_t losSteps         = defaultLosSteps;
        std::uint32_t marchSteps       = defaultMarchSteps;
        bool          relightShadows   = true;
        std::uint32_t shadowSteps      = defaultShadowSteps;
        // Edge-aware spatial filter radius on the gather output (pixels, 0 =
        // pass-through copy). Removes the probe grid's spatial scale; the
        // depth/normal weights keep silhouettes hard.
        std::uint32_t spatialFilterRadius = defaultSpatialFilterRadius;
        // Angular mapping of the field's directional domain (see the enum's
        // comment): octahedron texel grid, cubemap face grid, or SH projection
        // evaluated in the gather. The direction budget stays ~base_rays in
        // every mode.
        CascadeAngularMapping mapping = CascadeAngularMapping::Octahedron;
        // Light transport bounces per frame (1 = the historical single
        // bounce). Every bounce re-runs march + compose + gather; the march
        // of bounce b > 0 adds the previous bounce's gathered transport at
        // its hit points.
        std::uint32_t bounces = defaultBounces;
    };

    ScreenSpaceRadianceCascadesPass(
        Settings settings, LightingCapability *lightingCapability
    )
        : settings(settings), lighting(lightingCapability)
    {
    }

    const char *techniqueName() const override
    {
        return "screen_space_radiance_cascades";
    }

    LightingTarget targets() const override
    {
        return LightingTarget::IndirectDiffuse;
    }

    void record(
        vk::raii::CommandBuffer &commandBuffer, const FrameRenderContext &context
    ) override
    {
        if (cascadeCount() == 0 || context.extent != cachedExtent || needsRecreate)
        {
            rebuild(context);
        }

        updateLightUniform();

        const std::uint32_t cascades = cascadeCount();
        // Angular schedule: CONSTANT across cascades. Every cascade carries
        // the same direction count; the spatial grid refines per cascade
        // (probe stride doubles) while the interval partition grows. The
        // former per-cascade halving with a 2x3 floor handed the entire far
        // field six directions, which is where the composed field's angular
        // structure (giant soft blobs) came from.
        const std::uint32_t directions = directionCount;
        const auto gridWidth      = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return (context.extent.width + stride - 1) / stride;
        };
        const auto gridHeight = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return (context.extent.height + stride - 1) / stride;
        };
        const auto intervalStart = [this](std::uint32_t cascade)
        {
            return cascade == 0
                     ? 0.0f
                     : settings.intervalBase * std::pow(settings.intervalFactor, static_cast<float>(cascade - 1));
        };
        const auto intervalEnd = [this](std::uint32_t cascade)
        { return settings.intervalBase * std::pow(settings.intervalFactor, static_cast<float>(cascade)); };

        // ---- Light transport: one march -> compose -> gather sequence per
        // bounce. Bounce 0 relights march hits with the direct lights; every
        // later bounce also adds the PREVIOUS bounce's gathered transport at
        // the hit point (the march samples the gather output through the
        // hit's screen position). The gather of the LAST bounce runs with the
        // starved-pixel direct fallback (it feeds the composite); earlier
        // bounces run in pure-transport mode so direct light is never
        // counted twice along the bounce chain.
        for (std::uint32_t bounce = 0; bounce < settings.bounces; ++bounce)
        {
            const bool lastBounce = bounce + 1 == settings.bounces;

            // ---- March: every cascade traces its own interval. ---------
            commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **marchPipeline);
            commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **marchPipelineLayout, 0, **sharedSet, {});
            for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
            {
                commandBuffer.bindDescriptorSets(
                    vk::PipelineBindPoint::eCompute, **marchPipelineLayout, 1, **marchSets[cascade], {}
                );
                const MarchPush pushValue{
                    .interval = {intervalStart(cascade), intervalEnd(cascade), 0.0f, 0.0f},
                    .flags    = {context.primitiveCount, settings.marchSteps, cascade > 0 ? 1u : 0u, bounce},
                    .grid     = {gridWidth(cascade), gridHeight(cascade), directions, 0u},
                    .extentStride = {
                        context.extent.width,
                        context.extent.height,
                        settings.probeStride << cascade,
                        settings.probeStride << cascade
                    }
                };
                commandBuffer.pushConstants(
                    **marchPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pushValue), &pushValue
                );
                commandBuffer.dispatch(
                    (gridWidth(cascade) + 3) / 4,
                    (gridHeight(cascade) + 3) / 4,
                    directions
                );
            }

            // march[c] writes -> compose reads.
            marchBarriers(cascadeImageScratch);
            barrierCascades(
                commandBuffer,
                cascadeImageScratch,
                vk::PipelineStageFlagBits2::eComputeShader,
                vk::PipelineStageFlagBits2::eComputeShader
            );

            // ---- Compose: front-to-back interval chaining. -------------
            commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **composePipeline);
            commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **composePipelineLayout, 0, **sharedSet, {});
            for (std::uint32_t cascade = cascades; cascade-- > 0;)
            {
                commandBuffer.bindDescriptorSets(
                    vk::PipelineBindPoint::eCompute, **composePipelineLayout, 1, **composeSets[cascade], {}
                );
                const ComposePush pushValue{
                    .coarsestFlag = {0.0f, 0.0f, 0.0f, cascade == cascades - 1 ? 1.0f : 0.0f},
                    .fineGrid   = {gridWidth(cascade), gridHeight(cascade), directions, 0u},
                    .coarseGrid = {
                        gridWidth(cascade + 1),
                        gridHeight(cascade + 1),
                        directions,
                        0u
                    }
                };
                commandBuffer.pushConstants(
                    **composePipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pushValue), &pushValue
                );
                commandBuffer.dispatch(
                    (gridWidth(cascade) + 3) / 4,
                    (gridHeight(cascade) + 3) / 4,
                    directions
                );

                // full[c] write -> full[c-1] read in the next iteration.
                std::array<vk::ImageMemoryBarrier2, 1> barrier;
                barrier[0].setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
                barrier[0].setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
                barrier[0].setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
                barrier[0].setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
                barrier[0].setOldLayout(vk::ImageLayout::eGeneral);
                barrier[0].setNewLayout(vk::ImageLayout::eGeneral);
                barrier[0].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                barrier[0].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                barrier[0].setImage(**fullImages[cascade].image);
                barrier[0].setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
                vk::DependencyInfo dependency;
                dependency.setImageMemoryBarriers(barrier);
                commandBuffer.pipelineBarrier2(dependency);
            }

            // ---- Gather: per pixel diffuse irradiance from cascade 0. ---
            // The gather writes the module temp image; the NEXT bounce's
            // march reads it as its transport source (bounce > 0), and the
            // spatial filter of the last bounce turns the final one into the
            // renderer-owned indirect target.
            commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **gatherPipeline);
            commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **gatherPipelineLayout, 0, **sharedSet, {});
            commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **gatherPipelineLayout, 1, **gatherSet, {});
            const GatherPush gatherPush{
                .counts = {context.primitiveCount, settings.losSteps, settings.gatherVisibility ? 1u : 0u, cascades},
                .schedule =
                    {settings.intervalBase, settings.intervalFactor, static_cast<float>(settings.probeStride),
                     lastBounce ? 0.0f : 1.0f},
                .dirs = {directions, static_cast<std::uint32_t>(settings.mapping), 0u, 0u}
            };
            commandBuffer.pushConstants(
                **gatherPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(gatherPush), &gatherPush
            );
            commandBuffer.dispatch((context.extent.width + 7) / 8, (context.extent.height + 7) / 8, 1);

            // gather write -> next reader (the next bounce's march, or the
            // spatial filter below on the last bounce).
            barrierGatherTemp(
                commandBuffer,
                vk::AccessFlagBits2::eShaderStorageWrite,
                vk::AccessFlagBits2::eShaderRead
            );
        }

        // ---- Spatial filter: edge-aware cleanup of the gather output. -----
        // (Runs once, on the last bounce's gather output - intermediate
        // bounces feed the transport chain, not the composite.)
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **denoisePipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **denoisePipelineLayout, 0, **sharedSet, {});
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **denoisePipelineLayout, 1, **denoiseSet, {});
        const FilterPush filterPush{
            .params =
                {static_cast<std::int32_t>(settings.spatialFilterRadius), filterNormalPower, filterDepthBandPercent, 0},
            .cameraPlanes = {context.cameraNearPlane, context.cameraFarPlane, 0.0f, 0.0f}
        };
        commandBuffer.pushConstants(
            **denoisePipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(filterPush), &filterPush
        );
        commandBuffer.dispatch((context.extent.width + 7) / 8, (context.extent.height + 7) / 8, 1);
    }

    void notifyFrameResourcesRecreated() override
    {
        needsRecreate = true;
    }

private:
    std::uint32_t cascadeCount() const
    {
        return settings.cascades;
    }

    void barrierCascades(
        vk::raii::CommandBuffer      &commandBuffer,
        const std::vector<vk::Image> &images,
        vk::PipelineStageFlagBits2    sourceStage,
        vk::PipelineStageFlagBits2    destinationStage
    )
    {
        // Scratch members (below): record() runs every frame and fresh
        // vectors per barrier group were pure heap churn on the record path.
        std::vector<vk::ImageMemoryBarrier2> &barriers = cascadeBarrierScratch;
        barriers.clear();
        for (const vk::Image &image : images)
        {
            vk::ImageMemoryBarrier2 barrier;
            barrier.setSrcStageMask(sourceStage);
            barrier.setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
            barrier.setDstStageMask(destinationStage);
            barrier.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
            barrier.setOldLayout(vk::ImageLayout::eGeneral);
            barrier.setNewLayout(vk::ImageLayout::eGeneral);
            barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barrier.setImage(image);
            barrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
            barriers.push_back(barrier);
        }
        vk::DependencyInfo dependency;
        dependency.setImageMemoryBarriers(barriers);
        commandBuffer.pipelineBarrier2(dependency);
    }

    void marchBarriers(
        std::vector<vk::Image> &images
    )
    {
        images.clear();
        for (const CascadeImage &cascade : marchImages)
        {
            images.push_back(**cascade.image);
        }
    }

    // One image memory barrier on the gather temp (the bounce ping-pong
    // image): every gather writes it, the next bounce's march samples it as
    // its transport source and the last bounce's spatial filter reads it.
    void barrierGatherTemp(
        vk::raii::CommandBuffer &commandBuffer,
        vk::AccessFlagBits2      sourceAccess,
        vk::AccessFlagBits2      destinationAccess
    )
    {
        vk::ImageMemoryBarrier2 barrier;
        barrier.setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
        barrier.setSrcAccessMask(sourceAccess);
        barrier.setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
        barrier.setDstAccessMask(destinationAccess);
        barrier.setOldLayout(vk::ImageLayout::eGeneral);
        barrier.setNewLayout(vk::ImageLayout::eGeneral);
        barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setImage(**gatherTemp.image);
        barrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
        std::array<vk::ImageMemoryBarrier2, 1> barriers{barrier};
        vk::DependencyInfo                     dependency;
        dependency.setImageMemoryBarriers(barriers);
        commandBuffer.pipelineBarrier2(dependency);
    }

    void createCascadeImage(
        CascadeImage             &target,
        const FrameRenderContext &context,
        vk::Extent2D              grid,
        std::uint32_t             layers,
        vk::ImageViewType         viewType = vk::ImageViewType::e2DArray
    )
    {
        vk::ImageCreateInfo imageInfo;
        imageInfo.setImageType(vk::ImageType::e2D);
        imageInfo.setFormat(vk::Format::eR16G16B16A16Sfloat);
        imageInfo.setExtent(vk::Extent3D(grid.width, grid.height, 1));
        imageInfo.setMipLevels(1);
        imageInfo.setArrayLayers(layers);
        imageInfo.setSamples(vk::SampleCountFlagBits::e1);
        imageInfo.setTiling(vk::ImageTiling::eOptimal);
        imageInfo.setUsage(vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage);
        imageInfo.setSharingMode(vk::SharingMode::eExclusive);
        imageInfo.setInitialLayout(vk::ImageLayout::eUndefined);
        target.image                              = std::make_unique<vk::raii::Image>(context.device, imageInfo);
        const vk::MemoryRequirements requirements = target.image->getMemoryRequirements();
        vk::MemoryAllocateInfo       allocate;
        allocate.setAllocationSize(requirements.size);
        allocate.setMemoryTypeIndex(findMemoryType(
            context.physicalDevice, requirements.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal
        ));
        target.memory = std::make_unique<vk::raii::DeviceMemory>(context.device, allocate);
        target.image->bindMemory(*target.memory, 0);
        vk::ImageViewCreateInfo viewInfo;
        viewInfo.setImage(**target.image);
        viewInfo.setViewType(viewType);
        viewInfo.setFormat(vk::Format::eR16G16B16A16Sfloat);
        viewInfo.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, layers));
        target.view = std::make_unique<vk::raii::ImageView>(context.device, viewInfo);
        // UNDEFINED -> GENERAL through a one-shot submission on the renderer's
        // graphics queue: storage image writes and sampled reads both require
        // a defined layout before the first dispatch that touches the image.
        vk::CommandPoolCreateInfo poolInfo;
        poolInfo.setFlags(vk::CommandPoolCreateFlagBits::eTransient);
        poolInfo.setQueueFamilyIndex(context.graphicsQueueFamily);
        vk::raii::CommandPool         setupPool(context.device, poolInfo);
        vk::CommandBufferAllocateInfo allocateInfo;
        allocateInfo.setCommandPool(*setupPool);
        allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
        allocateInfo.setCommandBufferCount(1);
        vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(context.device, allocateInfo).front());
        setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
        vk::ImageMemoryBarrier2 barrier;
        barrier.setSrcStageMask(vk::PipelineStageFlagBits2::eAllCommands);
        barrier.setSrcAccessMask(vk::AccessFlags2());
        barrier.setDstStageMask(vk::PipelineStageFlagBits2::eAllCommands);
        barrier.setDstAccessMask(vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite);
        barrier.setOldLayout(vk::ImageLayout::eUndefined);
        barrier.setNewLayout(vk::ImageLayout::eGeneral);
        barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setImage(**target.image);
        barrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, layers));
        std::array<vk::ImageMemoryBarrier2, 1> barriers{barrier};
        vk::DependencyInfo                     dependency;
        dependency.setImageMemoryBarriers(barriers);
        setupBuffer.pipelineBarrier2(dependency);
        setupBuffer.end();
        vk::SubmitInfo submitInfo;
        submitInfo.setCommandBuffers(*setupBuffer);
        context.graphicsQueue.submit(submitInfo);
        context.graphicsQueue.waitIdle();
    }

    void updateLightUniform()
    {
        GpuLightUniform             uniform;
        const bool                  hasLighting = lighting != nullptr;
        DirectionalLightDescription sun;
        const bool                  sunEnabled = hasLighting && lighting->hasSunLight();
        if (sunEnabled)
        {
            sun = lighting->sunLight();
        }
        const std::uint32_t count = hasLighting ? std::min(lighting->pointLightCount(), maxPointLights) : 0;
        std::array<PointLightDescription, maxPointLights> pointLights{};
        for (std::uint32_t index = 0; index < count; ++index)
        {
            pointLights[index] = lighting->pointLight(index);
        }
        fillSunAndPointLights(uniform, sunEnabled, sun, pointLights.data(), count);
        uniform.lightCountsAndFlags = {
            static_cast<std::int32_t>(count),
            settings.relightShadows ? 1 : 0,
            static_cast<std::int32_t>(settings.shadowSteps),
            0
        };
        std::memcpy(lightStorage.mapped, &uniform, sizeof(uniform));
    }

    void rebuild(
        const FrameRenderContext &context
    )
    {
        // Teardown in strict reverse dependency order: descriptor sets, pools,
        // pipelines, pipeline layouts, descriptor set layouts, then images,
        // buffers and samplers. Pipelines embed their layouts and layouts
        // embed the descriptor set layouts, so a layout must never die while
        // a pipeline that references it is alive (docs/robustness.md,
        // invariant 25).
        gatherSet.reset();
        gatherPool.reset();
        denoiseSet.reset();
        denoisePool.reset();
        for (auto &set : composeSets)
        {
            set.reset();
        }
        composePool.reset();
        for (auto &set : marchSets)
        {
            set.reset();
        }
        marchPool.reset();
        sharedSet.reset();
        sharedPool.reset();
        gatherPipeline.reset();
        gatherPipelineLayout.reset();
        denoisePipeline.reset();
        denoisePipelineLayout.reset();
        composePipeline.reset();
        composePipelineLayout.reset();
        marchPipeline.reset();
        marchPipelineLayout.reset();
        sharedSetLayout.reset();
        marchSetLayout.reset();
        composeSetLayout.reset();
        gatherSetLayout.reset();
        filterSetLayout.reset();
        for (auto &image : fullImages)
        {
            image = CascadeImage{};
        }
        for (auto &image : marchImages)
        {
            image = CascadeImage{};
        }
        gatherTemp = CascadeImage{};
        skyImage   = CascadeImage{};
        lightStorage.reset();
        directionStorage.reset();
        composeSampler.reset();
        nearestSampler.reset();

        const std::uint32_t cascades = cascadeCount();
        // Direction table per angular mapping (Settings.mapping): every mode
        // spends approximately base_rays directions - the octahedron texel
        // grid, the cubemap face grid, or the octahedron grid again for the
        // SH projection (the march always runs on discrete directions; the
        // mapping only changes the gather's irradiance evaluation). One vec4
        // per direction - xyz the unit direction, w the exact solid angle -
        // uploaded to the host-visible storage buffer bound at shared set 0 /
        // binding 7 (cascade_common.glsl). Layer order matches the march's
        // image layers.
        const std::vector<std::array<float, 4>> directionTable =
            buildDirectionTable(settings.mapping, settings.baseRays);
        directionCount = static_cast<std::uint32_t>(directionTable.size());
        {
            directionStorage = createMappedHostVisibleBuffer(
                context.device,
                context.physicalDevice,
                static_cast<vk::DeviceSize>(directionTable.size() * sizeof(std::array<float, 4>)),
                vk::BufferUsageFlagBits::eStorageBuffer
            );
            std::memcpy(directionStorage.mapped, directionTable.data(), directionTable.size() * sizeof(std::array<float, 4>));
        }

        const auto gridWidth = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return std::max(1u, (context.extent.width + stride - 1) / stride);
        };
        const auto gridHeight = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return std::max(1u, (context.extent.height + stride - 1) / stride);
        };
        const auto layers = [this](std::uint32_t) { return directionCount; };

        // Cascade storage: march (radiance + transmittance) and full (composed
        // radiance) per cascade. Sized to the cascade grid, so memory decays
        // by 4x per cascade (memory rule).
        marchImages.resize(cascades);
        fullImages.resize(cascades);
        for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
        {
            const vk::Extent2D grid{gridWidth(cascade), gridHeight(cascade)};
            createCascadeImage(marchImages[cascade], context, grid, layers(cascade));
            createCascadeImage(fullImages[cascade], context, grid, layers(cascade));
        }
        createCascadeImage(skyImage, context, vk::Extent2D{1, 1}, 1);

        nearestSampler = makeNearestClampSampler(context.device);

        vk::SamplerCreateInfo linearInfo;
        linearInfo.setMagFilter(vk::Filter::eLinear);
        linearInfo.setMinFilter(vk::Filter::eLinear);
        linearInfo.setMipmapMode(vk::SamplerMipmapMode::eNearest);
        linearInfo.setAddressModeU(vk::SamplerAddressMode::eClampToEdge);
        linearInfo.setAddressModeV(vk::SamplerAddressMode::eClampToEdge);
        linearInfo.setAddressModeW(vk::SamplerAddressMode::eClampToEdge);
        composeSampler = std::make_unique<vk::raii::Sampler>(context.device, linearInfo);

        // Light uniform buffer (own copy: the GI relight configures shadow
        // behaviour from [lighting.screen_space_radiance_cascades], independent of the
        // direct module).
        lightStorage = createMappedHostVisibleBuffer(
            context.device, context.physicalDevice, sizeof(GpuLightUniform), vk::BufferUsageFlagBits::eUniformBuffer
        );

        // Layouts -----------------------------------------------------------
        {
            // Shared set 0: camera UBO, primitives SSBO, light UBO (the two
            // relighting passes), material table SSBO, albedo/data texture
            // arrays (textured relight), environment UBO (the compose sky
            // base), cascade direction table SSBO (binding 7,
            // cascade_common.glsl).
            std::array<vk::DescriptorSetLayoutBinding, 8> bindings;
            bindings[0].setBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[3].setBinding(3).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[3].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[4]
                .setBinding(4)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                .setDescriptorCount(1);
            bindings[4].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[5]
                .setBinding(5)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                .setDescriptorCount(1);
            bindings[5].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[6].setBinding(6).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[6].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[7].setBinding(7).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[7].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            sharedSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            // March role set: depth, normal (sampled), the cascade storage
            // image (written), and (bounce > 0) the previous bounce's gather
            // output as the transport source (sampled). Bounce 0 never reads
            // it (the push flag gates the sample), but the binding must stay
            // valid.
            std::array<vk::DescriptorSetLayoutBinding, 4> bindings;
            for (std::uint32_t binding = 0; binding < 2; ++binding)
            {
                bindings[binding]
                    .setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[3].setBinding(3).setDescriptorType(vk::DescriptorType::eCombinedImageSampler).setDescriptorCount(1);
            bindings[3].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            marchSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            std::array<vk::DescriptorSetLayoutBinding, 3> bindings;
            bindings[0]
                .setBinding(0)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                .setDescriptorCount(1);
            bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            composeSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            // Spatial filter: gathered GI, depth, normal + the output storage
            // image.
            std::array<vk::DescriptorSetLayoutBinding, 4> bindings;
            for (std::uint32_t binding = 0; binding < 3; ++binding)
            {
                bindings[binding]
                    .setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[3].setBinding(3).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[3].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            filterSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            // Gather: one composed-cascade sampler per cascade slot (the
            // shader switches over the per-pixel cascade index), depth,
            // normal, and the gather output storage image.
            std::array<vk::DescriptorSetLayoutBinding, 11> bindings;
            for (std::uint32_t binding = 0; binding < 10; ++binding)
            {
                bindings[binding]
                    .setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[10].setBinding(10).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[10].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            gatherSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }

        // Pipelines ---------------------------------------------------------
        auto makeComputePipeline = [&context](
                                       const std::vector<char>             &code,
                                       const vk::raii::DescriptorSetLayout &sharedLayout,
                                       const vk::raii::DescriptorSetLayout &roleLayout,
                                       std::uint32_t                        pushSize
                                   )
        {
            vk::PushConstantRange pushRange;
            pushRange.setStageFlags(vk::ShaderStageFlagBits::eCompute);
            pushRange.setOffset(0);
            pushRange.setSize(pushSize);
            vk::PipelineLayoutCreateInfo                 layoutInfo;
            const std::array<vk::DescriptorSetLayout, 2> layouts{*sharedLayout, *roleLayout};
            layoutInfo.setSetLayouts(layouts);
            layoutInfo.setPushConstantRanges(pushRange);
            auto layout = std::make_unique<vk::raii::PipelineLayout>(context.device, layoutInfo);

            vk::ShaderModuleCreateInfo moduleInfo;
            moduleInfo.setCodeSize(code.size()).setPCode(reinterpret_cast<const std::uint32_t *>(code.data()));
            vk::raii::ShaderModule            module(context.device, moduleInfo);
            vk::PipelineShaderStageCreateInfo stage;
            stage.setStage(vk::ShaderStageFlagBits::eCompute).setModule(*module).setPName("main");
            vk::ComputePipelineCreateInfo pipelineInfo;
            pipelineInfo.setStage(stage);
            pipelineInfo.setLayout(**layout);
            auto pipeline = std::make_unique<vk::raii::Pipeline>(context.device, nullptr, pipelineInfo);
            struct Built
            {
                std::unique_ptr<vk::raii::PipelineLayout> layout;
                std::unique_ptr<vk::raii::Pipeline>       pipeline;
            };
            return Built{.layout = std::move(layout), .pipeline = std::move(pipeline)};
        };

        auto marchBuilt = makeComputePipeline(
            readShaderBytes(context.shaderDirectory / "rc_march.comp.spv", moduleContext),
            *sharedSetLayout,
            *marchSetLayout,
            sizeof(MarchPush)
        );
        marchPipelineLayout = std::move(marchBuilt.layout);
        marchPipeline       = std::move(marchBuilt.pipeline);
        auto composeBuilt   = makeComputePipeline(
            readShaderBytes(context.shaderDirectory / "rc_compose.comp.spv", moduleContext),
            *sharedSetLayout,
            *composeSetLayout,
            sizeof(ComposePush)
        );
        composePipelineLayout = std::move(composeBuilt.layout);
        composePipeline       = std::move(composeBuilt.pipeline);
        auto gatherBuilt      = makeComputePipeline(
            readShaderBytes(context.shaderDirectory / "rc_gather.comp.spv", moduleContext),
            *sharedSetLayout,
            *gatherSetLayout,
            sizeof(GatherPush)
        );
        gatherPipelineLayout = std::move(gatherBuilt.layout);
        gatherPipeline       = std::move(gatherBuilt.pipeline);
        // The filter pass has its own small layout (3 sampled images + 1
        // storage image) - the gather layout grew per-cascade sampler
        // bindings the filter does not use. Push block: params + camera
        // planes (32 bytes) - the planes linearize buffer depth for the
        // edge-aware depth band.
        auto denoiseBuilt = makeComputePipeline(
            readShaderBytes(context.shaderDirectory / "rc_denoise.comp.spv", moduleContext),
            *sharedSetLayout,
            *filterSetLayout,
            sizeof(FilterPush)
        );
        denoisePipelineLayout = std::move(denoiseBuilt.layout);
        denoisePipeline       = std::move(denoiseBuilt.pipeline);

        // Gather output temp: full screen extent, single layer, same format
        // as the indirect target. The gather and filter shaders declare plain
        // image2D/sampler2D, so the view is 2D (not the cascade 2D array).
        createCascadeImage(
            gatherTemp, context, vk::Extent2D{context.extent.width, context.extent.height}, 1, vk::ImageViewType::e2D
        );

        // Descriptor sets ---------------------------------------------------
        {
            std::array<vk::DescriptorPoolSize, 3> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer, 3),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 3),
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 2)
            };
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            sharedPool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**sharedPool);
            allocate.setSetLayouts(**sharedSetLayout);
            sharedSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front())
            );

            vk::DescriptorBufferInfo cameraInfo(context.cameraBuffer, 0, sizeof(CameraData));
            vk::DescriptorBufferInfo primitivesInfo(context.primitivesBuffer, 0, vk::WholeSize);
            vk::DescriptorBufferInfo lightInfo(**lightStorage.buffer, 0, sizeof(GpuLightUniform));
            vk::DescriptorBufferInfo materialsInfo(context.materialsBuffer, 0, vk::WholeSize);
            vk::DescriptorImageInfo  albedoArrayInfo(
                **nearestSampler, context.albedoArrayView, vk::ImageLayout::eShaderReadOnlyOptimal
            );
            vk::DescriptorImageInfo dataArrayInfo(
                **nearestSampler, context.dataArrayView, vk::ImageLayout::eShaderReadOnlyOptimal
            );
            vk::DescriptorBufferInfo              environmentInfo(context.environmentBuffer, 0, vk::WholeSize);
            vk::DescriptorBufferInfo              directionsInfo(**directionStorage.buffer, 0, vk::WholeSize);
            std::array<vk::WriteDescriptorSet, 8> writes;
            writes[0].setDstSet(**sharedSet).setDstBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[0].setBufferInfo(cameraInfo);
            writes[1].setDstSet(**sharedSet).setDstBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[1].setBufferInfo(primitivesInfo);
            writes[2].setDstSet(**sharedSet).setDstBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[2].setBufferInfo(lightInfo);
            writes[3].setDstSet(**sharedSet).setDstBinding(3).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[3].setBufferInfo(materialsInfo);
            writes[4]
                .setDstSet(**sharedSet)
                .setDstBinding(4)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[4].setImageInfo(albedoArrayInfo);
            writes[5]
                .setDstSet(**sharedSet)
                .setDstBinding(5)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[5].setImageInfo(dataArrayInfo);
            writes[6].setDstSet(**sharedSet).setDstBinding(6).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[6].setBufferInfo(environmentInfo);
            writes[7].setDstSet(**sharedSet).setDstBinding(7).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[7].setBufferInfo(directionsInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        const vk::ImageView dummyView = *skyImage.view;
        marchSets.resize(cascades);
        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, cascades * 3),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, cascades)
            };
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(cascades);
            poolInfo.setPoolSizes(poolSizes);
            marchPool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
            {
                vk::DescriptorSetAllocateInfo allocate;
                allocate.setDescriptorPool(**marchPool);
                allocate.setSetLayouts(**marchSetLayout);
                marchSets[cascade] = std::make_unique<vk::raii::DescriptorSet>(
                    std::move(vk::raii::DescriptorSets(context.device, allocate).front())
                );
                vk::DescriptorImageInfo depthInfo(**nearestSampler, context.depthView, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo normalInfo(**nearestSampler, context.normalView, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo outInfo(
                    **nearestSampler, *marchImages[cascade].view, vk::ImageLayout::eGeneral
                );
                // The bounce source: the gather temp (full extent 2D view).
                // Bounce 0's march never samples it - the push flags.w gates
                // the read - but the descriptor stays bound.
                vk::DescriptorImageInfo bounceSourceInfo(**nearestSampler, *gatherTemp.view, vk::ImageLayout::eGeneral);
                std::array<vk::WriteDescriptorSet, 4> writes;
                writes[0]
                    .setDstSet(**marchSets[cascade])
                    .setDstBinding(0)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[0].setImageInfo(depthInfo);
                writes[1]
                    .setDstSet(**marchSets[cascade])
                    .setDstBinding(1)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[1].setImageInfo(normalInfo);
                writes[2]
                    .setDstSet(**marchSets[cascade])
                    .setDstBinding(2)
                    .setDescriptorType(vk::DescriptorType::eStorageImage);
                writes[2].setImageInfo(outInfo);
                writes[3]
                    .setDstSet(**marchSets[cascade])
                    .setDstBinding(3)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[3].setImageInfo(bounceSourceInfo);
                context.device.updateDescriptorSets(writes, {});
            }
        }

        composeSets.resize(cascades);
        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, cascades),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, cascades * 2)
            };
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(cascades);
            poolInfo.setPoolSizes(poolSizes);
            composePool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
            {
                vk::DescriptorSetAllocateInfo allocate;
                allocate.setDescriptorPool(**composePool);
                allocate.setSetLayouts(**composeSetLayout);
                composeSets[cascade] = std::make_unique<vk::raii::DescriptorSet>(
                    std::move(vk::raii::DescriptorSets(context.device, allocate).front())
                );
                // The coarsest cascade has no coarser neighbor: bind the sky
                // dummy (the shader branches to the sky constant before it
                // could ever read it).
                const vk::ImageView     coarseView = cascade + 1 < cascades ? *fullImages[cascade + 1].view : dummyView;
                vk::DescriptorImageInfo coarseInfo(**composeSampler, coarseView, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo outInfo(**nearestSampler, *fullImages[cascade].view, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo marchInfo(
                    **nearestSampler, *marchImages[cascade].view, vk::ImageLayout::eGeneral
                );
                std::array<vk::WriteDescriptorSet, 3> writes;
                writes[0]
                    .setDstSet(**composeSets[cascade])
                    .setDstBinding(0)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[0].setImageInfo(coarseInfo);
                writes[1]
                    .setDstSet(**composeSets[cascade])
                    .setDstBinding(1)
                    .setDescriptorType(vk::DescriptorType::eStorageImage);
                writes[1].setImageInfo(outInfo);
                writes[2]
                    .setDstSet(**composeSets[cascade])
                    .setDstBinding(2)
                    .setDescriptorType(vk::DescriptorType::eStorageImage);
                writes[2].setImageInfo(marchInfo);
                context.device.updateDescriptorSets(writes, {});
            }
        }

        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 10),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 1)
            };
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            gatherPool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**gatherPool);
            allocate.setSetLayouts(**gatherSetLayout);
            gatherSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front())
            );
            // Bindings 0..7: the composed cascade of that index; slots beyond
            // the configured cascade count bind the sky stand-in (the gather
            // selection clamps to the configured count and never reads them).
            // The image info array must outlive updateDescriptorSets: the
            // write structs only reference it.
            std::array<vk::WriteDescriptorSet, 11> writes;
            std::array<vk::DescriptorImageInfo, 8> cascadeInfos;
            for (std::uint32_t cascade = 0; cascade < 8; ++cascade)
            {
                const vk::ImageView cascadeView = cascade < cascades ? *fullImages[cascade].view : *skyImage.view;
                cascadeInfos[cascade]           = vk::DescriptorImageInfo(
                    **nearestSampler, cascadeView, vk::ImageLayout::eGeneral
                );
                writes[cascade]
                    .setDstSet(**gatherSet)
                    .setDstBinding(cascade)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[cascade].setImageInfo(cascadeInfos[cascade]);
            }
            vk::DescriptorImageInfo depthInfo(**nearestSampler, context.depthView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo normalInfo(**nearestSampler, context.normalView, vk::ImageLayout::eGeneral);
            // The gather writes the MODULE temp; the filter pass turns it
            // into the renderer-owned indirect target.
            vk::DescriptorImageInfo outInfo(**nearestSampler, *gatherTemp.view, vk::ImageLayout::eGeneral);
            writes[8]
                .setDstSet(**gatherSet)
                .setDstBinding(8)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[8].setImageInfo(depthInfo);
            writes[9]
                .setDstSet(**gatherSet)
                .setDstBinding(9)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[9].setImageInfo(normalInfo);
            writes[10].setDstSet(**gatherSet).setDstBinding(10).setDescriptorType(vk::DescriptorType::eStorageImage);
            writes[10].setImageInfo(outInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 3),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 1)
            };
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            denoisePool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**denoisePool);
            allocate.setSetLayouts(**filterSetLayout);
            denoiseSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front())
            );
            vk::DescriptorImageInfo giInfo(**nearestSampler, *gatherTemp.view, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo depthInfo(**nearestSampler, context.depthView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo normalInfo(**nearestSampler, context.normalView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo outInfo(**nearestSampler, context.indirectRadianceView, vk::ImageLayout::eGeneral);
            std::array<vk::WriteDescriptorSet, 4> writes;
            writes[0]
                .setDstSet(**denoiseSet)
                .setDstBinding(0)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[0].setImageInfo(giInfo);
            writes[1]
                .setDstSet(**denoiseSet)
                .setDstBinding(1)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[1].setImageInfo(depthInfo);
            writes[2]
                .setDstSet(**denoiseSet)
                .setDstBinding(2)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[2].setImageInfo(normalInfo);
            writes[3].setDstSet(**denoiseSet).setDstBinding(3).setDescriptorType(vk::DescriptorType::eStorageImage);
            writes[3].setImageInfo(outInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        cachedExtent  = context.extent;
        needsRecreate = false;
    }

    Settings            settings;
    LightingCapability *lighting = nullptr;

    // Direction count of the active angular mapping (the direction table's
    // size): the march/compose third dispatch dimension, the cascade image
    // layer count, and the gather's quadrature loop bound.
    std::uint32_t directionCount = 0;
    vk::Extent2D  cachedExtent{};
    bool          needsRecreate = false;

    std::vector<CascadeImage> marchImages;
    std::vector<CascadeImage> fullImages;
    CascadeImage              gatherTemp; // full extent gather output, filtered into the target
    CascadeImage              skyImage;   // 1x1x1 stand-in for the coarsest compose binding

    std::unique_ptr<vk::raii::Sampler> nearestSampler;
    std::unique_ptr<vk::raii::Sampler> composeSampler;
    MappedHostVisibleBuffer            lightStorage;
    MappedHostVisibleBuffer            directionStorage;

    std::unique_ptr<vk::raii::DescriptorSetLayout>        sharedSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout>        marchSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout>        composeSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout>        gatherSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout>        filterSetLayout;
    std::unique_ptr<vk::raii::DescriptorPool>             sharedPool;
    std::unique_ptr<vk::raii::DescriptorSet>              sharedSet;
    std::unique_ptr<vk::raii::DescriptorPool>             marchPool;
    std::vector<std::unique_ptr<vk::raii::DescriptorSet>> marchSets;
    std::unique_ptr<vk::raii::DescriptorPool>             composePool;
    std::vector<std::unique_ptr<vk::raii::DescriptorSet>> composeSets;
    std::unique_ptr<vk::raii::DescriptorPool>             gatherPool;
    std::unique_ptr<vk::raii::DescriptorSet>              gatherSet;

    std::unique_ptr<vk::raii::PipelineLayout> marchPipelineLayout;
    std::unique_ptr<vk::raii::Pipeline>       marchPipeline;
    std::unique_ptr<vk::raii::PipelineLayout> composePipelineLayout;
    std::unique_ptr<vk::raii::Pipeline>       composePipeline;
    std::unique_ptr<vk::raii::PipelineLayout> gatherPipelineLayout;
    std::unique_ptr<vk::raii::Pipeline>       gatherPipeline;
    std::unique_ptr<vk::raii::PipelineLayout> denoisePipelineLayout;
    std::unique_ptr<vk::raii::Pipeline>       denoisePipeline;
    std::unique_ptr<vk::raii::DescriptorPool> denoisePool;
    std::unique_ptr<vk::raii::DescriptorSet>  denoiseSet;

    // Record-path scratch reused across frames (see barrierCascades above).
    std::vector<vk::ImageMemoryBarrier2> cascadeBarrierScratch;
    std::vector<vk::Image>               cascadeImageScratch;
};

// ---- Module lifecycle ---------------------------------------------------------------

ScreenSpaceRadianceCascadesModule::ScreenSpaceRadianceCascadesModule()
{
    componentManifest.name = "screen_space_radiance_cascades";
    componentManifest.kind = ComponentKind::Module;
    // "indirect_diffuse" is this technique's radiance layer: the run demands
    // it when the GI transport pass should attach to the renderer.
    componentManifest.providedCapabilities = {"indirect_diffuse"};
    componentManifest.requiredCapabilities = {"logging", "renderer"};
    // "lighting" is OPTIONAL (the first manifest use of the mechanism): the
    // pass relights march hits with the scene lights when a provider is
    // active and degrades to emissive + sky transport without one. Declaring
    // it through optionalCapabilities gives the module everything the old
    // hand-rolled pattern (an explicit capability.lighting permission plus a
    // try/catch around the acquire) could not: the resolver orders this
    // module after the lighting provider when one exists - construct's
    // acquire below is a property of the assembly, never of plugin scan
    // order - and the capability.<name> permission is derived from the
    // declaration. Optional capabilities never join the demand closure, so
    // this declaration does not pull a light provider into runs that have no
    // other use for one.
    componentManifest.optionalCapabilities = {"lighting"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.renderer"};
    // The pass settings the module reads from [lighting.screen_space_radiance_cascades].
    componentManifest.knownConfigKeys = {
        "lighting.screen_space_radiance_cascades.cascades",
        "lighting.screen_space_radiance_cascades.interval_base",
        "lighting.screen_space_radiance_cascades.interval_factor",
        "lighting.screen_space_radiance_cascades.probe_stride",
        "lighting.screen_space_radiance_cascades.base_rays",
        "lighting.screen_space_radiance_cascades.mapping",
        "lighting.screen_space_radiance_cascades.bounces",
        "lighting.screen_space_radiance_cascades.gather_visibility",
        "lighting.screen_space_radiance_cascades.los_steps",
        "lighting.screen_space_radiance_cascades.march_steps",
        "lighting.screen_space_radiance_cascades.relight_shadows",
        "lighting.screen_space_radiance_cascades.shadow_steps",
        "lighting.screen_space_radiance_cascades.spatial_filter",
    };
}

ScreenSpaceRadianceCascadesModule::~ScreenSpaceRadianceCascadesModule()
{
    stop();
}

void ScreenSpaceRadianceCascadesModule::construct(
    ComponentContext &context
)
{
    logger = &context.acquireCapability<LoggerCapability>();

    ScreenSpaceRadianceCascadesPass::Settings settings;
    settings.cascades         = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "screen_space_radiance_cascades.cascades").value_or(defaultCascades), 1, 8
    ));
    settings.intervalBase     = static_cast<float>(context
            .configurationFloatingInSection("lighting", "screen_space_radiance_cascades.interval_base")
            .value_or(defaultIntervalBase));
    settings.intervalFactor   = static_cast<float>(context
            .configurationFloatingInSection("lighting", "screen_space_radiance_cascades.interval_factor")
            .value_or(defaultIntervalFactor));
    settings.probeStride      = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "screen_space_radiance_cascades.probe_stride")
            .value_or(defaultProbeStride),
        1,
        64
    ));
    settings.baseRays         = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "screen_space_radiance_cascades.base_rays").value_or(defaultBaseRays),
        4,
        256
    ));
    settings.gatherVisibility = context.configurationBooleanInSection("lighting", "screen_space_radiance_cascades.gather_visibility")
                                    .value_or(true);
    settings.losSteps         = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "screen_space_radiance_cascades.los_steps").value_or(defaultLosSteps),
        4,
        256
    ));
    settings.marchSteps       = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "screen_space_radiance_cascades.march_steps").value_or(defaultMarchSteps),
        4,
        512
    ));
    settings.relightShadows   = context.configurationBooleanInSection("lighting", "screen_space_radiance_cascades.relight_shadows")
                                    .value_or(true);
    settings.shadowSteps      = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "screen_space_radiance_cascades.shadow_steps")
            .value_or(defaultShadowSteps),
        4,
        256
    ));
    // Filter radius clamp follows the shader's (rc_denoise.comp clamps 0..16):
    // the shipped demo asks for 10 (one probe cell spans 8 pixels at stride 8,
    // so the window must average ACROSS cells), and an 8-cap here silently
    // applied 8 for 10 - the same stale dense-probe-era limit the shader side
    // already shed.
    settings.spatialFilterRadius = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "screen_space_radiance_cascades.spatial_filter")
            .value_or(defaultSpatialFilterRadius),
        0,
        16
    ));
    // Angular mapping: octahedron (default), cubemap, or spherical_harmonic.
    // An unknown spelling is a precise startup error, not a silent fallback:
    // the mapping changes the GI's look, so a typo must never quietly render
    // with a different one.
    const std::string mappingName = context.configurationStringInSection(
                                        "lighting", "screen_space_radiance_cascades.mapping"
    )
                                        .value_or(defaultMappingName);
    if (!parseMappingName(mappingName, settings.mapping))
    {
        throw std::runtime_error(
            "screen_space_radiance_cascades module: [lighting.screen_space_radiance_cascades] mapping '" + mappingName
            + "' is not a known mapping (octahedron, cubemap, spherical_harmonic)"
        );
    }
    // Light transport bounces: 1 reproduces the historical single bounce;
    // every extra bounce re-runs the cascade sequence (cost linear in
    // bounces). There is NO upper bound by design: the bounce count is a
    // quality/performance trade the configuration owns. The engine states
    // the cost here and reports the configured value at start; it does not
    // second-guess the machine the run executes on. The only validation is
    // the structural floor - 0 bounces would disable the transport this pass
    // exists for, and negative counts are a typo.
    const std::int64_t configuredBouncesValue = context.configurationIntegerInSection(
                                                    "lighting", "screen_space_radiance_cascades.bounces"
    )
                                                    .value_or(defaultBounces);
    if (configuredBouncesValue < 1)
    {
        throw std::runtime_error(
            "screen_space_radiance_cascades module: [lighting.screen_space_radiance_cascades] bounces must be >= 1 (0 would disable the "
            "light transport; drop the indirect_diffuse demand instead of configuring the pass empty)"
        );
    }
    settings.bounces = static_cast<std::uint32_t>(configuredBouncesValue);
    // Start-log reporting copies (the pass keeps the authoritative settings).
    configuredMappingName = mappingName;
    configuredBounces     = settings.bounces;
    if (settings.intervalBase <= 0.0f || settings.intervalFactor < 1.0f)
    {
        throw std::runtime_error("screen_space_radiance_cascades module: interval_base must be > 0 and interval_factor >= 1");
    }

    // The lighting capability is optional (declared in the manifest): the
    // resolver ordered this module after the provider when one is active, so
    // the probe below is a stable answer, never scan-order luck. A denied or
    // missing provider only disables the first-bounce relight.
    LightingCapability *lighting = nullptr;
    if (context.isCapabilityAvailable(LightingCapability::capabilityName))
    {
        lighting = &context.acquireCapability<LightingCapability>();
    }

    // Attachment follows demand: this module only activates when the run
    // required the indirect_diffuse layer it provides, so an active module
    // always attaches. Asking about one's OWN capability is the one question
    // the demand set answers.
    passAttached      = context.isCapabilityDemanded("indirect_diffuse");
    lightingAvailable = lighting != nullptr;

    pass = std::make_shared<ScreenSpaceRadianceCascadesPass>(settings, lighting);
    if (passAttached)
    {
        RendererCapability &rendererCapability = context.acquireCapability<RendererCapability>();
        auto               *rendererModule     = static_cast<RendererModule *>(&rendererCapability);
        rendererModule->attachLightingTechnique(pass);
    }
}

void ScreenSpaceRadianceCascadesModule::start()
{
    logger->logMessage(
        LogLevel::Information,
        std::string("screen_space_radiance_cascades module started; technique ")
            + (passAttached ? "attached (indirect radiance)" : "indirect_diffuse not demanded; idle")
            + ", light capability " + (lightingAvailable ? "present" : "absent (no first-bounce relight)")
            + ", mapping " + configuredMappingName
            + ", bounces " + std::to_string(configuredBounces)
    );
}

void ScreenSpaceRadianceCascadesModule::stop()
{
    passAttached = false;
    pass.reset();
    logger = nullptr;
}

void *ScreenSpaceRadianceCascadesModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    // "indirect_diffuse" is a demand marker: it carries no call interface, so
    // the implementation behind it is the module object itself. The kernel
    // type-erases the pointer; whoever demands the marker consumes the work
    // the module attaches to the frame, not a cast.
    if (capabilityName == "indirect_diffuse")
    {
        return this;
    }
    return nullptr;
}

} // namespace engine
