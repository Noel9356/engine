// Directional-sun shadow mapping for the headless backend.
//
// The rasterizer's sun term is otherwise unoccluded: any surface facing the
// light receives the full radiance, so nothing grounds the objects it draws.
// This pass rasterizes the frame's opaque, lit draws into an orthographic
// depth map aligned with the sun (the software twin of a shadow-map depth
// pass) and shades the sun term through a small percentage-closer filter, so
// casters drop real shadows with a penumbra the filter width controls.
//
// The map is built at endFrame from the draws submitted that frame and
// consumed by the next frame's shading: the caster set only exists once
// every draw has been seen, and rebuilding after the last draw keeps the
// build off the shaded pixels' critical path. A hash over the sun direction
// and every caster's (mesh, transform) pair skips the rebuild while the
// scene is static, so a converged view pays nothing - the same amortization
// the transform cache and the module's static-frame skip use.
//
// Casters are the draws the backend actually rasterized as opaque lit
// geometry: unlit surfaces (sky domes, emissive panels - the decorative
// geometry the lighting environment already accounts for) and transparent
// surfaces (their transmitted share is the reflection field's chain, not a
// hard occluder) do not block the sun, matching the shadow-ray semantics
// the radiance_cache module applies to its own trace.
#pragma once

#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace engine
{
namespace renderer
{
namespace sunshadows
{

// One opaque draw captured for the shadow pass: the mesh handle plus the
// column-major model matrix of the submission (the mesh geometry itself is
// looked up at build time, so a handle destroyed between draw and build
// simply drops out of the caster set).
struct SunShadowCaster
{
    std::uint32_t mesh     = 0;
    float         transform[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f};
};

// Geometry + placement one caster resolves to at build time. The caller
// borrows the mesh's planar position array and index list and supplies the
// draw's world-space bounds box (the mesh's local bounds box transformed by
// the model matrix - exact for any rotation/scale combination, unlike a
// radius scaled by a basis norm).
struct SunShadowGeometry
{
    const float         *positions        = nullptr; // tightly packed xyz triplets
    std::size_t          positionCount    = 0;
    const std::uint32_t *indices          = nullptr;
    std::size_t          indexCount       = 0;
    const float         *transformColumnMajor = nullptr;
    glm::vec3            worldBoundsMin{0.0f};
    glm::vec3            worldBoundsMax{0.0f};
};

// The built shadow map: an orthographic depth buffer looking from the scene
// toward the sun, stored as unit-space depth (0 = nearest the light,
// 1 = the far plane or empty). The projection is carried as a basis + extents
// instead of a matrix so the per-sample project is three dot products, and
// texel/bias scales are precomputed in world units for the normal-offset
// bias the sampler applies.
struct SunShadowMap
{
    std::uint32_t     resolution = 0;
    std::vector<float> depth;      // row-major, resolution * resolution, 1 = empty
    glm::vec3         sunDirection{0.0f, 1.0f, 0.0f};

    // Light-space frame: origin at the fit box's light-space center, forward
    // pointing toward the sun (depth increases along forward).
    glm::vec3 origin{0.0f};
    glm::vec3 right{1.0f, 0.0f, 0.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    glm::vec3 forward{0.0f, 0.0f, 1.0f};
    float     halfWidth      = 1.0f; // light-space u half extent
    float     halfHeight     = 1.0f; // light-space v half extent
    float     minDepth       = 0.0f; // forward-projected near extent
    float     depthExtent    = 1.0f; // far - near along forward
    float     texelWorldSize = 1.0f; // world units per map texel (larger axis)
    bool      valid          = false;
};

// Rasterizes the casters into the map. The orthographic u/v fit follows
// the RECEIVER box (the camera frustum's world bounds, what the screen can
// actually see) padded by a fifth on every side, so the texel density tracks
// the visible extent rather than the scene's - a huge floor under a tight
// view keeps centimeter texels instead of decimeter ones. The depth range
// stays the casters' full span along the light axis (depth resolution does
// not trade against texel density, and tall casters above the view must
// still land in the range). receiverMin/receiverMax may be null, which fits
// to the casters' own bounds instead. Empty caster lists or a degenerate sun
// direction leave the map invalid.
inline void buildSunShadowMap(SunShadowMap &map, const std::vector<SunShadowGeometry> &casters,
    std::uint32_t resolution, const glm::vec3 &sunDirection, const glm::vec3 *receiverMin = nullptr,
    const glm::vec3 *receiverMax = nullptr)
{
    map.valid = false;
    const bool degenerateSun = sunDirection.x == 0.0f && sunDirection.y == 0.0f && sunDirection.z == 0.0f;
    if (casters.empty() || resolution < 8u || degenerateSun)
    {
        return;
    }
    const glm::vec3 sunWard = glm::normalize(sunDirection);
    // Depth runs along the light's travel direction (from the sun into the
    // scene): the surface nearest the light carries the SMALLEST depth, the
    // depth buffer's min-store keeps it, and a receiver is lit exactly when
    // nothing nearer the light than itself covers the texel.
    const glm::vec3 forward = -sunWard;
    // Stable right/up basis: project the world up onto the light plane,
    // falling back to the world x axis for a vertical sun.
    glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    if (std::abs(glm::dot(worldUp, forward)) > 0.999f)
    {
        worldUp = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    const glm::vec3 right = glm::normalize(glm::cross(worldUp, forward));
    const glm::vec3 up    = glm::cross(forward, right);

    // Casters' union in LIGHT space: each caster contributes its world bounds
    // box's eight corners projected onto (right, up, forward), so tilted suns
    // measure the extents along the light axes (the depth range and the
    // fallback u/v fit).
    glm::vec3 minimum( std::numeric_limits<float>::max());
    glm::vec3 maximum(std::numeric_limits<float>::lowest());
    for (const SunShadowGeometry &caster : casters)
    {
        for (std::int32_t dz = 0; dz < 2; ++dz)
        {
            for (std::int32_t dy = 0; dy < 2; ++dy)
            {
                for (std::int32_t dx = 0; dx < 2; ++dx)
                {
                    const glm::vec3 corner(dx ? caster.worldBoundsMax.x : caster.worldBoundsMin.x,
                        dy ? caster.worldBoundsMax.y : caster.worldBoundsMin.y,
                        dz ? caster.worldBoundsMax.z : caster.worldBoundsMin.z);
                    const glm::vec3 lightCorner(
                        glm::dot(corner, right), glm::dot(corner, up), glm::dot(corner, forward));
                    minimum = glm::min(minimum, lightCorner);
                    maximum = glm::max(maximum, lightCorner);
                }
            }
        }
    }
    if (!(maximum.x > minimum.x) || !(maximum.y > minimum.y) || !(maximum.z > minimum.z))
    {
        return; // degenerate fit (single-point scene)
    }

    // Light-space extents: u/v from the receiver box when provided (padded
    // a fifth so shadows entering from just outside the view still
    // rasterize), depth always the casters' full span. The light-space
    // coordinates are taken against the world origin, so the map's origin is
    // the world point whose coordinates equal the chosen centers.
    float uCenter = (minimum.x + maximum.x) * 0.5f;
    float vCenter = (minimum.y + maximum.y) * 0.5f;
    float halfU   = (maximum.x - minimum.x) * 0.5f;
    float halfV   = (maximum.y - minimum.y) * 0.5f;
    if (receiverMin != nullptr && receiverMax != nullptr && receiverMax->x > receiverMin->x
        && receiverMax->y > receiverMin->y && std::isfinite(receiverMin->x) && std::isfinite(receiverMax->x))
    {
        glm::vec3 receiverLight[8];
        std::size_t corner = 0;
        for (std::int32_t dz = 0; dz < 2; ++dz)
        {
            for (std::int32_t dy = 0; dy < 2; ++dy)
            {
                for (std::int32_t dx = 0; dx < 2; ++dx)
                {
                    const glm::vec3 worldCorner(dx ? receiverMax->x : receiverMin->x,
                        dy ? receiverMax->y : receiverMin->y,
                        dz ? receiverMax->z : receiverMin->z);
                    receiverLight[corner++] = glm::vec3(glm::dot(worldCorner, right), glm::dot(worldCorner, up),
                        glm::dot(worldCorner, forward));
                }
            }
        }
        float uMin = receiverLight[0].x, uMax = receiverLight[0].x;
        float vMin = receiverLight[0].y, vMax = receiverLight[0].y;
        for (const glm::vec3 &c : receiverLight)
        {
            uMin = std::min(uMin, c.x);
            uMax = std::max(uMax, c.x);
            vMin = std::min(vMin, c.y);
            vMax = std::max(vMax, c.y);
        }
        // Intersect the view's range with the casters' own span: a far
        // plane reaching past the scene must not dilute the texel density
        // (the receivers shadowing anything are the casters themselves),
        // and a tight view keeps its fine texels. An empty intersection
        // (the view looking entirely away from the casters) keeps the
        // casters' range, which the raster then mostly clips away anyway.
        uMin = std::max(uMin, minimum.x);
        uMax = std::min(uMax, maximum.x);
        vMin = std::max(vMin, minimum.y);
        vMax = std::min(vMax, maximum.y);
        if (uMax <= uMin)
        {
            uMin = minimum.x;
            uMax = maximum.x;
        }
        if (vMax <= vMin)
        {
            vMin = minimum.y;
            vMax = maximum.y;
        }
        constexpr float kReceiverPadding = 1.2f;
        uCenter = (uMin + uMax) * 0.5f;
        vCenter = (vMin + vMax) * 0.5f;
        halfU   = (uMax - uMin) * 0.5f * kReceiverPadding + 0.25f;
        halfV   = (vMax - vMin) * 0.5f * kReceiverPadding + 0.25f;
    }
    const float dCenter     = (minimum.z + maximum.z) * 0.5f;

    map.resolution    = resolution;
    map.sunDirection  = sunWard;
    map.origin        = right * uCenter + up * vCenter + forward * dCenter;
    map.right         = right;
    map.up            = up;
    map.forward       = forward;
    map.halfWidth     = halfU;
    map.halfHeight    = halfV;
    map.minDepth      = (minimum.z - maximum.z) * 0.5f;
    map.depthExtent   = std::max(maximum.z - minimum.z, 1.0e-3f);
    map.texelWorldSize
        = std::max(map.halfWidth, map.halfHeight) * 2.0f / static_cast<float>(resolution);

    map.depth.assign(static_cast<std::size_t>(resolution) * resolution, 1.0f);

    const float invHalfWidth  = 1.0f / map.halfWidth;
    const float invHalfHeight = 1.0f / map.halfHeight;
    const float invDepth      = 1.0f / map.depthExtent;
    const float mapSize       = static_cast<float>(resolution);

    // Project one world position into the map's unit cube (u, v in [0, 1],
    // depth 0 = nearest the light).
    const auto project = [&](const glm::vec3 &position)
    {
        const glm::vec3 delta = position - map.origin;
        return glm::vec3((glm::dot(delta, right) * invHalfWidth + 1.0f) * 0.5f,
            (glm::dot(delta, up) * invHalfHeight + 1.0f) * 0.5f,
            (glm::dot(delta, forward) - map.minDepth) * invDepth);
    };

    for (const SunShadowGeometry &caster : casters)
    {
        if (caster.positions == nullptr || caster.indices == nullptr || caster.transformColumnMajor == nullptr)
        {
            continue;
        }
        const glm::mat4 model = glm::make_mat4(caster.transformColumnMajor);

        const std::size_t triangleCount = caster.indexCount / 3;
        for (std::size_t triangle = 0; triangle < triangleCount; ++triangle)
        {
            glm::vec3 world[3];
            glm::vec3 unit[3];
            bool      inside = true;
            for (std::uint32_t corner = 0; corner < 3; ++corner)
            {
                const std::uint32_t vertexIndex = caster.indices[triangle * 3 + corner];
                if (vertexIndex >= caster.positionCount)
                {
                    inside = false;
                    break;
                }
                const glm::vec4 local(caster.positions[vertexIndex * 3], caster.positions[vertexIndex * 3 + 1],
                    caster.positions[vertexIndex * 3 + 2], 1.0f);
                world[corner] = glm::vec3(model * local);
                unit[corner]  = project(world[corner]);
                if (unit[corner].x < -0.05f || unit[corner].x > 1.05f || unit[corner].y < -0.05f
                    || unit[corner].y > 1.05f || unit[corner].z < 0.0f || unit[corner].z > 1.0f)
                {
                    // The fit box contains every caster's bounds, so this only
                    // rejects guard-band padding artifacts; both sides of the
                    // clip keep the conservative min-depth rasterization safe.
                    inside = false;
                    break;
                }
            }
            if (!inside)
            {
                continue;
            }

            // Edge-function scan over the triangle's map-space bounding box;
            // barycentric depth interpolation with a min-store (closest to
            // the light wins, the depth-buffer rule).
            const float area = (unit[1].x - unit[0].x) * (unit[2].y - unit[0].y)
                - (unit[2].x - unit[0].x) * (unit[1].y - unit[0].y);
            if (std::abs(area) < 1.0e-12f)
            {
                continue;
            }
            const float invArea = 1.0f / area;
            const float xMin = std::max((std::min(unit[0].x, std::min(unit[1].x, unit[2].x))) * mapSize, 0.0f);
            const float xMax = std::min((std::max(unit[0].x, std::max(unit[1].x, unit[2].x))) * mapSize,
                mapSize - 1.0f);
            const float yMin = std::max((std::min(unit[0].y, std::min(unit[1].y, unit[2].y))) * mapSize, 0.0f);
            const float yMax = std::min((std::max(unit[0].y, std::max(unit[1].y, unit[2].y))) * mapSize,
                mapSize - 1.0f);
            if (xMax < xMin || yMax < yMin)
            {
                continue;
            }

            // Incremental edge functions stepping one texel in x/y.
            const float edgeA = (unit[1].y - unit[2].y) * invArea;
            const float edgeB = (unit[2].x - unit[1].x) * invArea;
            const float edgeC = (unit[2].y - unit[0].y) * invArea;
            const float edgeD = (unit[0].x - unit[2].x) * invArea;
            const float edgeE = (unit[0].y - unit[1].y) * invArea;
            const float edgeF = (unit[1].x - unit[0].x) * invArea;

            const std::int32_t columnBegin = static_cast<std::int32_t>(std::floor(xMin));
            const std::int32_t columnEnd   = static_cast<std::int32_t>(std::floor(xMax));
            const std::int32_t rowBegin    = static_cast<std::int32_t>(std::floor(yMin));
            const std::int32_t rowEnd      = static_cast<std::int32_t>(std::floor(yMax));

            for (std::int32_t row = rowBegin; row <= rowEnd; ++row)
            {
                float *depthRow = map.depth.data() + static_cast<std::size_t>(row) * resolution;
                const float sampleY = (static_cast<float>(row) + 0.5f) / mapSize;
                for (std::int32_t column = columnBegin; column <= columnEnd; ++column)
                {
                    const float sampleX = (static_cast<float>(column) + 0.5f) / mapSize;
                    // Barycentric weights in map space; a sample on the
                    // triangle's plane only when all three are within the
                    // conservative epsilon the raster edge walk uses.
                    const float w0 = edgeA * (sampleX - unit[1].x) + edgeB * (sampleY - unit[1].y);
                    const float w1 = edgeC * (sampleX - unit[2].x) + edgeD * (sampleY - unit[2].y);
                    const float w2 = edgeE * (sampleX - unit[0].x) + edgeF * (sampleY - unit[0].y);
                    if (w0 < -0.001f || w1 < -0.001f || w2 < -0.001f)
                    {
                        continue;
                    }
                    const float depth = w0 * unit[0].z + w1 * unit[1].z + w2 * unit[2].z;
                    float &cell = depthRow[static_cast<std::uint32_t>(column)];
                    if (depth < cell)
                    {
                        cell = depth;
                    }
                }
            }
        }
    }
    map.valid = true;
}

// Sun visibility of one shaded point, percentage-closer filtered over a
// filter-radius box of taps. The sample position is offset along the
// geometric normal before projection (the standard normal-offset bias: it
// moves the depth comparison off the caster's own surface, which at grazing
// sun angles is the acne source), and the depth comparison carries a
// slope-scaled bias in unit depth. Points outside the map's fit (the camera
// looking past the caster union) and an invalid map read as fully lit.
inline float sunShadowVisibility(const SunShadowMap &map, const glm::vec3 &worldPosition,
    const glm::vec3 &normal, float filterRadiusTexels)
{
    if (!map.valid || map.resolution < 8u)
    {
        return 1.0f;
    }
    const float nDotL = glm::dot(normal, map.sunDirection);
    if (nDotL <= 0.0f)
    {
        return 1.0f; // facing away from the sun: the shading weight is already zero
    }
    const float offsetScale = 1.5f * map.texelWorldSize / std::max(nDotL, 0.35f);
    const glm::vec3 offsetPosition = worldPosition + normal * offsetScale;

    const glm::vec3 delta   = offsetPosition - map.origin;
    const float     u       = (glm::dot(delta, map.right) / map.halfWidth + 1.0f) * 0.5f;
    const float     v       = (glm::dot(delta, map.up) / map.halfHeight + 1.0f) * 0.5f;
    const float     depth   = (glm::dot(delta, map.forward) - map.minDepth) / map.depthExtent;
    if (u <= 0.0f || v <= 0.0f || u >= 1.0f || v >= 1.0f || depth <= 0.0f || depth >= 1.0f)
    {
        return 1.0f;
    }

    const float slope      = std::sqrt(std::max(0.0f, 1.0f - nDotL * nDotL));
    const float bias       = (2.0f + 3.0f * slope) * map.texelWorldSize / map.depthExtent;
    const float refDepth   = depth - bias;
    const float mapSize    = static_cast<float>(map.resolution);
    const float radius     = std::max(filterRadiusTexels, 0.0f);
    // Soft-compare ramp width: roughly one and a half texels' worth of depth
    // (the scale the stored depths quantize at), so penumbra edges shade
    // smoothly instead of stepping.
    const float compareRamp = 1.5f * map.texelWorldSize / map.depthExtent;
    const float invRamp     = compareRamp > 1.0e-9f ? 1.0f / compareRamp : 0.0f;

    const auto tapLit = [&](float tapX, float tapY)
    {
        const std::int32_t column = static_cast<std::int32_t>(tapX);
        const std::int32_t row    = static_cast<std::int32_t>(tapY);
        if (column < 0 || row < 0 || column >= static_cast<std::int32_t>(map.resolution)
            || row >= static_cast<std::int32_t>(map.resolution))
        {
            return 1.0f;
        }
        const float stored = map.depth[static_cast<std::size_t>(row) * map.resolution
            + static_cast<std::uint32_t>(column)];
        // Soft depth compare: instead of the hard lit/blocked step (whose
        // quantization the 4-tap average cannot hide), the comparison ramps
        // over roughly one texel's worth of depth, so penumbra edges shade
        // smoothly and the box taps blend spatially.
        return glm::clamp((stored - refDepth) * invRamp + 0.5f, 0.0f, 1.0f);
    };

    if (radius < 0.25f)
    {
        return tapLit(u * mapSize - 0.5f, v * mapSize - 0.5f);
    }
    // 2x2 percentage-closer taps at the filter radius: the four box corners
    // averaged (0, 0.25, 0.5, 0.75, 1 steps of penumbra).
    const float sampleU = u * mapSize - 0.5f;
    const float sampleV = v * mapSize - 0.5f;
    const float sum = tapLit(sampleU - radius, sampleV - radius) + tapLit(sampleU + radius, sampleV - radius)
        + tapLit(sampleU - radius, sampleV + radius) + tapLit(sampleU + radius, sampleV + radius);
    return sum * 0.25f;
}

} // namespace sunshadows
} // namespace renderer
} // namespace engine
