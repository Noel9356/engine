// frustum.hpp - view-projection frustum extraction for the spatial_index module.
//
// Converts a column-major OpenGL clip-space view-projection matrix into six
// inward-facing world-space planes (normalized when their normal is nonzero).

#ifndef ENGINE_SPATIAL_INDEX_FRUSTUM_HPP
#define ENGINE_SPATIAL_INDEX_FRUSTUM_HPP

#include <engine/spatial_index/types.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace engine::spatial
{

// Extracts left, right, bottom, top, near, and far planes from a column-major
// view-projection matrix using OpenGL clip depth (-w <= z <= w). Plane
// equations use dot(normal, point) + distance >= 0 for points inside. A
// zero-normal plane with non-negative distance is emitted as an inactive
// plane, which supports an infinite far plane. The output is unchanged when
// the matrix or any extracted plane is invalid.
[[nodiscard]] inline bool extractFrustum(const float viewProjection[16], Frustum3 *outFrustum) noexcept
{
    if (viewProjection == nullptr || outFrustum == nullptr)
        return false;

    for (std::size_t index = 0; index < 16; ++index)
        if (!std::isfinite(viewProjection[index]))
            return false;

    Frustum3 result{};
    for (std::size_t planeIndex = 0; planeIndex < result.planes.size(); ++planeIndex)
    {
        const std::size_t axis = planeIndex / 2u;
        const double sign = (planeIndex % 2u) == 0u ? 1.0 : -1.0;
        double coefficients[4]{};
        for (std::size_t component = 0; component < 4; ++component)
        {
            const double row3 = viewProjection[component * 4u + 3u];
            const double axisRow = viewProjection[component * 4u + axis];
            coefficients[component] = row3 + sign * axisRow;
        }

        const double normalLength = std::sqrt(coefficients[0] * coefficients[0]
            + coefficients[1] * coefficients[1] + coefficients[2] * coefficients[2]);
        if (!std::isfinite(normalLength))
            return false;

        Plane3 &plane = result.planes[planeIndex];
        if (normalLength <= std::numeric_limits<double>::min())
        {
            if (coefficients[3] < 0.0)
                return false;
            plane.normal = {0.0f, 0.0f, 0.0f};
            plane.distance = 1.0f; // A non-negative constant plane imposes no spatial constraint.
            continue;
        }

        for (std::size_t component = 0; component < 3; ++component)
        {
            const double normalized = coefficients[component] / normalLength;
            if (!std::isfinite(normalized))
                return false;
            plane.normal[component] = static_cast<float>(normalized);
        }
        const double normalizedDistance = coefficients[3] / normalLength;
        if (!std::isfinite(normalizedDistance)
            || std::abs(normalizedDistance) > static_cast<double>(std::numeric_limits<float>::max()))
            return false;
        plane.distance = static_cast<float>(normalizedDistance);
    }

    if (!result.valid())
        return false;
    *outFrustum = result;
    return true;
}

} // namespace engine::spatial

#endif
