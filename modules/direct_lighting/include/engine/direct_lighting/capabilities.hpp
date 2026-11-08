// Public capability surface of the direct_lighting module.
//
// Declares the POD types and function-pointer table consumers receive through
// ParticipantContext::acquireCapability for the "direct_lighting" capability
// (ABI 1): a fixed-capacity registry of directional, point, and spot lights,
// a priority-sorted snapshot copy API for renderers, and a device-based
// profile resolver.

#ifndef ENGINE_DIRECT_LIGHTING_CAPABILITIES_HPP
#define ENGINE_DIRECT_LIGHTING_CAPABILITIES_HPP

#include <engine/renderer/device_profile.hpp>

#include <cstdint>

namespace engine
{
namespace direct_lighting
{

// ABI version of this capability struct; consumers must match it.
inline constexpr std::uint32_t directLightingCapabilityAbi = 1u;
using DirectLightHandle                                    = std::uint32_t;
inline constexpr DirectLightHandle invalidDirectLight      = 0;
// Renderer-facing per-draw limit used by the current fixed shader layout.
// The module registry can hold more lights; copyLights may return a larger
// sorted snapshot when maximum_registered_lights is configured above this cap.
inline constexpr std::uint32_t maxDirectLights = 32u;

// Light kinds: Directional shines from infinity along direction, Point
// radiates from position, Spot additionally cuts a cone between
// innerConeRadians and outerConeRadians.
enum class DirectLightType : std::uint32_t
{
    Directional = 0,
    Point,
    Spot
};

// Status codes written through optional DirectLightingError out-parameters:
// malformed description, unknown handle, or the module/handle space being full.
enum class DirectLightingError : std::uint32_t
{
    None = 0,
    InvalidDescription,
    InvalidHandle,
    CapacityReached
};

// Light parameters. intensity is radiance in arbitrary units, range is the
// falloff distance in world units, cones are radians (outer <= pi/2).
// direction must be non-degenerate for Directional/Spot and is normalized by
// the module on input. layerMask is opaque to the module; priority controls
// the ordering of copyLights output.
struct DirectLightDescription
{
    DirectLightType type             = DirectLightType::Point;
    float           position[3]      = {0.0f, 0.0f, 0.0f};
    float           direction[3]     = {0.0f, -1.0f, 0.0f};
    float           color[3]         = {1.0f, 1.0f, 1.0f};
    float           intensity        = 100.0f;
    float           range            = 10.0f;
    float           innerConeRadians = 0.25f;
    float           outerConeRadians = 0.5f;
    std::int32_t    priority         = 0;

    std::uint32_t layerMask = 0xffffffffu;
    bool          enabled   = true;
};

// Snapshot entry: handle plus the (normalized) description as of the copy.
struct DirectLight
{
    DirectLightHandle      handle = invalidDirectLight;
    DirectLightDescription description;
};

// Renderer budget resolved by resolveProfile: the largest number of lights a
// device should evaluate per draw.
struct DirectLightingProfile
{
    std::uint32_t maxLights = maxDirectLights;
};

// Function-pointer table for the "direct_lighting" capability. Mutators are
// thread-safe (one module mutex); copyLights is the read side. Calls never
// throw; failures are reported via the error out-parameters.
struct DirectLightingCapability
{
    static constexpr const char *capabilityName = "direct_lighting";

    std::uint32_t abi  = directLightingCapabilityAbi;
    void         *self = nullptr;

    DirectLightHandle (*createLight)(void *self, const DirectLightDescription *description, DirectLightingError *error);
    bool (*updateLight)(
        void *self, DirectLightHandle light, const DirectLightDescription *description, DirectLightingError *error);
    bool (*destroyLight)(void *self, DirectLightHandle light);
    bool (*setEnabled)(void *self, DirectLightHandle light, bool enabled);

    // Reports the current version even with a null/zero-capacity buffer, so
    // callers can detect changes before sizing their buffer. Returns the
    // number of lights copied (priority-sorted, truncated to capacity).
    std::uint32_t (*copyLights)(void *self, DirectLight *output, std::uint32_t capacity, std::uint64_t *outVersion);
    bool (*resolveProfile)(void *self, const renderer::DeviceProfile *device, DirectLightingProfile *outProfile);
};

} // namespace direct_lighting
} // namespace engine

#endif
