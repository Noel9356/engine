#ifndef ENGINE_CAPABILITIES_LIGHTING_CONTRACT_HPP
#define ENGINE_CAPABILITIES_LIGHTING_CONTRACT_HPP

// Mod-facing contract: implementation-, platform- and vendor-agnostic. Light
// descriptions are plain primitive values; the lighting passes behind this
// contract may be PBR compute passes today and another technique tomorrow.
// Script mods do not call this directly yet (the Lua bridge is the script ABI
// and grows deliberately); C++ mods acquire this capability like any other.

#include <cstdint>

namespace engine
{

// One directional light ("sun"). Direction points FROM the light TOWARDS the
// scene (the vector the light travels along); it is expected normalized.
struct DirectionalLightDescription
{
    float direction[3] = {0.0f, -1.0f, 0.0f};
    float color[3]     = {1.0f, 1.0f, 1.0f};
    float intensity    = 0.0f;
};

// One positional light with inverse-square falloff. radius cuts the light off
// beyond that world-space distance (0 = no cutoff) so scenes can bound the
// number of shaded fragments per light.
struct PointLightDescription
{
    float position[3] = {0.0f, 0.0f, 0.0f};
    float color[3]    = {1.0f, 1.0f, 1.0f};
    float intensity   = 0.0f;
    float radius      = 0.0f;
};

// Capability "lighting": the scene light state. The state is consumed once per
// frame by the lighting passes; mutating it mid-frame is allowed but only the
// frame that reads it afterwards observes the change. Single threaded by the
// engine's threading contract, like the event bus.
class LightingCapability
{
public:
    static constexpr const char *capabilityName = "lighting";

    // Point lights are addressed by handle; handles are stable until removed.
    // The implementation caps the number of live point lights (see the
    // configuration documentation); addPointLight returns 0 when the cap is
    // reached, so callers can react instead of overwriting an existing light.
    using PointLightHandle = std::uint32_t;

    static constexpr PointLightHandle invalidPointLight = 0;

    virtual ~LightingCapability() = default;

    virtual void                        setSunLight(const DirectionalLightDescription &light) = 0;
    virtual void                        clearSunLight()                                       = 0;
    virtual bool                        hasSunLight() const                                   = 0;
    virtual DirectionalLightDescription sunLight() const                                      = 0;

    virtual PointLightHandle      addPointLight(const PointLightDescription &light) = 0;
    virtual void                  removePointLight(PointLightHandle handle)         = 0;
    virtual void                  clearPointLights()                                = 0;
    virtual std::uint32_t         pointLightCount() const                           = 0;
    virtual PointLightDescription pointLight(std::uint32_t index) const             = 0;
};

} // namespace engine

#endif // ENGINE_CAPABILITIES_LIGHTING_CONTRACT_HPP
