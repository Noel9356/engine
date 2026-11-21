// First-person surface walk controller for the planet demo.
//
// Owns the planet module's SurfaceWalker plus the demo's view orientation
// (yaw/pitch around the walker's local up), turning camera-style input into
// walk intents and producing the camera pose each frame. In auto mode it
// walks a deterministic path (fixed forward intent, gentle constant turn) so
// headless captures exercise the same code as interactive play.
#pragma once

#include <engine/camera/capabilities.hpp>
#include <engine/planet/capabilities.hpp>
#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include <engine/planet/walker.hpp>

#include <glm/glm.hpp>

namespace demo::planet_demo
{

class PlanetWalkController final
{
public:
    PlanetWalkController(const engine::planet::PlanetGenerationParams &params,
        const glm::dvec3 &planetCenter);

    // Places the walker on the terrain top at `direction` (planet-local unit
    // vector from the planet center) and resets the view to look along the
    // horizon tangent.
    void spawn(const engine::planet::Vec3 &unitDirection);

    // Advances the walk one frame from a camera input sample (mouse look +
    // WASD + Space jump + Shift sprint). `enabled == false` freezes input
    // (zero movement) while the walker still settles under gravity.
    void update(const engine::camera::CameraInputState &input, bool enabled, double deltaSeconds);

    // Deterministic automatic walk used by --orbit captures.
    void setAutoWalk(bool enabled)
    {
        autoWalk_ = enabled;
    }

    [[nodiscard]] const engine::planet::WalkState &state() const
    {
        return walker_.state();
    }

    // Camera pose derived from the walk state and the view orientation.
    [[nodiscard]] glm::dvec3 eye() const;
    [[nodiscard]] glm::dvec3 target() const;
    [[nodiscard]] glm::dvec3 up() const
    {
        return glm::dvec3(state().up.x, state().up.y, state().up.z);
    }

    [[nodiscard]] bool isSwimming() const
    {
        return state().swimming;
    }

private:
    void clampPitch();

    engine::planet::PlanetGenerationParams params_{};
    glm::dvec3  planetCenter_{0.0};
    engine::planet::DensityField field_;
    engine::planet::SurfaceWalker walker_;

    glm::dvec3 viewForward_{0.0, 0.0, 1.0}; // world-space look direction, re-orthogonalized each frame
    double     autoWalkYaw_ = 0.0;
    bool       autoWalk_    = false;
};

} // namespace demo::planet_demo

