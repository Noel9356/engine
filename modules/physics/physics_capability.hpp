#ifndef ENGINE_MODULES_PHYSICS_PHYSICS_CAPABILITY_HPP
#define ENGINE_MODULES_PHYSICS_PHYSICS_CAPABILITY_HPP

// The published capability surface of the physics module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic: bodies are
// plain values, the simulation contract is numbers in and numbers out.
//
// Provider model: this header is the family contract ("physics"). The
// built-in provider in this module is the implementation-bound member
// "physics.builtin" - a compact rigid body simulator sized for the engine's
// analytic scene (spheres, boxes, a ground plane). A full engine provider
// (for example one wrapping Jolt) declares its own member of the family and
// wins through the [providers] configuration section; mods keep demanding
// "physics" and notice nothing but the behavior.

#include "scene_capability.hpp"

#include <array>
#include <cstdint>

namespace engine
{

// Capability "physics": rigid bodies driving scene registry objects.
//
// The contract binds bodies to scene objects: a body STEPS and writes its
// transform back through the scene registry, so the renderer, the animation
// module and every script see moved objects exactly like any other scene
// mutation. A body does not own geometry - the bound object's kind (sphere
// radius / box half extents) IS the collision shape, read live from the
// registry: resize the object and the collider follows.
class PhysicsCapability
{
public:
    static constexpr const char *capabilityName = "physics";

    using BodyHandle = std::uint32_t;

    static constexpr BodyHandle invalidBody = 0;

    virtual ~PhysicsCapability() = default;

    // One body's tunables. Every field has a sane default; restitution 0.4
    // with the built-in provider's default gravity gives a believable bounce
    // without tuning.
    struct BodyDescription
    {
        float restitution   = 0.4f; // bounce factor on contact (0 = dead stop, 1 = perfect bounce)
        float linearDamping = 0.02f; // velocity decay per second
        bool  startsAsleep  = false; // unused by the built-in provider (it has no sleep states yet)
    };

    // Binds one dynamic body to a live scene registry object. The object
    // must exist (the registry is the source of truth); returns invalidBody
    // for unknown objects. The body's initial state: the object's current
    // transform, zero velocity.
    virtual BodyHandle addDynamicBody(
        SceneCapability::ObjectHandle objectHandle, const BodyDescription &description
    ) = 0;

    // Unbinds one body. The scene object stays (removing the object is the
    // registry's job); returns false for unknown handles.
    virtual bool removeBody(
        BodyHandle handle
    ) = 0;

    // Direct velocity control (impulses are velocity deltas away).
    virtual bool setBodyVelocity(
        BodyHandle handle, const std::array<float, 3> &velocity
    ) = 0;

    virtual bool bodyVelocity(
        BodyHandle handle, std::array<float, 3> &outVelocity
    ) const = 0;

    // World gravity, shared by every dynamic body. The built-in default is
    // earth gravity.
    virtual void setGravity(
        const std::array<float, 3> &gravity
    ) = 0;

    virtual std::array<float, 3> gravity() const = 0;

    virtual std::uint32_t bodyCount() const = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_PHYSICS_PHYSICS_CAPABILITY_HPP
