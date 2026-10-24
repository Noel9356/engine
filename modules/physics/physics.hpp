#ifndef ENGINE_MODULES_PHYSICS_HPP
#define ENGINE_MODULES_PHYSICS_HPP

#include "engine/kernel/bounding_volume_hierarchy.hpp"
#include "logging_capability.hpp"
#include "physics_capability.hpp"
#include "renderer_capability.hpp"
#include "scene_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine
{

// Module "physics": the built-in rigid body provider (family member
// "physics.builtin").
//
// Simulation contract, in one paragraph: the module subscribes to the frame
// event, integrates every dynamic body with semi-implicit Euler under the
// configured gravity, resolves contacts (bodies vs the ground plane, sphere
// vs sphere, sphere vs box) with restitution, and writes the resulting
// transforms back through the scene registry. The registry stays the source
// of truth - a body whose object was removed drops, a resized object resizes
// its collider, and the renderer's incremental rebuild picks every moved
// object up with no physics-specific renderer path.
//
// Collision pair culling runs through the kernel's bounding volume hierarchy
// over the bodies' swept bounds (rebuilt per step; at the registry cap that
// costs microseconds) - the same structure a future CPU SDF bake would use.
class PhysicsModule final : public ModuleInterface, public PhysicsCapability
{
public:
    PhysicsModule();
    ~PhysicsModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // PhysicsCapability.
    BodyHandle                   addDynamicBody(SceneCapability::ObjectHandle objectHandle, const BodyDescription &description)
        override;
    bool                         removeBody(BodyHandle handle) override;
    bool                         setBodyVelocity(BodyHandle handle, const std::array<float, 3> &velocity) override;
    bool                         bodyVelocity(BodyHandle handle, std::array<float, 3> &outVelocity) const override;
    void                         setGravity(const std::array<float, 3> &gravity) override;
    std::array<float, 3>         gravity() const override;
    std::uint32_t                bodyCount() const override;

private:
    struct Body
    {
        BodyHandle                            handle        = invalidBody;
        SceneCapability::ObjectHandle         objectHandle  = SceneCapability::invalidObject;
        BodyDescription                       description{};
        std::array<float, 3>                  velocity      = {0.0f, 0.0f, 0.0f};
    };

    // One simulation step of the given length (clamped internally; a stalled
    // frame must not teleport bodies through geometry).
    void step(
        double seconds
    );
    void forwardFrameBegin(const FrameBeginEvent &event);

    // Contact resolution helpers. All positions come from the registry right
    // before the step, all results go back through it.
    void resolveGroundContact(
        Body &body, const SceneObjectDescriptor &object, const AxisAlignedBoundingBox &worldBounds, double dt
    );
    void resolveSphereSphere(
        Body &first, const SceneObjectDescriptor &firstObject,
        Body &second, const SceneObjectDescriptor &secondObject
    );
    void resolveSphereBox(
        Body &sphereBody, const SceneObjectDescriptor &sphereObject,
        Body &boxBody, const SceneObjectDescriptor &boxObject
    );
    // Broadphase (kernel BVH) + narrowphase over all live body pairs.
    void resolveBodyPairs();

    ComponentManifest   componentManifest;
    ComponentContext   *boundContext    = nullptr;
    LoggerCapability   *logger          = nullptr;
    SceneCapability    *sceneCapability = nullptr;

    EventSubscription   frameBeginSubscription;
    EventChannel<FrameBeginEvent> *frameBeginChannel = nullptr;

    std::unordered_map<BodyHandle, Body> bodies;
    BodyHandle                           nextHandle = invalidBody + 1;

    // Simulation parameters (the [physics] configuration section; defaults
    // are earth gravity and a ground plane at y=0).
    std::array<float, 3> gravity_        = {0.0f, -9.81f, 0.0f};
    bool                 groundEnabled   = true;
    float                groundHeight    = 0.0f;
    float                maxStepSeconds  = 1.0f / 30.0f;

    std::chrono::steady_clock::time_point lastTick{};
    bool                                  clockSeeded = false;
};

} // namespace engine

#endif // ENGINE_MODULES_PHYSICS_HPP
