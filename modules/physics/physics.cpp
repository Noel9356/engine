#include "physics.hpp"

#include <algorithm>
#include <cmath>

namespace engine
{

namespace
{

// The world-space AABB of one registry object (spheres use their radius).
AxisAlignedBoundingBox objectBounds(
    const SceneObjectDescriptor &object
)
{
    AxisAlignedBoundingBox bounds;
    const float            radius = object.size[0];
    if (object.type == scene_object_type::sphere)
    {
        bounds.minPoint = {object.position[0] - radius, object.position[1] - radius, object.position[2] - radius};
        bounds.maxPoint = {object.position[0] + radius, object.position[1] + radius, object.position[2] + radius};
    }
    else
    {
        // Axis-aligned approximation: the box's rotation is ignored for
        // contact purposes (documented limitation - rotated box colliders
        // need the full OBB path a heavier provider brings).
        bounds.minPoint = {object.position[0] - object.size[0], object.position[1] - object.size[1],
                           object.position[2] - object.size[2]};
        bounds.maxPoint = {object.position[0] + object.size[0], object.position[1] + object.size[1],
                           object.position[2] + object.size[2]};
    }
    return bounds;
}

float clampf(
    float value, float low, float high
)
{
    return std::clamp(value, low, high);
}

} // namespace

PhysicsModule::PhysicsModule()
{
    componentManifest.name = "physics";
    componentManifest.kind = ComponentKind::Module;
    // The implementation-bound member name of the "physics" family: a heavier
    // provider (Jolt, for example) declares its own member and wins through
    // the [providers] configuration; mods keep demanding the family.
    componentManifest.providedCapabilities = {"physics.builtin"};
    componentManifest.requiredCapabilities = {"logging", "scene"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.scene"};
    componentManifest.knownConfigKeys      = {
        "physics.gravity",
        "physics.ground_enabled",
        "physics.ground_height",
        "physics.max_step_seconds",
    };
}

PhysicsModule::~PhysicsModule()
{
    stop();
}

void PhysicsModule::construct(
    ComponentContext &context
)
{
    boundContext    = &context;
    logger          = &context.acquireCapability<LoggerCapability>();
    sceneCapability = &context.acquireCapability<SceneCapability>();

    const std::vector<double> configuredGravity = context.configurationFloatingArrayInSection("physics", "gravity");
    if (configuredGravity.size() == 3)
    {
        gravity_ = {static_cast<float>(configuredGravity[0]), static_cast<float>(configuredGravity[1]),
                    static_cast<float>(configuredGravity[2])};
    }
    groundEnabled = context.configurationBooleanInSection("physics", "ground_enabled").value_or(true);
    groundHeight  = static_cast<float>(context.configurationFloatingInSection("physics", "ground_height").value_or(0.0));
    maxStepSeconds =
        static_cast<float>(context.configurationFloatingInSection("physics", "max_step_seconds").value_or(1.0 / 30.0));

    EventBus                      &bus           = context.eventBus();
    frameBeginChannel                            = &bus.channel<FrameBeginEvent>();
    frameBeginSubscription                       = EventSubscription(
        &bus, typeid(FrameBeginEvent), frameBeginChannel->subscribe<PhysicsModule, &PhysicsModule::forwardFrameBegin>(this)
    );
}

void PhysicsModule::start()
{
    logger->logMessage(
        LogLevel::Information,
        "physics module started (builtin provider, gravity [" + std::to_string(gravity_[0]) + " "
            + std::to_string(gravity_[1]) + " " + std::to_string(gravity_[2]) + "], ground "
            + (groundEnabled ? "enabled" : "disabled") + ")"
    );
}

void PhysicsModule::stop()
{
    frameBeginSubscription.release();
    bodies.clear();
    boundContext    = nullptr;
    logger          = nullptr;
    sceneCapability = nullptr;
}

void *PhysicsModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "physics.builtin")
    {
        return static_cast<PhysicsCapability *>(this);
    }
    return nullptr;
}

PhysicsCapability::BodyHandle PhysicsModule::addDynamicBody(
    SceneCapability::ObjectHandle objectHandle, const BodyDescription &description
)
{
    SceneObjectDescriptor object;
    if (objectHandle == SceneCapability::invalidObject || !sceneCapability->object(objectHandle, object))
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning,
                "physics module: addDynamicBody refused (object " + std::to_string(objectHandle) + " is not live)"
            );
        }
        return invalidBody;
    }
    Body body;
    body.handle       = nextHandle++;
    body.objectHandle = objectHandle;
    body.description  = description;
    bodies.emplace(body.handle, body);
    if (logger != nullptr)
    {
        logger->logMessage(
            LogLevel::Debug,
            "physics module: body " + std::to_string(body.handle) + " bound to object " + std::to_string(objectHandle)
        );
    }
    return body.handle;
}

bool PhysicsModule::removeBody(
    BodyHandle handle
)
{
    return bodies.erase(handle) != 0;
}

bool PhysicsModule::setBodyVelocity(
    BodyHandle handle, const std::array<float, 3> &velocity
)
{
    const auto found = bodies.find(handle);
    if (found == bodies.end())
    {
        return false;
    }
    found->second.velocity = velocity;
    return true;
}

bool PhysicsModule::bodyVelocity(
    BodyHandle handle, std::array<float, 3> &outVelocity
) const
{
    const auto found = bodies.find(handle);
    if (found == bodies.end())
    {
        return false;
    }
    outVelocity = found->second.velocity;
    return true;
}

void PhysicsModule::setGravity(
    const std::array<float, 3> &gravity
)
{
    gravity_ = gravity;
}

std::array<float, 3> PhysicsModule::gravity() const
{
    return gravity_;
}

std::uint32_t PhysicsModule::bodyCount() const
{
    return static_cast<std::uint32_t>(bodies.size());
}

void PhysicsModule::forwardFrameBegin(
    const FrameBeginEvent & /*event*/
)
{
    const auto now = std::chrono::steady_clock::now();
    if (!clockSeeded)
    {
        clockSeeded = true;
        lastTick    = now;
        return;
    }
    const double frameDelta = std::chrono::duration<double>(now - lastTick).count();
    lastTick                = now;
    // Clamp, then step in slices: a stalled frame must not teleport bodies
    // through geometry, and a 1/30 s cap keeps the analytic contacts stable
    // without a substep tuning story.
    const double clamped = std::min(frameDelta, static_cast<double>(maxStepSeconds));
    if (clamped > 0.0)
    {
        step(clamped);
    }
}

void PhysicsModule::step(
    double seconds
)
{
    if (bodies.empty())
    {
        return;
    }
    // Snapshot the live state of every bound object, integrate, resolve,
    // write back. A body whose object vanished mid-step (removed by a script
    // between frames) is dropped.
    std::vector<BodyHandle> deadHandles;
    for (auto &entry : bodies)
    {
        Body            &body   = entry.second;
        SceneObjectDescriptor object;
        if (!sceneCapability->object(body.objectHandle, object))
        {
            deadHandles.push_back(body.handle);
            continue;
        }

        const double dt = seconds;
        // Semi-implicit Euler: velocity first, then position - the stable
        // order for gravity-driven motion under restitution.
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            body.velocity[axis] += gravity_[axis] * static_cast<float>(dt);
            body.velocity[axis] *= 1.0f - body.description.linearDamping * static_cast<float>(dt);
            object.position[axis] += body.velocity[axis] * static_cast<float>(dt);
        }

        AxisAlignedBoundingBox bounds = objectBounds(object);
        resolveGroundContact(body, object, bounds, dt);

        sceneCapability->setObjectTransform(body.objectHandle, object.position, object.rotationYRadians);
    }
    for (BodyHandle handle : deadHandles)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Debug,
                "physics module: body " + std::to_string(handle) + " dropped (its scene object was removed)"
            );
        }
        bodies.erase(handle);
    }

    resolveBodyPairs();
}

void PhysicsModule::resolveGroundContact(
    Body &body, const SceneObjectDescriptor &object, const AxisAlignedBoundingBox &worldBounds, double dt
)
{
    (void) dt;
    if (!groundEnabled)
    {
        return;
    }
    // Penetration of the object's lowest point into the ground plane.
    const float penetration = groundHeight - worldBounds.minPoint[1];
    if (penetration <= 0.0f)
    {
        return;
    }
    // Push out along Y, reflect the vertical velocity per restitution, damp
    // the tangential velocity a touch (the built-in provider's stand-in for
    // friction - documented as such).
    std::array<float, 3> position = object.position;
    position[1] += penetration;
    if (body.velocity[1] < 0.0f)
    {
        body.velocity[1] = -body.velocity[1] * body.description.restitution;
        if (std::abs(body.velocity[1]) < 0.15f)
        {
            body.velocity[1] = 0.0f; // rest: no micro-bouncing forever
        }
        body.velocity[0] *= 0.98f;
        body.velocity[2] *= 0.98f;
    }
    sceneCapability->setObjectTransform(body.objectHandle, position, object.rotationYRadians);
}

void PhysicsModule::resolveBodyPairs()
{
    if (bodies.size() < 2)
    {
        return;
    }
    // Broadphase: one BVH over the bodies' current world bounds, then the
    // narrowphase on the surviving pairs. The BVH is rebuilt per step (at the
    // registry cap: microseconds) - the kernel structure keeps the pair loop
    // near-linear without any tuning surface.
    std::vector<AxisAlignedBoundingBox> bounds;
    std::vector<BodyHandle>             handles;
    bounds.reserve(bodies.size());
    handles.reserve(bodies.size());
    for (auto &entry : bodies)
    {
        SceneObjectDescriptor object;
        if (sceneCapability->object(entry.second.objectHandle, object))
        {
            bounds.push_back(objectBounds(object));
            handles.push_back(entry.first);
        }
    }
    BoundingVolumeHierarchy   bvh;
    bvh.build(bounds);

    std::vector<std::uint32_t> neighbors;
    for (std::size_t index = 0; index < handles.size(); ++index)
    {
        neighbors.clear();
        bvh.queryOverlaps(bounds[index], neighbors);
        for (std::uint32_t neighborIndex : neighbors)
        {
            if (static_cast<std::size_t>(neighborIndex) <= index)
            {
                continue; // each unordered pair once
            }
            Body            *first  = &bodies[handles[index]];
            Body            *second = &bodies[handles[neighborIndex]];
            SceneObjectDescriptor firstObject;
            SceneObjectDescriptor secondObject;
            if (!sceneCapability->object(first->objectHandle, firstObject)
                || !sceneCapability->object(second->objectHandle, secondObject))
            {
                continue;
            }
            const bool firstIsSphere  = firstObject.type == scene_object_type::sphere;
            const bool secondIsSphere = secondObject.type == scene_object_type::sphere;
            if (firstIsSphere && secondIsSphere)
            {
                resolveSphereSphere(*first, firstObject, *second, secondObject);
            }
            else if (firstIsSphere != secondIsSphere)
            {
                if (firstIsSphere)
                {
                    resolveSphereBox(*first, firstObject, *second, secondObject);
                }
                else
                {
                    resolveSphereBox(*second, secondObject, *first, firstObject);
                }
            }
            // Box vs box is out of scope for the built-in provider (rotated
            // boxes need the full OBT path); the broadphase simply skips the
            // pair and the objects pass through each other.
        }
    }
}

void PhysicsModule::resolveSphereSphere(
    Body &first, const SceneObjectDescriptor &firstObject,
    Body &second, const SceneObjectDescriptor &secondObject
)
{
    const float radiusSum = firstObject.size[0] + secondObject.size[0];
    std::array<float, 3> delta = {secondObject.position[0] - firstObject.position[0],
                                  secondObject.position[1] - firstObject.position[1],
                                  secondObject.position[2] - firstObject.position[2]};
    const float distance = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    if (distance >= radiusSum || distance < 1e-6f)
    {
        return;
    }
    const std::array<float, 3> normal = {delta[0] / distance, delta[1] / distance, delta[2] / distance};
    const float                penetration = radiusSum - distance;

    // Split the positional correction between the two bodies, then resolve
    // the relative velocity along the normal with restitution.
    const std::array<float, 3> firstPosition = {firstObject.position[0] - normal[0] * penetration * 0.5f,
                                                firstObject.position[1] - normal[1] * penetration * 0.5f,
                                                firstObject.position[2] - normal[2] * penetration * 0.5f};
    const std::array<float, 3> secondPosition = {secondObject.position[0] + normal[0] * penetration * 0.5f,
                                                 secondObject.position[1] + normal[1] * penetration * 0.5f,
                                                 secondObject.position[2] + normal[2] * penetration * 0.5f};
    const float relativeNormalVelocity = (second.velocity[0] - first.velocity[0]) * normal[0]
                                       + (second.velocity[1] - first.velocity[1]) * normal[1]
                                       + (second.velocity[2] - first.velocity[2]) * normal[2];
    if (relativeNormalVelocity < 0.0f)
    {
        const float restitution = std::min(first.description.restitution, second.description.restitution);
        const float impulse     = -(1.0f + restitution) * relativeNormalVelocity * 0.5f;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            first.velocity[axis] -= normal[axis] * impulse;
            second.velocity[axis] += normal[axis] * impulse;
        }
    }
    sceneCapability->setObjectTransform(first.objectHandle, firstPosition, firstObject.rotationYRadians);
    sceneCapability->setObjectTransform(second.objectHandle, secondPosition, secondObject.rotationYRadians);
}

void PhysicsModule::resolveSphereBox(
    Body &sphereBody, const SceneObjectDescriptor &sphereObject,
    Body &boxBody, const SceneObjectDescriptor &boxObject
)
{
    const AxisAlignedBoundingBox boxBounds = objectBounds(boxObject);
    // Closest point on the box to the sphere center (axis-aligned).
    const std::array<float, 3> closest = {clampf(sphereObject.position[0], boxBounds.minPoint[0], boxBounds.maxPoint[0]),
                                          clampf(sphereObject.position[1], boxBounds.minPoint[1], boxBounds.maxPoint[1]),
                                          clampf(sphereObject.position[2], boxBounds.minPoint[2], boxBounds.maxPoint[2])};
    std::array<float, 3> delta = {sphereObject.position[0] - closest[0],
                                  sphereObject.position[1] - closest[1],
                                  sphereObject.position[2] - closest[2]};
    const float distance = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    const float radius   = sphereObject.size[0];
    if (distance >= radius || distance < 1e-6f)
    {
        return;
    }
    // Normal from the box surface towards the sphere center; the degenerate
    // case (center inside the box) pushes along the shallowest exit axis.
    std::array<float, 3> normal{};
    if (distance > 1e-4f)
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            normal[axis] = delta[axis] / distance;
        }
    }
    else
    {
        // Inside: exit through the nearest face.
        std::size_t shallowest = 0;
        float       bestDepth  = 1e9f;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const float toMin  = sphereObject.position[axis] - boxBounds.minPoint[axis];
            const float toMax  = boxBounds.maxPoint[axis] - sphereObject.position[axis];
            const float depth  = std::min(toMin, toMax);
            if (depth < bestDepth)
            {
                bestDepth  = depth;
                shallowest = axis;
            }
        }
        normal[shallowest] = sphereObject.position[shallowest] - (boxBounds.minPoint[shallowest]
                                                                + boxBounds.maxPoint[shallowest]) * 0.5f > 0.0f
                                 ? 1.0f
                                 : -1.0f;
    }
    const float penetration = radius - distance;

    std::array<float, 3> spherePosition = sphereObject.position;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        spherePosition[axis] += normal[axis] * penetration;
    }
    const float relativeNormalVelocity = (boxBody.velocity[0] - sphereBody.velocity[0]) * normal[0]
                                       + (boxBody.velocity[1] - sphereBody.velocity[1]) * normal[1]
                                       + (boxBody.velocity[2] - sphereBody.velocity[2]) * normal[2];
    if (relativeNormalVelocity < 0.0f)
    {
        const float restitution = std::min(sphereBody.description.restitution, boxBody.description.restitution);
        const float impulse     = -(1.0f + restitution) * relativeNormalVelocity * 0.5f;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            sphereBody.velocity[axis] -= normal[axis] * impulse;
            boxBody.velocity[axis] += normal[axis] * impulse;
        }
    }
    sceneCapability->setObjectTransform(sphereBody.objectHandle, spherePosition, sphereObject.rotationYRadians);
}

} // namespace engine
