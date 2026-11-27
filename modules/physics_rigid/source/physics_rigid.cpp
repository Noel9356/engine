// physics_rigid module implementation.
//
// The module owns the handle-keyed worlds and exposes the
// PhysicsRigidCapability function table. Simulation mechanics live in
// dedicated translation units (math.hpp, shapes, broadphase, solver, world)
// so this file stays focused on lifecycle, configuration, and capability
// dispatch.

#include "broadphase.hpp"
#include "math.hpp"
#include "records.hpp"
#include "shapes.hpp"
#include "solver.hpp"
#include "world.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/step_accumulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_rigid
{
namespace
{

// Fills a public shape description from the module's runtime shape record.
void fillShapeDescription(const Shape &shape, ShapeDescription &outDescription)
{
    outDescription.kind                = shape.kind;
    outDescription.sphereRadius        = shape.sphereRadius;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        outDescription.boxHalfExtents[axis] = shape.boxHalfExtents[axis];
    }
    outDescription.capsuleRadius       = shape.capsuleRadius;
    outDescription.capsuleHalfLength   = shape.capsuleHalfLength;
    outDescription.cylinderRadius      = shape.cylinderRadius;
    outDescription.cylinderHalfHeight  = shape.cylinderHalfHeight;
    outDescription.planeNormal[0]      = shape.planeNormal.x;
    outDescription.planeNormal[1]      = shape.planeNormal.y;
    outDescription.planeNormal[2]      = shape.planeNormal.z;
    outDescription.planeOffset         = shape.planeOffset;
}

// Fills a public body description from the module's runtime body record.
void fillBodyDescriptionFromRecord(const Body &body, BodyDescription &outDescription)
{
    fillShapeDescription(body.shape, outDescription.shape);
    outDescription.transform                = body.transform;
    outDescription.kind                     = body.kind;
    outDescription.density                  = body.densitySource;
    outDescription.mass                     = body.densitySource > 0.0 ? 0.0 : body.explicitMassSource;
    outDescription.material                 = body.material;
    outDescription.linearDamping            = body.linearDamping;
    outDescription.angularDamping           = body.angularDamping;
    outDescription.sleepingEnabled          = body.sleepingEnabled;
    outDescription.gravityEnabled           = body.gravityEnabled;
    outDescription.categoryBits             = body.categoryBits;
    outDescription.maskBits                 = body.maskBits;
    outDescription.userTag                  = body.userTag;
    outDescription.continuousCollisionEnabled = body.continuousCollisionEnabled;
}

class PhysicsRigidModule final : public ModuleInterface
{
public:
    PhysicsRigidModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        readLimits(context);
        readSolverSettings(context);
        readSleepSettings(context);
        readSpaceSettings(context);
        readLodSettings(context);

        if (const auto gravity = context.configurationFloatingArrayInSection("physics_rigid", "gravity");
            gravity.size() == 3)
        {
            settings_.gravity = math::Vec3{gravity[0], gravity[1], gravity[2]};
            if (!math::isFinite(settings_.gravity))
            {
                settings_.gravity = math::Vec3{0.0, -9.81, 0.0};
            }
        }

        worlds_.reserve(settings_.maximumWorlds);
    }

    void start() override
    {}

    void stop() override
    {
        worlds_.clear();
        worldIndexByHandle_.clear();
        nextWorldHandle_ = 1;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == PhysicsRigidCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_rigid",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_rigid"}};
    }

    template <typename Target>
    static void readPositiveLimit(ParticipantContext &context, const char *key, std::int64_t maximum, Target &target)
    {
        if (const auto value = context.configurationIntegerInSection("physics_rigid", key);
            value && *value > 0 && *value <= maximum)
        {
            target = static_cast<Target>(*value);
        }
    }

    void readLimits(ParticipantContext &context)
    {
        readPositiveLimit(context, "maximum_worlds", 4096, settings_.maximumWorlds);
        readPositiveLimit(context, "maximum_bodies", 10'000'000, settings_.maximumBodies);
        readPositiveLimit(context, "maximum_joints", 10'000'000, settings_.maximumJoints);
        readPositiveLimit(context, "maximum_triggers", 100'000, settings_.maximumTriggers);
        readPositiveLimit(context, "maximum_trigger_events", 1'000'000, settings_.maximumTriggerEvents);
        readPositiveLimit(context, "maximum_steps_per_simulate", 1000, settings_.maximumStepsPerSimulate);
        readPositiveLimit(context, "maximum_substeps", 64, settings_.maximumSubsteps);
    }

    void readSolverSettings(ParticipantContext &context)
    {
        readPositiveLimit(context, "solver_velocity_iterations", 1000, settings_.velocityIterations);
        readPositiveLimit(context, "solver_position_iterations", 1000, settings_.positionIterations);
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "contact_margin");
            value && *value > 0.0 && *value <= 0.5 && std::isfinite(*value))
            settings_.contactMargin = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "contact_slop");
            value && *value > 0.0 && *value < 1.0 && std::isfinite(*value))
            settings_.contactSlop = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "joint_baumgarte_factor");
            value && *value > 0.0 && *value <= 1.0 && std::isfinite(*value))
            settings_.jointBaumgarteFactor = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "position_correction_factor");
            value && *value > 0.0 && *value <= 1.0 && std::isfinite(*value))
            settings_.positionCorrectionFactor = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "maximum_linear_correction");
            value && *value > 0.0 && *value < 10.0 && std::isfinite(*value))
            settings_.maximumLinearCorrection = *value;
        if (const auto value
            = context.configurationFloatingInSection("physics_rigid", "restitution_velocity_threshold");
            value && *value >= 0.0 && *value < 100.0 && std::isfinite(*value))
            settings_.restitutionVelocityThreshold = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "maximum_linear_velocity");
            value && *value >= 1.0 && std::isfinite(*value))
            settings_.maximumLinearVelocity = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "maximum_angular_velocity");
            value && *value >= 1.0 && std::isfinite(*value))
            settings_.maximumAngularVelocity = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "fixed_rate_hz");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.fixedRateHz = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "substep_maximum_delta");
            value && *value >= 0.0 && *value <= 0.1 && std::isfinite(*value))
            settings_.substepMaximumDelta = *value;
        settings_.warmStarting = context.configurationBooleanInSection("physics_rigid", "warm_starting").value_or(true);
    }

    void readSleepSettings(ParticipantContext &context)
    {
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "linear_sleep_tolerance");
            value && *value > 0.0 && std::isfinite(*value))
            settings_.linearSleepTolerance = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "angular_sleep_tolerance");
            value && *value > 0.0 && std::isfinite(*value))
            settings_.angularSleepTolerance = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "time_to_sleep");
            value && *value >= 0.0 && *value <= 60.0 && std::isfinite(*value))
            settings_.timeToSleep = *value;
    }

    void readSpaceSettings(ParticipantContext &context)
    {
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "spatial_hash_cell_size");
            value && *value > 1.0e-4 && *value <= 1.0e6 && std::isfinite(*value))
            settings_.spatialHashCellSize = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rigid", "ccd_velocity_threshold");
            value && *value >= 0.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.ccdVelocityThreshold = *value;
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_rigid");
        // Per-band solver iteration budgets: five values, one per fidelity
        // band (Cached, Animation, Procedural, Simplified, Full). A 0 entry
        // keeps the derived budget (Full keeps the configured iterations,
        // each lower band halves, floored at one); explicit values clamp to
        // [1, configured iterations] at world creation.
        const auto readBudget = [&context](const char *key, std::uint32_t *targets, std::uint32_t maximum) {
            const auto values = context.configurationFloatingArrayInSection("physics_rigid", key);
            if (values.size() < engine::util::kSimulationFidelityBandCount)
            {
                return;
            }
            for (std::uint32_t band = 0; band < engine::util::kSimulationFidelityBandCount; ++band)
            {
                const long long value = static_cast<long long>(values[band]);
                if (value < 0 || value > static_cast<long long>(maximum))
                {
                    continue;
                }
                targets[band] = static_cast<std::uint32_t>(value);
            }
        };
        readBudget("lod_velocity_iterations", settings_.lodVelocityIterations, settings_.velocityIterations);
        readBudget("lod_position_iterations", settings_.lodPositionIterations, settings_.positionIterations);
    }

    static void setError(PhysicsError *output, PhysicsError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    World *findWorld(WorldHandle handle)
    {
        const auto iterator = worldIndexByHandle_.find(handle);
        return iterator != worldIndexByHandle_.end() ? &worlds_[iterator->second] : nullptr;
    }

    // ---- capability implementations ---------------------------------------

    WorldHandle createWorldImpl(const WorldDescription *description, PhysicsError *error)
    {
        setError(error, PhysicsError::None);
        if (description == nullptr)
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidWorld;
        }
        if (worlds_.size() >= settings_.maximumWorlds)
        {
            setError(error, PhysicsError::CapacityExceeded);
            return invalidWorld;
        }
        const WorldDescription validated
            = description->useConfiguredGravity ? *description : WorldDescription{description->gravity, false};
        if (!math::isFinite(validated.gravity))
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidWorld;
        }
        World *world = createWorld(worlds_, worldIndexByHandle_, nextWorldHandle_, settings_, validated);
        if (world == nullptr)
        {
            setError(error, PhysicsError::CapacityExceeded);
            return invalidWorld;
        }
        return world->handle;
    }

    void destroyWorldImpl(WorldHandle handle)
    {
        const auto iterator = worldIndexByHandle_.find(handle);
        if (iterator == worldIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        worldIndexByHandle_.erase(iterator);
        if (index + 1 != worlds_.size())
        {
            std::swap(worlds_[index], worlds_.back());
            worldIndexByHandle_[worlds_[index].handle] = index;
        }
        worlds_.pop_back();
    }

    bool setWorldGravityImpl(WorldHandle handle, const double gravity[3])
    {
        World *world = findWorld(handle);
        if (world == nullptr || gravity == nullptr)
        {
            return false;
        }
        const math::Vec3 candidate{gravity[0], gravity[1], gravity[2]};
        if (!math::isFinite(candidate))
        {
            return false;
        }
        world->gravity = candidate;
        return true;
    }

    bool copyWorldGravityImpl(WorldHandle handle, double outGravity[3])
    {
        World *world = findWorld(handle);
        if (world == nullptr || outGravity == nullptr)
        {
            return false;
        }
        outGravity[0] = world->gravity.x;
        outGravity[1] = world->gravity.y;
        outGravity[2] = world->gravity.z;
        return true;
    }

    BodyHandle createBodyImpl(WorldHandle worldHandle, const BodyDescription *description, PhysicsError *error)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            setError(error, PhysicsError::InvalidHandle);
            return invalidBody;
        }
        if (description == nullptr)
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidBody;
        }
        Body body;
        if (!validateBodyDescription(*world, world->settings, *description, body, error))
        {
            return invalidBody;
        }
        if (nextBodyHandle_ == invalidBody)
        {
            setError(error, PhysicsError::CapacityExceeded);
            return invalidBody;
        }
        body.handle     = nextBodyHandle_;
        nextBodyHandle_ = nextBodyHandle_ == std::numeric_limits<BodyHandle>::max() ? invalidBody : nextBodyHandle_ + 1;
        world->bodies.push_back(std::move(body));
        world->bodyIndexByHandle.emplace(world->bodies.back().handle, world->bodies.size() - 1);
        if (world->bodies.back().shape.kind == ShapeKind::Plane)
        {
            world->planeBodyHandles.push_back(world->bodies.back().handle);
        }
        setError(error, PhysicsError::None);
        return world->bodies.back().handle;
    }

    void destroyBodyImpl(WorldHandle worldHandle, BodyHandle handle)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return;
        }
        destroyBodyFromWorld(*world, handle);
    }

    std::uint32_t bodyCountImpl(WorldHandle worldHandle)
    {
        World *world = findWorld(worldHandle);
        return world != nullptr ? static_cast<std::uint32_t>(world->bodies.size()) : 0u;
    }

    bool copyBodyHandlesImpl(
        WorldHandle worldHandle, BodyHandle *outHandles, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        if (outRequired != nullptr)
        {
            *outRequired = static_cast<std::uint32_t>(world->bodies.size());
        }
        if (outHandles == nullptr || capacity < world->bodies.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < world->bodies.size(); ++index)
        {
            outHandles[index] = world->bodies[index].handle;
        }
        return true;
    }

    bool copyBodyDescriptionImpl(WorldHandle worldHandle, BodyHandle handle, BodyDescription *outDescription)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outDescription == nullptr)
        {
            return false;
        }
        const Body *body = [&]()
        {
            const auto iterator = world->bodyIndexByHandle.find(handle);
            return iterator != world->bodyIndexByHandle.end() ? &world->bodies[iterator->second] : nullptr;
        }();
        if (body == nullptr)
        {
            return false;
        }
        BodyDescription description;
        fillBodyDescriptionFromRecord(*body, description);
        *outDescription = std::move(description);
        return true;
    }

    bool copyBodyStateImpl(WorldHandle worldHandle, BodyHandle handle, BodyState *outState)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outState == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        const Body &body          = world->bodies[iterator->second];
        outState->transform       = body.transform;
        outState->linearVelocity  = body.linearVelocity;
        outState->angularVelocity = body.angularVelocity;
        outState->awake           = body.awake || body.kind != BodyKind::Dynamic;
        return true;
    }

    // Mass properties of one body: total mass and the local-frame principal
    // inertia diagonal (zeros for static and kinematic bodies).
    bool copyBodyMassPropertiesImpl(
        WorldHandle worldHandle, BodyHandle handle, BodyMassProperties *outMass)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outMass == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        const Body &body = world->bodies[iterator->second];
        *outMass         = BodyMassProperties{};
        outMass->mass    = body.mass;
        outMass->inertiaLocal = body.inertiaLocal;
        return true;
    }

    bool setBodyTransformImpl(WorldHandle worldHandle, BodyHandle handle, const RigidTransform *transform, bool wake)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || transform == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body &body = world->bodies[iterator->second];
        if (!math::isFinite(transform->position) || !math::isFinite(transform->rotation))
        {
            return false;
        }
        const math::Quaternion rotation = math::normalize(transform->rotation);
        if (math::dot(rotation, rotation) < 1.0e-12)
        {
            return false;
        }
        body.transform.position = transform->position;
        body.transform.rotation = rotation;
        refreshBodyAabb(body);
        if (wake)
        {
            wakeRecord(body);
        }
        else if (!body.awake && body.kind == BodyKind::Dynamic)
        {
            body.linearVelocity  = math::Vec3{};
            body.angularVelocity = math::Vec3{};
        }
        return true;
    }

    bool setBodyKindImpl(WorldHandle worldHandle, BodyHandle handle, BodyKind kind)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body &body = world->bodies[iterator->second];
        if (body.shape.kind == ShapeKind::Plane && kind != BodyKind::Static)
        {
            return false;
        }
        body.kind = kind;
        recomputeMassProperties(body);
        wakeRecord(body);
        return true;
    }

    bool setBodyLinearVelocityImpl(WorldHandle worldHandle, BodyHandle handle, const double velocity[3])
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || velocity == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body            &body = world->bodies[iterator->second];
        const math::Vec3 candidate{velocity[0], velocity[1], velocity[2]};
        if (!math::isFinite(candidate))
        {
            return false;
        }
        body.linearVelocity = candidate;
        wakeRecord(body);
        return true;
    }

    bool setBodyAngularVelocityImpl(WorldHandle worldHandle, BodyHandle handle, const double velocity[3])
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || velocity == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body            &body = world->bodies[iterator->second];
        const math::Vec3 candidate{velocity[0], velocity[1], velocity[2]};
        if (!math::isFinite(candidate))
        {
            return false;
        }
        body.angularVelocity = candidate;
        wakeRecord(body);
        return true;
    }

    bool setBodyMaterialImpl(WorldHandle worldHandle, BodyHandle handle, const MaterialDescription *material)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || material == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body &body = world->bodies[iterator->second];
        if (!std::isfinite(material->friction) || !std::isfinite(material->restitution)
            || !std::isfinite(material->rollingResistance))
        {
            return false;
        }
        body.material.friction           = std::clamp(material->friction, 0.0, 4.0);
        body.material.restitution        = std::clamp(material->restitution, 0.0, 1.0);
        body.material.rollingResistance  = std::clamp(material->rollingResistance, 0.0, 1.0);
        return true;
    }

    bool setBodyContinuousCollisionImpl(WorldHandle worldHandle, BodyHandle handle, bool enabled)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        world->bodies[iterator->second].continuousCollisionEnabled = enabled;
        return true;
    }

    bool setBodyImportanceImpl(WorldHandle worldHandle, BodyHandle handle, double importance)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        // Importance is a policy input, not motion: the band resolves on the
        // next step and the body keeps its sleep state.
        world->bodies[iterator->second].importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool setBodyAwakeImpl(WorldHandle worldHandle, BodyHandle handle, bool awake)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body &body = world->bodies[iterator->second];
        if (awake)
        {
            wakeRecord(body);
        }
        else if (body.kind == BodyKind::Dynamic)
        {
            body.awake           = false;
            body.linearVelocity  = math::Vec3{};
            body.angularVelocity = math::Vec3{};
        }
        return true;
    }

    bool applyForceAtPointImpl(
        WorldHandle worldHandle, BodyHandle handle, const double force[3], const double worldPoint[3])
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || force == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body            &body = world->bodies[iterator->second];
        const math::Vec3 candidate{force[0], force[1], force[2]};
        if (!math::isFinite(candidate))
        {
            return false;
        }
        if (body.kind != BodyKind::Dynamic)
        {
            return false;
        }
        // Validate the optional anchor before touching any state, so a
        // rejected call leaves the accumulators exactly as they were.
        math::Vec3 point;
        if (worldPoint != nullptr)
        {
            point = math::Vec3{worldPoint[0], worldPoint[1], worldPoint[2]};
            if (!math::isFinite(point))
            {
                return false;
            }
        }
        wakeRecord(body);
        body.forceAccumulator += candidate;
        if (worldPoint != nullptr)
        {
            body.torqueAccumulator += math::cross(point - body.transform.position, candidate);
        }
        return true;
    }

    bool applyTorqueImpl(WorldHandle worldHandle, BodyHandle handle, const double torque[3])
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || torque == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body            &body = world->bodies[iterator->second];
        const math::Vec3 candidate{torque[0], torque[1], torque[2]};
        if (!math::isFinite(candidate) || body.kind != BodyKind::Dynamic)
        {
            return false;
        }
        wakeRecord(body);
        body.torqueAccumulator += candidate;
        return true;
    }

    bool applyImpulseAtPointImpl(
        WorldHandle worldHandle, BodyHandle handle, const double impulse[3], const double worldPoint[3])
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || impulse == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body            &body = world->bodies[iterator->second];
        const math::Vec3 candidate{impulse[0], impulse[1], impulse[2]};
        if (!math::isFinite(candidate) || body.kind != BodyKind::Dynamic)
        {
            return false;
        }
        // Validate the optional anchor before touching any state, so a
        // rejected call leaves the velocities exactly as they were.
        math::Vec3 point;
        if (worldPoint != nullptr)
        {
            point = math::Vec3{worldPoint[0], worldPoint[1], worldPoint[2]};
            if (!math::isFinite(point))
            {
                return false;
            }
        }
        wakeRecord(body);
        body.linearVelocity += candidate * body.inverseMass;
        if (worldPoint != nullptr)
        {
            const math::Mat3 inverseInertia = computeWorldInverseInertia(body);
            body.angularVelocity
                += math::multiplyMatrixVector(inverseInertia, math::cross(point - body.transform.position, candidate));
        }
        return true;
    }

    bool applyAngularImpulseImpl(WorldHandle worldHandle, BodyHandle handle, const double impulse[3])
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || impulse == nullptr)
        {
            return false;
        }
        const auto iterator = world->bodyIndexByHandle.find(handle);
        if (iterator == world->bodyIndexByHandle.end())
        {
            return false;
        }
        Body            &body = world->bodies[iterator->second];
        const math::Vec3 candidate{impulse[0], impulse[1], impulse[2]};
        if (!math::isFinite(candidate) || body.kind != BodyKind::Dynamic)
        {
            return false;
        }
        wakeRecord(body);
        const math::Mat3 inverseInertia = computeWorldInverseInertia(body);
        body.angularVelocity += math::multiplyMatrixVector(inverseInertia, candidate);
        return true;
    }

    JointHandle createJointImpl(WorldHandle worldHandle, const JointDescription *description, PhysicsError *error)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            setError(error, PhysicsError::InvalidHandle);
            return invalidJoint;
        }
        setError(error, PhysicsError::None);
        if (description == nullptr)
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidJoint;
        }
        const auto bodyAIterator = world->bodyIndexByHandle.find(description->bodyA);
        const auto bodyBIterator = world->bodyIndexByHandle.find(description->bodyB);
        if (bodyAIterator == world->bodyIndexByHandle.end() || bodyBIterator == world->bodyIndexByHandle.end()
            || description->bodyA == description->bodyB)
        {
            setError(error, PhysicsError::InvalidHandle);
            return invalidJoint;
        }
        if (world->joints.size() >= world->settings.maximumJoints)
        {
            setError(error, PhysicsError::CapacityExceeded);
            return invalidJoint;
        }
        if (!math::isFinite(description->anchorLocalA) || !math::isFinite(description->anchorLocalB)
            || !std::isfinite(description->distance) || description->distance < 0.0)
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidJoint;
        }
        Joint joint;
        joint.description = *description;
        if (joint.description.kind == JointKind::Hinge || joint.description.kind == JointKind::Prismatic)
        {
            const math::Vec3 axis{description->axisLocalA.x, description->axisLocalA.y, description->axisLocalA.z};
            if (!math::isFinite(axis) || math::lengthSquared(axis) < 1.0e-12)
            {
                setError(error, PhysicsError::InvalidDescription);
                return invalidJoint;
            }
            joint.axisLocalA = math::normalized(axis);
        }
        else
        {
            joint.axisLocalA = math::Vec3{0.0, 1.0, 0.0};
        }
        if (joint.description.limitsEnabled)
        {
            const bool hingeLimitsValid    = joint.description.kind == JointKind::Hinge
                                          && std::isfinite(joint.description.limitLowerRadians)
                                          && std::isfinite(joint.description.limitUpperRadians)
                                          && joint.description.limitLowerRadians < joint.description.limitUpperRadians
                                          && joint.description.limitLowerRadians > -3.14159265358979323846
                                          && joint.description.limitUpperRadians < 3.14159265358979323846;
            const bool ballLimitsValid     = joint.description.kind == JointKind::BallSocket
                                          && std::isfinite(joint.description.coneHalfAngleRadians)
                                          && std::isfinite(joint.description.twistHalfAngleRadians)
                                          && joint.description.coneHalfAngleRadians > 0.0
                                          && joint.description.coneHalfAngleRadians <= 3.14159265358979323846
                                          && joint.description.twistHalfAngleRadians > 0.0
                                          && joint.description.twistHalfAngleRadians <= 3.14159265358979323846;
            const bool prismaticLimitsValid = joint.description.kind == JointKind::Prismatic
                                           && std::isfinite(joint.description.limitLowerRadians)
                                           && std::isfinite(joint.description.limitUpperRadians)
                                           && joint.description.limitLowerRadians < joint.description.limitUpperRadians;
            const bool distanceOrUnlimited = !joint.description.limitsEnabled;
            if (!distanceOrUnlimited && !hingeLimitsValid && !ballLimitsValid && !prismaticLimitsValid)
            {
                setError(error, PhysicsError::InvalidDescription);
                return invalidJoint;
            }
        }
        if (joint.description.motorEnabled
            && (!std::isfinite(joint.description.motorTargetVelocity)
                || !std::isfinite(joint.description.maximumMotorTorque) || joint.description.maximumMotorTorque < 0.0))
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidJoint;
        }
        if (joint.description.springEnabled
            && (!std::isfinite(joint.description.springFrequencyHz) || joint.description.springFrequencyHz < 0.0
                || !std::isfinite(joint.description.springDampingRatio) || joint.description.springDampingRatio < 0.0))
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidJoint;
        }
        Body &bodyA = world->bodies[bodyAIterator->second];
        Body &bodyB = world->bodies[bodyBIterator->second];
        joint.referenceRelativeRotation
            = math::normalize(math::conjugate(bodyA.transform.rotation) * bodyB.transform.rotation);
        if (joint.description.kind == JointKind::Distance)
        {
            const math::Vec3 anchorWorldA
                = bodyA.transform.position + math::rotate(bodyA.transform.rotation, joint.description.anchorLocalA);
            const math::Vec3 anchorWorldB
                = bodyB.transform.position + math::rotate(bodyB.transform.rotation, joint.description.anchorLocalB);
            joint.restLength = joint.description.distance > 0.0 ? joint.description.distance
                                                                : math::length(anchorWorldB - anchorWorldA);
        }
        else if (joint.description.kind == JointKind::Prismatic)
        {
            const math::Vec3 anchorWorldA
                = bodyA.transform.position + math::rotate(bodyA.transform.rotation, joint.description.anchorLocalA);
            const math::Vec3 anchorWorldB
                = bodyB.transform.position + math::rotate(bodyB.transform.rotation, joint.description.anchorLocalB);
            const math::Vec3 axisWorld = math::rotate(bodyA.transform.rotation, joint.axisLocalA);
            joint.sliderRestDistance   = math::dot(anchorWorldB - anchorWorldA, axisWorld);
        }
        else
        {
            joint.restLength = 0.0;
        }
        wakeRecord(bodyA);
        wakeRecord(bodyB);
        if (nextJointHandle_ == invalidJoint)
        {
            setError(error, PhysicsError::CapacityExceeded);
            return invalidJoint;
        }
        joint.handle = nextJointHandle_;
        nextJointHandle_
            = nextJointHandle_ == std::numeric_limits<JointHandle>::max() ? invalidJoint : nextJointHandle_ + 1;
        world->joints.push_back(std::move(joint));
        world->jointIndexByHandle.emplace(world->joints.back().handle, world->joints.size() - 1);
        return world->joints.back().handle;
    }

    void destroyJointImpl(WorldHandle worldHandle, JointHandle handle)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return;
        }
        destroyJointFromWorld(*world, handle);
    }

    bool copyJointDescriptionImpl(WorldHandle worldHandle, JointHandle handle, JointDescription *outDescription)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outDescription == nullptr)
        {
            return false;
        }
        const auto iterator = world->jointIndexByHandle.find(handle);
        if (iterator == world->jointIndexByHandle.end())
        {
            return false;
        }
        *outDescription = world->joints[iterator->second].description;
        return true;
    }

    bool setJointMotorImpl(
        WorldHandle worldHandle, JointHandle handle, bool enabled, double targetVelocity, double maximumTorque)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        const auto iterator = world->jointIndexByHandle.find(handle);
        if (iterator == world->jointIndexByHandle.end())
        {
            return false;
        }
        Joint &joint = world->joints[iterator->second];
        // Motor parameters stay validated whichever way the switch goes, so
        // a disabled motor never stores values a later copyJointDescription
        // would echo as garbage.
        if (!std::isfinite(targetVelocity) || !std::isfinite(maximumTorque) || maximumTorque < 0.0)
        {
            return false;
        }
        joint.description.motorEnabled        = enabled;
        joint.description.motorTargetVelocity = targetVelocity;
        joint.description.maximumMotorTorque  = maximumTorque;
        const auto bodyAIterator              = world->bodyIndexByHandle.find(joint.description.bodyA);
        const auto bodyBIterator              = world->bodyIndexByHandle.find(joint.description.bodyB);
        if (bodyAIterator != world->bodyIndexByHandle.end())
        {
            wakeRecord(world->bodies[bodyAIterator->second]);
        }
        if (bodyBIterator != world->bodyIndexByHandle.end())
        {
            wakeRecord(world->bodies[bodyBIterator->second]);
        }
        return true;
    }

    bool setJointSpringImpl(
        WorldHandle worldHandle, JointHandle handle, bool enabled, double frequencyHz, double dampingRatio)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        const auto iterator = world->jointIndexByHandle.find(handle);
        if (iterator == world->jointIndexByHandle.end())
        {
            return false;
        }
        Joint &joint = world->joints[iterator->second];
        if (joint.description.kind != JointKind::Distance)
        {
            return false;
        }
        // Spring parameters stay validated whichever way the switch goes,
        // so a disabled spring never stores values a later
        // copyJointDescription would echo as garbage.
        if (!std::isfinite(frequencyHz) || frequencyHz < 0.0 || !std::isfinite(dampingRatio) || dampingRatio < 0.0)
        {
            return false;
        }
        joint.description.springEnabled      = enabled;
        joint.description.springFrequencyHz  = frequencyHz;
        joint.description.springDampingRatio = dampingRatio;
        joint.distanceImpulse                = 0.0;
        const auto bodyAIterator             = world->bodyIndexByHandle.find(joint.description.bodyA);
        const auto bodyBIterator             = world->bodyIndexByHandle.find(joint.description.bodyB);
        if (bodyAIterator != world->bodyIndexByHandle.end())
        {
            wakeRecord(world->bodies[bodyAIterator->second]);
        }
        if (bodyBIterator != world->bodyIndexByHandle.end())
        {
            wakeRecord(world->bodies[bodyBIterator->second]);
        }
        return true;
    }

    bool stepImpl(WorldHandle worldHandle, double fixedDeltaSeconds)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        return stepWorld(*world, fixedDeltaSeconds, scratch_);
    }

    std::uint32_t simulateImpl(WorldHandle worldHandle, double elapsedSeconds)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / world->settings.fixedRateHz;
        return world->accumulator.drain(fixedDelta, world->settings.maximumStepsPerSimulate, elapsedSeconds,
            [this, world, fixedDelta]() { return stepWorld(*world, fixedDelta, scratch_); });
    }

    bool rayCastImpl(WorldHandle worldHandle, const double origin[3], const double direction[3], double maximumDistance,
        RayHit *outHit)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || origin == nullptr || direction == nullptr || outHit == nullptr)
        {
            return false;
        }
        return rayCastWorld(*world,
            math::Vec3{origin[0], origin[1], origin[2]},
            math::Vec3{direction[0], direction[1], direction[2]},
            maximumDistance,
            *outHit);
    }

    bool rayCastIgnoringImpl(WorldHandle worldHandle, const double origin[3], const double direction[3],
        double maximumDistance, const BodyHandle *ignoredBodies, std::uint32_t ignoredBodyCount, RayHit *outHit)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || origin == nullptr || direction == nullptr || outHit == nullptr
            || (ignoredBodyCount > 0 && ignoredBodies == nullptr))
        {
            return false;
        }
        return rayCastWorldIgnoring(*world,
            math::Vec3{origin[0], origin[1], origin[2]},
            math::Vec3{direction[0], direction[1], direction[2]},
            maximumDistance,
            ignoredBodies,
            ignoredBodyCount,
            *outHit);
    }

    bool collideSphereImpl(WorldHandle worldHandle, const double center[3], double radius, double outNormal[3],
        double *outDepth, BodyHandle *outBody)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || center == nullptr || outNormal == nullptr || outDepth == nullptr)
        {
            return false;
        }
        math::Vec3 normal;
        double     depth = 0.0;
        BodyHandle body  = invalidBody;
        if (!collideSphereWorld(*world, math::Vec3{center[0], center[1], center[2]}, radius, normal, depth, body))
        {
            return false;
        }
        outNormal[0] = normal.x;
        outNormal[1] = normal.y;
        outNormal[2] = normal.z;
        *outDepth    = depth;
        if (outBody != nullptr)
        {
            *outBody = body;
        }
        return true;
    }

    bool worldStatisticsImpl(WorldHandle worldHandle, WorldStatistics *outStatistics)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        fillWorldStatistics(*world, *outStatistics);
        return true;
    }

    bool sphereCastImpl(
        WorldHandle worldHandle, const double start[3], const double delta[3], double radius, SweepHit *outHit)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || start == nullptr || delta == nullptr || outHit == nullptr)
        {
            return false;
        }
        const math::Vec3 startVector{start[0], start[1], start[2]};
        const math::Vec3 deltaVector{delta[0], delta[1], delta[2]};
        if (!math::isFinite(startVector) || !math::isFinite(deltaVector))
        {
            return false;
        }
        return sphereCastWorld(*world, startVector, deltaVector, radius, *outHit);
    }

    // ---- batched query implementations (ABI 5) -----------------------------

    std::uint32_t collideSpheresImpl(
        WorldHandle worldHandle, const SphereOverlapQuery *queries, std::uint32_t queryCount, SphereOverlapResult *outResults)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return 0u;
        }
        return collideSpheresWorld(*world, queries, queryCount, outResults);
    }

    template <typename OutRecord, typename Fill>
    static std::uint32_t copyBodiesBatched(
        World *world, const BodyHandle *bodies, std::uint32_t bodyCount, OutRecord *outRecords, Fill &&fillOne)
    {
        if (world == nullptr || bodies == nullptr || outRecords == nullptr)
        {
            return 0u;
        }
        std::uint32_t resolved = 0;
        for (std::uint32_t index = 0; index < bodyCount; ++index)
        {
            // The capability documents that entries for unknown handles stay
            // default-constructed, so every slot is written: resolved handles
            // get their record, unresolved ones get the documented default
            // instead of the caller's uninitialized buffer contents.
            OutRecord &record = outRecords[index];
            record            = OutRecord{};
            const auto iterator = world->bodyIndexByHandle.find(bodies[index]);
            if (iterator != world->bodyIndexByHandle.end())
            {
                fillOne(record, world->bodies[iterator->second]);
                ++resolved;
            }
        }
        return resolved;
    }

    std::uint32_t copyBodyStatesImpl(
        WorldHandle worldHandle, const BodyHandle *bodies, std::uint32_t bodyCount, BodyState *outStates)
    {
        return copyBodiesBatched(findWorld(worldHandle), bodies, bodyCount, outStates, [](BodyState &target, const Body &body)
        {
            BodyState state;
            state.transform         = body.transform;
            state.linearVelocity    = body.linearVelocity;
            state.angularVelocity   = body.angularVelocity;
            // The same awake semantics the single-body copy reports: static
            // and kinematic bodies never sleep, so they always read awake.
            state.awake             = body.awake || body.kind != BodyKind::Dynamic;
            target                  = state;
        });
    }

    std::uint32_t copyBodyDescriptionsImpl(WorldHandle worldHandle, const BodyHandle *bodies, std::uint32_t bodyCount,
        BodyDescription *outDescriptions)
    {
        return copyBodiesBatched(
            findWorld(worldHandle), bodies, bodyCount, outDescriptions, [](BodyDescription &target, const Body &body)
            {
                BodyDescription description;
                fillBodyDescriptionFromRecord(body, description);
                target = description;
            });
    }

    // ---- trigger volume implementations ------------------------------------

    Trigger *findTrigger(World &world, TriggerHandle handle)
    {
        const auto iterator = world.triggerIndexByHandle.find(handle);
        return iterator != world.triggerIndexByHandle.end() ? &world.triggers[iterator->second] : nullptr;
    }

    TriggerHandle createTriggerImpl(WorldHandle worldHandle, const TriggerDescription *description, PhysicsError *error)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            setError(error, PhysicsError::InvalidHandle);
            return invalidTrigger;
        }
        if (description == nullptr)
        {
            setError(error, PhysicsError::InvalidDescription);
            return invalidTrigger;
        }
        Trigger trigger;
        if (!validateTriggerDescription(*world, *description, trigger, error))
        {
            return invalidTrigger;
        }
        if (nextTriggerHandle_ == invalidTrigger)
        {
            setError(error, PhysicsError::CapacityExceeded);
            return invalidTrigger;
        }
        trigger.handle        = nextTriggerHandle_;
        nextTriggerHandle_ = nextTriggerHandle_ == std::numeric_limits<TriggerHandle>::max()
            ? invalidTrigger
            : nextTriggerHandle_ + 1;
        world->triggers.push_back(std::move(trigger));
        world->triggerIndexByHandle.emplace(world->triggers.back().handle, world->triggers.size() - 1);
        setError(error, PhysicsError::None);
        return world->triggers.back().handle;
    }

    void destroyTriggerImpl(WorldHandle worldHandle, TriggerHandle handle)
    {
        World *world = findWorld(worldHandle);
        if (world != nullptr)
        {
            destroyTriggerFromWorld(*world, handle);
        }
    }

    bool copyTriggerDescriptionImpl(
        WorldHandle worldHandle, TriggerHandle handle, TriggerDescription *outDescription)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outDescription == nullptr)
        {
            return false;
        }
        const Trigger *trigger = findTrigger(*world, handle);
        if (trigger == nullptr)
        {
            return false;
        }
        TriggerDescription description;
        description.shape.kind          = trigger->shape.kind;
        description.shape.sphereRadius  = trigger->shape.sphereRadius;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            description.shape.boxHalfExtents[axis] = trigger->shape.boxHalfExtents[axis];
        }
        description.shape.capsuleRadius      = trigger->shape.capsuleRadius;
        description.shape.capsuleHalfLength  = trigger->shape.capsuleHalfLength;
        description.shape.cylinderRadius     = trigger->shape.cylinderRadius;
        description.shape.cylinderHalfHeight = trigger->shape.cylinderHalfHeight;
        description.transform                = trigger->localTransform;
        description.attachedBody             = trigger->attachedBody;
        description.categoryBits             = trigger->categoryBits;
        description.maskBits                 = trigger->maskBits;
        description.enabled                  = trigger->enabled;
        description.userTag                  = trigger->userTag;
        *outDescription                      = description;
        return true;
    }

    bool setTriggerTransformImpl(WorldHandle worldHandle, TriggerHandle handle, const RigidTransform *transform)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || transform == nullptr)
        {
            return false;
        }
        Trigger *trigger = findTrigger(*world, handle);
        if (trigger == nullptr || !math::isFinite(transform->position) || !math::isFinite(transform->rotation))
        {
            return false;
        }
        const math::Quaternion rotation = math::normalize(transform->rotation);
        if (math::dot(rotation, rotation) < 1.0e-12)
        {
            return false;
        }
        if (trigger->attachedBody != invalidBody)
        {
            trigger->localTransform.position = transform->position;
            trigger->localTransform.rotation = rotation;
        }
        else
        {
            trigger->transform.position = transform->position;
            trigger->transform.rotation = rotation;
            computeShapeAabb(trigger->shape, trigger->transform, trigger->aabbMinimum, trigger->aabbMaximum);
        }
        return true;
    }

    bool setTriggerEnabledImpl(WorldHandle worldHandle, TriggerHandle handle, bool enabled)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        Trigger *trigger = findTrigger(*world, handle);
        if (trigger == nullptr)
        {
            return false;
        }
        trigger->enabled = enabled;
        return true;
    }

    bool triggerEventCountImpl(WorldHandle worldHandle, std::uint32_t *outCount, bool *outOverflowed)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outCount == nullptr)
        {
            return false;
        }
        *outCount = static_cast<std::uint32_t>(world->pendingTriggerEvents.size());
        if (outOverflowed != nullptr)
        {
            *outOverflowed = world->triggerEventOverflow;
        }
        return true;
    }

    bool copyTriggerEventsImpl(WorldHandle worldHandle, std::uint32_t firstIndex, TriggerEvent *outEvents,
        std::uint32_t capacity, std::uint32_t *outCopied)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || (outEvents == nullptr && capacity > 0))
        {
            return false;
        }
        const std::uint32_t pending = static_cast<std::uint32_t>(world->pendingTriggerEvents.size());
        const std::uint32_t copied
            = firstIndex < pending ? std::min(capacity, pending - firstIndex) : 0u;
        for (std::uint32_t index = 0; index < copied; ++index)
        {
            outEvents[index] = world->pendingTriggerEvents[firstIndex + index];
        }
        if (outCopied != nullptr)
        {
            *outCopied = copied;
        }
        return true;
    }

    bool clearTriggerEventsImpl(WorldHandle worldHandle)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        world->pendingTriggerEvents.clear();
        world->triggerEventOverflow = false;
        return true;
    }

    bool triggerOverlapCountImpl(WorldHandle worldHandle, TriggerHandle handle, std::uint32_t *outCount)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr || outCount == nullptr)
        {
            return false;
        }
        const Trigger *trigger = findTrigger(*world, handle);
        if (trigger == nullptr)
        {
            return false;
        }
        *outCount = static_cast<std::uint32_t>(trigger->overlapping.size());
        return true;
    }

    bool copyTriggerOverlapsImpl(WorldHandle worldHandle, TriggerHandle handle, BodyHandle *outBodies,
        std::uint32_t capacity, std::uint32_t *outRequired)
    {
        World *world = findWorld(worldHandle);
        if (world == nullptr)
        {
            return false;
        }
        const Trigger *trigger = findTrigger(*world, handle);
        if (trigger == nullptr)
        {
            return false;
        }
        if (outRequired != nullptr)
        {
            *outRequired = static_cast<std::uint32_t>(trigger->overlapping.size());
        }
        if (outBodies == nullptr || capacity < trigger->overlapping.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < trigger->overlapping.size(); ++index)
        {
            outBodies[index] = trigger->overlapping[index];
        }
        return true;
    }

    static void wakeRecord(Body &body)
    {
        body.awake      = true;
        body.sleepTimer = 0.0;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsRigidCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_RIGID_BIND(field, method)                                                                       \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    { return static_cast<PhysicsRigidModule *>(self)->method(args...); }
        ENGINE_PHYSICS_RIGID_BIND(createWorld, createWorldImpl);
        ENGINE_PHYSICS_RIGID_BIND(destroyWorld, destroyWorldImpl);
        ENGINE_PHYSICS_RIGID_BIND(setWorldGravity, setWorldGravityImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyWorldGravity, copyWorldGravityImpl);
        ENGINE_PHYSICS_RIGID_BIND(createBody, createBodyImpl);
        ENGINE_PHYSICS_RIGID_BIND(destroyBody, destroyBodyImpl);
        ENGINE_PHYSICS_RIGID_BIND(bodyCount, bodyCountImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyBodyHandles, copyBodyHandlesImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyBodyDescription, copyBodyDescriptionImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyBodyState, copyBodyStateImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyBodyMassProperties, copyBodyMassPropertiesImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyTransform, setBodyTransformImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyKind, setBodyKindImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyLinearVelocity, setBodyLinearVelocityImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyAngularVelocity, setBodyAngularVelocityImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyMaterial, setBodyMaterialImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyAwake, setBodyAwakeImpl);
        ENGINE_PHYSICS_RIGID_BIND(applyForceAtPoint, applyForceAtPointImpl);
        ENGINE_PHYSICS_RIGID_BIND(applyTorque, applyTorqueImpl);
        ENGINE_PHYSICS_RIGID_BIND(applyImpulseAtPoint, applyImpulseAtPointImpl);
        ENGINE_PHYSICS_RIGID_BIND(applyAngularImpulse, applyAngularImpulseImpl);
        ENGINE_PHYSICS_RIGID_BIND(createJoint, createJointImpl);
        ENGINE_PHYSICS_RIGID_BIND(destroyJoint, destroyJointImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyJointDescription, copyJointDescriptionImpl);
        ENGINE_PHYSICS_RIGID_BIND(setJointMotor, setJointMotorImpl);
        ENGINE_PHYSICS_RIGID_BIND(step, stepImpl);
        ENGINE_PHYSICS_RIGID_BIND(simulate, simulateImpl);
        ENGINE_PHYSICS_RIGID_BIND(rayCast, rayCastImpl);
        ENGINE_PHYSICS_RIGID_BIND(rayCastIgnoring, rayCastIgnoringImpl);
        ENGINE_PHYSICS_RIGID_BIND(collideSphere, collideSphereImpl);
        ENGINE_PHYSICS_RIGID_BIND(worldStatistics, worldStatisticsImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyContinuousCollision, setBodyContinuousCollisionImpl);
        ENGINE_PHYSICS_RIGID_BIND(setJointSpring, setJointSpringImpl);
        ENGINE_PHYSICS_RIGID_BIND(sphereCast, sphereCastImpl);
        ENGINE_PHYSICS_RIGID_BIND(createTrigger, createTriggerImpl);
        ENGINE_PHYSICS_RIGID_BIND(destroyTrigger, destroyTriggerImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyTriggerDescription, copyTriggerDescriptionImpl);
        ENGINE_PHYSICS_RIGID_BIND(setTriggerTransform, setTriggerTransformImpl);
        ENGINE_PHYSICS_RIGID_BIND(setTriggerEnabled, setTriggerEnabledImpl);
        ENGINE_PHYSICS_RIGID_BIND(triggerEventCount, triggerEventCountImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyTriggerEvents, copyTriggerEventsImpl);
        ENGINE_PHYSICS_RIGID_BIND(clearTriggerEvents, clearTriggerEventsImpl);
        ENGINE_PHYSICS_RIGID_BIND(triggerOverlapCount, triggerOverlapCountImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyTriggerOverlaps, copyTriggerOverlapsImpl);
        ENGINE_PHYSICS_RIGID_BIND(collideSpheres, collideSpheresImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyBodyStates, copyBodyStatesImpl);
        ENGINE_PHYSICS_RIGID_BIND(copyBodyDescriptions, copyBodyDescriptionsImpl);
        ENGINE_PHYSICS_RIGID_BIND(setBodyImportance, setBodyImportanceImpl);
#undef ENGINE_PHYSICS_RIGID_BIND
    }

    ParticipantManifest    manifest_;
    PhysicsRigidCapability capability_{};

    SimulationSettings settings_;

    std::vector<World>                           worlds_;
    std::unordered_map<WorldHandle, std::size_t> worldIndexByHandle_;
    WorldHandle                                  nextWorldHandle_   = 1;
    BodyHandle                                   nextBodyHandle_    = 1;
    JointHandle                                  nextJointHandle_   = 1;
    TriggerHandle                                nextTriggerHandle_ = 1;

    StepScratch scratch_;
};

} // namespace

} // namespace engine::physics_rigid

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_rigid capability; no required capabilities, no permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion    = engine::engineModuleAbiVersion,
        .moduleName    = "physics_rigid",
        .moduleVersion = "0.1.0",
        .createModule  = []() -> engine::ModuleInterface  *{ return new engine::physics_rigid::PhysicsRigidModule(); },
        .providedCapabilities    = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
