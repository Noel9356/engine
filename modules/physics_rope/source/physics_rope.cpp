// physics_rope module implementation.
//
// Simulation LOD reduces the internal substep count for unimportant ropes
// without touching the fixed step size; a diverged step rolls back to its
// pre-step node state.
//
// XPBD rope chains over a physics_rigid world: nodes integrate velocities,
// stretch and bending constraints solve in Gauss-Seidel order with
// compliance-scaled corrections (accumulating a per-segment tension readout),
// over-stretched segments break permanently when the description enables it,
// attached nodes follow rigid bodies or pin positions, and free nodes resolve
// against the rigid world's bodies and planes through one batched
// sphere-overlap query per substep. Stepping happens in whole fixed steps
// with internal substeps (the "small steps" XPBD schedule), so the simulation
// is independent of frame rate; contact friction retains the documented share
// of tangential velocity per second of contact.

#include "records.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>
#include <engine/util/xpbd_particles.hpp>
#include <engine/physics_rope/capabilities.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_rope
{
namespace
{

namespace math = engine::physics_rigid::math;

class RopeModule final : public ModuleInterface
{
public:
    RopeModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_rope", "maximum_ropes");
            value && *value > 0 && *value <= 4096)
            settings_.maximumRopes = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_rope", "maximum_nodes");
            value && *value > 0 && *value <= 10'000'000)
            settings_.maximumNodes = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_rope", "maximum_nodes_per_rope");
            value && *value > 0 && *value <= 100'000)
            settings_.maximumNodesPerRope = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_rope", "substeps_per_step");
            value && *value > 0 && *value <= 64)
            settings_.substepsPerStep = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_rope", "maximum_steps_per_simulate");
            value && *value > 0 && *value <= 1000)
            settings_.maximumStepsPerSimulate = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_rope", "fixed_rate_hz");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.fixedRateHz = *value;
        if (const auto value = context.configurationFloatingInSection("physics_rope", "maximum_velocity");
            value && *value > 0.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.maximumVelocity = *value;
        readLodSettings(context);

        rigid_ = context.tryAcquireCapability<engine::physics_rigid::PhysicsRigidCapability>();
        if (rigid_ != nullptr && rigid_->abi != engine::physics_rigid::physicsRigidCapabilityAbi)
        {
            rigid_ = nullptr;
        }
    }

    void start() override
    {}

    void stop() override
    {
        ropes_.clear();
        ropeIndexByHandle_.clear();
        nextRopeHandle_ = 1;
        totalNodes_     = 0;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == RopeCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_rope",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_rope"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_rope");
    }

    static void setError(RopeError *output, RopeError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    RopeRecord *findRope(RopeHandle handle)
    {
        const auto iterator = ropeIndexByHandle_.find(handle);
        return iterator != ropeIndexByHandle_.end() ? &ropes_[iterator->second] : nullptr;
    }

    std::size_t nodeOffset(std::uint32_t nodeIndex) const
    {
        return static_cast<std::size_t>(nodeIndex) * 3;
    }

    // World position of an attached node: its pin position (refreshed below
    // for body attachments each substep).
    void refreshAttachmentTargets(RopeRecord &rope)
    {
        if (rigid_ == nullptr)
        {
            return;
        }
        for (std::size_t node = 0; node < rope.attachments.size(); ++node)
        {
            if (rope.attachments[node] != NodeAttachment::Body)
            {
                continue;
            }
            engine::physics_rigid::BodyState state;
            if (!rigid_->copyBodyState(rigid_->self, rope.world, rope.attachedBodies[node], &state))
            {
                continue; // a destroyed body freezes its node at the last target
            }
            const math::Vec3 anchor = state.transform.position
                                      + math::rotate(state.transform.rotation,
                                          math::Vec3{rope.attachedLocalAnchors[node * 3],
                                              rope.attachedLocalAnchors[node * 3 + 1],
                                              rope.attachedLocalAnchors[node * 3 + 2]});
            rope.pinPositions[node * 3]     = anchor.x;
            rope.pinPositions[node * 3 + 1] = anchor.y;
            rope.pinPositions[node * 3 + 2] = anchor.z;
        }
    }

    // ---- XPBD stepping ------------------------------------------------------

    void integrateSubstep(RopeRecord &rope, double substepDelta)
    {
        const std::size_t nodeCount    = rope.positions.size() / 3;
        const double      dampingFactor = 1.0 / (1.0 + substepDelta * rope.damping);
        for (std::size_t node = 0; node < nodeCount; ++node)
        {
            const std::size_t offset = node * 3;
            if (rope.attachments[node] != NodeAttachment::Free)
            {
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    rope.velocities[offset + axis]        = 0.0;
                    rope.positions[offset + axis]         = rope.pinPositions[offset + axis];
                    rope.previousPositions[offset + axis] = rope.positions[offset + axis];
                }
                continue;
            }
            rope.previousPositions[offset]     = rope.positions[offset];
            rope.previousPositions[offset + 1] = rope.positions[offset + 1];
            rope.previousPositions[offset + 2] = rope.positions[offset + 2];
            rope.velocities[offset] += (rope.gravity[0] + rope.windAcceleration[0]) * substepDelta;
            rope.velocities[offset + 1] += (rope.gravity[1] + rope.windAcceleration[1]) * substepDelta;
            rope.velocities[offset + 2] += (rope.gravity[2] + rope.windAcceleration[2]) * substepDelta;
            rope.velocities[offset] *= dampingFactor;
            rope.velocities[offset + 1] *= dampingFactor;
            rope.velocities[offset + 2] *= dampingFactor;
            rope.positions[offset] += rope.velocities[offset] * substepDelta;
            rope.positions[offset + 1] += rope.velocities[offset + 1] * substepDelta;
            rope.positions[offset + 2] += rope.velocities[offset + 2] * substepDelta;
        }
    }

    // Strain check ahead of each substep solve: a segment stretched beyond
    // restLength * (1 + breakStrain) breaks permanently (stretch only — a
    // compressed segment never snaps). Breaking is a flag flip, so it can
    // never trip the non-finite failure path.
    void checkSegmentStrain(RopeRecord &rope)
    {
        const double strainLimit = 1.0 + rope.breakStrain;
        for (std::size_t index = 0; index < rope.stretchConstraints.size(); ++index)
        {
            if (rope.brokenSegments[index] != 0u)
            {
                continue;
            }
            const RopeRecord::StretchConstraint &constraint = rope.stretchConstraints[index];
            const std::size_t offsetA           = nodeOffset(constraint.nodeA);
            const std::size_t offsetB           = nodeOffset(constraint.nodeB);
            const double      deltaX            = rope.positions[offsetB] - rope.positions[offsetA];
            const double      deltaY            = rope.positions[offsetB + 1] - rope.positions[offsetA + 1];
            const double      deltaZ            = rope.positions[offsetB + 2] - rope.positions[offsetA + 2];
            const double      length            = std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
            if (length > constraint.restLength * strainLimit)
            {
                rope.brokenSegments[index] = 1u;
                rope.brokenSegmentCount += 1;
            }
        }
    }

    void solveConstraints(RopeRecord &rope, double substepDelta)
    {
        if (rope.breakStrain > 0.0)
        {
            checkSegmentStrain(rope);
        }
        for (std::size_t index = 0; index < rope.stretchConstraints.size(); ++index)
        {
            const RopeRecord::StretchConstraint &constraint = rope.stretchConstraints[index];
            if (rope.brokenSegments[index] != 0u)
            {
                continue; // a broken segment no longer pulls its nodes together
            }
            // The shared XPBD solve keeps the lambda math identical to the
            // cloth module; the returned lambda lands in the segment's
            // tension accumulator (bend rows pass a null accumulator).
            const double lambda = engine::util::solveXpbdDistance(rope.positions.data(),
                constraint.nodeA,
                constraint.nodeB,
                rope.inverseMasses[constraint.nodeA],
                rope.inverseMasses[constraint.nodeB],
                constraint.restLength,
                constraint.compliance,
                substepDelta);
            rope.tensionAccumulator[index] += std::abs(lambda);
        }
        for (const RopeRecord::BendConstraint &constraint : rope.bendConstraints)
        {
            // A bend row spans two stretch segments (its record names them);
            // either one breaking releases the row, since holding the
            // two-apart distance across a break would drag the separated
            // pieces back together.
            if (rope.brokenSegments[constraint.segmentA] != 0u || rope.brokenSegments[constraint.segmentB] != 0u)
            {
                continue;
            }
            engine::util::solveXpbdDistance(rope.positions.data(),
                constraint.nodeA,
                constraint.nodeB,
                rope.inverseMasses[constraint.nodeA],
                rope.inverseMasses[constraint.nodeB],
                constraint.restLength,
                constraint.compliance,
                substepDelta);
        }
    }

    // Resolves every free node against the rigid world in one batched
    // sphere query per substep (the shared engine::util batched
    // projection). The caller clears the per-node contact scratch; the
    // helper writes flags and normals for the hits and pushes the node
    // positions out of the obstacles.
    void resolveCollisions(RopeRecord &rope)
    {
        if (rigid_ == nullptr || !rope.collideWithWorld)
        {
            return;
        }
        const std::size_t nodeCount = rope.positions.size() / 3;
        for (std::size_t node = 0; node < nodeCount; ++node)
        {
            rope.contactFlags[node] = 0u;
        }
        engine::util::projectParticleSphereOverlaps(rigid_,
            rope.world,
            rope.positions.data(),
            nodeCount,
            rope.nodeRadius,
            [&rope](std::size_t node) { return rope.attachments[node] != NodeAttachment::Free; },
            collisionQueries_,
            collisionSlotIndices_,
            collisionResults_,
            rope.contactNormals.data(),
            rope.contactFlags.data());
    }

    void finalizeSubstepVelocities(RopeRecord &rope, double substepDelta)
    {
        const std::size_t nodeCount    = rope.positions.size() / 3;
        const double      inverseDeltaTime = 1.0 / substepDelta;
        // Friction retention per contact substep: the capability documents
        // the tangential velocity kept per SECOND of contact, so the
        // per-substep multiplier is friction^(delta) - the same wall-clock
        // contact time removes the same tangential speed at any step rate
        // (a raw per-substep multiplier would not; the cloth module applies
        // the identical conversion).
        const double frictionRetention = rope.friction > 0.0 ? std::pow(rope.friction, substepDelta) : 0.0;
        for (std::size_t node = 0; node < nodeCount; ++node)
        {
            const std::size_t offset = node * 3;
            if (rope.attachments[node] != NodeAttachment::Free)
            {
                continue;
            }
            // The shared finalize re-derives the velocity from the position
            // delta, removes the approaching velocity along the contact
            // normal, scales the tangential remainder by the retention, and
            // clamps to the module's configured maximum.
            engine::util::finalizeParticleVelocity(&rope.velocities[offset],
                &rope.positions[offset],
                &rope.previousPositions[offset],
                inverseDeltaTime,
                rope.contactFlags[node] != 0u ? &rope.contactNormals[offset] : nullptr,
                frictionRetention,
                settings_.maximumVelocity);
        }
    }

    bool stepRope(RopeRecord &rope, double fixedDeltaSeconds, std::uint32_t substeps)
    {
        const auto stepStart = std::chrono::steady_clock::now();
        if (!(fixedDeltaSeconds > 0.0) || fixedDeltaSeconds > 0.1 || !std::isfinite(fixedDeltaSeconds))
        {
            return false;
        }
        const std::size_t   nodeCount    = rope.positions.size() / 3;
        rope.contactFlags.assign(nodeCount, 0u);
        rope.contactNormals.assign(nodeCount * 3, 0.0);
        rope.tensionAccumulator.assign(rope.segments, 0.0); // fresh |lambda| sums for this step
        const double        substepDelta = fixedDeltaSeconds / static_cast<double>(substeps);
        // Pre-step copies for the rollback path: a diverged step restores
        // this state and reports failure, so the caller sees an untouched
        // rope.
        rope.preStepPositions.resize(rope.positions.size());
        rope.preStepVelocities.resize(rope.velocities.size());
        std::copy(rope.positions.begin(), rope.positions.end(), rope.preStepPositions.begin());
        std::copy(rope.velocities.begin(), rope.velocities.end(), rope.preStepVelocities.begin());
        for (std::uint32_t substep = 0; substep < substeps; ++substep)
        {
            refreshAttachmentTargets(rope);
            integrateSubstep(rope, substepDelta);
            solveConstraints(rope, substepDelta);
            resolveCollisions(rope);
            finalizeSubstepVelocities(rope, substepDelta);
        }
        rope.lastContactCount = static_cast<std::uint32_t>(
            std::count(rope.contactFlags.begin(), rope.contactFlags.end(), std::uint8_t{1}));
        bool finite = true;
        for (const double value : rope.positions)
        {
            if (!std::isfinite(value))
            {
                finite = false;
                break;
            }
        }
        for (const double value : rope.velocities)
        {
            if (!std::isfinite(value))
            {
                finite = false;
                break;
            }
        }
        if (!finite)
        {
            std::copy(rope.preStepPositions.begin(), rope.preStepPositions.end(), rope.positions.begin());
            std::copy(rope.preStepVelocities.begin(), rope.preStepVelocities.end(), rope.velocities.begin());
            return false;
        }
        rope.stepsSimulated += 1;
        rope.substepsExecuted += substeps;
        // Publish the tension readout: the step's accumulated |lambda| per
        // segment converts to force units through the fixed delta squared,
        // and the maximum rides along for the statistics.
        const double inverseDeltaSquared = 1.0 / (fixedDeltaSeconds * fixedDeltaSeconds);
        double       maximumTension      = 0.0;
        for (std::size_t segment = 0; segment < rope.lastTension.size(); ++segment)
        {
            rope.lastTension[segment] = rope.tensionAccumulator[segment] * inverseDeltaSquared;
            maximumTension            = std::max(maximumTension, rope.lastTension[segment]);
        }
        rope.maximumTension = maximumTension;
        const auto stepEnd         = std::chrono::steady_clock::now();
        rope.lastStepMilliseconds = std::chrono::duration<double, std::milli>(stepEnd - stepStart).count();
        return true;
    }

    // ---- capability implementations ---------------------------------------

    RopeHandle createRopeImpl(
        engine::physics_rigid::WorldHandle world, const RopeDescription *description, RopeError *error)
    {
        setError(error, RopeError::None);
        if (rigid_ == nullptr)
        {
            setError(error, RopeError::InvalidHandle);
            return invalidRope;
        }
        if (description == nullptr)
        {
            setError(error, RopeError::InvalidDescription);
            return invalidRope;
        }
        if (ropes_.size() >= settings_.maximumRopes || totalNodes_ >= settings_.maximumNodes)
        {
            setError(error, RopeError::CapacityExceeded);
            return invalidRope;
        }
        // Node counts compute in 64-bit before any comparison, so a huge
        // segment count cannot wrap past the per-rope or module budgets.
        const std::size_t nodeCount = static_cast<std::size_t>(description->segments) + 1u;
        if (description->segments < 1 || nodeCount > settings_.maximumNodesPerRope
            || totalNodes_ + nodeCount > settings_.maximumNodes)
        {
            setError(error, RopeError::CapacityExceeded);
            return invalidRope;
        }
        if (!std::isfinite(description->restLength) || description->restLength <= 0.0
            || !std::isfinite(description->nodeRadius) || description->nodeRadius < 0.0
            || !std::isfinite(description->nodeMass) || description->nodeMass <= 0.0
            || !std::isfinite(description->stretchCompliance) || description->stretchCompliance < 0.0
            || !std::isfinite(description->bendingCompliance) || description->bendingCompliance < 0.0
            || !std::isfinite(description->damping) || description->damping < 0.0
            || !std::isfinite(description->friction) || description->friction < 0.0 || description->friction > 1.0
            || !std::isfinite(description->breakStrain) || description->breakStrain < 0.0
            || !std::isfinite(description->start[0]) || !std::isfinite(description->start[1])
            || !std::isfinite(description->start[2]) || !std::isfinite(description->end[0])
            || !std::isfinite(description->end[1]) || !std::isfinite(description->end[2]))
        {
            setError(error, RopeError::InvalidDescription);
            return invalidRope;
        }
        if ((description->attachStartToBody && description->startBody == engine::physics_rigid::invalidBody)
            || (description->attachEndToBody && description->endBody == engine::physics_rigid::invalidBody))
        {
            setError(error, RopeError::InvalidDescription);
            return invalidRope;
        }
        if (description->attachStartToBody || description->attachEndToBody)
        {
            engine::physics_rigid::BodyState existenceProbe;
            if ((description->attachStartToBody
                    && !rigid_->copyBodyState(
                        rigid_->self, world, description->startBody, &existenceProbe))
                || (description->attachEndToBody
                    && !rigid_->copyBodyState(
                        rigid_->self, world, description->endBody, &existenceProbe)))
            {
                setError(error, RopeError::InvalidHandle);
                return invalidRope;
            }
        }

        if (nextRopeHandle_ == invalidRope)
        {
            setError(error, RopeError::CapacityExceeded);
            return invalidRope;
        }

        RopeRecord rope;
        rope.handle = nextRopeHandle_;
        nextRopeHandle_
            = nextRopeHandle_ == std::numeric_limits<RopeHandle>::max() ? invalidRope : nextRopeHandle_ + 1;
        rope.world            = world;
        rope.segments         = description->segments;
        rope.nodeMass         = description->nodeMass;
        rope.nodeRadius       = description->nodeRadius;
        rope.damping          = description->damping;
        rope.friction         = description->friction;
        rope.collideWithWorld = description->collideWithWorld;
        rope.breakStrain      = description->breakStrain;
        // Capturing the gravity doubles as the world-handle validation: a
        // world the rigid module does not know is rejected here instead of
        // silently falling back to the default gravity.
        if (!rigid_->copyWorldGravity(rigid_->self, world, rope.gravity))
        {
            setError(error, RopeError::InvalidHandle);
            return invalidRope;
        }

        rope.positions.resize(nodeCount * 3);
        rope.velocities.assign(nodeCount * 3, 0.0);
        rope.previousPositions.resize(nodeCount * 3);
        rope.inverseMasses.assign(nodeCount, description->nodeMass > 0.0 ? 1.0 / description->nodeMass : 0.0);
        rope.attachments.assign(nodeCount, NodeAttachment::Free);
        rope.pinPositions.resize(nodeCount * 3);
        rope.attachedBodies.assign(nodeCount, engine::physics_rigid::invalidBody);
        rope.attachedLocalAnchors.assign(nodeCount * 3, 0.0);
        const double inverseSegments = 1.0 / static_cast<double>(description->segments);
        for (std::size_t node = 0; node < nodeCount; ++node)
        {
            const double fraction = static_cast<double>(node) * inverseSegments;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                rope.positions[node * 3 + axis] = description->start[axis]
                                                  + (description->end[axis] - description->start[axis]) * fraction;
            }
        }

        const double segmentLength = description->restLength * inverseSegments;
        rope.stretchConstraints.reserve(description->segments);
        for (std::uint32_t segment = 0; segment < description->segments; ++segment)
        {
            RopeRecord::StretchConstraint constraint;
            constraint.nodeA      = segment;
            constraint.nodeB      = segment + 1;
            constraint.restLength = segmentLength;
            constraint.compliance = description->stretchCompliance;
            rope.stretchConstraints.push_back(constraint);
        }
        if (description->segments >= 2)
        {
            rope.bendConstraints.reserve(description->segments - 1);
            for (std::uint32_t segment = 0; segment + 2 <= description->segments; ++segment)
            {
                RopeRecord::BendConstraint constraint;
                constraint.nodeA      = segment;
                constraint.nodeB      = segment + 2;
                constraint.segmentA   = segment;
                constraint.segmentB   = segment + 1;
                constraint.restLength = segmentLength * 2.0;
                constraint.compliance = description->bendingCompliance;
                rope.bendConstraints.push_back(constraint);
            }
        }

        // Breaking and tension bookkeeping start clean: nothing broken, zero
        // tension until the first step completes.
        rope.brokenSegments.assign(description->segments, 0u);
        rope.tensionAccumulator.assign(description->segments, 0.0);
        rope.lastTension.assign(description->segments, 0.0);

        if (description->pinStart || description->attachStartToBody)
        {
            const bool bodyAttached = description->attachStartToBody;
            rope.attachments[0]     = bodyAttached ? NodeAttachment::Body : NodeAttachment::Pinned;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                rope.pinPositions[axis] = bodyAttached ? rope.positions[axis] : description->start[axis];
            }
            if (bodyAttached)
            {
                rope.attachedBodies[0] = description->startBody;
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    rope.attachedLocalAnchors[axis] = description->startLocalAnchor[axis];
                }
                refreshAttachmentTargets(rope);
            }
            rope.inverseMasses[0] = 0.0;
        }
        if (description->pinEnd || description->attachEndToBody)
        {
            const std::size_t lastNode = nodeCount - 1;
            const bool bodyAttached   = description->attachEndToBody;
            rope.attachments[lastNode] = bodyAttached ? NodeAttachment::Body : NodeAttachment::Pinned;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                rope.pinPositions[lastNode * 3 + axis]
                    = bodyAttached ? rope.positions[lastNode * 3 + axis] : description->end[axis];
            }
            if (bodyAttached)
            {
                rope.attachedBodies[lastNode] = description->endBody;
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    rope.attachedLocalAnchors[lastNode * 3 + axis] = description->endLocalAnchor[axis];
                }
                refreshAttachmentTargets(rope);
            }
            rope.inverseMasses[lastNode] = 0.0;
        }

        rope.previousPositions = rope.positions;
        totalNodes_ += nodeCount;
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal, so ropes sharing an interval spread their work across
        // steps.
        rope.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        ropes_.push_back(std::move(rope));
        ropeIndexByHandle_.emplace(ropes_.back().handle, ropes_.size() - 1);
        return ropes_.back().handle;
    }

    void destroyRopeImpl(RopeHandle handle)
    {
        const auto iterator = ropeIndexByHandle_.find(handle);
        if (iterator == ropeIndexByHandle_.end())
        {
            return;
        }
        totalNodes_ -= ropes_[iterator->second].positions.size() / 3;
        const std::size_t index = iterator->second;
        ropeIndexByHandle_.erase(iterator);
        if (index + 1 != ropes_.size())
        {
            ropes_[index]                                 = ropes_.back();
            ropeIndexByHandle_[ropes_[index].handle]      = index;
        }
        ropes_.pop_back();
    }

    bool ropeInfoImpl(RopeHandle handle, RopeInfo *outInfo)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || outInfo == nullptr)
        {
            return false;
        }
        outInfo->segments        = rope->segments;
        outInfo->nodeCount       = static_cast<std::uint32_t>(rope->positions.size() / 3);
        outInfo->constraintCount = static_cast<std::uint32_t>(
            rope->stretchConstraints.size() + rope->bendConstraints.size());
        return true;
    }

    bool copyNodePositionsImpl(RopeHandle handle, double *outPositions, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || outPositions == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(rope->positions.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(rope->positions.begin(), rope->positions.end(), outPositions);
        return true;
    }

    bool copyNodeVelocitiesImpl(
        RopeHandle handle, double *outVelocities, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || outVelocities == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(rope->velocities.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(rope->velocities.begin(), rope->velocities.end(), outVelocities);
        return true;
    }

    bool copyRopeTensionImpl(
        RopeHandle handle, double *outTension, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || outTension == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(rope->lastTension.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(rope->lastTension.begin(), rope->lastTension.end(), outTension);
        return true;
    }

    bool setRopeLengthImpl(RopeHandle handle, double length)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || !std::isfinite(length) || length <= 0.0)
        {
            return false;
        }
        // Rest lengths retarget immediately: stretch rows take the total
        // length split evenly across the segments, and bend rows keep the
        // twice-the-segment rest they were built with, so reeling is never
        // fought by stale bend rests. No warm start exists to reset; nodes
        // respond from the next step on.
        const double segmentLength = length / static_cast<double>(rope->segments);
        for (RopeRecord::StretchConstraint &constraint : rope->stretchConstraints)
        {
            constraint.restLength = segmentLength;
        }
        for (RopeRecord::BendConstraint &constraint : rope->bendConstraints)
        {
            constraint.restLength = segmentLength * 2.0;
        }
        return true;
    }

    bool setNodePinnedImpl(RopeHandle handle, std::uint32_t nodeIndex, bool pinned)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || nodeIndex >= rope->attachments.size())
        {
            return false;
        }
        const std::size_t offset = nodeOffset(nodeIndex);
        if (pinned)
        {
            if (rope->attachments[nodeIndex] == NodeAttachment::Free)
            {
                rope->pinPositions[offset]     = rope->positions[offset];
                rope->pinPositions[offset + 1] = rope->positions[offset + 1];
                rope->pinPositions[offset + 2] = rope->positions[offset + 2];
            }
            // Pinning supersedes any attachment (a body-attached node
            // releases its body and holds its current world position),
            // matching pinNodeToPosition.
            rope->attachments[nodeIndex]    = NodeAttachment::Pinned;
            rope->inverseMasses[nodeIndex]  = 0.0;
            rope->attachedBodies[nodeIndex] = engine::physics_rigid::invalidBody;
        }
        else if (rope->attachments[nodeIndex] == NodeAttachment::Pinned)
        {
            rope->attachments[nodeIndex]   = NodeAttachment::Free;
            rope->inverseMasses[nodeIndex] = rope->nodeMass > 0.0 ? 1.0 / rope->nodeMass : 0.0;
        }
        return true;
    }

    bool pinNodeToPositionImpl(RopeHandle handle, std::uint32_t nodeIndex, const double position[3])
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || nodeIndex >= rope->attachments.size() || position == nullptr)
        {
            return false;
        }
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
        {
            return false;
        }
        rope->attachments[nodeIndex]          = NodeAttachment::Pinned;
        rope->inverseMasses[nodeIndex]        = 0.0;
        rope->attachedBodies[nodeIndex]       = engine::physics_rigid::invalidBody;
        const std::size_t offset              = nodeOffset(nodeIndex);
        rope->pinPositions[offset]            = position[0];
        rope->pinPositions[offset + 1]        = position[1];
        rope->pinPositions[offset + 2]        = position[2];
        rope->positions[offset]               = position[0];
        rope->positions[offset + 1]           = position[1];
        rope->positions[offset + 2]           = position[2];
        return true;
    }

    bool attachNodeToBodyImpl(
        RopeHandle handle, std::uint32_t nodeIndex, engine::physics_rigid::BodyHandle body, const double localAnchor[3])
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || nodeIndex >= rope->attachments.size() || localAnchor == nullptr
            || rigid_ == nullptr || body == engine::physics_rigid::invalidBody)
        {
            return false;
        }
        if (!std::isfinite(localAnchor[0]) || !std::isfinite(localAnchor[1]) || !std::isfinite(localAnchor[2]))
        {
            return false;
        }
        engine::physics_rigid::BodyState state;
        if (!rigid_->copyBodyState(rigid_->self, rope->world, body, &state))
        {
            return false;
        }
        rope->attachments[nodeIndex]    = NodeAttachment::Body;
        rope->inverseMasses[nodeIndex]  = 0.0;
        rope->attachedBodies[nodeIndex] = body;
        const std::size_t offset        = nodeOffset(nodeIndex);
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            rope->attachedLocalAnchors[offset + axis] = localAnchor[axis];
        }
        refreshAttachmentTargets(*rope);
        return true;
    }

    bool detachNodeImpl(RopeHandle handle, std::uint32_t nodeIndex)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || nodeIndex >= rope->attachments.size())
        {
            return false;
        }
        rope->attachments[nodeIndex]    = NodeAttachment::Free;
        rope->attachedBodies[nodeIndex] = engine::physics_rigid::invalidBody;
        rope->inverseMasses[nodeIndex]  = rope->nodeMass > 0.0 ? 1.0 / rope->nodeMass : 0.0;
        return true;
    }

    bool isNodeAttachedImpl(RopeHandle handle, std::uint32_t nodeIndex, bool *outAttached)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || nodeIndex >= rope->attachments.size() || outAttached == nullptr)
        {
            return false;
        }
        *outAttached = rope->attachments[nodeIndex] != NodeAttachment::Free;
        return true;
    }

    bool setWindAccelerationImpl(RopeHandle handle, const double wind[3])
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || wind == nullptr)
        {
            return false;
        }
        if (!std::isfinite(wind[0]) || !std::isfinite(wind[1]) || !std::isfinite(wind[2]))
        {
            return false;
        }
        rope->windAcceleration[0] = wind[0];
        rope->windAcceleration[1] = wind[1];
        rope->windAcceleration[2] = wind[2];
        return true;
    }

    // Internal substep count for one fidelity band: lower bands keep the
    // fixed step size and cut the substep count instead (the small-steps
    // XPBD schedule tolerates the coarser solve).
    std::uint32_t substepsForBand(engine::util::SimulationFidelity band) const
    {
        return engine::util::simulationLodSubstepsForBand(band, settings_.substepsPerStep);
    }

    bool stepImpl(RopeHandle handle, double fixedDeltaSeconds)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence; a
        // due step keeps the fixed delta and spends the band's substep
        // budget (constrained systems never enlarge the step). Cached
        // ropes freeze. Without lod_enabled every rope runs Full every
        // step.
        if (engine::util::simulationLodResolvePass(rope->lodBand,
                rope->cadence,
                settings_.lod.thresholds,
                settings_.lod.intervals,
                settings_.lod.enabled,
                rope->importance,
                &rope->lodSkippedSteps)
            == 0u)
        {
            return true;
        }
        if (!stepRope(*rope, fixedDeltaSeconds, substepsForBand(rope->lodBand)))
        {
            return false;
        }
        return true;
    }

    std::uint32_t simulateImpl(RopeHandle handle, double elapsedSeconds)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        return rope->accumulator.drain(fixedDelta, settings_.maximumStepsPerSimulate, elapsedSeconds,
            [this, handle, fixedDelta]() { return stepImpl(handle, fixedDelta); });
    }

    bool setRopeImportanceImpl(RopeHandle handle, double importance)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        rope->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool ropeStatisticsImpl(RopeHandle handle, RopeStatistics *outStatistics)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        outStatistics->stepsSimulated       = rope->stepsSimulated;
        outStatistics->substepsExecuted     = rope->substepsExecuted;
        outStatistics->lastContactCount     = rope->lastContactCount;
        outStatistics->lastStepMilliseconds = rope->lastStepMilliseconds;
        outStatistics->brokenSegmentCount   = rope->brokenSegmentCount;
        outStatistics->maximumTension       = rope->maximumTension;
        outStatistics->lodSkippedSteps      = rope->lodSkippedSteps;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsRopeCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_ROPE_BIND(field, method)                                                                        \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<RopeModule *>(self)->method(args...); }
        ENGINE_PHYSICS_ROPE_BIND(createRope, createRopeImpl);
        ENGINE_PHYSICS_ROPE_BIND(destroyRope, destroyRopeImpl);
        ENGINE_PHYSICS_ROPE_BIND(ropeInfo, ropeInfoImpl);
        ENGINE_PHYSICS_ROPE_BIND(copyNodePositions, copyNodePositionsImpl);
        ENGINE_PHYSICS_ROPE_BIND(copyNodeVelocities, copyNodeVelocitiesImpl);
        ENGINE_PHYSICS_ROPE_BIND(setNodePinned, setNodePinnedImpl);
        ENGINE_PHYSICS_ROPE_BIND(pinNodeToPosition, pinNodeToPositionImpl);
        ENGINE_PHYSICS_ROPE_BIND(attachNodeToBody, attachNodeToBodyImpl);
        ENGINE_PHYSICS_ROPE_BIND(detachNode, detachNodeImpl);
        ENGINE_PHYSICS_ROPE_BIND(isNodeAttached, isNodeAttachedImpl);
        ENGINE_PHYSICS_ROPE_BIND(setWindAcceleration, setWindAccelerationImpl);
        ENGINE_PHYSICS_ROPE_BIND(step, stepImpl);
        ENGINE_PHYSICS_ROPE_BIND(simulate, simulateImpl);
        ENGINE_PHYSICS_ROPE_BIND(ropeStatistics, ropeStatisticsImpl);
        ENGINE_PHYSICS_ROPE_BIND(setRopeImportance, setRopeImportanceImpl);
        ENGINE_PHYSICS_ROPE_BIND(copyRopeTension, copyRopeTensionImpl);
        ENGINE_PHYSICS_ROPE_BIND(setRopeLength, setRopeLengthImpl);
#undef ENGINE_PHYSICS_ROPE_BIND
    }

    ParticipantManifest manifest_;
    RopeCapability      capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    RopeSettings settings_;

    std::vector<RopeRecord>                     ropes_;
    std::unordered_map<RopeHandle, std::size_t> ropeIndexByHandle_;
    RopeHandle                                  nextRopeHandle_ = 1;
    std::size_t                                 totalNodes_     = 0;

    // Batched sphere-overlap scratch reused by the stepping path.
    std::vector<std::size_t>                               collisionSlotIndices_;
    std::vector<engine::physics_rigid::SphereOverlapQuery>  collisionQueries_;
    std::vector<engine::physics_rigid::SphereOverlapResult> collisionResults_;

    // Registration ordinal for LOD stagger phases.
    std::uint64_t registrationOrdinal_ = 0;
};

} // namespace
} // namespace engine::physics_rope

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_rope capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_rope"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_rope",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_rope::RopeModule(); },
        .providedCapabilities = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = required,
        .requiredCapabilityCount = 1,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
