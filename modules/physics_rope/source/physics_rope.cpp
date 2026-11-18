// physics_rope module implementation.
//
// XPBD rope chains over a physics_rigid world: nodes integrate velocities,
// stretch and bending constraints solve in Gauss-Seidel order with
// compliance-scaled corrections (accumulating a per-segment tension readout),
// over-stretched segments break permanently when the description enables it,
// attached nodes follow rigid bodies or pin positions, and free nodes resolve
// against the rigid world's bodies and planes through the collideSphere query.
// Stepping happens in whole fixed steps with internal substeps (the "small
// steps" XPBD schedule), so the simulation is independent of frame rate.

#include "records.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
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

    static void setError(RopeError *output, RopeError value) noexcept
    {
        if (output != nullptr)
        {
            *output = value;
        }
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
        const double inverseDeltaTimeSquared = 1.0 / (substepDelta * substepDelta);
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
            solveDistanceConstraint(rope,
                constraint.nodeA,
                constraint.nodeB,
                constraint.restLength,
                constraint.compliance,
                inverseDeltaTimeSquared,
                &rope.tensionAccumulator[index]);
        }
        for (const RopeRecord::BendConstraint &constraint : rope.bendConstraints)
        {
            // A bend row spans the two stretch segments between its node
            // pair; either one breaking releases the row, since holding the
            // two-apart distance across a break would drag the separated
            // pieces back together.
            if (rope.brokenSegments[constraint.nodeA] != 0u || rope.brokenSegments[constraint.nodeA + 1] != 0u)
            {
                continue;
            }
            solveDistanceConstraint(rope,
                constraint.nodeA,
                constraint.nodeB,
                constraint.restLength,
                constraint.compliance,
                inverseDeltaTimeSquared,
                nullptr);
        }
    }

    void solveDistanceConstraint(RopeRecord &rope, std::uint32_t nodeA, std::uint32_t nodeB, double restLength,
        double compliance, double inverseDeltaTimeSquared, double *lambdaAccumulator)
    {
        const std::size_t offsetA = nodeOffset(nodeA);
        const std::size_t offsetB = nodeOffset(nodeB);
        const double      inverseMassA = rope.inverseMasses[nodeA];
        const double      inverseMassB = rope.inverseMasses[nodeB];
        const double      weightSum    = inverseMassA + inverseMassB;
        if (weightSum <= 0.0)
        {
            return;
        }
        const double deltaX = rope.positions[offsetB] - rope.positions[offsetA];
        const double deltaY = rope.positions[offsetB + 1] - rope.positions[offsetA + 1];
        const double deltaZ = rope.positions[offsetB + 2] - rope.positions[offsetA + 2];
        const double length = std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
        if (length <= 1.0e-12)
        {
            return;
        }
        const double complianceScaled = compliance * inverseDeltaTimeSquared;
        const double lambda           = (-length + restLength) / (weightSum + complianceScaled);
        if (lambdaAccumulator != nullptr)
        {
            // Tension bookkeeping: the stretch row's |lambda| lands in its
            // owning segment's accumulator (bend rows pass nullptr).
            *lambdaAccumulator += std::abs(lambda);
        }
        const double scaleX = deltaX / length * lambda;
        const double scaleY = deltaY / length * lambda;
        const double scaleZ = deltaZ / length * lambda;
        rope.positions[offsetA] -= inverseMassA * scaleX;
        rope.positions[offsetA + 1] -= inverseMassA * scaleY;
        rope.positions[offsetA + 2] -= inverseMassA * scaleZ;
        rope.positions[offsetB] += inverseMassB * scaleX;
        rope.positions[offsetB + 1] += inverseMassB * scaleY;
        rope.positions[offsetB + 2] += inverseMassB * scaleZ;
    }

    void resolveCollisions(RopeRecord &rope)
    {
        rope.lastContactCount = 0;
        if (rigid_ == nullptr || !rope.collideWithWorld)
        {
            return;
        }
        const std::size_t nodeCount = rope.positions.size() / 3;
        for (std::size_t node = 0; node < nodeCount; ++node)
        {
            const std::size_t offset = node * 3;
            if (rope.attachments[node] != NodeAttachment::Free)
            {
                continue;
            }
            double                            normal[3] = {0.0, 0.0, 0.0};
            double                            depth     = 0.0;
            engine::physics_rigid::BodyHandle body      = engine::physics_rigid::invalidBody;
            if (!rigid_->collideSphere(
                    rigid_->self, rope.world, &rope.positions[offset], rope.nodeRadius, normal, &depth, &body)
                || depth <= 0.0)
            {
                continue;
            }
            // Store the contact for the velocity pass through the pin slot
            // scratch: contacts land in contactNormals_ keyed by node.
            rope.lastContactCount += 1;
            contactFlags_[node] = 1u;
            contactNormals_[node * 3]     = normal[0];
            contactNormals_[node * 3 + 1] = normal[1];
            contactNormals_[node * 3 + 2] = normal[2];
            // Project the node out of the obstacle along the push-out
            // direction; velocity damping happens in the final velocity pass.
            rope.positions[offset] += normal[0] * depth;
            rope.positions[offset + 1] += normal[1] * depth;
            rope.positions[offset + 2] += normal[2] * depth;
        }
    }

    void finalizeSubstepVelocities(RopeRecord &rope, double substepDelta)
    {
        const std::size_t nodeCount    = rope.positions.size() / 3;
        const double      inverseDeltaTime = 1.0 / substepDelta;
        for (std::size_t node = 0; node < nodeCount; ++node)
        {
            const std::size_t offset = node * 3;
            if (rope.attachments[node] != NodeAttachment::Free)
            {
                continue;
            }
            double velocityX = (rope.positions[offset] - rope.previousPositions[offset]) * inverseDeltaTime;
            double velocityY = (rope.positions[offset + 1] - rope.previousPositions[offset + 1]) * inverseDeltaTime;
            double velocityZ = (rope.positions[offset + 2] - rope.previousPositions[offset + 2]) * inverseDeltaTime;
            if (contactFlags_[node] != 0u)
            {
                // Split into normal and tangential parts; the normal part is
                // removed and the tangential part scales by the friction
                // factor (the fraction of sliding velocity kept per substep).
                const double normalX        = contactNormals_[node * 3];
                const double normalY        = contactNormals_[node * 3 + 1];
                const double normalZ        = contactNormals_[node * 3 + 2];
                const double normalVelocity = normalX * velocityX + normalY * velocityY + normalZ * velocityZ;
                velocityX -= normalX * normalVelocity;
                velocityY -= normalY * normalVelocity;
                velocityZ -= normalZ * normalVelocity;
                velocityX *= rope.friction;
                velocityY *= rope.friction;
                velocityZ *= rope.friction;
            }
            rope.velocities[offset]     = velocityX;
            rope.velocities[offset + 1] = velocityY;
            rope.velocities[offset + 2] = velocityZ;
        }
    }

    bool stepRope(RopeRecord &rope, double fixedDeltaSeconds)
    {
        const auto stepStart = std::chrono::steady_clock::now();
        if (!(fixedDeltaSeconds > 0.0) || fixedDeltaSeconds > 0.1 || !std::isfinite(fixedDeltaSeconds))
        {
            return false;
        }
        const std::size_t   nodeCount    = rope.positions.size() / 3;
        contactFlags_.assign(nodeCount, 0u);
        contactNormals_.assign(nodeCount * 3, 0.0);
        rope.tensionAccumulator.assign(rope.segments, 0.0); // fresh |lambda| sums for this step
        const std::uint32_t substeps     = settings_.substepsPerStep;
        const double        substepDelta = fixedDeltaSeconds / static_cast<double>(substeps);
        for (std::uint32_t substep = 0; substep < substeps; ++substep)
        {
            refreshAttachmentTargets(rope);
            integrateSubstep(rope, substepDelta);
            solveConstraints(rope, substepDelta);
            resolveCollisions(rope);
            finalizeSubstepVelocities(rope, substepDelta);
        }
        rope.lastContactCount = static_cast<std::uint32_t>(
            std::count(contactFlags_.begin(), contactFlags_.end(), std::uint8_t{1}));
        bool finite = true;
        for (const double value : rope.positions)
        {
            if (!std::isfinite(value))
            {
                finite = false;
                break;
            }
        }
        if (!finite)
        {
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
        if (rigid_ == nullptr || description == nullptr)
        {
            setError(error, RopeError::InvalidDescription);
            return invalidRope;
        }
        if (ropes_.size() >= settings_.maximumRopes || totalNodes_ >= settings_.maximumNodes)
        {
            setError(error, RopeError::CapacityExceeded);
            return invalidRope;
        }
        if (description->segments < 1 || description->segments + 1 > settings_.maximumNodesPerRope
            || totalNodes_ + description->segments + 1 > settings_.maximumNodes)
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
        if (!rigid_->copyWorldGravity(rigid_->self, world, rope.gravity))
        {
            rope.gravity[0] = 0.0;
            rope.gravity[1] = -9.81;
            rope.gravity[2] = 0.0;
        }

        const std::size_t nodeCount = static_cast<std::size_t>(description->segments) + 1;
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
        if (pinned && rope->attachments[nodeIndex] == NodeAttachment::Free)
        {
            rope->attachments[nodeIndex]          = NodeAttachment::Pinned;
            rope->inverseMasses[nodeIndex]        = 0.0;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                rope->pinPositions[offset + axis] = rope->positions[offset + axis];
            }
        }
        else if (!pinned && rope->attachments[nodeIndex] == NodeAttachment::Pinned)
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

    bool stepImpl(RopeHandle handle, double fixedDeltaSeconds)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr)
        {
            return false;
        }
        return stepRope(*rope, fixedDeltaSeconds);
    }

    std::uint32_t simulateImpl(RopeHandle handle, double elapsedSeconds)
    {
        RopeRecord *rope = findRope(handle);
        if (rope == nullptr || !std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        rope->simulatedTimeBudget += elapsedSeconds;
        std::uint32_t executed = 0;
        while (rope->simulatedTimeBudget >= fixedDelta && executed < settings_.maximumStepsPerSimulate)
        {
            rope->simulatedTimeBudget -= fixedDelta;
            if (!stepRope(*rope, fixedDelta))
            {
                break;
            }
            ++executed;
        }
        if (executed == settings_.maximumStepsPerSimulate)
        {
            rope->simulatedTimeBudget = 0.0;
        }
        return executed;
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

    // Per-substep collision scratch shared by the stepping path.
    std::vector<std::uint8_t> contactFlags_;
    std::vector<double>       contactNormals_;
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
