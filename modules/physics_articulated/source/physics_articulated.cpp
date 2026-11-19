// physics_articulated module implementation.
//
// Figures mirror into a physics_rigid world: each link becomes a rigid body
// (with a per-figure collision category so links skip each other unless
// self-collision is requested) and each link-parent connection becomes a
// rigid hinge or ball-socket joint. The module adds the articulated layer on
// top: assembly-frame joint measurements, per-joint PD drives mapped onto
// the rigid hinge motors, and the collision-category pool. Simulation
// stepping itself stays in the rigid core; the demo calls updateDrives once
// per fixed step and steps the rigid world once.

#include "records.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_articulated
{
namespace
{

// The rigid module's public math vocabulary (double precision, operating on
// the capability's plain-data types).
namespace math = engine::physics_rigid::math;

class PhysicsArticulatedModule final : public ModuleInterface
{
public:
    PhysicsArticulatedModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_articulated", "maximum_figures");
            value && *value > 0 && *value <= 4096)
            settings_.maximumFigures = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_articulated", "maximum_links_per_figure");
            value && *value > 0 && *value <= 4096)
            settings_.maximumLinksPerFigure = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_articulated", "maximum_drive_velocity");
            value && *value >= 1.0 && *value <= 10000.0 && std::isfinite(*value))
            settings_.maximumDriveVelocity = *value;
        if (const auto value = context.configurationFloatingInSection("physics_articulated", "maximum_drive_torque");
            value && *value >= 1.0 && *value <= 1000000.0 && std::isfinite(*value))
            settings_.maximumDriveTorque = *value;

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
        releaseAllFigures();
        figures_.clear();
        figureIndexByHandle_.clear();
        nextFigureHandle_ = 1;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == ArticulatedCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_articulated",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_articulated"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    static void setError(ArticulatedError *output, ArticulatedError value) noexcept
    {
        if (output != nullptr)
        {
            *output = value;
        }
    }

    void releaseAllFigures()
    {
        for (const FigureHandle handle : figureOrder_)
        {
            destroyFigureRecord(handle);
        }
        figureOrder_.clear();
        categoryPool_ = 0;
    }

    // Releases the rigid bodies and joints of one figure and frees its
    // collision category.
    void destroyFigureRecord(FigureHandle handle)
    {
        const auto iterator = figureIndexByHandle_.find(handle);
        if (iterator == figureIndexByHandle_.end() || rigid_ == nullptr)
        {
            return;
        }
        FigureRecord &figure = figures_[iterator->second];
        for (LinkRecord &link : figure.links)
        {
            if (link.joint != engine::physics_rigid::invalidJoint)
            {
                rigid_->destroyJoint(rigid_->self, figure.world, link.joint);
            }
            if (link.body != engine::physics_rigid::invalidBody)
            {
                rigid_->destroyBody(rigid_->self, figure.world, link.body);
            }
        }
        categoryPool_ &= ~figure.categoryBit;
        figureIndexByHandle_.erase(iterator);
    }

    FigureRecord *findFigure(FigureHandle handle)
    {
        const auto iterator = figureIndexByHandle_.find(handle);
        return iterator != figureIndexByHandle_.end() ? &figures_[iterator->second] : nullptr;
    }

    // ---- capability implementations ---------------------------------------

    FigureHandle createFigureImpl(
        engine::physics_rigid::WorldHandle world, const FigureDescription *description, ArticulatedError *error)
    {
        setError(error, ArticulatedError::None);
        if (rigid_ == nullptr)
        {
            setError(error, ArticulatedError::Unsupported);
            return invalidFigure;
        }
        if (description == nullptr || description->links == nullptr || description->linkCount == 0)
        {
            setError(error, ArticulatedError::InvalidDescription);
            return invalidFigure;
        }
        if (figures_.size() >= settings_.maximumFigures || description->linkCount > settings_.maximumLinksPerFigure)
        {
            setError(error, ArticulatedError::CapacityExceeded);
            return invalidFigure;
        }

        // Parent indices must reference earlier links (trees, assembly order
        // root first) and no link may parent itself.
        for (std::uint32_t index = 0; index < description->linkCount; ++index)
        {
            const LinkDescription &link = description->links[index];
            if (link.parentIndex < -1 || link.parentIndex >= static_cast<std::int32_t>(description->linkCount)
                || link.parentIndex == static_cast<std::int32_t>(index)
                || (link.parentIndex >= 0 && static_cast<std::uint32_t>(link.parentIndex) >= index))
            {
                setError(error, ArticulatedError::InvalidDescription);
                return invalidFigure;
            }
        }

        // Collision category: one bit from the pool; exhausted means figures
        // cannot be isolated from themselves any more.
        std::uint32_t categoryBit = 0;
        if (!description->selfCollision)
        {
            for (std::uint32_t bit = 1; bit <= 31; ++bit)
            {
                if ((categoryPool_ & (1u << bit)) == 0u)
                {
                    categoryBit = 1u << bit;
                    break;
                }
            }
            if (categoryBit == 0u)
            {
                setError(error, ArticulatedError::CapacityExceeded);
                return invalidFigure;
            }
        }

        FigureRecord figure;
        figure.world           = world;
        figure.selfCollision   = description->selfCollision;
        figure.categoryBit     = categoryBit;
        figure.sleepingEnabled = description->sleepingEnabled;
        figure.links.resize(description->linkCount);

        // Create bodies and joints bottom-up; on failure release everything
        // created so far so the world never keeps partial figures.
        for (std::uint32_t index = 0; index < description->linkCount; ++index)
        {
            const LinkDescription &link   = description->links[index];
            LinkRecord            &record = figure.links[index];
            record.parentIndex            = link.parentIndex;
            record.jointToParent          = link.jointToParent;
            record.hingeAxisLocalParent   = link.hingeAxisLocalParent;
            record.drive                  = JointDrive{};

            engine::physics_rigid::BodyDescription bodyDescription;
            bodyDescription.shape = link.shape;
            if (link.parentIndex < 0)
            {
                bodyDescription.transform = link.relativeTransform;
            }
            else
            {
                // Compose parent world transform with the relative placement
                // through the rigid capability's own math vocabulary.
                engine::physics_rigid::BodyState parentState{};
                if (!rigid_->copyBodyState(rigid_->self,
                        world,
                        figure.links[static_cast<std::uint32_t>(link.parentIndex)].body,
                        &parentState))
                {
                    setError(error, ArticulatedError::InvalidDescription);
                    rollbackFigure(figure);
                    return invalidFigure;
                }
                bodyDescription.transform.position
                    = parentState.transform.position
                    + math::rotate(parentState.transform.rotation, link.relativeTransform.position);
                bodyDescription.transform.rotation = parentState.transform.rotation * link.relativeTransform.rotation;
            }
            bodyDescription.kind            = link.parentIndex < 0 && !description->rootDynamic
                                                ? engine::physics_rigid::BodyKind::Static
                                                : engine::physics_rigid::BodyKind::Dynamic;
            bodyDescription.density         = link.density;
            bodyDescription.mass            = link.mass;
            bodyDescription.material        = link.material;
            bodyDescription.sleepingEnabled = description->sleepingEnabled;
            bodyDescription.gravityEnabled  = true;
            bodyDescription.categoryBits    = categoryBit != 0u ? categoryBit : 1u;
            bodyDescription.maskBits        = categoryBit != 0u ? (~categoryBit) : 0xFFFFFFFFu;

            engine::physics_rigid::PhysicsError bodyError = engine::physics_rigid::PhysicsError::None;
            record.body = rigid_->createBody(rigid_->self, world, &bodyDescription, &bodyError);
            if (record.body == engine::physics_rigid::invalidBody)
            {
                setError(error,
                    bodyError == engine::physics_rigid::PhysicsError::CapacityExceeded
                        ? ArticulatedError::CapacityExceeded
                        : ArticulatedError::InvalidDescription);
                rollbackFigure(figure);
                return invalidFigure;
            }

            if (link.parentIndex >= 0)
            {
                engine::physics_rigid::JointDescription jointDescription;
                jointDescription.kind                 = link.jointToParent == ArticulatedJointKind::Hinge
                                                          ? engine::physics_rigid::JointKind::Hinge
                                                          : engine::physics_rigid::JointKind::BallSocket;
                jointDescription.bodyA                = figure.links[static_cast<std::uint32_t>(link.parentIndex)].body;
                jointDescription.bodyB                = record.body;
                jointDescription.anchorLocalA         = link.jointAnchorLocalParent;
                jointDescription.anchorLocalB         = link.jointAnchorLocalChild;
                jointDescription.axisLocalA           = link.hingeAxisLocalParent;
                jointDescription.limitsEnabled        = link.limitsEnabled;
                jointDescription.limitLowerRadians    = link.limitLowerRadians;
                jointDescription.limitUpperRadians    = link.limitUpperRadians;
                jointDescription.coneHalfAngleRadians = link.coneHalfAngleRadians;
                jointDescription.twistHalfAngleRadians         = link.twistHalfAngleRadians;
                engine::physics_rigid::PhysicsError jointError = engine::physics_rigid::PhysicsError::None;
                record.joint = rigid_->createJoint(rigid_->self, world, &jointDescription, &jointError);
                if (record.joint == engine::physics_rigid::invalidJoint)
                {
                    setError(error, ArticulatedError::InvalidDescription);
                    rollbackFigure(figure);
                    return invalidFigure;
                }
            }

            // Assembly reference: relative rotation parent^-1 * link.
            if (link.parentIndex >= 0)
            {
                engine::physics_rigid::BodyState parentState{};
                engine::physics_rigid::BodyState childState{};
                rigid_->copyBodyState(
                    rigid_->self, world, figure.links[static_cast<std::uint32_t>(link.parentIndex)].body, &parentState);
                rigid_->copyBodyState(rigid_->self, world, record.body, &childState);
                record.referenceRelativeRotation
                    = math::normalize(math::conjugate(parentState.transform.rotation) * childState.transform.rotation);
            }
        }

        if (nextFigureHandle_ == invalidFigure)
        {
            setError(error, ArticulatedError::CapacityExceeded);
            rollbackFigure(figure);
            return invalidFigure;
        }
        figure.handle = nextFigureHandle_;
        nextFigureHandle_
            = nextFigureHandle_ == std::numeric_limits<FigureHandle>::max() ? invalidFigure : nextFigureHandle_ + 1;
        categoryPool_ |= categoryBit;
        figures_.push_back(std::move(figure));
        figureIndexByHandle_.emplace(figures_.back().handle, figures_.size() - 1);
        figureOrder_.push_back(figures_.back().handle);
        return figures_.back().handle;
    }

    void rollbackFigure(FigureRecord &figure)
    {
        if (rigid_ == nullptr)
        {
            return;
        }
        for (LinkRecord &link : figure.links)
        {
            if (link.joint != engine::physics_rigid::invalidJoint)
            {
                rigid_->destroyJoint(rigid_->self, figure.world, link.joint);
            }
            if (link.body != engine::physics_rigid::invalidBody)
            {
                rigid_->destroyBody(rigid_->self, figure.world, link.body);
            }
        }
        figure.links.clear();
    }

    void destroyFigureImpl(FigureHandle handle)
    {
        destroyFigureRecord(handle);
        for (std::size_t index = 0; index < figureOrder_.size(); ++index)
        {
            if (figureOrder_[index] == handle)
            {
                figureOrder_[index] = figureOrder_.back();
                figureOrder_.pop_back();
                break;
            }
        }
        // Compact the record vector, keeping the handle map consistent.
        const auto iterator = figureIndexByHandle_.find(handle);
        if (iterator == figureIndexByHandle_.end() && figures_.empty())
        {
            return;
        }
        // The record was erased from the map in destroyFigureRecord; remove
        // the vector slot by swapping the last element into its place.
        for (std::size_t index = 0; index < figures_.size(); ++index)
        {
            if (figures_[index].handle == handle)
            {
                if (index + 1 != figures_.size())
                {
                    figures_[index]                              = figures_.back();
                    figureIndexByHandle_[figures_[index].handle] = index;
                }
                figures_.pop_back();
                break;
            }
        }
    }

    bool figureInfoImpl(FigureHandle handle, FigureInfo *outInfo)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || outInfo == nullptr)
        {
            return false;
        }
        std::uint32_t jointCount = 0;
        for (const LinkRecord &link : figure->links)
        {
            if (link.joint != engine::physics_rigid::invalidJoint)
            {
                ++jointCount;
            }
        }
        outInfo->linkCount  = static_cast<std::uint32_t>(figure->links.size());
        outInfo->jointCount = jointCount;
        return true;
    }

    bool figureStatisticsImpl(FigureHandle handle, ArticulatedFigureStatistics *outStatistics)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        std::uint32_t activeDrives = 0;
        for (const LinkRecord &link : figure->links)
        {
            if (link.drive.enabled)
            {
                ++activeDrives;
            }
        }
        outStatistics->linkCount        = static_cast<std::uint32_t>(figure->links.size());
        outStatistics->activeDriveCount = activeDrives;
        outStatistics->categoryBit      = figure->categoryBit;
        return true;
    }

    bool copyLinkBodyHandlesImpl(FigureHandle handle, engine::physics_rigid::BodyHandle *outHandles,
        std::uint32_t capacity, std::uint32_t *outRequired)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr)
        {
            return false;
        }
        if (outRequired != nullptr)
        {
            *outRequired = static_cast<std::uint32_t>(figure->links.size());
        }
        if (outHandles == nullptr || capacity < figure->links.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < figure->links.size(); ++index)
        {
            outHandles[index] = figure->links[index].body;
        }
        return true;
    }

    bool linkBodyHandleImpl(FigureHandle handle, std::uint32_t linkIndex, engine::physics_rigid::BodyHandle *outHandle)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || outHandle == nullptr || linkIndex >= figure->links.size())
        {
            return false;
        }
        *outHandle = figure->links[linkIndex].body;
        return true;
    }

    bool setJointDriveImpl(FigureHandle handle, std::uint32_t linkIndex, const JointDrive *drive)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || drive == nullptr || linkIndex >= figure->links.size())
        {
            return false;
        }
        LinkRecord &link = figure->links[linkIndex];
        if (link.joint == engine::physics_rigid::invalidJoint)
        {
            setError(nullptr, ArticulatedError::Unsupported);
            return false;
        }
        if (!std::isfinite(drive->stiffness) || !std::isfinite(drive->damping)
            || !std::isfinite(drive->targetAngleRadians) || !std::isfinite(drive->maximumVelocity)
            || !std::isfinite(drive->maximumTorque) || drive->maximumVelocity < 0.0 || drive->maximumTorque < 0.0)
        {
            return false;
        }
        link.drive                    = *drive;
        link.drive.stiffness          = std::clamp(drive->stiffness, 0.0, 1.0e9);
        link.drive.damping            = std::clamp(drive->damping, 0.0, 1.0e9);
        link.drive.targetAngleRadians = std::clamp(drive->targetAngleRadians, -kTwoPi, kTwoPi);
        link.drive.maximumVelocity    = std::clamp(drive->maximumVelocity, 0.0, settings_.maximumDriveVelocity);
        link.drive.maximumTorque      = std::clamp(drive->maximumTorque, 0.0, settings_.maximumDriveTorque);
        if (!link.drive.enabled)
        {
            rigid_->setJointMotor(rigid_->self, figure->world, link.joint, false, 0.0, 0.0);
        }
        return true;
    }

    bool copyJointDriveImpl(FigureHandle handle, std::uint32_t linkIndex, JointDrive *outDrive)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || outDrive == nullptr || linkIndex >= figure->links.size())
        {
            return false;
        }
        *outDrive = figure->links[linkIndex].drive;
        return true;
    }

    // Measures the joint angle of one link against its parent: the twist
    // about the hinge axis relative to the assembly frame. Ball-socket
    // drives power the twist component; swing stays limit-controlled.
    bool updateDrivesImpl(FigureHandle handle, double fixedDeltaSeconds)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || rigid_ == nullptr)
        {
            return false;
        }
        if (!(fixedDeltaSeconds > 0.0) || !std::isfinite(fixedDeltaSeconds))
        {
            return false;
        }
        for (LinkRecord &link : figure->links)
        {
            if (link.joint == engine::physics_rigid::invalidJoint || !link.drive.enabled)
            {
                continue;
            }
            engine::physics_rigid::JointDescription description{};
            if (!rigid_->copyJointDescription(rigid_->self, figure->world, link.joint, &description))
            {
                continue;
            }
            engine::physics_rigid::BodyState parentState{};
            engine::physics_rigid::BodyState childState{};
            const bool parentLinkExists = std::any_of(figure->links.begin(),
                figure->links.end(),
                [&](const LinkRecord &candidate) { return candidate.body == description.bodyA; });
            if (!parentLinkExists
                || !rigid_->copyBodyState(rigid_->self, figure->world, description.bodyA, &parentState)
                || !rigid_->copyBodyState(rigid_->self, figure->world, description.bodyB, &childState))
            {
                continue;
            }

            const engine::physics_rigid::Quaternion relativeRotation
                = math::normalize(math::conjugate(parentState.transform.rotation) * childState.transform.rotation);
            const engine::physics_rigid::Vec3 axisLocal = description.kind == engine::physics_rigid::JointKind::Hinge
                                                            ? description.axisLocalA
                                                            : engine::physics_rigid::Vec3{0.0, 1.0, 0.0};
            const double assemblyTwist = math::twistAngleRadians(referenceRelativeRotationOf(link), axisLocal);
            const double currentTwist  = math::twistAngleRadians(relativeRotation, axisLocal);
            const double angle         = currentTwist - assemblyTwist;

            // Relative angular velocity about the world hinge axis.
            const engine::physics_rigid::Vec3 axisWorld = math::rotate(parentState.transform.rotation, axisLocal);
            const engine::physics_rigid::Vec3 relativeAngularVelocity
                = childState.angularVelocity - parentState.angularVelocity;
            const double angularSpeed = math::dot(relativeAngularVelocity, axisWorld);

            double velocityTarget
                = link.drive.stiffness * (link.drive.targetAngleRadians - angle) - link.drive.damping * angularSpeed;
            velocityTarget = std::clamp(velocityTarget, -link.drive.maximumVelocity, link.drive.maximumVelocity);
            rigid_->setJointMotor(
                rigid_->self, figure->world, link.joint, true, velocityTarget, link.drive.maximumTorque);
        }
        return true;
    }

    bool copyJointStateImpl(FigureHandle handle, std::uint32_t linkIndex, FigureJointState *outState)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || outState == nullptr || rigid_ == nullptr || linkIndex >= figure->links.size())
        {
            return false;
        }
        const LinkRecord &link = figure->links[linkIndex];
        if (link.joint == engine::physics_rigid::invalidJoint)
        {
            return false;
        }
        engine::physics_rigid::JointDescription description{};
        if (!rigid_->copyJointDescription(rigid_->self, figure->world, link.joint, &description))
        {
            return false;
        }
        engine::physics_rigid::BodyState parentState{};
        engine::physics_rigid::BodyState childState{};
        if (!rigid_->copyBodyState(rigid_->self, figure->world, description.bodyA, &parentState)
            || !rigid_->copyBodyState(rigid_->self, figure->world, description.bodyB, &childState))
        {
            return false;
        }
        const engine::physics_rigid::Quaternion relativeRotation
            = math::normalize(math::conjugate(parentState.transform.rotation) * childState.transform.rotation);
        const engine::physics_rigid::Quaternion assembly = referenceRelativeRotationOf(link);
        if (description.kind == engine::physics_rigid::JointKind::Hinge)
        {
            const engine::physics_rigid::Vec3 axisLocal = description.axisLocalA;
            outState->angleRadians
                = math::twistAngleRadians(relativeRotation, axisLocal) - math::twistAngleRadians(assembly, axisLocal);
            outState->twistRadians = outState->angleRadians;
            outState->swingRadians = 0.0;
        }
        else
        {
            const engine::physics_rigid::Vec3 axisLocal{0.0, 1.0, 0.0};
            outState->twistRadians
                = math::twistAngleRadians(relativeRotation, axisLocal) - math::twistAngleRadians(assembly, axisLocal);
            outState->swingRadians
                = math::swingAngleRadians(math::conjugate(assembly) * relativeRotation, math::Vec3{0.0, 1.0, 0.0});
            outState->angleRadians = outState->twistRadians;
        }
        return true;
    }

    static engine::physics_rigid::Quaternion referenceRelativeRotationOf(const LinkRecord &link)
    {
        return link.referenceRelativeRotation;
    }

    // Assembly reference of one joint: the parent/child body pair, the
    // twist/hinge axis in the parent's frame, and the relative rotation
    // captured at figure creation.
    bool copyJointReferenceImpl(FigureHandle handle, std::uint32_t linkIndex, FigureJointReference *outReference)
    {
        FigureRecord *figure = findFigure(handle);
        if (figure == nullptr || outReference == nullptr || linkIndex >= figure->links.size())
        {
            return false;
        }
        const LinkRecord &link = figure->links[linkIndex];
        if (link.parentIndex < 0 || link.joint == engine::physics_rigid::invalidJoint)
        {
            setError(nullptr, ArticulatedError::Unsupported);
            return false;
        }
        const LinkRecord &parent = figure->links[static_cast<std::uint32_t>(link.parentIndex)];
        *outReference            = FigureJointReference{};
        outReference->parentBody = parent.body;
        outReference->childBody  = link.body;
        outReference->axisLocalParent[0] = link.hingeAxisLocalParent.x;
        outReference->axisLocalParent[1] = link.hingeAxisLocalParent.y;
        outReference->axisLocalParent[2] = link.hingeAxisLocalParent.z;
        const engine::physics_rigid::Quaternion reference = referenceRelativeRotationOf(link);
        outReference->referenceRelativeRotation[0] = reference.x;
        outReference->referenceRelativeRotation[1] = reference.y;
        outReference->referenceRelativeRotation[2] = reference.z;
        outReference->referenceRelativeRotation[3] = reference.w;
        outReference->kind = link.jointToParent == ArticulatedJointKind::Hinge
                                ? engine::physics_rigid::JointKind::Hinge
                                : engine::physics_rigid::JointKind::BallSocket;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsArticulatedCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_ARTICULATED_BIND(field, method)                                                                 \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    { return static_cast<PhysicsArticulatedModule *>(self)->method(args...); }
        ENGINE_PHYSICS_ARTICULATED_BIND(createFigure, createFigureImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(destroyFigure, destroyFigureImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(figureInfo, figureInfoImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(figureStatistics, figureStatisticsImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(copyLinkBodyHandles, copyLinkBodyHandlesImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(linkBodyHandle, linkBodyHandleImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(setJointDrive, setJointDriveImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(copyJointDrive, copyJointDriveImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(updateDrives, updateDrivesImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(copyJointState, copyJointStateImpl);
        ENGINE_PHYSICS_ARTICULATED_BIND(copyJointReference, copyJointReferenceImpl);
#undef ENGINE_PHYSICS_ARTICULATED_BIND
    }

    static constexpr double kTwoPi = 6.283185307179586;

    ParticipantManifest   manifest_;
    ArticulatedCapability capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    ArticulatedSettings settings_;

    std::vector<FigureRecord>                     figures_;
    std::unordered_map<FigureHandle, std::size_t> figureIndexByHandle_;
    std::vector<FigureHandle>                     figureOrder_;
    FigureHandle                                  nextFigureHandle_ = 1;
    std::uint32_t                                 categoryPool_     = 0;
};

} // namespace

} // namespace engine::physics_articulated

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_articulated capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_articulated"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion    = engine::engineModuleAbiVersion,
        .moduleName    = "physics_articulated",
        .moduleVersion = "0.1.0",
        .createModule  = []() -> engine::ModuleInterface *
        { return new engine::physics_articulated::PhysicsArticulatedModule(); },
        .providedCapabilities    = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = required,
        .requiredCapabilityCount = 1,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
