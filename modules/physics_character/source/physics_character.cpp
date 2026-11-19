// physics_character module implementation.
//
// Character physics assembled from the rigid and articulated capabilities:
// a humanoid ragdoll template builder, active-ragdoll character control
// (pose targets through the articulated PD drives, a drive-strength blend,
// and an upright assist torque), jiggle chains for secondary motion, and a
// procedural two-foot planter with ground ray casts and a two-bone IK
// solve. The module owns no simulation state beyond control records; all
// forces and bodies live in the required capabilities.

#include "records.hpp"

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_character/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_character
{
namespace
{

namespace math = engine::physics_rigid::math;
// Humanoid template geometry at scale 1 (meters, pelvis origin). Part
// order: pelvis (root), torso, head, upper arm L, lower arm L, upper arm R,
// lower arm R, upper leg L, lower leg L, upper leg R, lower leg R.

class CharacterModule final : public ModuleInterface
{
public:
    CharacterModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_character", "maximum_characters");
            value && *value > 0 && *value <= 4096)
            settings_.maximumCharacters = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_character", "maximum_jiggle_chains");
            value && *value > 0 && *value <= 4096)
            settings_.maximumJiggleChains = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_character", "maximum_foot_planters");
            value && *value > 0 && *value <= 4096)
            settings_.maximumFootPlanters = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_character", "maximum_hand_planters");
            value && *value > 0 && *value <= 4096)
            settings_.maximumHandPlanters = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_character", "maximum_lean_controllers");
            value && *value > 0 && *value <= 4096)
            settings_.maximumLeanControllers = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_character", "jiggle_gravity_magnitude");
            value && *value > 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.jiggleGravityMagnitude = *value;
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_character");

        rigid_ = context.tryAcquireCapability<engine::physics_rigid::PhysicsRigidCapability>();
        if (rigid_ != nullptr && rigid_->abi != engine::physics_rigid::physicsRigidCapabilityAbi)
        {
            rigid_ = nullptr;
        }
        articulated_ = context.tryAcquireCapability<engine::physics_articulated::ArticulatedCapability>();
        if (articulated_ != nullptr
            && articulated_->abi != engine::physics_articulated::physicsArticulatedCapabilityAbi)
        {
            articulated_ = nullptr;
        }
    }

    void start() override
    {}

    void stop() override
    {
        characters_.clear();
        characterIndexByHandle_.clear();
        chains_.clear();
        chainIndexByHandle_.clear();
        planters_.clear();
        planterIndexByHandle_.clear();
        handPlanters_.clear();
        handPlanterIndexByHandle_.clear();
        leanControllers_.clear();
        leanIndexByHandle_.clear();
        nextCharacterHandle_    = 1;
        nextChainHandle_        = 1;
        nextPlanterHandle_      = 1;
        nextHandPlanterHandle_  = 1;
        nextLeanHandle_         = 1;
        registrationOrdinal_    = 0;
        statistics_             = CharacterStatistics{};
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == CharacterCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_character",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_character"},
            .requiredCapabilities = {"physics_rigid", "physics_articulated"}};
    }

    static void setError(CharacterError *output, CharacterError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    CharacterRecord *findCharacter(CharacterHandle handle)
    {
        const auto iterator = characterIndexByHandle_.find(handle);
        return iterator != characterIndexByHandle_.end() ? &characters_[iterator->second] : nullptr;
    }

    JiggleChainRecord *findChain(JiggleChainHandle handle)
    {
        const auto iterator = chainIndexByHandle_.find(handle);
        return iterator != chainIndexByHandle_.end() ? &chains_[iterator->second] : nullptr;
    }

    FootPlanterRecord *findPlanter(FootPlanterHandle handle)
    {
        const auto iterator = planterIndexByHandle_.find(handle);
        return iterator != planterIndexByHandle_.end() ? &planters_[iterator->second] : nullptr;
    }

    HandPlanterRecord *findHandPlanter(HandPlanterHandle handle)
    {
        const auto iterator = handPlanterIndexByHandle_.find(handle);
        return iterator != handPlanterIndexByHandle_.end() ? &handPlanters_[iterator->second] : nullptr;
    }

    LeanRecord *findLean(LeanHandle handle)
    {
        const auto iterator = leanIndexByHandle_.find(handle);
        return iterator != leanIndexByHandle_.end() ? &leanControllers_[iterator->second] : nullptr;
    }

    static bool finiteArray(const double *values, std::uint32_t count)
    {
        for (std::uint32_t index = 0; index < count; ++index)
        {
            if (!std::isfinite(values[index]))
            {
                return false;
            }
        }
        return true;
    }

    // ---- ragdoll construction ----------------------------------------------

    RagdollInfo createHumanoidRagdollImpl(
        engine::physics_rigid::WorldHandle world, const RagdollParameters *parameters, CharacterError *error)
    {
        setError(error, CharacterError::None);
        RagdollInfo info;
        if (rigid_ == nullptr || articulated_ == nullptr || parameters == nullptr)
        {
            setError(error, CharacterError::InvalidDescription);
            return info;
        }
        engine::physics_articulated::ArticulatedError articulatedError
            = engine::physics_articulated::ArticulatedError::None;
        if (!finiteArray(parameters->position, 3) || !finiteArray(parameters->rotation, 4)
            || !std::isfinite(parameters->scale) || parameters->scale <= 0.0
            || !std::isfinite(parameters->totalMass) || parameters->totalMass <= 0.0
            || !std::isfinite(parameters->friction) || parameters->friction < 0.0)
        {
            setError(error, CharacterError::InvalidDescription);
            return info;
        }
        math::Quaternion rotation = math::normalize(math::Quaternion{
            parameters->rotation[0], parameters->rotation[1], parameters->rotation[2], parameters->rotation[3]});

        engine::physics_articulated::FigureDescription figure;
        std::vector<engine::physics_articulated::LinkDescription> links(kHumanoidPartCount);
        for (std::uint32_t part = 0; part < kHumanoidPartCount; ++part)
        {
            const HumanoidPart &templatePart = kHumanoidParts[part];
            engine::physics_articulated::LinkDescription &link = links[part];
            link.parentIndex = templatePart.parentIndex;
            engine::physics_rigid::ShapeDescription &shape = link.shape;
            shape.kind = templatePart.shapeKind;
            if (shape.kind == engine::physics_rigid::ShapeKind::Box)
            {
                shape.boxHalfExtents[0] = templatePart.boxHalf[0] * parameters->scale;
                shape.boxHalfExtents[1] = templatePart.boxHalf[1] * parameters->scale;
                shape.boxHalfExtents[2] = templatePart.boxHalf[2] * parameters->scale;
            }
            else if (shape.kind == engine::physics_rigid::ShapeKind::Capsule)
            {
                shape.capsuleRadius     = templatePart.capsuleRadius * parameters->scale;
                shape.capsuleHalfLength = templatePart.capsuleHalfLength * parameters->scale;
            }
            else
            {
                shape.sphereRadius = templatePart.sphereRadius * parameters->scale;
            }
            link.density      = 0.0;
            link.mass         = parameters->totalMass * templatePart.massFraction / kHumanoidMassFractionSum;
            link.material     = engine::physics_rigid::MaterialDescription{parameters->friction, 0.0, 0.0};
            if (templatePart.parentIndex < 0)
            {
                link.relativeTransform.position = math::Vec3{parameters->position[0], parameters->position[1], parameters->position[2]};
                link.relativeTransform.rotation = rotation;
            }
            else
            {
                link.relativeTransform.position = math::Vec3{templatePart.relativePosition[0] * parameters->scale,
                    templatePart.relativePosition[1] * parameters->scale,
                    templatePart.relativePosition[2] * parameters->scale};
                link.relativeTransform.rotation = math::Quaternion{0.0, 0.0, 0.0, 1.0};
            }
            link.jointToParent = templatePart.hinge ? engine::physics_articulated::ArticulatedJointKind::Hinge
                                                    : engine::physics_articulated::ArticulatedJointKind::BallSocket;
            link.jointAnchorLocalChild = math::Vec3{templatePart.jointAnchorChild[0] * parameters->scale,
                templatePart.jointAnchorChild[1] * parameters->scale,
                templatePart.jointAnchorChild[2] * parameters->scale};
            link.jointAnchorLocalParent = math::Vec3{templatePart.jointAnchorParent[0] * parameters->scale,
                templatePart.jointAnchorParent[1] * parameters->scale,
                templatePart.jointAnchorParent[2] * parameters->scale};
            link.hingeAxisLocalParent   = math::Vec3{templatePart.hingeAxis[0], templatePart.hingeAxis[1], templatePart.hingeAxis[2]};
            link.limitsEnabled          = templatePart.limits;
            link.limitLowerRadians      = templatePart.limitLower;
            link.limitUpperRadians      = templatePart.limitUpper;
            link.coneHalfAngleRadians   = templatePart.coneHalfAngle;
            link.twistHalfAngleRadians  = templatePart.twistHalfAngle;
        }
        figure.links       = links.data();
        figure.linkCount   = static_cast<std::uint32_t>(links.size());
        figure.selfCollision = false;
        figure.rootDynamic   = !parameters->rootAnchored;
        figure.sleepingEnabled = parameters->sleepingEnabled;

        info.figure = articulated_->createFigure(articulated_->self, world, &figure, &articulatedError);
        if (info.figure == engine::physics_articulated::invalidFigure)
        {
            // Propagate the articulated module's rejection: a full figure
            // budget reports capacity, everything else (a rejected link
            // description, an unknown world) reports a bad description.
            setError(error, articulatedError == engine::physics_articulated::ArticulatedError::CapacityExceeded
                                ? CharacterError::CapacityExceeded
                                : CharacterError::InvalidDescription);
            return info;
        }
        info.partCount = kHumanoidPartCount;
        return info;
    }

    // ---- active ragdoll / physical animation --------------------------------

    CharacterHandle createCharacterImpl(const CharacterDescription *description, CharacterError *error)
    {
        setError(error, CharacterError::None);
        if (articulated_ == nullptr || rigid_ == nullptr || description == nullptr)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidCharacter;
        }
        engine::physics_articulated::FigureInfo figureInfo;
        if (description->figure == engine::physics_articulated::invalidFigure
            || !articulated_->figureInfo(articulated_->self, description->figure, &figureInfo))
        {
            setError(error, CharacterError::InvalidHandle);
            return invalidCharacter;
        }
        if (description->torsoLinkIndex >= figureInfo.linkCount || !std::isfinite(description->uprightTorque)
            || description->uprightTorque < 0.0)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidCharacter;
        }
        // The character's world must be the figure's rigid world: probe it
        // through the rigid capability so a mismatched handle is rejected at
        // creation instead of silently skipping every joint drive and the
        // upright assist on each update.
        {
            double worldGravity[3] = {0.0, -9.81, 0.0};
            if (!rigid_->copyWorldGravity(rigid_->self, description->world, worldGravity))
            {
                setError(error, CharacterError::InvalidHandle);
                return invalidCharacter;
            }
        }
        if (characters_.size() >= settings_.maximumCharacters)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidCharacter;
        }
        engine::physics_rigid::BodyHandle torsoBody = engine::physics_rigid::invalidBody;
        if (!articulated_->linkBodyHandle(
                articulated_->self, description->figure, description->torsoLinkIndex, &torsoBody)
            || torsoBody == engine::physics_rigid::invalidBody)
        {
            setError(error, CharacterError::InvalidHandle);
            return invalidCharacter;
        }
        if (nextCharacterHandle_ == invalidCharacter)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidCharacter;
        }
        CharacterRecord character;
        character.handle        = nextCharacterHandle_;
        nextCharacterHandle_    = nextCharacterHandle_ == std::numeric_limits<CharacterHandle>::max()
                                      ? invalidCharacter
                                      : nextCharacterHandle_ + 1;
        character.world         = description->world;
        character.figure        = description->figure;
        character.torsoLinkIndex = description->torsoLinkIndex;
        character.torsoBody     = torsoBody;
        character.uprightTorque = description->uprightTorque;
        character.joints.assign(figureInfo.linkCount, CharacterRecord::JointTarget{});
        // The articulated figure does not expose its world handle; the
        // consumer's fixed update passes the world implicitly through the
        // rigid calls on the torso body handle.
        // Stagger phase from the registration ordinal (shared counter
        // across every record type, like the fields module).
        character.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        characters_.push_back(std::move(character));
        characterIndexByHandle_.emplace(characters_.back().handle, characters_.size() - 1);
        statistics_.characterCount = static_cast<std::uint32_t>(characters_.size());
        return characters_.back().handle;
    }

    void destroyCharacterImpl(CharacterHandle handle)
    {
        const auto iterator = characterIndexByHandle_.find(handle);
        if (iterator == characterIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        characterIndexByHandle_.erase(iterator);
        if (index + 1 != characters_.size())
        {
            characters_[index]                                       = characters_.back();
            characterIndexByHandle_[characters_[index].handle]       = index;
        }
        characters_.pop_back();
        statistics_.characterCount = static_cast<std::uint32_t>(characters_.size());
    }

    bool setCharacterModeImpl(CharacterHandle handle, CharacterMode mode)
    {
        CharacterRecord *character = findCharacter(handle);
        if (character == nullptr)
        {
            return false;
        }
        character->mode = mode;
        return true;
    }

    bool setCharacterDriveStrengthImpl(CharacterHandle handle, double strength)
    {
        CharacterRecord *character = findCharacter(handle);
        if (character == nullptr || !std::isfinite(strength) || strength < 0.0 || strength > 1.0)
        {
            return false;
        }
        character->driveStrength = strength;
        return true;
    }

    bool setCharacterJointTargetImpl(CharacterHandle handle, std::uint32_t linkIndex, double targetAngleRadians)
    {
        CharacterRecord *character = findCharacter(handle);
        if (character == nullptr || linkIndex >= character->joints.size() || !std::isfinite(targetAngleRadians))
        {
            return false;
        }
        if (linkIndex == 0)
        {
            // The root link carries no joint; the Unsupported error is
            // reserved exactly for this case.
            return false; // the root link has no joint
        }
        character->joints[linkIndex].enabled            = true;
        character->joints[linkIndex].targetAngleRadians = targetAngleRadians;
        return true;
    }

    bool setCharacterJointGainsImpl(
        CharacterHandle handle, std::uint32_t linkIndex, double stiffness, double damping, double maximumTorque)
    {
        CharacterRecord *character = findCharacter(handle);
        if (character == nullptr || linkIndex >= character->joints.size() || !std::isfinite(stiffness)
            || stiffness < 0.0 || !std::isfinite(damping) || damping < 0.0 || !std::isfinite(maximumTorque)
            || maximumTorque < 0.0)
        {
            return false;
        }
        if (linkIndex == 0)
        {
            return false; // the root link has no joint, like setCharacterJointTarget
        }
        character->joints[linkIndex].stiffness     = stiffness;
        character->joints[linkIndex].damping       = damping;
        character->joints[linkIndex].maximumTorque = maximumTorque;
        return true;
    }

    bool updateCharacterImpl(CharacterHandle handle, double fixedDeltaSeconds)
    {
        CharacterRecord *character = findCharacter(handle);
        if (character == nullptr || articulated_ == nullptr || rigid_ == nullptr
            || character->world == engine::physics_rigid::invalidWorld || !std::isfinite(fixedDeltaSeconds)
            || fixedDeltaSeconds <= 0.0 || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence. A due
        // update scales its PD and upright torques by the interval so the
        // impulse per wall time matches the full-rate schedule (the gain
        // bounds account for the enlarged step, because the torque acts
        // once per interval), and the Cached band stops driving entirely -
        // the ragdoll goes passive until the score rises. Without
        // lod_enabled every character updates every step.
        const std::uint32_t lodInterval = engine::util::simulationLodResolvePass(character->lodBand,
            character->cadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            character->importance,
            &character->lodSkippedSteps);
        if (lodInterval == 0u)
        {
            return true;
        }
        const double driveTimeScale = static_cast<double>(lodInterval);
        const bool  drivesActive    = character->mode != CharacterMode::Dynamic;
        if (drivesActive && character->driveStrength > 0.0)
        {
            // Pose tracking: a torque PD on every enabled joint target. The
            // target frame is the assembly pose composed with the scalar
            // target angle about the joint axis. Hinges receive their torque
            // projected onto the hinge axis; ball-sockets receive the full
            // three-axis pose torque. Each row applies an equal and opposite
            // pair (child +, parent -) so the control acts like a muscle
            // across the joint, and the magnitude clamp bounds every muscle
            // to its torque budget. Unlike a velocity-motor servo this
            // direct-torque form stays stable at any gain the caller picks.
            for (std::uint32_t linkIndex = 1; linkIndex < character->joints.size(); ++linkIndex)
            {
                const CharacterRecord::JointTarget &target = character->joints[linkIndex];
                if (!target.enabled)
                {
                    continue;
                }
                engine::physics_articulated::FigureJointReference reference;
                if (!articulated_->copyJointReference(articulated_->self, character->figure, linkIndex, &reference)
                    || reference.parentBody == engine::physics_rigid::invalidBody
                    || reference.childBody == engine::physics_rigid::invalidBody)
                {
                    continue;
                }
                engine::physics_rigid::BodyState parentState;
                engine::physics_rigid::BodyState childState;
                if (!rigid_->copyBodyState(rigid_->self, character->world, reference.parentBody, &parentState)
                    || !rigid_->copyBodyState(rigid_->self, character->world, reference.childBody, &childState))
                {
                    continue;
                }
                const math::Vec3 axisLocal{reference.axisLocalParent[0],
                    reference.axisLocalParent[1],
                    reference.axisLocalParent[2]};
                const math::Vec3 axisWorld = math::normalized(math::rotate(parentState.transform.rotation, axisLocal));
                const math::Quaternion relativeRotation
                    = math::normalize(math::conjugate(parentState.transform.rotation) * childState.transform.rotation);
                const math::Quaternion assemblyRotation = math::normalize(math::Quaternion{
                    reference.referenceRelativeRotation[0],
                    reference.referenceRelativeRotation[1],
                    reference.referenceRelativeRotation[2],
                    reference.referenceRelativeRotation[3]});
                const double halfAngle                 = target.targetAngleRadians * 0.5;
                const math::Quaternion targetTwist{axisLocal.x * std::sin(halfAngle),
                    axisLocal.y * std::sin(halfAngle),
                    axisLocal.z * std::sin(halfAngle),
                    std::cos(halfAngle)};
                const math::Quaternion targetRotation  = math::normalize(assemblyRotation * targetTwist);
                const math::Quaternion frameError      = math::conjugate(targetRotation) * relativeRotation;
                const math::Vec3 omegaRelative         = childState.angularVelocity - parentState.angularVelocity;
                // Gain bounds from the child limb's inertia about the joint
                // axis: a torque PD is only discretely stable when the
                // damping and stiffness cannot flip the limb's angular
                // velocity within one step, so heavy limbs keep the caller's
                // gains while light limbs get them scaled down.
                double stiffness                       = target.stiffness * character->driveStrength;
                double damping                         = target.damping * character->driveStrength;
                const double maximumTorque             = target.maximumTorque * character->driveStrength;
                engine::physics_rigid::BodyMassProperties childMass;
                if (rigid_->copyBodyMassProperties(
                        rigid_->self, character->world, reference.childBody, &childMass)
                    && childMass.mass > 0.0)
                {
                    const math::Vec3 axisLocalBody
                        = math::rotate(math::conjugate(childState.transform.rotation), axisWorld);
                    const double limbInertia = childMass.inertiaLocal.x * axisLocalBody.x * axisLocalBody.x
                                             + childMass.inertiaLocal.y * axisLocalBody.y * axisLocalBody.y
                                             + childMass.inertiaLocal.z * axisLocalBody.z * axisLocalBody.z;
                    if (limbInertia > 1.0e-9)
                    {
                        // The bound uses the drive's effective step: the
                        // interval-scaled torque acts for one fixed step
                        // every interval steps.
                        const double effectiveStep = fixedDeltaSeconds * driveTimeScale;
                        damping = std::min(damping, 1.6 * limbInertia / effectiveStep);
                        stiffness = std::min(stiffness, damping * damping / (4.0 * limbInertia));
                    }
                }
                math::Vec3 torque;
                if (reference.kind == engine::physics_rigid::JointKind::Hinge)
                {
                    // One degree of freedom: the twist error about the axis.
                    const double errorAngle = math::dot(math::rotationVectorRadians(frameError), axisLocal);
                    const double omegaAngle = math::dot(omegaRelative, axisWorld);
                    torque                  = axisWorld * (stiffness * errorAngle - damping * omegaAngle);
                }
                else
                {
                    const math::Vec3 errorLocal = math::rotationVectorRadians(frameError);
                    const math::Vec3 errorWorld = math::rotate(parentState.transform.rotation, errorLocal);
                    torque                      = errorWorld * stiffness - omegaRelative * damping;
                }
                torque = torque * driveTimeScale;
                const double maximumScaledTorque = maximumTorque * driveTimeScale;
                if (math::length(torque) > maximumScaledTorque)
                {
                    torque = torque * (maximumScaledTorque / math::length(torque));
                }
                const double childTorque[3]  = {torque.x, torque.y, torque.z};
                const double parentTorque[3] = {-torque.x, -torque.y, -torque.z};
                rigid_->applyTorque(rigid_->self, character->world, reference.childBody, childTorque);
                rigid_->applyTorque(rigid_->self, character->world, reference.parentBody, parentTorque);
            }

            // Upright assist: torque that rotates the torso's up vector
            // toward the world up, self-limiting through the sine of the
            // tilt angle carried in the cross product magnitude. The cross
            // product axis already points along the corrective rotation.
            engine::physics_rigid::BodyState torsoState;
            if (character->uprightTorque > 0.0 && character->driveStrength > 0.0
                && rigid_->copyBodyState(rigid_->self, character->world, character->torsoBody, &torsoState))
            {
                const math::Vec3 bodyUp = math::rotate(torsoState.transform.rotation, math::Vec3{0.0, 1.0, 0.0});
                const math::Vec3 axis   = math::cross(bodyUp, math::Vec3{0.0, 1.0, 0.0});
                const math::Vec3 torque
                    = axis * (character->uprightTorque * character->driveStrength * driveTimeScale);
                const double torqueVector[3] = {torque.x, torque.y, torque.z};
                rigid_->applyTorque(rigid_->self, character->world, character->torsoBody, torqueVector);
            }
        }
        return true;
    }

    // ---- secondary motion ---------------------------------------------------

    JiggleChainHandle createJiggleChainImpl(const JiggleChainDescription *description, CharacterError *error)
    {
        setError(error, CharacterError::None);
        if (description == nullptr || description->restOffsets == nullptr || description->boneCount < 1
            || description->boneCount > 256 || chains_.size() >= settings_.maximumJiggleChains)
        {
            setError(error, description == nullptr || description->restOffsets == nullptr || description->boneCount < 1
                                || description->boneCount > 256
                                ? CharacterError::InvalidDescription
                                : CharacterError::CapacityExceeded);
            return invalidJiggleChain;
        }
        if (!std::isfinite(description->stiffness) || description->stiffness <= 0.0
            || !std::isfinite(description->damping) || description->damping < 0.0
            || !std::isfinite(description->gravityInfluence) || description->gravityInfluence < 0.0
            || description->gravityInfluence > 2.0 || !std::isfinite(description->maximumAngleRadians)
            || description->maximumAngleRadians <= 0.0)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidJiggleChain;
        }
        for (std::uint32_t bone = 0; bone < description->boneCount; ++bone)
        {
            if (!finiteArray(description->restOffsets + bone * 3, 3))
            {
                setError(error, CharacterError::InvalidDescription);
                return invalidJiggleChain;
            }
            const math::Vec3 offset{description->restOffsets[bone * 3], description->restOffsets[bone * 3 + 1],
                description->restOffsets[bone * 3 + 2]};
            if (math::lengthSquared(offset) < 1.0e-12)
            {
                setError(error, CharacterError::InvalidDescription);
                return invalidJiggleChain;
            }
        }
        if (nextChainHandle_ == invalidJiggleChain)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidJiggleChain;
        }
        JiggleChainRecord chain;
        chain.handle       = nextChainHandle_;
        nextChainHandle_   = nextChainHandle_ == std::numeric_limits<JiggleChainHandle>::max()
                                 ? invalidJiggleChain
                                 : nextChainHandle_ + 1;
        chain.description  = *description;
        chain.restOffsets.resize(description->boneCount);
        for (std::uint32_t bone = 0; bone < description->boneCount; ++bone)
        {
            chain.restOffsets[bone] = {description->restOffsets[bone * 3], description->restOffsets[bone * 3 + 1],
                description->restOffsets[bone * 3 + 2]};
        }
        chain.bones.assign(description->boneCount, JiggleBoneState{});
        chain.worldRotations.assign(description->boneCount, math::Quaternion{0.0, 0.0, 0.0, 1.0});
        chain.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        chains_.push_back(std::move(chain));
        chainIndexByHandle_.emplace(chains_.back().handle, chains_.size() - 1);
        statistics_.jiggleChainCount = static_cast<std::uint32_t>(chains_.size());
        return chains_.back().handle;
    }

    void destroyJiggleChainImpl(JiggleChainHandle handle)
    {
        const auto iterator = chainIndexByHandle_.find(handle);
        if (iterator == chainIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        chainIndexByHandle_.erase(iterator);
        if (index + 1 != chains_.size())
        {
            chains_[index]                                   = chains_.back();
            chainIndexByHandle_[chains_[index].handle]       = index;
        }
        chains_.pop_back();
        statistics_.jiggleChainCount = static_cast<std::uint32_t>(chains_.size());
    }

    bool updateJiggleChainImpl(JiggleChainHandle handle, const engine::physics_rigid::RigidTransform *root,
        const double rootVelocity[3], double fixedDeltaSeconds)
    {
        JiggleChainRecord *chain = findChain(handle);
        if (chain == nullptr || root == nullptr || rootVelocity == nullptr || !std::isfinite(fixedDeltaSeconds)
            || fixedDeltaSeconds <= 0.0 || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        if (!finiteArray(rootVelocity, 3) || !std::isfinite(root->position.x) || !std::isfinite(root->position.y)
            || !std::isfinite(root->position.z))
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence; a due
        // pass integrates interval x the caller's delta (a spring system,
        // so the enlarged step keeps the wall-time pacing), and the Cached
        // band freezes the chain. Without lod_enabled every chain updates
        // every step.
        const std::uint32_t lodInterval = engine::util::simulationLodResolvePass(chain->lodBand,
            chain->cadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            chain->importance,
            &chain->lodSkippedSteps);
        if (lodInterval == 0u)
        {
            return true;
        }
        const double stepDeltaSeconds = fixedDeltaSeconds * static_cast<double>(lodInterval);
        const math::Vec3 velocity{rootVelocity[0], rootVelocity[1], rootVelocity[2]};
        math::Vec3 acceleration{};
        if (chain->hasPreviousVelocity)
        {
            // The velocities are stepDeltaSeconds apart on a due pass (the
            // LOD interval), so the finite difference divides by the step
            // the pair actually spans; dividing by the bare fixed delta
            // would amplify the excitation by the interval and break the
            // module's wall-time pacing rule.
            acceleration = (velocity - chain->previousRootVelocity) * (1.0 / stepDeltaSeconds);
        }
        chain->previousRootVelocity = velocity;
        chain->hasPreviousVelocity  = true;

        const math::Quaternion rootRotation = math::normalize(root->rotation);
        math::Quaternion parentRotation = rootRotation; // composed frame of the parent bone
        for (std::uint32_t bone = 0; bone < chain->bones.size(); ++bone)
        {
            JiggleBoneState &state  = chain->bones[bone];
            const math::Vec3 offset{chain->restOffsets[bone][0], chain->restOffsets[bone][1], chain->restOffsets[bone][2]};

            // Virtual force per unit mass at the bone tip in the root's
            // accelerating frame: gravity scaled by the influence factor
            // minus the root acceleration.
            const math::Vec3 localTip = offset + math::Vec3{
                state.offsetVector[0], state.offsetVector[1], state.offsetVector[2]};
            const math::Vec3 tipWorld   = math::rotate(parentRotation, localTip);
            const math::Vec3 forceWorld = math::Vec3{0.0, -settings_.jiggleGravityMagnitude * chain->description.gravityInfluence, 0.0}
                                          - acceleration;
            // Torque about the bone origin from the tip force, expressed in
            // the parent bone frame (the spring state lives there).
            const math::Vec3 torqueWorld  = math::cross(tipWorld, forceWorld);
            const math::Vec3 torqueLocal  = math::inverseRotate(parentRotation, torqueWorld);
            const double     leverArm     = std::max(math::length(offset), 1.0e-6);
            const math::Vec3 excitation   = torqueLocal * (1.0 / (leverArm * leverArm));

            // Semi-implicit Euler stability bounds for the spring: the
            // discrete solve stays stable while h^2 k < 4 and h c < 2, so the
            // gains clamp to safety fractions of those bounds (the same
            // protection the foliage module applies to its bend springs).
            // The clamps absorb the enlarged step of a reduced-cadence LOD
            // pass, which would otherwise destabilize a lively chain.
            const double stiffness = std::min(chain->description.stiffness, kSpringStiffnessStabilityFactor / (stepDeltaSeconds * stepDeltaSeconds));
            const double damping   = std::min(chain->description.damping, kSpringDampingStabilityFactor / stepDeltaSeconds);
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                state.angularVelocity[axis] += (-stiffness * math::component(excitation, axis)
                                                    - damping * state.angularVelocity[axis]
                                                    - stiffness * state.offsetVector[axis])
                                               * stepDeltaSeconds;
                state.offsetVector[axis] += state.angularVelocity[axis] * stepDeltaSeconds;
            }
            const double angleSquared = state.offsetVector[0] * state.offsetVector[0]
                                        + state.offsetVector[1] * state.offsetVector[1]
                                        + state.offsetVector[2] * state.offsetVector[2];
            const double maximumAngle = chain->description.maximumAngleRadians;
            if (angleSquared > maximumAngle * maximumAngle)
            {
                const double scale = maximumAngle / std::sqrt(angleSquared);
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    state.offsetVector[axis] *= scale;
                    state.angularVelocity[axis] *= 0.5;
                }
            }

            // Compose the simulated frame: parent frame translated by the
            // rest offset and rotated by the spring offset.
            const math::Vec3 offsetAxisVector{state.offsetVector[0], state.offsetVector[1], state.offsetVector[2]};
            const double     angle = math::length(offsetAxisVector);
            math::Quaternion offsetRotation{0.0, 0.0, 0.0, 1.0};
            if (angle > 1.0e-9)
            {
                const math::Vec3 axisDirection = offsetAxisVector * (1.0 / angle);
                const double     halfSin       = std::sin(angle * 0.5);
                offsetRotation = math::Quaternion{axisDirection.x * halfSin, axisDirection.y * halfSin,
                    axisDirection.z * halfSin, std::cos(angle * 0.5)};
            }
            chain->worldRotations[bone] = math::normalize(parentRotation * offsetRotation);
            parentRotation = chain->worldRotations[bone];
        }
        bool finite = true;
        for (const JiggleBoneState &state : chain->bones)
        {
            if (!std::isfinite(state.offsetVector[0]) || !std::isfinite(state.offsetVector[1])
                || !std::isfinite(state.offsetVector[2]) || !std::isfinite(state.angularVelocity[0])
                || !std::isfinite(state.angularVelocity[1]) || !std::isfinite(state.angularVelocity[2]))
            {
                finite = false;
                break;
            }
        }
        return finite;
    }

    bool copyChainBoneRotationsImpl(
        JiggleChainHandle handle, double *outRotations, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        JiggleChainRecord *chain = findChain(handle);
        if (chain == nullptr || outRequired == nullptr)
        {
            return false;
        }
        const std::uint32_t total = static_cast<std::uint32_t>(chain->bones.size()) * 4;
        *outRequired = total;
        if (outRotations == nullptr)
        {
            return false;
        }
        if (capacity < total)
        {
            return false;
        }
        for (std::size_t bone = 0; bone < chain->bones.size(); ++bone)
        {
            const math::Quaternion &rotation = chain->worldRotations[bone];
            outRotations[bone * 4]     = rotation.x;
            outRotations[bone * 4 + 1] = rotation.y;
            outRotations[bone * 4 + 2] = rotation.z;
            outRotations[bone * 4 + 3] = rotation.w;
        }
        return true;
    }

    // ---- procedural foot placement -------------------------------------------

    FootPlanterHandle createFootPlanterImpl(
        engine::physics_rigid::WorldHandle world, const FootPlanterDescription *description, CharacterError *error)
    {
        setError(error, CharacterError::None);
        if (rigid_ == nullptr)
        {
            setError(error, CharacterError::InvalidHandle);
            return invalidFootPlanter;
        }
        if (description == nullptr)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidFootPlanter;
        }
        if (description->legUpperLength <= 0.0 || description->legLowerLength <= 0.0
            || description->footRadius <= 0.0 || description->maxStepHeight < 0.0 || description->strideThreshold <= 0.0
            || description->rayCastUp <= 0.0 || description->leadTime < 0.0
            || !std::isfinite(description->legUpperLength) || !std::isfinite(description->legLowerLength)
            || !std::isfinite(description->hipAnchor[0]) || !std::isfinite(description->hipAnchor[1])
            || !std::isfinite(description->hipAnchor[2]) || !finiteArray(description->footOffsets[0], 3)
            || !finiteArray(description->footOffsets[1], 3))
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidFootPlanter;
        }
        if (description->legUpperLength + description->legLowerLength <= description->footRadius)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidFootPlanter;
        }
        if (description->ignoredBodyCount > 0 && description->ignoredBodies == nullptr)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidFootPlanter;
        }
        if (planters_.size() >= settings_.maximumFootPlanters)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidFootPlanter;
        }
        if (nextPlanterHandle_ == invalidFootPlanter)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidFootPlanter;
        }
        // The planter's world handle validates through a gravity probe: a
        // world the rigid module does not know is rejected here.
        double gravityProbe[3] = {0.0, 0.0, 0.0};
        if (!rigid_->copyWorldGravity(rigid_->self, world, gravityProbe))
        {
            setError(error, CharacterError::InvalidHandle);
            return invalidFootPlanter;
        }
        FootPlanterRecord planter;
        planter.handle      = nextPlanterHandle_;
        nextPlanterHandle_  = nextPlanterHandle_ == std::numeric_limits<FootPlanterHandle>::max()
                                  ? invalidFootPlanter
                                  : nextPlanterHandle_ + 1;
        planter.world       = world;
        planter.description = *description;
        planter.ignoredBodies.assign(
            description->ignoredBodies, description->ignoredBodies + description->ignoredBodyCount);
        if (!planter.ignoredBodies.empty())
        {
            // The record owns the list; point the description's view at it.
            planter.description.ignoredBodies = planter.ignoredBodies.data();
        }
        else
        {
            planter.description.ignoredBodies = nullptr;
        }
        planter.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        planters_.push_back(std::move(planter));
        planterIndexByHandle_.emplace(planters_.back().handle, planters_.size() - 1);
        statistics_.footPlanterCount = static_cast<std::uint32_t>(planters_.size());
        return planters_.back().handle;
    }

    void destroyFootPlanterImpl(FootPlanterHandle handle)
    {
        const auto iterator = planterIndexByHandle_.find(handle);
        if (iterator == planterIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        planterIndexByHandle_.erase(iterator);
        if (index + 1 != planters_.size())
        {
            planters_[index]                                     = planters_.back();
            planterIndexByHandle_[planters_[index].handle]       = index;
        }
        planters_.pop_back();
        statistics_.footPlanterCount = static_cast<std::uint32_t>(planters_.size());
    }

    bool updateFootPlanterImpl(FootPlanterHandle handle, const engine::physics_rigid::RigidTransform *root,
        const double rootVelocity[3], double fixedDeltaSeconds)
    {
        FootPlanterRecord *planter = findPlanter(handle);
        if (planter == nullptr || rigid_ == nullptr || root == nullptr || rootVelocity == nullptr
            || !std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0 || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        if (!finiteArray(rootVelocity, 3))
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence; a due
        // pass runs the ground probes, a skipped pass keeps the last planted
        // feet (the plant latch holds until the next due pass), and the
        // Cached band freezes them entirely. Without lod_enabled every
        // planter updates every step.
        if (engine::util::simulationLodResolvePass(planter->lodBand,
                planter->cadence,
                settings_.lod.thresholds,
                settings_.lod.intervals,
                settings_.lod.enabled,
                planter->importance,
                &planter->lodSkippedSteps)
            == 0u)
        {
            return true;
        }
        const FootPlanterDescription &description = planter->description;
        const math::Quaternion rootRotation = math::normalize(root->rotation);
        const math::Vec3 rootPosition      = root->position;
        const math::Vec3 velocity{rootVelocity[0], rootVelocity[1], rootVelocity[2]};
        const math::Vec3 hipWorld = rootPosition + math::rotate(rootRotation, math::Vec3{description.hipAnchor[0], description.hipAnchor[1], description.hipAnchor[2]});
        const double legReach     = description.legUpperLength + description.legLowerLength;

        for (std::uint32_t foot = 0; foot < 2; ++foot)
        {
            const math::Vec3 footOffset = math::rotate(rootRotation,
                math::Vec3{description.footOffsets[foot][0], description.footOffsets[foot][1], description.footOffsets[foot][2]});
            const math::Vec3 desired = rootPosition + footOffset + velocity * description.leadTime;

            // Ground probe: cast down from above the desired point.
            const math::Vec3 rayOrigin    = desired + math::Vec3{0.0, description.rayCastUp, 0.0};
            const double     rayOriginV[3] = {rayOrigin.x, rayOrigin.y, rayOrigin.z};
            const double     down[3]       = {0.0, -1.0, 0.0};
            engine::physics_rigid::RayHit hit;
            math::Vec3 target    = desired;
            math::Vec3 normal{0.0, 1.0, 0.0};
            bool     groundKnown = false;
            const engine::physics_rigid::BodyHandle *ignored = planter->ignoredBodies.empty()
                                                                   ? nullptr
                                                                   : planter->ignoredBodies.data();
            const std::uint32_t ignoredCount = static_cast<std::uint32_t>(planter->ignoredBodies.size());
            const bool groundHit = ignored != nullptr
                                       ? rigid_->rayCastIgnoring(rigid_->self,
                                             planter->world,
                                             rayOriginV,
                                             down,
                                             description.rayCastUp * 2.0,
                                             ignored,
                                             ignoredCount,
                                             &hit)
                                       : rigid_->rayCast(rigid_->self, planter->world, rayOriginV, down,
                                             description.rayCastUp * 2.0, &hit);
            if (groundHit)
            {
                target      = math::Vec3{hit.point.x, hit.point.y, hit.point.z};
                normal      = math::Vec3{hit.normal.x, hit.normal.y, hit.normal.z};
                groundKnown = true;
            }

            FootRuntime &runtime = planter->feet[foot];
            if (!runtime.planted)
            {
                runtime.plantPosition = target;
                runtime.plantNormal   = normal;
                runtime.planted       = groundKnown;
            }
            else
            {
                const math::Vec3 offset  = target - runtime.plantPosition;
                const double horizontal  = std::sqrt(offset.x * offset.x + offset.z * offset.z);
                // Replant when the desired point strides away from the
                // planted point, when the ground rises past the step limit,
                // or when no ground exists anymore at the desired point.
                if (horizontal > description.strideThreshold
                    || (groundKnown && offset.y > description.maxStepHeight)
                    || (!groundKnown && offset.y < -description.strideThreshold))
                {
                    runtime.plantPosition = target;
                    runtime.plantNormal   = normal;
                    runtime.planted       = groundKnown;
                }
            }

            // Two-bone IK from the hip to the planted foot: the knee bends
            // toward the root's forward direction, and the hip drops when
            // the foot is unreachable at full extension. The knee bend is
            // the general law-of-cosines exterior angle (the hip-interior
            // plus foot-interior angles of the hip-foot-thigh triangle), so
            // asymmetric leg bones solve exactly like the hand planter does
            // for arms.
            const math::Vec3 footPoint = runtime.plantPosition + math::Vec3{0.0, description.footRadius, 0.0};
            const math::Vec3 hipToFoot = footPoint - hipWorld;
            const double distance      = math::length(hipToFoot);
            const double clamped       = std::clamp(distance, 1.0e-6, legReach - 1.0e-6);
            const double upperLength   = description.legUpperLength;
            const double lowerLength   = description.legLowerLength;
            const double cosHip        = (upperLength * upperLength + clamped * clamped - lowerLength * lowerLength)
                                 / (2.0 * upperLength * clamped);
            const double cosFoot       = (clamped * clamped + lowerLength * lowerLength - upperLength * upperLength)
                                 / (2.0 * clamped * lowerLength);
            const double hipAngle      = std::acos(std::clamp(cosHip, -1.0, 1.0));
            const double footAngle     = std::acos(std::clamp(cosFoot, -1.0, 1.0));
            const double kneeBend      = hipAngle + footAngle;
            const double hipDrop       = distance > legReach ? distance - legReach : 0.0;

            runtime.kneeDirection   = math::rotate(rootRotation, math::Vec3{0.0, 0.0, 1.0});
            runtime.kneeBendRadians = std::max(kneeBend, 0.0);
            runtime.hipDropMeters   = hipDrop;
        }
        (void)fixedDeltaSeconds;
        return true;
    }

    bool copyFootStatesImpl(FootPlanterHandle handle, FootState *outStates, std::uint32_t capacity,
        std::uint32_t *outRequired)
    {
        FootPlanterRecord *planter = findPlanter(handle);
        if (planter == nullptr || outRequired == nullptr)
        {
            return false;
        }
        *outRequired = 2;
        if (outStates == nullptr || capacity < 2)
        {
            return false;
        }
        for (std::uint32_t foot = 0; foot < 2; ++foot)
        {
            const FootRuntime &runtime = planter->feet[foot];
            FootState state;
            state.planted = runtime.planted;
            state.footPosition[0] = runtime.plantPosition.x;
            state.footPosition[1] = runtime.plantPosition.y;
            state.footPosition[2] = runtime.plantPosition.z;
            state.footNormal[0] = runtime.plantNormal.x;
            state.footNormal[1] = runtime.plantNormal.y;
            state.footNormal[2] = runtime.plantNormal.z;
            state.kneeDirection[0] = runtime.kneeDirection.x;
            state.kneeDirection[1] = runtime.kneeDirection.y;
            state.kneeDirection[2] = runtime.kneeDirection.z;
            state.kneeBendRadians = runtime.kneeBendRadians;
            state.hipDropMeters   = runtime.hipDropMeters;
            outStates[foot] = state;
        }
        return true;
    }

    bool characterStatisticsImpl(CharacterStatistics *outStatistics)
    {
        if (outStatistics == nullptr)
        {
            return false;
        }
        std::uint64_t lodSkippedSteps = 0;
        for (const CharacterRecord &character : characters_)
        {
            lodSkippedSteps += character.lodSkippedSteps;
        }
        for (const JiggleChainRecord &chain : chains_)
        {
            lodSkippedSteps += chain.lodSkippedSteps;
        }
        for (const FootPlanterRecord &planter : planters_)
        {
            lodSkippedSteps += planter.lodSkippedSteps;
        }
        for (const LeanRecord &lean : leanControllers_)
        {
            lodSkippedSteps += lean.lodSkippedSteps;
        }
        statistics_.lodSkippedSteps = lodSkippedSteps;
        *outStatistics              = statistics_;
        return true;
    }

    // ---- hand placement ------------------------------------------------------

    HandPlanterHandle createHandPlanterImpl(const HandPlanterDescription *description, CharacterError *error)
    {
        setError(error, CharacterError::None);
        if (description == nullptr || !std::isfinite(description->armUpperLength)
            || description->armUpperLength <= 0.0 || !std::isfinite(description->armLowerLength)
            || description->armLowerLength <= 0.0 || !finiteArray(&description->shoulderOffsets[0][0], 6)
            || !finiteArray(&description->handOffsets[0][0], 6) || !finiteArray(description->elbowDirection, 3))
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidHandPlanter;
        }
        const math::Vec3 elbow{
            description->elbowDirection[0], description->elbowDirection[1], description->elbowDirection[2]};
        if (math::lengthSquared(elbow) < 1.0e-12)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidHandPlanter;
        }
        if (description->armUpperLength + description->armLowerLength <= 0.0)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidHandPlanter;
        }
        if (handPlanters_.size() >= settings_.maximumHandPlanters)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidHandPlanter;
        }
        if (nextHandPlanterHandle_ == invalidHandPlanter)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidHandPlanter;
        }
        HandPlanterRecord planter;
        planter.handle      = nextHandPlanterHandle_;
        nextHandPlanterHandle_ = nextHandPlanterHandle_ == std::numeric_limits<HandPlanterHandle>::max()
            ? invalidHandPlanter
            : nextHandPlanterHandle_ + 1;
        planter.description = *description;
        handPlanters_.push_back(std::move(planter));
        handPlanterIndexByHandle_.emplace(handPlanters_.back().handle, handPlanters_.size() - 1);
        statistics_.handPlanterCount = static_cast<std::uint32_t>(handPlanters_.size());
        return handPlanters_.back().handle;
    }

    void destroyHandPlanterImpl(HandPlanterHandle handle)
    {
        const auto iterator = handPlanterIndexByHandle_.find(handle);
        if (iterator == handPlanterIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        handPlanterIndexByHandle_.erase(iterator);
        if (index + 1 != handPlanters_.size())
        {
            handPlanters_[index]                                       = handPlanters_.back();
            handPlanterIndexByHandle_[handPlanters_[index].handle]     = index;
        }
        handPlanters_.pop_back();
        statistics_.handPlanterCount = static_cast<std::uint32_t>(handPlanters_.size());
    }

    bool setHandTargetImpl(HandPlanterHandle handle, std::uint32_t handIndex, const double target[3])
    {
        HandPlanterRecord *planter = findHandPlanter(handle);
        if (planter == nullptr || handIndex > 1 || target == nullptr || !finiteArray(target, 3))
        {
            return false;
        }
        HandRuntime &runtime = planter->hands[handIndex];
        runtime.target       = math::Vec3{target[0], target[1], target[2]};
        runtime.hasTarget    = true;
        return true;
    }

    bool updateHandPlanterImpl(HandPlanterHandle handle, const engine::physics_rigid::RigidTransform *root)
    {
        HandPlanterRecord *planter = findHandPlanter(handle);
        if (planter == nullptr || root == nullptr)
        {
            return false;
        }
        if (!std::isfinite(root->position.x) || !std::isfinite(root->position.y) || !std::isfinite(root->position.z))
        {
            return false;
        }
        const HandPlanterDescription &description = planter->description;
        const math::Quaternion rootRotation = math::normalize(root->rotation);
        const math::Vec3 rootPosition       = root->position;
        const double     reach              = description.armUpperLength + description.armLowerLength;
        const math::Vec3 elbowPoleLocal
            = math::normalized(math::Vec3{description.elbowDirection[0], description.elbowDirection[1], description.elbowDirection[2]});
        const math::Vec3 elbowPoleWorld = math::rotate(rootRotation, elbowPoleLocal);

        for (std::uint32_t hand = 0; hand < 2; ++hand)
        {
            HandRuntime &runtime = planter->hands[hand];
            const math::Vec3 shoulderWorld = rootPosition + math::rotate(rootRotation,
                math::Vec3{description.shoulderOffsets[hand][0], description.shoulderOffsets[hand][1],
                    description.shoulderOffsets[hand][2]});
            if (!runtime.hasTarget)
            {
                runtime.target = rootPosition + math::rotate(rootRotation,
                    math::Vec3{description.handOffsets[hand][0], description.handOffsets[hand][1],
                        description.handOffsets[hand][2]});
            }

            // Two-bone IK: clamp the target to full extension, then the law
            // of cosines gives the elbow interior angle.
            const math::Vec3 shoulderToTarget = runtime.target - shoulderWorld;
            const double     distance         = math::length(shoulderToTarget);
            const double     clamped          = std::clamp(distance, 1.0e-6, reach - 1.0e-6);
            runtime.withinReach               = distance <= reach;
            runtime.stretchMeters             = distance > reach ? distance - reach : 0.0;
            runtime.handPosition              = shoulderWorld
                                  + (distance > 1.0e-6 ? shoulderToTarget * (clamped / distance) : math::Vec3{0.0, 0.0, 0.0});
            const double cosElbow = (description.armUpperLength * description.armUpperLength
                                        + description.armLowerLength * description.armLowerLength
                                        - clamped * clamped)
                / (2.0 * description.armUpperLength * description.armLowerLength);
            runtime.elbowBendRadians = std::max(math::kPi - std::acos(std::clamp(cosElbow, -1.0, 1.0)), 0.0);
            runtime.elbowDirection   = elbowPoleWorld;
        }
        return true;
    }

    bool copyHandStatesImpl(
        HandPlanterHandle handle, HandState *outStates, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        HandPlanterRecord *planter = findHandPlanter(handle);
        if (planter == nullptr || outRequired == nullptr)
        {
            return false;
        }
        *outRequired = 2;
        if (outStates == nullptr || capacity < 2)
        {
            return false;
        }
        for (std::uint32_t hand = 0; hand < 2; ++hand)
        {
            const HandRuntime &runtime = planter->hands[hand];
            HandState state;
            state.withinReach = runtime.withinReach;
            state.handPosition[0] = runtime.handPosition.x;
            state.handPosition[1] = runtime.handPosition.y;
            state.handPosition[2] = runtime.handPosition.z;
            state.elbowDirection[0] = runtime.elbowDirection.x;
            state.elbowDirection[1] = runtime.elbowDirection.y;
            state.elbowDirection[2] = runtime.elbowDirection.z;
            state.elbowBendRadians = runtime.elbowBendRadians;
            state.stretchMeters    = runtime.stretchMeters;
            outStates[hand] = state;
        }
        return true;
    }

    // ---- procedural lean -------------------------------------------------------

    LeanHandle createLeanControllerImpl(const LeanControllerDescription *description, CharacterError *error)
    {
        setError(error, CharacterError::None);
        if (description == nullptr || !std::isfinite(description->maximumLeanRadians)
            || description->maximumLeanRadians <= 0.0 || description->maximumLeanRadians > 1.2
            || !std::isfinite(description->stiffness) || description->stiffness <= 0.0
            || !std::isfinite(description->damping) || description->damping < 0.0
            || !std::isfinite(description->accelerationGain) || description->accelerationGain < 0.0)
        {
            setError(error, CharacterError::InvalidDescription);
            return invalidLeanController;
        }
        if (leanControllers_.size() >= settings_.maximumLeanControllers)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidLeanController;
        }
        if (nextLeanHandle_ == invalidLeanController)
        {
            setError(error, CharacterError::CapacityExceeded);
            return invalidLeanController;
        }
        LeanRecord lean;
        lean.handle      = nextLeanHandle_;
        nextLeanHandle_  = nextLeanHandle_ == std::numeric_limits<LeanHandle>::max()
                              ? invalidLeanController
                              : nextLeanHandle_ + 1;
        lean.description = *description;
        lean.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        leanControllers_.push_back(std::move(lean));
        leanIndexByHandle_.emplace(leanControllers_.back().handle, leanControllers_.size() - 1);
        statistics_.leanControllerCount = static_cast<std::uint32_t>(leanControllers_.size());
        return leanControllers_.back().handle;
    }

    void destroyLeanControllerImpl(LeanHandle handle)
    {
        const auto iterator = leanIndexByHandle_.find(handle);
        if (iterator == leanIndexByHandle_.end())
        {
            return;
        }
        const std::size_t index = iterator->second;
        leanIndexByHandle_.erase(iterator);
        if (index + 1 != leanControllers_.size())
        {
            leanControllers_[index]                              = leanControllers_.back();
            leanIndexByHandle_[leanControllers_[index].handle]   = index;
        }
        leanControllers_.pop_back();
        statistics_.leanControllerCount = static_cast<std::uint32_t>(leanControllers_.size());
    }

    bool updateLeanControllerImpl(LeanHandle handle, const engine::physics_rigid::RigidTransform *root,
        const double rootVelocity[3], double fixedDeltaSeconds, LeanState *outState)
    {
        LeanRecord *lean = findLean(handle);
        if (lean == nullptr || root == nullptr || rootVelocity == nullptr || outState == nullptr
            || !std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0 || fixedDeltaSeconds > 0.1)
        {
            return false;
        }
        if (!finiteArray(rootVelocity, 3) || !std::isfinite(root->position.x) || !std::isfinite(root->position.y)
            || !std::isfinite(root->position.z))
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence; a due
        // pass integrates interval x the caller's delta (a spring system),
        // and the Cached band holds the last lean. Without lod_enabled
        // every controller updates every step.
        const std::uint32_t lodInterval = engine::util::simulationLodResolvePass(lean->lodBand,
            lean->cadence,
            settings_.lod.thresholds,
            settings_.lod.intervals,
            settings_.lod.enabled,
            lean->importance,
            &lean->lodSkippedSteps);
        if (lodInterval == 0u)
        {
            // The caller still receives the held state.
            outState->leanRadians    = lean->leanAngleRadians;
            outState->leanAxis[0]    = lean->leanAxis.x;
            outState->leanAxis[1]    = lean->leanAxis.y;
            outState->leanAxis[2]    = lean->leanAxis.z;
            outState->leanRotation[0] = lean->leanRotation.x;
            outState->leanRotation[1] = lean->leanRotation.y;
            outState->leanRotation[2] = lean->leanRotation.z;
            outState->leanRotation[3] = lean->leanRotation.w;
            return true;
        }
        const double stepDeltaSeconds = fixedDeltaSeconds * static_cast<double>(lodInterval);
        const math::Vec3 velocity{rootVelocity[0], rootVelocity[1], rootVelocity[2]};
        math::Vec3 acceleration{};
        if (lean->hasPreviousVelocity)
        {
            // Same interval-aware finite difference as the jiggle chains:
            // the velocity pair spans the LOD-interval step.
            acceleration = (velocity - lean->previousVelocity) * (1.0 / stepDeltaSeconds);
        }
        lean->previousVelocity      = velocity;
        lean->hasPreviousVelocity   = true;

        // Lateral acceleration only: the vertical component is dropped so
        // jumping and falling never lean the character over.
        math::Vec3 lateral = acceleration;
        lateral.y          = 0.0;
        const double lateralMagnitude = math::length(lateral);

        double targetAngle  = 0.0;
        math::Vec3 targetAxis = lean->leanAxis;
        if (lateralMagnitude > 1.0e-6)
        {
            targetAxis = math::normalized(math::cross(math::Vec3{0.0, 1.0, 0.0}, lateral));
            targetAngle = std::clamp(
                lateralMagnitude * lean->description.accelerationGain, 0.0, lean->description.maximumLeanRadians);
        }

        // Spring-damper scalar lean around the (possibly retargeted) axis,
        // with the same stability-bound clamps the jiggle chains apply.
        const double leanStiffness = std::min(lean->description.stiffness,
            kSpringStiffnessStabilityFactor / (stepDeltaSeconds * stepDeltaSeconds));
        const double leanDamping   = std::min(lean->description.damping,
            kSpringDampingStabilityFactor / stepDeltaSeconds);
        lean->angularVelocity += (-leanStiffness * (lean->leanAngleRadians - targetAngle)
                                     - leanDamping * lean->angularVelocity)
            * stepDeltaSeconds;
        lean->leanAngleRadians += lean->angularVelocity * stepDeltaSeconds;
        lean->leanAngleRadians = std::clamp(lean->leanAngleRadians, -lean->description.maximumLeanRadians,
            lean->description.maximumLeanRadians);
        if (lateralMagnitude > 1.0e-6)
        {
            lean->leanAxis = targetAxis;
        }

        const double angle = std::abs(lean->leanAngleRadians);
        if (angle > 1.0e-9)
        {
            const math::Vec3 axis   = lean->leanAxis * (lean->leanAngleRadians >= 0.0 ? 1.0 : -1.0);
            const double     half   = angle * 0.5;
            const double     sine   = std::sin(half);
            lean->leanRotation = math::normalize(math::Quaternion{
                axis.x * sine, axis.y * sine, axis.z * sine, std::cos(half)});
        }
        else
        {
            lean->leanRotation = math::Quaternion{0.0, 0.0, 0.0, 1.0};
        }

        outState->leanRadians       = lean->leanAngleRadians;
        outState->leanAxis[0]       = lean->leanAxis.x;
        outState->leanAxis[1]       = lean->leanAxis.y;
        outState->leanAxis[2]       = lean->leanAxis.z;
        outState->leanRotation[0]   = lean->leanRotation.x;
        outState->leanRotation[1]   = lean->leanRotation.y;
        outState->leanRotation[2]   = lean->leanRotation.z;
        outState->leanRotation[3]   = lean->leanRotation.w;
        return std::isfinite(lean->leanAngleRadians) && std::isfinite(lean->angularVelocity);
    }

    // ---- physical pushing --------------------------------------------------------

    bool applyCharacterPushImpl(engine::physics_rigid::WorldHandle world, const double center[3], double radius,
        const double pushVelocity[3], double forceScale)
    {
        if (rigid_ == nullptr || center == nullptr || pushVelocity == nullptr || !finiteArray(center, 3)
            || !finiteArray(pushVelocity, 3) || !std::isfinite(radius) || radius <= 0.0 || !std::isfinite(forceScale)
            || forceScale < 0.0)
        {
            return false;
        }
        const std::uint32_t bodyCount = rigid_->bodyCount(rigid_->self, world);
        if (bodyCount == 0u)
        {
            return true;
        }
        // Batched reads over the whole world (the rigid capability's
        // copyBodyHandles / copyBodyStates / copyBodyDescriptions surface):
        // three calls instead of two per body, and every dynamic body in the
        // sphere receives the push no matter how large the world grew.
        pushHandles_.resize(bodyCount);
        pushStates_.resize(bodyCount);
        pushDescriptions_.resize(bodyCount);
        std::uint32_t required = 0u;
        if (!rigid_->copyBodyHandles(rigid_->self, world, pushHandles_.data(), bodyCount, &required))
        {
            return false;
        }
        const std::uint32_t liveCount = required < bodyCount ? required : bodyCount;
        // The batched reads resolve in order; the resolved counts bound the
        // loops so a partially-filled batch never reads stale entries.
        const std::uint32_t resolvedStates = rigid_->copyBodyStates(rigid_->self, world, pushHandles_.data(),
            liveCount, pushStates_.data());
        const std::uint32_t resolvedDescriptions = rigid_->copyBodyDescriptions(rigid_->self, world,
            pushHandles_.data(), liveCount, pushDescriptions_.data());
        const double radiusSquared = radius * radius;
        for (std::uint32_t index = 0;
            index < liveCount && index < resolvedStates && index < resolvedDescriptions; ++index)
        {
            if (pushDescriptions_[index].kind != engine::physics_rigid::BodyKind::Dynamic)
            {
                continue;
            }
            const engine::physics_rigid::RigidTransform &transform = pushStates_[index].transform;
            const double offsetX = transform.position.x - center[0];
            const double offsetY = transform.position.y - center[1];
            const double offsetZ = transform.position.z - center[2];
            if (offsetX * offsetX + offsetY * offsetY + offsetZ * offsetZ > radiusSquared)
            {
                continue;
            }
            const double impulse[3] = {pushVelocity[0] * forceScale, pushVelocity[1] * forceScale,
                pushVelocity[2] * forceScale};
            const double point[3]   = {transform.position.x, transform.position.y, transform.position.z};
            rigid_->applyImpulseAtPoint(rigid_->self, world, pushHandles_[index], impulse, point);
        }
        return true;
    }

    bool setCharacterImportanceImpl(CharacterHandle handle, double importance)
    {
        CharacterRecord *character = findCharacter(handle);
        if (character == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        character->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool setJiggleChainImportanceImpl(JiggleChainHandle handle, double importance)
    {
        JiggleChainRecord *chain = findChain(handle);
        if (chain == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        chain->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool setFootPlanterImportanceImpl(FootPlanterHandle handle, double importance)
    {
        FootPlanterRecord *planter = findPlanter(handle);
        if (planter == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        planter->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool setLeanControllerImportanceImpl(LeanHandle handle, double importance)
    {
        LeanRecord *lean = findLean(handle);
        if (lean == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        lean->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsCharacterCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_CHARACTER_BIND(field, method)                                                                   \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<CharacterModule *>(self)->method(args...); }
        ENGINE_PHYSICS_CHARACTER_BIND(createHumanoidRagdoll, createHumanoidRagdollImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(createCharacter, createCharacterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(destroyCharacter, destroyCharacterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setCharacterMode, setCharacterModeImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setCharacterDriveStrength, setCharacterDriveStrengthImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setCharacterJointTarget, setCharacterJointTargetImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setCharacterJointGains, setCharacterJointGainsImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(updateCharacter, updateCharacterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(createJiggleChain, createJiggleChainImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(destroyJiggleChain, destroyJiggleChainImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(updateJiggleChain, updateJiggleChainImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(copyChainBoneRotations, copyChainBoneRotationsImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(createFootPlanter, createFootPlanterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(destroyFootPlanter, destroyFootPlanterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(updateFootPlanter, updateFootPlanterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(copyFootStates, copyFootStatesImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(createHandPlanter, createHandPlanterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(destroyHandPlanter, destroyHandPlanterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setHandTarget, setHandTargetImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(updateHandPlanter, updateHandPlanterImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(copyHandStates, copyHandStatesImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(createLeanController, createLeanControllerImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(destroyLeanController, destroyLeanControllerImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(updateLeanController, updateLeanControllerImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(applyCharacterPush, applyCharacterPushImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(characterStatistics, characterStatisticsImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setCharacterImportance, setCharacterImportanceImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setJiggleChainImportance, setJiggleChainImportanceImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setFootPlanterImportance, setFootPlanterImportanceImpl);
        ENGINE_PHYSICS_CHARACTER_BIND(setLeanControllerImportance, setLeanControllerImportanceImpl);
#undef ENGINE_PHYSICS_CHARACTER_BIND
    }

    ParticipantManifest manifest_;
    CharacterCapability capability_{};

    engine::physics_rigid::PhysicsRigidCapability         *rigid_       = nullptr;
    engine::physics_articulated::ArticulatedCapability    *articulated_ = nullptr;

    CharacterSettings settings_;

    CharacterStatistics statistics_{};

    // Registration ordinal for the stagger phases (one counter shared by
    // every record type, like the fields module): records sharing an update
    // interval spread their due steps evenly across the cadence.
    std::uint64_t registrationOrdinal_ = 0;

    // Reused scratch for the batched push pass.
    std::vector<engine::physics_rigid::BodyHandle>      pushHandles_;
    std::vector<engine::physics_rigid::BodyState>       pushStates_;
    std::vector<engine::physics_rigid::BodyDescription> pushDescriptions_;

    std::vector<CharacterRecord>                          characters_;
    std::unordered_map<CharacterHandle, std::size_t>      characterIndexByHandle_;
    CharacterHandle                                       nextCharacterHandle_ = 1;
    std::vector<JiggleChainRecord>                        chains_;
    std::unordered_map<JiggleChainHandle, std::size_t>    chainIndexByHandle_;
    JiggleChainHandle                                     nextChainHandle_ = 1;
    std::vector<FootPlanterRecord>                        planters_;
    std::unordered_map<FootPlanterHandle, std::size_t>    planterIndexByHandle_;
    FootPlanterHandle                                     nextPlanterHandle_ = 1;
    std::vector<HandPlanterRecord>                        handPlanters_;
    std::unordered_map<HandPlanterHandle, std::size_t>    handPlanterIndexByHandle_;
    HandPlanterHandle                                     nextHandPlanterHandle_ = 1;
    std::vector<LeanRecord>                               leanControllers_;
    std::unordered_map<LeanHandle, std::size_t>           leanIndexByHandle_;
    LeanHandle                                            nextLeanHandle_ = 1;
};

} // namespace
} // namespace engine::physics_character

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_character capability and requiring physics_rigid plus
// physics_articulated.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_character"};
    static const char *const                    required[] = {"physics_rigid", "physics_articulated"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_character",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_character::CharacterModule(); },
        .providedCapabilities = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = required,
        .requiredCapabilityCount = 2,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
