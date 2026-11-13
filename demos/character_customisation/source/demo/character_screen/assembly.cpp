#include "../character_screen_internal.hpp"
#include "../character_metrics.gen.hpp"
#include "../geometry.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <utility>

namespace demo
{
using namespace screen_detail;

engine::character::InstanceId CharacterScreen::instanceByName(const std::string &name) const
{
    if (character_ == nullptr || characterHandle_ == engine::character::invalidCharacter)
    {
        return engine::character::invalidInstance;
    }
    const std::uint32_t count = character_->instanceCount(character_->self, characterHandle_);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const engine::character::InstanceId id = character_->instanceId(character_->self, characterHandle_, index);
        const char                         *instanceName = nullptr;
        if (character_->instanceInfo(character_->self, characterHandle_, id, &instanceName, nullptr, nullptr)
            && name == instanceName)
        {
            return id;
        }
    }
    return engine::character::invalidInstance;
}

// ---- assembly helpers (body plans, counts, variants) -------------------------

bool CharacterScreen::rebuildAssembly()
{
    // (Re)create the assembly from the active body-plan profile and re-apply
    // the current counts/variants. Plan changes alter the skeleton topology,
    // so the assembly is rebuilt; counts and variants re-attach afterwards.
    CharacterError error = CharacterError::None;
    if (characterHandle_ != engine::character::invalidCharacter)
    {
        character_->destroyAssembly(character_->self, characterHandle_);
        characterHandle_ = engine::character::invalidCharacter;
    }
    characterHandle_ = character_->createAssembly(character_->self, profileIdForPlan(activePlan_), &error);
    if (characterHandle_ == engine::character::invalidCharacter)
    {
        std::fprintf(stderr, "demo: could not create the %s assembly (%u)\n", profileIdForPlan(activePlan_),
            static_cast<unsigned>(error));
        return false;
    }
    const int heads = headCount_;
    const int wings = wingPairs_;
    const int arms = armPairs_;
    const bool tail = tailPresent_;
    headCount_ = 1;
    wingPairs_ = 0;
    armPairs_ = 1;
    tailPresent_ = false;
    setHeadCount(heads);
    setWingPairs(wings);
    setArmPairs(arms);
    setTail(tail && activePlan_ != 2 && activePlan_ != 3);
    applyParameterHeights();
    evaluateCharacter(true);
    return true;
}

bool CharacterScreen::setPlan(int plan)
{
    if (plan == activePlan_ || plan < 0 || plan > 3)
    {
        return true;
    }
    activePlan_ = plan;
    // Extra arm pairs and the reptile tail do not graft onto the quadruped
    // plan; the naga carries the serpent lower body instead of a tail.
    if (activePlan_ == 3)
    {
        armPairs_ = 1;
        tailPresent_ = false;
    }
    return rebuildAssembly();
}

bool CharacterScreen::setQuadSpecies(int species)
{
    if (species < 0 || species >= kQuadSpeciesCount)
    {
        return true;
    }
    const std::string bodyId = activePlan_ == 1
        ? std::string("quad_centaur_") + kQuadSpeciesIds[species]
        : std::string("quad_body_") + kQuadSpeciesIds[species];
    const InstanceId body = instanceByName("body");
    if (body == engine::character::invalidInstance)
    {
        return false;
    }
    // Skip only when the body instance already wears the requested part -
    // comparing species INDICES early-returned when the requested species
    // happened to equal the field default, silently keeping the profile's
    // default animal (the rev-7 "every species preset rendered the horse"
    // bug, reintroduced whenever the default index matched the request).
    const char *currentPartId = nullptr;
    if (character_->instanceInfo(character_->self, characterHandle_, body, nullptr, &currentPartId, nullptr)
        && currentPartId != nullptr && bodyId == currentPartId)
    {
        quadSpecies_ = species;
        return true;
    }
    quadSpecies_ = species;
    CharacterError error = CharacterError::None;
    const bool replaced = character_->replacePart(
        character_->self, characterHandle_, body, bodyId.c_str(), &error);
    applyParameterHeights();
    evaluateCharacter(true);
    return replaced;
}

bool CharacterScreen::setHeadCount(int count)
{
    count = std::clamp(count, 1, 3);
    CharacterError error = CharacterError::None;
    bool ok = true;
    // Head 1 comes from the profile; heads 2/3 attach beside it. Extra
    // skulls yaw outward and roll a touch so the pair reads as two skulls
    // sharing the collar instead of one skull split in half (rev-9 used a
    // pure roll, which made the outer faces fight each other).
    while (headCount_ > count)
    {
        const InstanceId extra = instanceByName(headCount_ == 3 ? "head3" : "head2");
        if (extra != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, extra) && ok;
        }
        headCount_ -= 1;
    }
    while (headCount_ < count)
    {
        headCount_ += 1;
        InstanceId parent = instanceByName(activePlan_ == 0 || activePlan_ == 2 ? "torso" : "chest");
        if (parent == engine::character::invalidInstance)
        {
            parent = instanceByName("body");
        }
        const InstanceId head = character_->addPart(character_->self, characterHandle_, activeHeadPart_.c_str(),
            headCount_ == 2 ? "head2" : "head3", &error);
        if (parent == engine::character::invalidInstance || head == engine::character::invalidInstance)
        {
            ok = false;
            break;
        }
        const float lateral = headCount_ == 2 ? -0.150f : 0.150f;
        const float offset[3] = {lateral, 0.014f, 0.006f};
        // Yaw outward around the shared collar: the second head turns left,
        // the third right, and both roll a little away from the centre line.
        const float outwardYaw = headCount_ == 2 ? -24.0f : 24.0f;
        const float outwardRoll = headCount_ == 2 ? -9.0f : 9.0f;
        const glm::quat yaw = glm::angleAxis(glm::radians(outwardYaw), glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::quat roll = glm::angleAxis(glm::radians(outwardRoll), glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::quat pose = yaw * roll;
        const float rotation[4] = {pose.x, pose.y, pose.z, pose.w};
        ok = character_->connect(character_->self, characterHandle_, parent, "neck.base", head, "head.root",
            offset, rotation, &error)
            && ok;
    }
    if (ok)
    {
        evaluateCharacter(true);
    }
    return ok;
}

bool CharacterScreen::setWingPairs(int pairs)
{
    pairs = std::clamp(pairs, 0, 3);
    CharacterError error = CharacterError::None;
    (void)error;
    bool ok = true;
    while (wingPairs_ > pairs)
    {
        const std::string leftName = "wing" + std::to_string(wingPairs_) + "_left";
        const std::string rightName = "wing" + std::to_string(wingPairs_) + "_right";
        const InstanceId left = instanceByName(leftName);
        const InstanceId right = instanceByName(rightName);
        if (left != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, left) && ok;
        }
        if (right != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, right) && ok;
        }
        if (wingPairs_ >= 0 && wingPairs_ < static_cast<int>(wingSlotRolls_.size()))
        {
            wingSlotRolls_[static_cast<std::size_t>(wingPairs_)] = 0.0f;
        }
        wingPairs_ -= 1;
    }
    while (wingPairs_ < pairs)
    {
        wingPairs_ += 1;
        ok = addWingPairToSlot(wingPairs_) && ok;
    }
    if (ok)
    {
        evaluateCharacter(true);
    }
    return ok;
}

bool CharacterScreen::addWingPairToSlot(int slot)
{
    CharacterError error = CharacterError::None;
    InstanceId parent = instanceByName(activePlan_ == 0 || activePlan_ == 2 ? "torso" : "chest");
    if (parent == engine::character::invalidInstance)
    {
        parent = instanceByName("body");
    }
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    char leftName[16];
    char rightName[16];
    std::snprintf(leftName, sizeof(leftName), "wing%d_left", slot);
    std::snprintf(rightName, sizeof(rightName), "wing%d_right", slot);
    char connector[2][16];
    std::snprintf(connector[0], sizeof(connector[0]), "wings%d.left", slot);
    std::snprintf(connector[1], sizeof(connector[1]), "wings%d.right", slot);
    const InstanceId left = character_->addPart(
        character_->self, characterHandle_, "wing_dragon_left", leftName, &error);
    const InstanceId right = left != engine::character::invalidInstance
        ? character_->addPart(character_->self, characterHandle_, "wing_dragon_right", rightName, &error)
        : engine::character::invalidInstance;
    if (left == engine::character::invalidInstance || right == engine::character::invalidInstance)
    {
        return false;
    }
    // Stacked pairs share the same wing geometry, so each slot rolls a
    // little further outward - otherwise three 1.55 m wings intersect into
    // one shark-fin wall (rev-7 frames 08/09). The authored roll is STORED
    // per slot so the flap driver can compose with it every frame instead
    // of overwriting it (the rev-9 driver replaced the connection rotation,
    // collapsing all pairs to the same roll as soon as they flapped).
    const float rollL = slot == 1 ? 0.0f : slot == 2 ? -12.0f : -22.0f;
    if (slot >= 0 && slot < static_cast<int>(wingSlotRolls_.size()))
    {
        wingSlotRolls_[static_cast<std::size_t>(slot)] = rollL;
    }
    const float rollR = -rollL;
    float leftRoll[4];
    float rightRoll[4];
    setRollRotation(leftRoll, rollL);
    setRollRotation(rightRoll, rollR);
    bool connected = character_->connect(
        character_->self, characterHandle_, parent, connector[0], left, "wing.root", nullptr, leftRoll, &error);
    connected = character_->connect(
                   character_->self, characterHandle_, parent, connector[1], right, "wing.root", nullptr, rightRoll,
                   &error)
        && connected;
    return connected;
}

bool CharacterScreen::setArmPairs(int pairs)
{
    pairs = std::clamp(pairs, 1, 3);
    if (activePlan_ == 3)
    {
        pairs = 1; // the quadruped plan keeps the animal's own four limbs
    }
    CharacterError error = CharacterError::None;
    bool ok = true;
    while (armPairs_ > pairs)
    {
        const std::string leftName = "arm" + std::to_string(armPairs_) + "_left";
        const std::string rightName = "arm" + std::to_string(armPairs_) + "_right";
        const InstanceId left = instanceByName(leftName);
        const InstanceId right = instanceByName(rightName);
        if (left != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, left) && ok;
        }
        if (right != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, right) && ok;
        }
        armPairs_ -= 1;
    }
    while (armPairs_ < pairs)
    {
        armPairs_ += 1;
        const int slot = armPairs_; // 2 or 3
        const InstanceId parent = instanceByName(activePlan_ == 0 || activePlan_ == 2 ? "torso" : "chest");
        if (parent == engine::character::invalidInstance)
        {
            ok = false;
            break;
        }
        const std::string leftName = "arm" + std::to_string(slot) + "_left";
        const std::string rightName = "arm" + std::to_string(slot) + "_right";
        const std::string connectorLeft = "arm" + std::to_string(slot) + ".left";
        const std::string connectorRight = "arm" + std::to_string(slot) + ".right";
        const InstanceId left = character_->addPart(
            character_->self, characterHandle_, "arm_human_left", leftName.c_str(), &error);
        const InstanceId right = left != engine::character::invalidInstance
            ? character_->addPart(character_->self, characterHandle_, "arm_human_right", rightName.c_str(), &error)
            : engine::character::invalidInstance;
        if (left == engine::character::invalidInstance || right == engine::character::invalidInstance)
        {
            ok = false;
            break;
        }
        // Extra pairs fan OUTWARD in the coronal plane, lower pairs swinging
        // wider than the first (marilith-style fan). A small upward offset
        // keeps the second/third shoulder rings from clipping the first
        // pair's deltoids; the roll about the forward axis does the visible
        // work (the rev-7 yaw attempt only spun hanging arms in place).
        const float fanRoll = slot == 2 ? 24.0f : 40.0f;
        const float lift = slot == 2 ? 0.030f : 0.010f;
        const float offset[3] = {0.0f, lift, 0.0f};
        float leftPose[4];
        float rightPose[4];
        setRollRotation(leftPose, -fanRoll);
        setRollRotation(rightPose, fanRoll);
        bool connected = character_->connect(character_->self, characterHandle_, parent, connectorLeft.c_str(), left,
            "shoulder.root", offset, leftPose, &error);
        connected = character_->connect(character_->self, characterHandle_, parent, connectorRight.c_str(), right,
            "shoulder.root", offset, rightPose, &error)
            && connected;
        ok = connected && ok;
    }
    if (ok)
    {
        evaluateCharacter(true);
    }
    return ok;
}

bool CharacterScreen::setTail(bool present)
{
    tailPresent_ = present;
    const InstanceId tail = instanceByName("tail");
    if (!present)
    {
        if (tail != engine::character::invalidInstance)
        {
            if (character_->removePart(character_->self, characterHandle_, tail))
            {
                evaluateCharacter(true);
                return true;
            }
            return false;
        }
        return true;
    }
    if (tail != engine::character::invalidInstance || activePlan_ == 2 || activePlan_ == 3)
    {
        return true; // already present, or the plan carries its own tail
    }
    CharacterError error = CharacterError::None;
    (void)error;
    const InstanceId parent = instanceByName(activePlan_ == 1 ? "chest" : "torso");
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    const InstanceId tailPart = character_->addPart(
        character_->self, characterHandle_, activeTailPart_.c_str(), "tail", &error);
    (void)error;
    if (tailPart == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const bool connected = character_->connect(
        character_->self, characterHandle_, parent, "tail.base", tailPart, "tail.root", nullptr, identity, &error);
    if (connected)
    {
        evaluateCharacter(true);
    }
    return connected;
}

bool CharacterScreen::setTailPart(const std::string &partId)
{
    activeTailPart_ = partId;
    const InstanceId tail = instanceByName("tail");
    if (tail == engine::character::invalidInstance || activePlan_ == 2 || activePlan_ == 3)
    {
        return true; // variant remembered; applies when a tail is attached
    }
    CharacterError error = CharacterError::None;
    const bool swapped = character_->replacePart(
        character_->self, characterHandle_, tail, partId.c_str(), &error);
    if (swapped)
    {
        evaluateCharacter(true);
    }
    return swapped;
}

void CharacterScreen::applyParameterHeights()
{
    // Keep the feet (or hooves) near the ground plane: limb length scales the
    // leg mesh about its hip anchor, so the assembly root drops by the scaled
    // sole offset below the anchor. Quadruped and centaur plans ground on the
    // animal body instead (the measured hooves' lowest point below the stub),
    // PER SPECIES - rev-9 grounded every centaur species on the horse's
    // hoof line, which left the smaller animals floating.
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float limbLength = 1.0f;
    character_->getParameter(character_->self, characterHandle_, "limbs.length", &limbLength);
    float height = 1.0f;
    character_->getParameter(character_->self, characterHandle_, "body.height", &height);
    const int species = std::clamp(quadSpecies_, 0, kQuadSpeciesCount - 1);
    if (activePlan_ == 3)
    {
        const float ground = kQuadGroundY[species] * height;
        const float translation[3] = {0.0f, -ground - 0.01f, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    if (activePlan_ == 1)
    {
        // Centaur: ground on the (headless) animal body - its hooves define
        // the lowest point below the Body-joint origin, per species.
        const float ground = kQuadGroundY[species] * height;
        const float translation[3] = {0.0f, -ground - 0.01f, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    if (activePlan_ == 2)
    {
        // Naga: ground on the pitched serpent lower body. The profile
        // connection drops the ring centre by 0.031 m below the tailbone
        // socket; the piece's measured lowest point sits serpentGroundY
        // (negative) below the part origin. The lift keeps the tube's
        // bottom surface just ABOVE the floor plane - the tube is a wide
        // flat surface, and letting it dip below y=0 z-fights the floor
        // quad into vanishing at grazing camera angles (the piece would
        // read as disconnected from the rear views).
        const float lift = -(kCharacterMetrics.serpentGroundY + 0.065f - 0.031f) * height;
        const float translation[3] = {0.0f, lift + 0.02f, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    const float legScale = 1.0f + (limbLength - 1.0f);
    // legFootY is the sole's offset BELOW the hip anchor (negative); the
    // root rises by its absolute value so the soles touch the ground plane.
    const float translation[3] = {0.0f, -kCharacterMetrics.legFootY * legScale - 0.01f, 0.0f};
    character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
}

void CharacterScreen::resetToDefault()
{
    activeHeadPart_ = "head_human";
    activeTailPart_ = "tail_fox";
    activePlan_ = 0;
    quadSpecies_ = 0;
    headCount_ = 1;
    wingPairs_ = 0;
    armPairs_ = 1;
    tailPresent_ = false;
    CharacterError error = CharacterError::None;
    character_->setParameter(character_->self, characterHandle_, "body.plan", 0.0f, &error);
    if (!rebuildAssembly())
    {
        return;
    }
    setStatusMessage("Character reset to the humanoid profile");
}

bool CharacterScreen::evaluateCharacter(bool validate)
{
    CharacterError error     = CharacterError::None;
    const bool     evaluated = character_->evaluate(character_->self, characterHandle_, &error);
    if (validate)
    {
        issues_.clear();
        const std::uint32_t issueCount = character_->validate(character_->self, characterHandle_, nullptr, 0);
        if (issueCount > 0)
        {
            issues_.resize(std::min<std::uint32_t>(issueCount, 16));
            character_->validate(
                character_->self, characterHandle_, issues_.data(), static_cast<std::uint32_t>(issues_.size()));
        }
    }
    return evaluated;
}

// ---- input ---------------------------------------------------------------------


} // namespace demo
