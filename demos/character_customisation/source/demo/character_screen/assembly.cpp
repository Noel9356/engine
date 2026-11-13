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

std::string CharacterScreen::partIdOfInstance(const std::string &name) const
{
    const InstanceId instance = instanceByName(name);
    if (instance == engine::character::invalidInstance)
    {
        return {};
    }
    const char *partId = nullptr;
    if (character_->instanceInfo(character_->self, characterHandle_, instance, nullptr, &partId, nullptr)
        && partId != nullptr)
    {
        return partId;
    }
    return {};
}

// ---- assembly helpers (body plans, variants, clothing) ------------------------

bool CharacterScreen::rebuildAssembly()
{
    // (Re)create the assembly from the active body-plan profile and re-apply
    // the current variants. Plan changes alter the skeleton topology, so the
    // assembly is rebuilt; variants re-attach afterwards.
    CharacterError error = CharacterError::None;
    if (characterHandle_ != engine::character::invalidCharacter)
    {
        character_->destroyAssembly(character_->self, characterHandle_);
        characterHandle_ = engine::character::invalidCharacter;
    }
    const char *profileId = activePlan_ == 1
        ? CharacterContent::taurProfileId
        : (activePlan_ == 2 ? CharacterContent::quadrupedProfileId : CharacterContent::humanoidProfileId);
    characterHandle_ = character_->createAssembly(character_->self, profileId, &error);
    if (characterHandle_ == engine::character::invalidCharacter)
    {
        std::fprintf(stderr, "demo: could not create the %s assembly (%u)\n", profileId,
            static_cast<unsigned>(error));
        return false;
    }
    // Species lower body for the taur/quadruped plans.
    if (activePlan_ != 0)
    {
        setQuadSpecies(quadSpecies_);
    }
    // Variants: head (+ears), outfit, tail, wings.
    setHeadPart(activeHeadPart_);
    setOutfit(clothed_);
    setTail(tailPresent_);
    setWings(wingsPresent_);
    applyParameterHeights();
    evaluateCharacter(true);
    return true;
}

bool CharacterScreen::setPlan(int plan)
{
    if (plan == activePlan_ || plan < 0 || plan > 2)
    {
        return true;
    }
    activePlan_ = plan;
    // Wings do not graft onto the quadruped plan's silhouette without a
    // withers socket pair; the taur keeps them.
    if (activePlan_ == 2)
    {
        wingsPresent_ = false;
    }
    return rebuildAssembly();
}

bool CharacterScreen::setQuadSpecies(int species)
{
    if (species < 0 || species >= metrics::kSpeciesCount)
    {
        return true;
    }
    const std::string bodyId = activePlan_ == 1
        ? std::string("quad_taur_") + metrics::kSpecies[species].id
        : std::string("quad_body_") + metrics::kSpecies[species].id;
    const InstanceId body = instanceByName("body");
    if (body == engine::character::invalidInstance)
    {
        return false;
    }
    // Skip only when the body instance already wears the requested part -
    // comparing part ids (comparing species indices silently skipped the
    // swap whenever the requested index matched the field default).
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
    // The taur's tail stays with the species: re-point the tail variant to
    // the matching species tail so the lower body reads as one animal.
    if (replaced && activePlan_ == 1)
    {
        setTailPart(std::string("tail_") + metrics::kSpecies[species].id);
    }
    applyParameterHeights();
    evaluateCharacter(true);
    return replaced;
}

bool CharacterScreen::headHasEarSockets() const
{
    if (activeHeadPart_.rfind("head_", 0) != 0 || activeHeadPart_ == "head_human"
        || activeHeadPart_ == "head_dragon")
    {
        return false;
    }
    const std::string species = activeHeadPart_.substr(5);
    for (int index = 0; index < metrics::kSpeciesCount; ++index)
    {
        if (species == metrics::kSpecies[index].id)
        {
            return metrics::kSpecies[index].hasEarSockets;
        }
    }
    return false;
}

bool CharacterScreen::setHeadPart(const std::string &partId)
{
    activeHeadPart_ = partId;
    const InstanceId head = instanceByName("head");
    if (head == engine::character::invalidInstance)
    {
        return true; // remembered; the plan rebuild attaches it
    }
    CharacterError error = CharacterError::None;
    const bool swapped = character_->replacePart(
        character_->self, characterHandle_, head, partId.c_str(), &error);
    if (!swapped)
    {
        return false;
    }
    // Species heads wear their matching ear pair by default; the human and
    // dragon heads carry their own ears and remove the parts.
    if (headHasEarSockets())
    {
        if (activeEarPart_.empty())
        {
            activeEarPart_ = std::string("ear_") + partId.substr(5) + "_left";
        }
        setEarParts(activeEarPart_);
    }
    else
    {
        activeEarPart_.clear();
        for (const char *earName : {"ear_left", "ear_right"})
        {
            const InstanceId ear = instanceByName(earName);
            if (ear != engine::character::invalidInstance)
            {
                character_->removePart(character_->self, characterHandle_, ear);
            }
        }
    }
    evaluateCharacter(true);
    return true;
}

bool CharacterScreen::setEarParts(const std::string &leftPartId)
{
    activeEarPart_ = leftPartId;
    const std::string rightPartId = leftPartId.substr(0, leftPartId.size() - 5) + "_right";
    CharacterError error = CharacterError::None;
    const std::tuple<const char *, const char *, std::string> sides[2] = {
        {"ear_left", "ear.left", leftPartId},
        {"ear_right", "ear.right", rightPartId},
    };
    for (const auto &[earName, earConnector, partId] : sides)
    {
        const InstanceId ear = instanceByName(earName);
        if (ear == engine::character::invalidInstance)
        {
            const InstanceId added = character_->addPart(character_->self, characterHandle_, partId.c_str(),
                earName, &error);
            if (added == engine::character::invalidInstance)
            {
                return false;
            }
            const InstanceId parent = instanceByName("head");
            if (parent == engine::character::invalidInstance)
            {
                return false;
            }
            const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
            if (!character_->connect(character_->self, characterHandle_, parent, earConnector, added, "ear.root",
                    nullptr, identity, &error))
            {
                return false;
            }
        }
        else
        {
            const char *currentPartId = nullptr;
            character_->instanceInfo(character_->self, characterHandle_, ear, nullptr, &currentPartId, nullptr);
            if (currentPartId == nullptr || partId != currentPartId)
            {
                character_->replacePart(character_->self, characterHandle_, ear, partId.c_str(), &error);
            }
        }
    }
    evaluateCharacter(true);
    return true;
}

bool CharacterScreen::setWings(bool present)
{
    wingsPresent_ = present;
    CharacterError error = CharacterError::None;
    const bool exists = instanceByName("wing_left") != engine::character::invalidInstance;
    if (!present)
    {
        if (exists)
        {
            for (const char *wingName : {"wing_left", "wing_right"})
            {
                const InstanceId wing = instanceByName(wingName);
                if (wing != engine::character::invalidInstance)
                {
                    character_->removePart(character_->self, characterHandle_, wing);
                }
            }
            evaluateCharacter(true);
        }
        return true;
    }
    if (exists || activePlan_ == 2)
    {
        return true; // already present, or the plan has no wing sockets
    }
    // Exactly one pair: on the humanoid torso's back sockets or the taur's
    // withers sockets.
    InstanceId parent = instanceByName(activePlan_ == 0 ? "torso" : "body");
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    (void)identity;
    bool ok = true;
    const std::tuple<const char *, const char *, const char *> wings[2] = {
        {"wing_left", "wing_dragon_left", "wings.left"},
        {"wing_right", "wing_dragon_right", "wings.right"},
    };
    for (const auto &[wingName, partId, connector] : wings)
    {
        const InstanceId wing = character_->addPart(character_->self, characterHandle_, partId, wingName, &error);
        if (wing == engine::character::invalidInstance)
        {
            ok = false;
            break;
        }
        // A small outward roll keeps the pair readable around the back.
        const float degrees = wingName[4] == 'l' ? -8.0f : 8.0f;
        const glm::quat roll = glm::angleAxis(glm::radians(degrees), glm::vec3(0.0f, 0.0f, 1.0f));
        const float rotation[4] = {roll.x, roll.y, roll.z, roll.w};
        ok = character_->connect(character_->self, characterHandle_, parent, connector, wing, "wing.root",
                   nullptr, rotation, &error)
            && ok;
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
    if (tail != engine::character::invalidInstance || activePlan_ == 2)
    {
        return true; // already present, or the plan carries its own tail
    }
    CharacterError error = CharacterError::None;
    // Taur: the lower body's own tail socket. Humanoid: the torso tailbone.
    const bool        taur  = activePlan_ == 1;
    const InstanceId  parent = instanceByName(taur ? "body" : "torso");
    const char       *parentConnector = taur ? "tail.mount" : "tail.base";
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    const InstanceId tailPart = character_->addPart(
        character_->self, characterHandle_, activeTailPart_.c_str(), "tail", &error);
    if (tailPart == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const bool connected = character_->connect(
        character_->self, characterHandle_, parent, parentConnector, tailPart, "tail.root", nullptr, identity, &error);
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
    if (tail == engine::character::invalidInstance || activePlan_ == 2)
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

bool CharacterScreen::setOutfit(bool clothed)
{
    clothed_ = clothed;
    CharacterError error = CharacterError::None;
    const char *torsoId   = clothed ? "torso_ranger" : "torso_bare";
    const char *armId     = clothed ? "arm_ranger_left" : "arm_bare_left";
    const char *armIdRight = clothed ? "arm_ranger_right" : "arm_bare_right";
    const char *legId     = clothed ? "leg_ranger_left" : "leg_bare_left";
    const char *legIdRight = clothed ? "leg_ranger_right" : "leg_bare_right";
    const char *torsoInstance = activePlan_ == 1 ? "chest" : "torso";
    bool ok = true;
    const auto swap = [&](const char *instanceName, const char *partId) {
        const InstanceId instance = instanceByName(instanceName);
        if (instance == engine::character::invalidInstance)
        {
            return; // plan without this slot (quadruped)
        }
        const char *currentPartId = nullptr;
        character_->instanceInfo(character_->self, characterHandle_, instance, nullptr, &currentPartId, nullptr);
        if (currentPartId != nullptr && partId == currentPartId)
        {
            return;
        }
        ok = character_->replacePart(character_->self, characterHandle_, instance, partId, &error) && ok;
    };
    swap(torsoInstance, torsoId);
    swap("arm_left", armId);
    swap("arm_right", armIdRight);
    swap("leg_left", legId);
    swap("leg_right", legIdRight);
    // The garment overlays only read correctly over the clothed variant;
    // strip them when going bare.
    if (!clothed)
    {
        setClothingPiece("belt", false);
        setClothingPiece("bracer_left", false);
        setClothingPiece("bracer_right", false);
        setClothingPiece("pauldron_left", false);
        setClothingPiece("pauldron_right", false);
        beltPresent_      = false;
        bracersPresent_  = false;
        pauldronsPresent_ = false;
    }
    else if (!beltPresent_ && !bracersPresent_ && !pauldronsPresent_)
    {
        // first switch back to the outfit: restore the shipped look
        beltPresent_      = true;
        bracersPresent_  = true;
        pauldronsPresent_ = true;
        setClothingPiece("belt", true);
        setClothingPiece("bracer_left", true);
        setClothingPiece("bracer_right", true);
        setClothingPiece("pauldron_left", true);
        setClothingPiece("pauldron_right", true);
    }
    applyParameterHeights();
    evaluateCharacter(true);
    return ok;
}

bool CharacterScreen::setClothingPiece(const char *instanceName, bool present)
{
    CharacterError error = CharacterError::None;
    const InstanceId piece = instanceByName(instanceName);
    if (!present)
    {
        if (piece != engine::character::invalidInstance)
        {
            const bool removed = character_->removePart(character_->self, characterHandle_, piece);
            evaluateCharacter(true);
            return removed;
        }
        return true;
    }
    if (piece != engine::character::invalidInstance)
    {
        return true;
    }
    // Parent + connector per piece.
    const char *partId = nullptr;
    const char *parentName = nullptr;
    const char *parentConnector = nullptr;
    const char *childConnector = "garment.root";
    if (std::strcmp(instanceName, "belt") == 0)
    {
        partId = "clothes_belt";
        parentName = activePlan_ == 1 ? "chest" : "torso";
        parentConnector = "belt.mount";
    }
    else if (std::strcmp(instanceName, "bracer_left") == 0)
    {
        partId = "clothes_bracer_left";
        parentName = "arm_left";
        parentConnector = "bracer.mount";
    }
    else if (std::strcmp(instanceName, "bracer_right") == 0)
    {
        partId = "clothes_bracer_right";
        parentName = "arm_right";
        parentConnector = "bracer.mount";
    }
    else if (std::strcmp(instanceName, "pauldron_left") == 0)
    {
        partId = "clothes_pauldron_left";
        parentName = activePlan_ == 1 ? "chest" : "torso";
        parentConnector = "pad.left";
    }
    else if (std::strcmp(instanceName, "pauldron_right") == 0)
    {
        partId = "clothes_pauldron_right";
        parentName = activePlan_ == 1 ? "chest" : "torso";
        parentConnector = "pad.right";
    }
    else if (std::strcmp(instanceName, "hood") == 0)
    {
        partId = "clothes_hood";
        parentName = "head";
        parentConnector = "hood.mount";
    }
    else
    {
        return false;
    }
    const InstanceId parent = instanceByName(parentName);
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    const InstanceId added = character_->addPart(character_->self, characterHandle_, partId, instanceName, &error);
    if (added == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const bool connected = character_->connect(
        character_->self, characterHandle_, parent, parentConnector, added, childConnector, nullptr, identity, &error);
    if (connected)
    {
        evaluateCharacter(true);
    }
    return connected;
}

void CharacterScreen::applyParameterHeights()
{
    // Keep the feet (or hooves) near the ground plane: limb length scales the
    // leg mesh about its hip anchor, so the assembly root drops by the scaled
    // sole offset below the anchor. The taur and quadruped plans ground on
    // the animal body instead (the measured hooves' lowest point below the
    // body origin, per species, riding the lower-body scale).
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const int species = std::clamp(quadSpecies_, 0, metrics::kSpeciesCount - 1);
    if (activePlan_ == 1 || activePlan_ == 2)
    {
        float taurScale = 1.0f;
        character_->getParameter(character_->self, characterHandle_, "taur.scale", &taurScale);
        const float ground = metrics::kSpecies[species].groundY * taurScale;
        const float translation[3] = {0.0f, -ground - 0.01f, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    const std::string legPart = partIdOfInstance("leg_left");
    const float footY = content_.legFootYForPart(legPart);
    float limbLength = 1.0f;
    character_->getParameter(character_->self, characterHandle_, "limbs.length", &limbLength);
    const float legScale = 1.0f + (limbLength - 1.0f);
    // legFootY is the sole's offset BELOW the hip anchor (negative); the
    // root rises by its absolute value so the soles touch the ground plane.
    const float translation[3] = {0.0f, -footY * legScale - 0.01f, 0.0f};
    character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
}

void CharacterScreen::resetToDefault()
{
    activeHeadPart_ = "head_human";
    activeEarPart_.clear();
    activeTailPart_ = "tail_fox";
    activePlan_ = 0;
    quadSpecies_ = 0;
    wingsPresent_ = false;
    tailPresent_ = false;
    clothed_ = true;
    beltPresent_ = true;
    bracersPresent_ = true;
    pauldronsPresent_ = true;
    hoodPresent_ = false;
    CharacterError error = CharacterError::None;
    character_->setParameter(character_->self, characterHandle_, "body.plan", 0.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "muzzle.length", 1.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "eyes.width", 1.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "fur.markings", 0.0f, &error);
    refreshDeformations();
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

} // namespace demo
