#include "../character_metrics.gen.hpp"
#include "../character_screen_internal.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <tuple>

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

void CharacterScreen::fitCameraToAssembly()
{
    // Frame the whole assembled character: the world-space bounds of every
    // instance (its part bounds through its instance transform) become the
    // orbit target and distance. Fixed framing leaves taurs and quadrupeds
    // tiny in frame; the refit runs on structural changes (plans, species,
    // drive presets) and never fights an explicit CLI framing.
    if (character_ == nullptr || characterHandle_ == engine::character::invalidCharacter || userCameraFraming_)
    {
        return;
    }
    glm::vec3           boundsMin(std::numeric_limits<float>::max());
    glm::vec3           boundsMax(std::numeric_limits<float>::lowest());
    bool                any   = false;
    const std::uint32_t count = character_->instanceCount(character_->self, characterHandle_);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const engine::character::InstanceId id     = character_->instanceId(character_->self, characterHandle_, index);
        const char                         *partId = nullptr;
        if (!character_->instanceInfo(character_->self, characterHandle_, id, nullptr, &partId, nullptr)
            || partId == nullptr)
        {
            continue;
        }
        const PartRenderInfo *renderInfo = content_.renderInfoForPart(partId);
        if (renderInfo == nullptr)
        {
            continue;
        }
        float localMin[3] = {0.0f, 0.0f, 0.0f};
        float localMax[3] = {0.0f, 0.0f, 0.0f};
        if (!content_.geometryExtents(renderInfo->geometryId, localMin, localMax))
        {
            continue;
        }
        float matrix[16] = {};
        if (!character_->instanceTransform(character_->self, characterHandle_, id, matrix))
        {
            continue;
        }
        const glm::mat4 transform = glm::make_mat4(matrix);
        for (int corner = 0; corner < 8; ++corner)
        {
            const glm::vec4 local(corner & 1 ? localMax[0] : localMin[0],
                corner & 2 ? localMax[1] : localMin[1],
                corner & 4 ? localMax[2] : localMin[2],
                1.0f);
            const glm::vec3 world = glm::vec3(transform * local);
            boundsMin             = glm::min(boundsMin, world);
            boundsMax             = glm::max(boundsMax, world);
            any                   = true;
        }
    }
    if (!any)
    {
        return;
    }
    const glm::vec3 center = (boundsMin + boundsMax) * 0.5f;
    const glm::vec3 size   = boundsMax - boundsMin;
    const float     radius = 0.5f * glm::length(size);
    // The 45-degree orbit yaw sees the diagonal; add headroom so the fit
    // keeps the full silhouette (ears and tail included) inside the frame.
    // The distance floor follows the silhouette: small animals need a
    // sub-metre orbit, so the refit never parks them far away.
    const float targetY  = center.y;
    const float distance = std::clamp(radius * 2.3f, 0.35f, 9.0f);
    configureCamera(52.0f, 10.0f, distance, targetY);
    // The orbit target is the box centre itself: long-tailed taurs centre
    // far behind their torso, and targeting the origin kept those presets
    // pushed to one side of the frame.
    const float orbitTarget[3] = {center.x, center.y, center.z};
    camera_->setOrbitTarget(camera_->self, cameraHandle_, orbitTarget);
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
    const char *profileId = activePlan_ == 1 ? CharacterContent::taurProfileId
                                             : (activePlan_ == 2 ? CharacterContent::quadrupedProfileId
                                                                 : CharacterContent::humanoidProfileId);
    characterHandle_      = character_->createAssembly(character_->self, profileId, &error);
    if (characterHandle_ == engine::character::invalidCharacter)
    {
        std::fprintf(stderr, "demo: could not create the %s assembly (%u)\n", profileId, static_cast<unsigned>(error));
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
    // The taur's upper assembly rides taur.scale (the graft weld and every
    // part above it). Leaving the plan resets it so the biped - whose parts
    // only bind the body sliders - never inherits a stale animal size.
    if (activePlan_ != 1)
    {
        CharacterError error = CharacterError::None;
        character_->setParameter(character_->self, characterHandle_, "taur.scale", 1.0f, &error);
    }
    const bool rebuilt = rebuildAssembly();
    // A plan change re-proportions the whole character (taurs sprawl, the
    // quadruped drops to four legs): refit the studio framing.
    fitCameraToAssembly();
    return rebuilt;
}

bool CharacterScreen::setQuadSpecies(int species)
{
    if (species < 0 || species >= metrics::kSpeciesCount)
    {
        return true;
    }
    const std::string bodyId = activePlan_ == 1 ? std::string("quad_taur_") + metrics::kSpecies[species].id
                                                : std::string("quad_body_") + metrics::kSpecies[species].id;
    const InstanceId  body   = instanceByName("body");
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
    quadSpecies_            = species;
    CharacterError error    = CharacterError::None;
    const bool     replaced = character_->replacePart(character_->self, characterHandle_, body, bodyId.c_str(), &error);
    // The taur's tail stays with the species: re-point the tail variant to
    // the matching species tail so the lower body reads as one animal (the
    // plan mapping swaps in the taur-scaled variant automatically).
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
    if (activeHeadPart_.rfind("head_", 0) != 0 || activeHeadPart_ == "head_human" || activeHeadPart_ == "head_dragon")
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
    activeHeadPart_       = partId;
    const InstanceId head = instanceByName("head");
    if (head == engine::character::invalidInstance)
    {
        return true; // remembered; the plan rebuild attaches it
    }
    CharacterError error   = CharacterError::None;
    const bool     swapped = character_->replacePart(character_->self, characterHandle_, head, partId.c_str(), &error);
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
    // The neck follows the head's species (the weld-ring chain continues
    // collar -> neck -> skull); the human and dragon heads ride the human
    // neck loft.
    std::string neckPart = "neck_human";
    if (partId.rfind("head_", 0) == 0 && partId != "head_human" && partId != "head_dragon")
    {
        neckPart = "neck_" + partId.substr(5);
    }
    setNeckPart(neckPart);
    evaluateCharacter(true);
    return true;
}

bool CharacterScreen::setNeckPart(const std::string &partId)
{
    activeNeckPart_       = partId;
    const InstanceId neck = instanceByName("neck");
    if (neck == engine::character::invalidInstance)
    {
        return true; // remembered; the plan rebuild attaches it
    }
    CharacterError error   = CharacterError::None;
    const bool     swapped = character_->replacePart(character_->self, characterHandle_, neck, partId.c_str(), &error);
    evaluateCharacter(true);
    return swapped;
}

bool CharacterScreen::setEarParts(const std::string &leftPartId)
{
    activeEarPart_                = leftPartId;
    const std::string rightPartId = leftPartId.substr(0, leftPartId.size() - 5) + "_right";
    CharacterError    error       = CharacterError::None;
    const std::tuple<const char *, const char *, std::string> sides[2] = {
        {"ear_left", "ear.left", leftPartId},
        {"ear_right", "ear.right", rightPartId},
    };
    for (const auto &[earName, earConnector, partId] : sides)
    {
        const InstanceId ear = instanceByName(earName);
        if (ear == engine::character::invalidInstance)
        {
            const InstanceId added
                = character_->addPart(character_->self, characterHandle_, partId.c_str(), earName, &error);
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
            if (!character_->connect(character_->self,
                    characterHandle_,
                    parent,
                    earConnector,
                    added,
                    "ear.root",
                    nullptr,
                    identity,
                    &error))
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
    wingsPresent_         = present;
    CharacterError error  = CharacterError::None;
    const bool     exists = instanceByName("wing_left") != engine::character::invalidInstance;
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
    // Exactly one pair: on the thoracic segment's back sockets or the taur's
    // withers sockets.
    InstanceId parent = instanceByName(activePlan_ == 0 ? "chest" : "body");
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    bool                                                       ok       = true;
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
        // The pair arcs up and back over the shoulders (a dihedral arch:
        // the source wing sweeps forward, so the connection pitches it
        // behind the torso) with a small outward roll for readability.
        const float     degrees          = wingName[4] == 'l' ? -8.0f : 8.0f;
        const glm::quat roll             = glm::angleAxis(glm::radians(degrees), glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::quat pitch            = glm::angleAxis(glm::radians(-105.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::quat rotation         = pitch * roll;
        const float     rotationArray[4] = {rotation.x, rotation.y, rotation.z, rotation.w};
        ok                               = character_->connect(character_->self,
                                               characterHandle_,
                                               parent,
                                               connector,
                                               wing,
                                               "wing.root",
                                               nullptr,
                                               rotationArray,
                                               &error)
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
    tailPresent_          = present;
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
    // Taur: the lower body's own coccyx socket. Humanoid: the sacrum-coccyx
    // segment's tailbone - the tail always rides the coccyx.
    const bool       taur            = activePlan_ == 1;
    const InstanceId parent          = instanceByName(taur ? "body" : "pelvis");
    const char      *parentConnector = taur ? "tail.mount" : "tail.base";
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    const InstanceId tailPart = character_->addPart(
        character_->self, characterHandle_, tailPartForPlan(activeTailPart_).c_str(), "tail", &error);
    if (tailPart == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const bool  connected   = character_->connect(
        character_->self, characterHandle_, parent, parentConnector, tailPart, "tail.root", nullptr, identity, &error);
    if (connected)
    {
        evaluateCharacter(true);
    }
    return connected;
}

std::string CharacterScreen::tailPartForPlan(const std::string &partId) const
{
    // The taur plan rides the taur-scaled tail variants (they bind
    // taur.scale, so the tail tracks the lower body at every slider value);
    // every other plan uses the natural-scale tails. The reptile tail has no
    // variant and is used as-is.
    if (partId == "tail_reptile")
    {
        return partId;
    }
    const bool taurVariant = partId.size() > 5 && partId.compare(partId.size() - 5, 5, "_taur") == 0;
    if (activePlan_ == 1 && !taurVariant)
    {
        return partId + "_taur";
    }
    if (activePlan_ != 1 && taurVariant)
    {
        return partId.substr(0, partId.size() - 5);
    }
    return partId;
}

bool CharacterScreen::setTailPart(const std::string &partId)
{
    activeTailPart_       = partId;
    const InstanceId tail = instanceByName("tail");
    if (tail == engine::character::invalidInstance || activePlan_ == 2)
    {
        return true; // variant remembered; applies when a tail is attached
    }
    CharacterError    error    = CharacterError::None;
    const std::string planPart = tailPartForPlan(partId);
    const bool swapped = character_->replacePart(character_->self, characterHandle_, tail, planPart.c_str(), &error);
    if (swapped)
    {
        evaluateCharacter(true);
    }
    return swapped;
}

bool CharacterScreen::setOutfit(bool clothed)
{
    clothed_                  = clothed;
    CharacterError error      = CharacterError::None;
    const char    *chestId    = clothed ? "torso_thoracic_ranger" : "torso_thoracic_bare";
    const char    *pelvisId   = clothed ? "pelvis_ranger" : "pelvis_bare";
    const char    *armId      = clothed ? "arm_ranger_left" : "arm_bare_left";
    const char    *armIdRight = clothed ? "arm_ranger_right" : "arm_bare_right";
    const char    *legId      = clothed ? "leg_ranger_left" : "leg_bare_left";
    const char    *legIdRight = clothed ? "leg_ranger_right" : "leg_bare_right";
    bool           ok         = true;
    const auto     swap       = [&](const char *instanceName, const char *partId)
    {
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
    swap("chest", chestId);
    // The sacrum-coccyx segment only exists on the biped plan.
    if (activePlan_ == 0)
    {
        swap("pelvis", pelvisId);
    }
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
        bracersPresent_   = false;
        pauldronsPresent_ = false;
    }
    else if (!beltPresent_ && !bracersPresent_ && !pauldronsPresent_)
    {
        // first switch back to the outfit: restore the shipped look
        beltPresent_      = true;
        bracersPresent_   = true;
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
    CharacterError   error = CharacterError::None;
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
    const char *partId          = nullptr;
    const char *parentName      = nullptr;
    const char *parentConnector = nullptr;
    const char *childConnector  = "garment.root";
    if (std::strcmp(instanceName, "belt") == 0)
    {
        partId          = "clothes_belt";
        parentName      = "chest";
        parentConnector = "belt.mount";
    }
    else if (std::strcmp(instanceName, "bracer_left") == 0)
    {
        partId          = "clothes_bracer_left";
        parentName      = "arm_left";
        parentConnector = "bracer.mount";
    }
    else if (std::strcmp(instanceName, "bracer_right") == 0)
    {
        partId          = "clothes_bracer_right";
        parentName      = "arm_right";
        parentConnector = "bracer.mount";
    }
    else if (std::strcmp(instanceName, "pauldron_left") == 0)
    {
        partId          = "clothes_pauldron_left";
        parentName      = "chest";
        parentConnector = "pad.left";
    }
    else if (std::strcmp(instanceName, "pauldron_right") == 0)
    {
        partId          = "clothes_pauldron_right";
        parentName      = "chest";
        parentConnector = "pad.right";
    }
    else if (std::strcmp(instanceName, "hood") == 0)
    {
        partId          = "clothes_hood";
        parentName      = "head";
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
    const bool  connected   = character_->connect(
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
    const int   species     = std::clamp(quadSpecies_, 0, metrics::kSpeciesCount - 1);
    if (activePlan_ == 1)
    {
        // The taur grounds on the normalised lower body's own line.
        float taurScale = 1.0f;
        character_->getParameter(character_->self, characterHandle_, "taur.scale", &taurScale);
        const float ground         = metrics::kSpecies[species].groundY * taurScale;
        const float translation[3] = {0.0f, -ground, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    if (activePlan_ == 2)
    {
        // The quadruped is the whole, natural-scale animal: its own ground
        // line, still riding the lower-body scale slider.
        float taurScale = 1.0f;
        character_->getParameter(character_->self, characterHandle_, "taur.scale", &taurScale);
        const float ground         = metrics::kSpecies[species].groundQuadY * taurScale;
        const float translation[3] = {0.0f, -ground, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    const std::string legPart    = partIdOfInstance("leg_left");
    const float       footY      = content_.legFootYForPart(legPart);
    float             limbLength = 1.0f;
    character_->getParameter(character_->self, characterHandle_, "limbs.length", &limbLength);
    const float legScale = 1.0f + (limbLength - 1.0f);
    // legFootY is the sole's offset BELOW the hip anchor (negative); the
    // root rises by its absolute value so the soles touch the ground plane.
    const float translation[3] = {0.0f, -footY * legScale, 0.0f};
    character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
}

void CharacterScreen::resetToDefault()
{
    activeHeadPart_ = "head_fox";
    activeNeckPart_ = "neck_fox";
    activeEarPart_.clear();
    activeTailPart_      = "tail_fox";
    activePlan_          = 0;
    quadSpecies_         = 0;
    wingsPresent_        = false;
    tailPresent_         = false;
    clothed_             = false;
    beltPresent_         = false;
    bracersPresent_      = false;
    pauldronsPresent_    = false;
    hoodPresent_         = false;
    CharacterError error = CharacterError::None;
    character_->setParameter(character_->self, characterHandle_, "body.plan", 0.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "muzzle.length", 1.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "eyes.width", 1.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "fur.markings", 0.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "fur.coat", 0.0f, &error);
    character_->setParameter(character_->self, characterHandle_, "body.coat", 0.0f, &error);
    if (!rebuildAssembly())
    {
        return;
    }
    applyDefaultLook();
    fitCameraToAssembly();
    setStatusMessage("Character reset to the anthro fox look");
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

void CharacterScreen::dumpInstanceBoxes()
{
    // Numeric framing ground truth: every instance's part id, local bounds,
    // world bounds, then the assembly's world box and the fitted camera
    // numbers. An instance whose world box dwarfs the rest is the one blowing
    // up the camera fit; a world box below the ground reveals sunken sockets.
    if (character_ == nullptr || characterHandle_ == engine::character::invalidCharacter)
    {
        return;
    }
    glm::vec3  worldMin(std::numeric_limits<float>::max());
    glm::vec3  worldMax(std::numeric_limits<float>::lowest());
    bool       any   = false;
    const auto count = character_->instanceCount(character_->self, characterHandle_);
    std::printf("instance-boxes: %u instances\n", count);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const engine::character::InstanceId id = character_->instanceId(character_->self, characterHandle_, index);
        const char                         *instanceName = nullptr;
        const char                         *partId       = nullptr;
        if (!character_->instanceInfo(character_->self, characterHandle_, id, &instanceName, &partId, nullptr)
            || partId == nullptr)
        {
            continue;
        }
        const PartRenderInfo *renderInfo  = content_.renderInfoForPart(partId);
        float                 localMin[3] = {0.0f, 0.0f, 0.0f};
        float                 localMax[3] = {0.0f, 0.0f, 0.0f};
        if (renderInfo == nullptr || !content_.geometryExtents(renderInfo->geometryId, localMin, localMax))
        {
            std::printf("  %-14s %-24s no bounds\n", instanceName ? instanceName : "?", partId);
            continue;
        }
        float matrix[16] = {};
        if (!character_->instanceTransform(character_->self, characterHandle_, id, matrix))
        {
            std::printf("  %-14s %-24s no transform\n", instanceName ? instanceName : "?", partId);
            continue;
        }
        const glm::mat4 transform = glm::make_mat4(matrix);
        glm::vec3       partMin(std::numeric_limits<float>::max());
        glm::vec3       partMax(std::numeric_limits<float>::lowest());
        for (int corner = 0; corner < 8; ++corner)
        {
            const glm::vec4 local(corner & 1 ? localMax[0] : localMin[0],
                corner & 2 ? localMax[1] : localMin[1],
                corner & 4 ? localMax[2] : localMin[2],
                1.0f);
            const glm::vec3 world = glm::vec3(transform * local);
            partMin               = glm::min(partMin, world);
            partMax               = glm::max(partMax, world);
        }
        worldMin = glm::min(worldMin, partMin);
        worldMax = glm::max(worldMax, partMax);
        any      = true;
        std::printf("  %-14s %-24s local[%6.3f %6.3f %6.3f .. %6.3f %6.3f %6.3f]  world[%6.3f %6.3f %6.3f .. %6.3f "
                    "%6.3f %6.3f]\n",
            instanceName ? instanceName : "?",
            partId,
            localMin[0],
            localMin[1],
            localMin[2],
            localMax[0],
            localMax[1],
            localMax[2],
            partMin.x,
            partMin.y,
            partMin.z,
            partMax.x,
            partMax.y,
            partMax.z);
    }
    if (any)
    {
        const glm::vec3 size   = worldMax - worldMin;
        const glm::vec3 center = (worldMin + worldMax) * 0.5f;
        std::printf("  world box: min[%6.3f %6.3f %6.3f] max[%6.3f %6.3f %6.3f] size[%6.3f %6.3f %6.3f] "
                    "center[%6.3f %6.3f %6.3f] radius %.3f\n",
            worldMin.x,
            worldMin.y,
            worldMin.z,
            worldMax.x,
            worldMax.y,
            worldMax.z,
            size.x,
            size.y,
            size.z,
            center.x,
            center.y,
            center.z,
            0.5f * glm::length(size));
    }
}

} // namespace demo
