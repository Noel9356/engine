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

// ---- named drive configurations ---------------------------------------------------

bool CharacterScreen::applyDrivePreset(const std::string &name)
{
    if (!created_ || character_ == nullptr || characterHandle_ == engine::character::invalidCharacter)
    {
        return false;
    }
    if (name.empty())
    {
        return true;
    }

    CharacterError error        = CharacterError::None;
    const auto     setParameter = [&](const char *parameter, float value)
    { return character_->setParameter(character_->self, characterHandle_, parameter, value, &error); };
    bool applied = true;

    if (name == "human")
    {
        // Head variant on the humanoid plan with the default fox tail.
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setHeadPart("head_human") && applied;
        setTail(true);
    }
    else if (name == "dragonhead")
    {
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setHeadPart("head_dragon") && applied;
        setTail(true);
    }
    else if (name == "furryfox")
    {
        // Furry-fandom look: fox head + fox ears + fox tail on the humanoid
        // plan.
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setHeadPart("head_fox") && applied;
        applied = setEarParts("ear_fox_left") && applied;
        applied = setTailPart("tail_fox") && applied;
        setTail(true);
    }
    else if (name == "furrywolf")
    {
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setHeadPart("head_wolf") && applied;
        applied = setEarParts("ear_wolf_left") && applied;
        applied = setTailPart("tail_wolf") && applied;
        setTail(true);
    }
    else if (name == "bare")
    {
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setOutfit(false) && applied;
    }
    else if (name == "taur")
    {
        applied = setPlan(1) && setParameter("body.plan", 1.0f);
        setTail(true);
    }
    else if (name == "quadruped")
    {
        applied = setPlan(2) && setParameter("body.plan", 2.0f);
    }
    else if (name == "fox" || name == "shiba" || name == "husky" || name == "wolf" || name == "donkey"
        || name == "cow" || name == "deer" || name == "bull" || name == "alpaca" || name == "horse"
        || name == "stag")
    {
        // Quadruped species variant: the WHOLE animal (own head, own tail).
        int speciesIndex = -1;
        for (int index = 0; index < metrics::kSpeciesCount; ++index)
        {
            if (name == metrics::kSpecies[index].id)
            {
                speciesIndex = index;
                break;
            }
        }
        if (speciesIndex < 0)
        {
            speciesIndex = 0;
        }
        if (activePlan_ != 2)
        {
            applied = setPlan(2) && setParameter("body.plan", 2.0f);
        }
        applied = setQuadSpecies(speciesIndex) && applied;
    }
    else if (name == "shibainu")
    {
        // the shiba's registry id differs from the preset name
        if (activePlan_ != 2)
        {
            applied = setPlan(2) && setParameter("body.plan", 2.0f);
        }
        applied = setQuadSpecies(1) && applied;
    }
    else if (name.rfind("taur_", 0) == 0)
    {
        // Taur plan on a chosen species: the plan first (the profile's
        // default hindquarters), then the species swap on the live assembly.
        applied = setPlan(1) && setParameter("body.plan", 1.0f);
        setTail(true);
        int speciesIndex = -1;
        const std::string species = name.substr(5);
        for (int index = 0; index < metrics::kSpeciesCount; ++index)
        {
            if (species == metrics::kSpecies[index].id)
            {
                speciesIndex = index;
                break;
            }
        }
        if (speciesIndex < 0)
        {
            std::fprintf(stderr, "demo: unknown taur species '%s'\n", species.c_str());
            applied = false;
        }
        else
        {
            applied = setQuadSpecies(speciesIndex) && applied;
        }
    }
    else if (name == "winged")
    {
        applied = setWings(true) && setTail(true);
    }
    else if (name == "tall")
    {
        // Height range ends: every anchor rides the part scale bindings.
        applied = setParameter("body.height", 1.35f) && setParameter("limbs.length", 1.35f);
        applyParameterHeights();
    }
    else if (name == "broad")
    {
        // Width/mass range ends: shoulders, hips, and arms must stay grafted.
        applied = setParameter("body.width", 1.45f) && setParameter("body.muscle_mass", 1.55f)
               && setParameter("limbs.thickness", 1.55f);
    }
    else if (name == "longmuzzle")
    {
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setHeadPart("head_fox") && applied;
        applied = setEarParts("ear_fox_left") && applied;
        applied = setParameter("muzzle.length", 1.35f) && setParameter("eyes.width", 1.20f) && applied;
        refreshDeformations();
    }
    else if (name == "marked")
    {
        applied = setParameter("fur.markings", 3.0f) && setParameter("fur.coat", 1.0f);
        refreshDeformations();
    }
    else if (name == "longtail")
    {
        // Tail length range end on the tailed humanoid.
        applied = setPlan(0) && setTail(true) && setParameter("tail.length", 1.50f)
               && setParameter("tail.thickness", 1.65f);
    }
    else if (name == "bigtaur")
    {
        // Lower-body size range end on the taur plan.
        applied = setPlan(1) && setParameter("taur.scale", 1.40f);
        applyParameterHeights();
    }
    else if (name == "compact")
    {
        // Everything small at once: the opposite range end of every parameter.
        applied = setParameter("body.height", 0.75f) && setParameter("body.width", 0.70f)
               && setParameter("limbs.length", 0.75f) && setParameter("head.size", 0.75f)
               && setParameter("tail.length", 0.55f) && setParameter("wings.span", 0.55f);
        applyParameterHeights();
    }
    else if (name == "noappend")
    {
        // Strip every optional appendage and the outfit.
        applied = setWings(false) && setTail(false) && setOutfit(false);
    }
    else
    {
        std::fprintf(stderr, "demo: unknown character drive preset '%s'\n", name.c_str());
        return false;
    }

    if (!applied)
    {
        std::fprintf(
            stderr, "demo: character drive preset '%s' failed (%u)\n", name.c_str(), static_cast<unsigned>(error));
        for (const ValidationIssue &issue : issues_)
        {
            std::fprintf(stderr, "  issue %u: %s: %s\n", static_cast<unsigned>(issue.code), issue.subject,
                issue.message);
        }
        return false;
    }
    std::cout << "demo: character drive preset '" << name << "' applied\n";
    applyParameterHeights();
    const bool evaluated = evaluateCharacter(true);
    if (!evaluated)
    {
        std::fprintf(stderr, "  evaluate failed: %s\n", character_->lastErrorMessage(character_->self));
        for (const ValidationIssue &issue : issues_)
        {
            std::fprintf(stderr, "  issue %u: %s: %s\n", static_cast<unsigned>(issue.code), issue.subject,
                issue.message);
        }
    }
    return evaluated;
}

// ---- the design document's assembly test ------------------------------------------

bool CharacterScreen::runSelfTest()
{
    if (!created_ || character_ == nullptr || characterHandle_ == engine::character::invalidCharacter)
    {
        std::printf("character-test: screen not ready\n");
        return false;
    }

    std::uint32_t passed  = 0;
    std::uint32_t failed  = 0;
    std::uint32_t skipped = 0;
    const auto    check   = [&](const char *name, bool ok)
    {
        if (ok)
        {
            ++passed;
            std::printf("  PASS  %s\n", name);
        }
        else
        {
            ++failed;
            std::printf("  FAIL  %s\n", name);
        }
    };

    std::printf("character-test: downloaded-model body-plan assembly test\n");

    // 1. The default humanoid profile assembles and validates.
    resetToDefault();
    check("humanoid profile validates", activePlan_ == 0 && evaluateCharacter(true) && issues_.empty());

    // 2. Every head variant swaps on the live assembly, and every species
    //    head auto-wears its matching ear pair.
    CharacterError error = CharacterError::None;
    bool headsOk = true;
    for (int species = 0; species < metrics::kSpeciesCount; ++species)
    {
        const bool socketed = metrics::kSpecies[species].hasEarSockets;
        activeEarPart_.clear(); // force the auto-match on the next swap
        headsOk = setHeadPart(std::string("head_") + metrics::kSpecies[species].id) && headsOk;
        if (socketed)
        {
            headsOk = activeEarPart_ == std::string("ear_") + metrics::kSpecies[species].id + "_left" && headsOk;
            headsOk = instanceByName("ear_left") != engine::character::invalidInstance && headsOk;
        }
        else
        {
            // ears (and horns) belong to that species' head mesh
            headsOk = instanceByName("ear_left") == engine::character::invalidInstance && headsOk;
        }
    }
    for (const char *headId : {"head_human", "head_dragon"})
    {
        headsOk = setHeadPart(headId) && headsOk;
    }
    check("head variants swap with ears", headsOk);
    setHeadPart("head_human");

    // 3. Ear styles cross-graft: any species ears on the fox head.
    {
        bool earsOk = setHeadPart("head_fox");
        earsOk = setEarParts("ear_wolf_left") && earsOk;
        earsOk = setEarParts("ear_deer_left") && earsOk;
        check("ear styles swap on species heads", earsOk && instanceByName("ear_left") != engine::character::invalidInstance);
        setHeadPart("head_human");
    }

    // 4. The tail variants attach at the humanoid tailbone and swap live.
    check("fox tail attaches", setTail(true) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("tail") != engine::character::invalidInstance);
    bool tailsOk = true;
    for (int species = 0; species < metrics::kSpeciesCount; ++species)
    {
        tailsOk = setTailPart(std::string("tail_") + metrics::kSpecies[species].id) && tailsOk;
    }
    tailsOk = setTailPart("tail_reptile") && tailsOk;
    check("tail variants swap", tailsOk && evaluateCharacter(true) && issues_.empty());
    setTailPart("tail_fox");
    check("tail removes", setTail(false) && instanceByName("tail") == engine::character::invalidInstance);

    // 5. The wing pair attaches and removes (exactly one pair).
    check("wing pair attaches", setWings(true) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("wing_left") != engine::character::invalidInstance);
    check("wing pair removes", setWings(false)
        && instanceByName("wing_left") == engine::character::invalidInstance);

    // 6. The outfit toggles: bare variants + garment overlays.
    check("outfit swaps to bare", setOutfit(false) && evaluateCharacter(true) && issues_.empty()
        && partIdOfInstance("torso") == "torso_bare");
    check("outfit swaps back to ranger", setOutfit(true) && evaluateCharacter(true) && issues_.empty()
        && partIdOfInstance("torso") == "torso_ranger"
        && partIdOfInstance("belt") == "clothes_belt");
    check("belt removes", setClothingPiece("belt", false) && instanceByName("belt") == engine::character::invalidInstance);
    setClothingPiece("belt", true);
    check("hood toggles", setClothingPiece("hood", true) && instanceByName("hood") != engine::character::invalidInstance
        && setClothingPiece("hood", false) && instanceByName("hood") == engine::character::invalidInstance);

    // 7. The taur plan rebuilds (lower body + chest + arms) and EVERY
    //    species grafts the chest at its own measured stump; the tail
    //    follows the lower body (the species tail by default, swappable).
    check("taur plan builds", setPlan(1) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("body") != engine::character::invalidInstance
        && instanceByName("chest") != engine::character::invalidInstance
        && instanceByName("leg_left") == engine::character::invalidInstance);
    bool taurSpeciesOk = true;
    for (int species = 0; species < metrics::kSpeciesCount; ++species)
    {
        taurSpeciesOk = setQuadSpecies(species) && taurSpeciesOk && evaluateCharacter(true)
            && issues_.empty();
        // the tail follows the species swap on the taur plan
        taurSpeciesOk = activeTailPart_ == std::string("tail_") + metrics::kSpecies[species].id
            && taurSpeciesOk;
    }
    check("taur species graft per species", taurSpeciesOk);
    // the tail option still affects the taur lower body
    check("taur tail attaches at the lower-body socket", setTail(true)
        && instanceByName("tail") != engine::character::invalidInstance);
    check("taur tail swaps cross-species",
        setTailPart("tail_fox") && partIdOfInstance("tail") == "tail_fox"
        && setTailPart("tail_wolf") && partIdOfInstance("tail") == "tail_wolf");
    check("taur tail removes", setTail(false) && instanceByName("tail") == engine::character::invalidInstance
        && setTail(true) && instanceByName("tail") != engine::character::invalidInstance);

    // 8. The quadruped plan rebuilds as the WHOLE animal (own head) and
    //    swaps species live through all eleven downloaded animals.
    check("quadruped plan builds", setPlan(2) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("body") != engine::character::invalidInstance);
    bool quadrupedSpeciesOk = true;
    for (int species = 0; species < metrics::kSpeciesCount; ++species)
    {
        quadrupedSpeciesOk = setQuadSpecies(species) && quadrupedSpeciesOk && evaluateCharacter(true)
            && issues_.empty();
    }
    check("quadruped species swap", quadrupedSpeciesOk);

    // 9. Parameter range ends keep every plan valid (extremes re-validated),
    //    including the lower-body scale on the taur.
    {
        setPlan(1);
        CharacterError rangeError = CharacterError::None;
        character_->setParameter(character_->self, characterHandle_, "body.height", 1.35f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "body.width", 1.45f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "body.muscle_mass", 1.55f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "head.size", 1.35f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "taur.scale", 1.40f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "muzzle.length", 1.35f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "eyes.width", 1.20f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "fur.markings", 3.0f, &rangeError);
        refreshDeformations();
        applyParameterHeights();
        const bool extremesOk = evaluateCharacter(true) && issues_.empty();
        if (!issues_.empty())
        {
            for (const ValidationIssue &issue : issues_)
            {
                std::printf("  issue %u: %s: %s\n", static_cast<unsigned>(issue.code), issue.subject, issue.message);
            }
        }
        check("parameter extremes validate", extremesOk);
        resetToDefault();
    }

    // 10. Recipes round-trip on the new structure.
    {
        setWings(true);
        setTail(true);
        const bool saved = character_->saveRecipe(character_->self, characterHandle_, kRecipeFileName, &error);
        bool loaded = false;
        if (saved)
        {
            loaded = character_->loadRecipe(character_->self, characterHandle_, kRecipeFileName, &error);
        }
        check("recipe save + load", saved && loaded);
        setWings(false);
        setTail(false);
    }

    // 11. Animation drivers run without changing the structure.
    updateAnimationDrivers(1.0 / 60.0, 1.0);
    evaluateCharacter(true);
    check("animation drivers keep assembly valid", issues_.empty());

    std::printf("character-test: %u passed / %u failed / %u skipped\n", passed, failed, skipped);
    resetToDefault();
    return failed == 0;
}

} // namespace demo
