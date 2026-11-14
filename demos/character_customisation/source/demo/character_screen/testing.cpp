#include "../character_metrics.gen.hpp"
#include "../character_screen_internal.hpp"

#include <cstdio>
#include <iostream>
#include <string>

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
        // The clothed human variant: human head, ranger outfit, skin body,
        // no appendages - a plain human.
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setOutfit(true) && applied;
        applied = setHeadPart("head_human") && applied;
        applied = setParameter("body.coat", 0.0f) && setParameter("fur.coat", 0.0f)
               && setParameter("fur.markings", 0.0f) && applied;
        setTail(false);
        refreshDeformations();
    }
    else if (name == "dragonhead")
    {
        // The dragon head keeps its reptile tail (the same-pack appendage).
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setOutfit(true) && applied;
        applied = setHeadPart("head_dragon") && applied;
        applied = setParameter("body.coat", 0.0f) && setParameter("fur.coat", 0.0f)
               && setParameter("fur.markings", 0.0f) && applied;
        setTailPart("tail_reptile") && setTail(true);
        refreshDeformations();
    }
    else if (name == "furryfox")
    {
        // Furry-fandom look: fox head + fox ears + fox tail + the species'
        // own paw limbs on the furred bare body (the match-head coat paints
        // the whole body in the fox's measured palette).
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setOutfit(false) && applied;
        applied = setHeadPart("head_fox") && applied;
        applied = setLimbPart("arm_left", "anthro_arm_fox_left") && applied;
        applied = setLimbPart("arm_right", "anthro_arm_fox_right") && applied;
        applied = setLimbPart("leg_left", "anthro_leg_fox_left") && applied;
        applied = setLimbPart("leg_right", "anthro_leg_fox_right") && applied;
        applied = setTailPart("tail_fox") && applied;
        applied = setParameter("body.coat", 1.0f) && setParameter("fur.coat", 0.0f)
               && setParameter("fur.markings", 3.0f) && applied;
        setTail(true);
        refreshDeformations();
    }
    else if (name == "furrywolf")
    {
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setOutfit(false) && applied;
        applied = setHeadPart("head_wolf") && applied;
        applied = setLimbPart("arm_left", "anthro_arm_wolf_left") && applied;
        applied = setLimbPart("arm_right", "anthro_arm_wolf_right") && applied;
        applied = setLimbPart("leg_left", "anthro_leg_wolf_left") && applied;
        applied = setLimbPart("leg_right", "anthro_leg_wolf_right") && applied;
        applied = setTailPart("tail_wolf") && applied;
        applied = setParameter("body.coat", 1.0f) && setParameter("fur.coat", 0.0f)
               && setParameter("fur.markings", 4.0f) && applied;
        setTail(true);
        refreshDeformations();
    }
    else if (name == "anthrolimbs")
    {
        // The species limb variants on the biped: digitigrade legs and paw
        // arms from the chosen species (the trex pair reads loudest).
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setOutfit(false) && applied;
        applied = setLimbPart("arm_left", "anthro_arm_trex_left") && applied;
        applied = setLimbPart("arm_right", "anthro_arm_trex_right") && applied;
        applied = setLimbPart("leg_left", "anthro_leg_trex_left") && applied;
        applied = setLimbPart("leg_right", "anthro_leg_trex_right") && applied;
        applied = setHeadPart("head_trex") && applied;
        applied = setParameter("body.coat", 1.0f) && setParameter("fur.coat", 0.0f) && applied;
        refreshDeformations();
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
        // The taur read continues the current look's species: the default
        // fox head rides the fox lower body in one pelt.
        applied = setPlan(1) && setParameter("body.plan", 1.0f);
        applied = setQuadSpecies(quadSpecies_) && applied;
        applied = setOutfit(false) && applied;
        applied = setHeadPart(activeHeadPart_) && applied;
        applied = setParameter("body.coat", 1.0f) && setParameter("fur.coat", 0.0f) && applied;
        setTail(true);
        refreshDeformations();
    }
    else if (name == "quadruped")
    {
        applied = setPlan(2) && setParameter("body.plan", 2.0f);
    }
    else if (name == "rabbit")
    {
        // The rabbit ships head-only parts (no whole-animal or taur body),
        // so the preset reads as the anthro rabbit: biped base, the rabbit
        // head with its own ear pair and neck, and the match-head coat
        // painting the body in the rabbit's measured palette.
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setOutfit(false) && applied;
        applied = setHeadPart("head_rabbit") && applied;
        applied = setEarParts("ear_rabbit_left") && applied;
        applied = setParameter("body.coat", 1.0f) && setParameter("fur.coat", 0.0f) && applied;
        setTail(false);
        refreshDeformations();
    }
    else if (name == "fox" || name == "shiba" || name == "husky" || name == "wolf" || name == "donkey" || name == "cow"
             || name == "deer" || name == "bull" || name == "alpaca" || name == "horse" || name == "stag"
             || name == "trex" || name == "raptor" || name == "triceratops" || name == "stegosaurus"
             || name == "apatosaurus" || name == "parasaurolophus" || name == "rat" || name == "shark"
             || name == "dolphin" || name == "frog")
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
        // The upper body follows the species too - its head (with ears where
        // the species has ear parts) and a fur-coloured bare torso - so a
        // taur_fox reads as one fox, not a human rider.
        applied = setPlan(1) && setParameter("body.plan", 1.0f);
        setTail(true);
        int         speciesIndex = -1;
        std::string species      = name.substr(5);
        if (species == "shiba")
        {
            species = "shibainu"; // the shiba drive name aliases the registry id
        }
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
        else if (!metrics::kSpecies[speciesIndex].hasTaurBody)
        {
            // Head-only species (the rabbit) have no hindquarters in the
            // roster: the taur read falls back to the species' own anthro
            // preset instead of a chimera with whichever lower body was live.
            return applyDrivePreset(species);
        }
        else
        {
            // The match-head coat (index 0) paints the upper body in the
            // species' own measured palette, so every taur reads as one
            // animal from nose to tail.
            applied = setQuadSpecies(speciesIndex) && applied;
            applied = setOutfit(false) && applied;
            applied = setHeadPart(std::string("head_") + species) && applied;
            applied = setParameter("body.coat", 1.0f) && setParameter("fur.coat", 0.0f)
                   && setParameter("fur.markings", 3.0f) && applied;
            refreshDeformations();
        }
    }
    else if (name == "winged")
    {
        applied = setWings(true) && setTail(true);
    }
    else if (name == "cat" || name == "jackal" || name == "fennec")
    {
        // Unified-head species looks: parameter settings on the canid heads
        // (short muzzle + wide eyes reads as a cat; a long muzzle as a
        // jackal).
        if (activePlan_ == 2)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        // The look settings: the cat reads roundest on the shiba base (short
        // muzzle, wide eyes, small ears), the fennec on the fox base with
        // maxed-out ear size, the jackal on the wolf base with a long muzzle.
        const bool  canine = name == "wolf" || name == "jackal";
        const char *head   = canine ? "head_wolf" : (name == "cat" ? "head_shibainu" : "head_fox");
        const char *ears   = canine ? "ear_wolf_left" : (name == "cat" ? "ear_shibainu_left" : "ear_fox_left");
        const char *tail   = canine ? "tail_wolf" : (name == "cat" ? "tail_shibainu" : "tail_fox");
        const float muzzle = name == "cat" ? 0.72f : (name == "fennec" ? 0.85f : 1.40f);
        const float eyes   = name == "cat" ? 1.22f : (name == "fennec" ? 1.26f : 1.00f);
        if (name == "fennec")
        {
            applied = setParameter("ears.size", 1.60f) && applied;
        }
        if (name == "cat")
        {
            applied = setParameter("ears.size", 0.72f) && applied;
        }
        applied            = setHeadPart(head) && applied;
        applied            = setEarParts(ears) && applied;
        applied            = setTailPart(tail) && applied;
        setTail(true);
        applied = setParameter("muzzle.length", muzzle) && setParameter("eyes.width", eyes) && applied;
        refreshDeformations();
    }
    else if (name == "furrybody")
    {
        // The anthro body read: a fox head and tail on the furred bare torso
        // and limbs - the fur palette and markings cover the whole body.
        applied = setPlan(0) && setParameter("body.plan", 0.0f);
        applied = setOutfit(false) && applied;
        applied = setHeadPart("head_fox") && applied;
        applied = setTailPart("tail_fox") && applied;
        setTail(true);
        applied = setParameter("body.coat", 1.0f) && setParameter("fur.coat", 2.0f)
               && setParameter("fur.markings", 3.0f) && applied;
        refreshDeformations();
    }
    else if (name == "tall")
    {
        // Height range ends: every anchor rides the part scale bindings.
        applied = setParameter("body.height", 1.55f) && setParameter("limbs.length", 1.65f);
        applyParameterHeights();
    }
    else if (name == "broad")
    {
        // Width/mass range ends: shoulders, hips, and arms must stay grafted.
        applied = setParameter("body.width", 1.90f) && setParameter("body.muscle_mass", 1.90f)
               && setParameter("limbs.thickness", 2.00f);
    }
    else if (name == "longmuzzle")
    {
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        applied = setHeadPart("head_wolf") && applied;
        applied = setEarParts("ear_wolf_left") && applied;
        applied = setParameter("muzzle.length", 1.55f) && setParameter("eyes.width", 1.35f) && applied;
        applied = setParameter("ears.size", 1.60f) && applied;
        refreshDeformations();
    }
    else if (name == "marked")
    {
        applied = setParameter("fur.markings", 3.0f) && setParameter("fur.coat", 0.0f)
               && setParameter("body.coat", 1.0f);
        refreshDeformations();
    }
    else if (name == "striped")
    {
        // The tabby stripes (marking mode 4) on the match-head coat.
        applied = setParameter("fur.markings", 4.0f) && setParameter("fur.coat", 0.0f)
               && setParameter("body.coat", 1.0f);
        refreshDeformations();
    }
    else if (name == "longtail")
    {
        // Tail length range end on the tailed humanoid.
        applied = setPlan(0) && setTail(true) && setParameter("tail.length", 2.60f)
               && setParameter("tail.thickness", 2.40f);
    }
    else if (name == "bigtaur")
    {
        // Lower-body size range end on the taur plan, the lower body still
        // matching the head's species.
        applied = setPlan(1) && setParameter("body.plan", 1.0f);
        applied = setQuadSpecies(quadSpecies_) && applied;
        applied = setParameter("taur.scale", 2.40f) && applied;
        applyParameterHeights();
    }
    else if (name == "compact")
    {
        // Everything small at once: the opposite range end of every parameter.
        applied = setParameter("body.height", 0.65f) && setParameter("body.width", 0.60f)
               && setParameter("limbs.length", 0.65f) && setParameter("head.size", 0.65f)
               && setParameter("tail.length", 0.40f) && setParameter("wings.span", 0.40f);
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
            std::fprintf(
                stderr, "  issue %u: %s: %s\n", static_cast<unsigned>(issue.code), issue.subject, issue.message);
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
            std::fprintf(
                stderr, "  issue %u: %s: %s\n", static_cast<unsigned>(issue.code), issue.subject, issue.message);
        }
    }
    // Presets re-proportion the character (plans, species, extreme sliders):
    // refit the studio framing so every capture frames the whole silhouette.
    fitCameraToAssembly();
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
    CharacterError error   = CharacterError::None;
    bool           headsOk = true;
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
        // the neck follows the head's species (the weld-ring chain)
        headsOk = activeNeckPart_ == std::string("neck_") + metrics::kSpecies[species].id && headsOk;
        headsOk = instanceByName("neck") != engine::character::invalidInstance && headsOk;
        headsOk = partIdOfInstance("neck") == std::string("neck_") + metrics::kSpecies[species].id && headsOk;
    }
    for (const char *headId : {"head_human", "head_dragon"})
    {
        headsOk = setHeadPart(headId) && headsOk;
        headsOk = activeNeckPart_ == "neck_human" && headsOk;
    }
    check("head variants swap with ears + neck", headsOk);
    setHeadPart("head_human");

    // 3. Ear styles cross-graft: any species ears on the fox head.
    {
        bool earsOk = setHeadPart("head_fox");
        earsOk      = setEarParts("ear_wolf_left") && earsOk;
        earsOk      = setEarParts("ear_deer_left") && earsOk;
        check("ear styles swap on species heads",
            earsOk && instanceByName("ear_left") != engine::character::invalidInstance);
        setHeadPart("head_human");
    }

    // 4. The tail variants attach at the humanoid tailbone and swap live.
    check("fox tail attaches",
        setTail(true) && evaluateCharacter(true) && issues_.empty()
            && instanceByName("tail") != engine::character::invalidInstance);

    // 3b. The anthro limb variants: every legged species' hind leg lands on
    //     the pelvis hip socket and its foreleg on the shoulder socket (the
    //     ring sinks by the measured fraction; the extremes stay grafted).
    {
        bool limbsOk = true;
        for (int species = 0; species < metrics::kSpeciesCount; ++species)
        {
            if (!metrics::kSpecies[species].hasLimbs)
            {
                continue;
            }
            const std::string id = metrics::kSpecies[species].id;
            limbsOk = setLimbPart("leg_left", "anthro_leg_" + id + "_left") && limbsOk;
            limbsOk = setLimbPart("leg_right", "anthro_leg_" + id + "_right") && limbsOk;
            limbsOk = setLimbPart("arm_left", "anthro_arm_" + id + "_left") && limbsOk;
            limbsOk = setLimbPart("arm_right", "anthro_arm_" + id + "_right") && limbsOk;
            limbsOk = evaluateCharacter(true) && issues_.empty() && limbsOk;
        }
        check("anthro limb variants graft per species", limbsOk);
        // the human legs come back for the rest of the suite
        setLimbPart("leg_left", "leg_bare_left");
        setLimbPart("leg_right", "leg_bare_right");
        setLimbPart("arm_left", "arm_bare_left");
        setLimbPart("arm_right", "arm_bare_right");
    }

    bool tailsOk = true;
    for (int species = 0; species < metrics::kSpeciesCount; ++species)
    {
        if (!metrics::kSpecies[species].hasTail)
        {
            continue; // tail-less species (the frog, the rabbit)
        }
        tailsOk = setTailPart(std::string("tail_") + metrics::kSpecies[species].id) && tailsOk;
    }
    tailsOk = setTailPart("tail_reptile") && tailsOk;
    check("tail variants swap", tailsOk && evaluateCharacter(true) && issues_.empty());
    setTailPart("tail_fox");
    check("tail removes", setTail(false) && instanceByName("tail") == engine::character::invalidInstance);

    // 5. The wing pair attaches and removes (exactly one pair).
    check("wing pair attaches",
        setWings(true) && evaluateCharacter(true) && issues_.empty()
            && instanceByName("wing_left") != engine::character::invalidInstance);
    check("wing pair removes", setWings(false) && instanceByName("wing_left") == engine::character::invalidInstance);

    // 6. The outfit toggles: bare variants + garment overlays.
    check("outfit swaps to bare",
        setOutfit(false) && evaluateCharacter(true) && issues_.empty()
            && partIdOfInstance("chest") == "torso_thoracic_bare" && partIdOfInstance("pelvis") == "pelvis_bare");
    check("outfit swaps back to ranger",
        setOutfit(true) && evaluateCharacter(true) && issues_.empty()
            && partIdOfInstance("chest") == "torso_thoracic_ranger" && partIdOfInstance("pelvis") == "pelvis_ranger"
            && partIdOfInstance("belt") == "clothes_belt");
    check("belt removes",
        setClothingPiece("belt", false) && instanceByName("belt") == engine::character::invalidInstance);
    setClothingPiece("belt", true);
    check("hood toggles",
        setClothingPiece("hood", true) && instanceByName("hood") != engine::character::invalidInstance
            && setClothingPiece("hood", false) && instanceByName("hood") == engine::character::invalidInstance);

    // 7. The taur plan rebuilds (lower body + chest + arms) and EVERY
    //    species grafts the chest at its own measured stump; the tail
    //    follows the lower body (the species tail by default, swappable).
    check("taur plan builds",
        setPlan(1) && evaluateCharacter(true) && issues_.empty()
            && instanceByName("body") != engine::character::invalidInstance
            && instanceByName("chest") != engine::character::invalidInstance
            && instanceByName("leg_left") == engine::character::invalidInstance);
    bool taurSpeciesOk = true;
    for (int species = 0; species < metrics::kSpeciesCount; ++species)
    {
        taurSpeciesOk = setQuadSpecies(species) && taurSpeciesOk && evaluateCharacter(true) && issues_.empty();
        // the tail variant follows the species swap when the species has a
        // tail; the tail-less species (the frog) removes it instead
        if (metrics::kSpecies[species].hasTail)
        {
            taurSpeciesOk = activeTailPart_ == std::string("tail_") + metrics::kSpecies[species].id && taurSpeciesOk;
        }
        else
        {
            taurSpeciesOk = instanceByName("tail") == engine::character::invalidInstance && taurSpeciesOk;
        }
    }
    check("taur species graft per species", taurSpeciesOk);
    // the tail option still affects the taur lower body
    check("taur tail attaches at the lower-body socket",
        setTail(true) && instanceByName("tail") != engine::character::invalidInstance);
    check("taur tail swaps cross-species",
        setTailPart("tail_fox") && partIdOfInstance("tail") == "tail_fox_taur" && setTailPart("tail_wolf")
            && partIdOfInstance("tail") == "tail_wolf_taur");
    check("taur tail removes",
        setTail(false) && instanceByName("tail") == engine::character::invalidInstance && setTail(true)
            && instanceByName("tail") != engine::character::invalidInstance);

    // 8. The quadruped plan rebuilds as the WHOLE animal (own head) and
    //    swaps species live through all eleven downloaded animals.
    check("quadruped plan builds",
        setPlan(2) && evaluateCharacter(true) && issues_.empty()
            && instanceByName("body") != engine::character::invalidInstance);
    bool quadrupedSpeciesOk = true;
    for (int species = 0; species < metrics::kSpeciesCount; ++species)
    {
        quadrupedSpeciesOk
            = setQuadSpecies(species) && quadrupedSpeciesOk && evaluateCharacter(true) && issues_.empty();
    }
    check("quadruped species swap", quadrupedSpeciesOk);

    // 9. Parameter range ends keep every plan valid (extremes re-validated),
    //    including the lower-body scale on the taur.
    {
        setPlan(1);
        CharacterError rangeError = CharacterError::None;
        character_->setParameter(character_->self, characterHandle_, "body.height", 1.55f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "body.width", 1.90f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "body.muscle_mass", 1.90f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "head.size", 1.60f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "taur.scale", 2.40f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "muzzle.length", 1.55f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "eyes.width", 1.35f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "ears.size", 1.60f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "tail.length", 2.60f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "tail.thickness", 2.40f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "body.coat", 1.0f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "fur.markings", 5.0f, &rangeError);
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
        const bool saved  = character_->saveRecipe(character_->self, characterHandle_, kRecipeFileName, &error);
        bool       loaded = false;
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
