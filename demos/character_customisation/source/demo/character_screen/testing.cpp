#include "../character_screen_internal.hpp"
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

    if (name == "human" || name == "dragonhead")
    {
        // Head variant on the humanoid plan.
        if (activePlan_ != 0)
        {
            applied = setPlan(0) && setParameter("body.plan", 0.0f);
        }
        activeHeadPart_ = name == "human" ? "head_human" : "head_dragon";
        setHeadCount(1);
        const InstanceId head = instanceByName("head");
        applied = applied && head != engine::character::invalidInstance
            && character_->replacePart(character_->self, characterHandle_, head, activeHeadPart_.c_str(), &error);
        setTail(name == "human");
    }
    else if (name == "centaur")
    {
        applied = setPlan(1) && setParameter("body.plan", 1.0f);
        setHeadCount(1);
    }
    else if (name == "naga")
    {
        applied = setPlan(2) && setParameter("body.plan", 2.0f);
        setHeadCount(1);
    }
    else if (name == "quadruped")
    {
        applied = setPlan(3) && setParameter("body.plan", 3.0f);
        setHeadCount(1);
    }
    else if (name == "wolf" || name == "deer" || name == "bull")
    {
        // Quadruped species variant: the WHOLE animal (own head, own tail).
        // quadSpecies_ is deliberately NOT assigned here: setQuadSpecies()
        // early-returns when the field already equals the request, so
        // pre-assigning it silently skipped the body replacement (rev-7
        // bug - every species preset rendered the horse).
        if (activePlan_ != 3)
        {
            applied = setPlan(3) && setParameter("body.plan", 3.0f);
        }
        applied = setQuadSpecies(name == "wolf" ? 1 : name == "deer" ? 2 : 3) && applied;
    }
    else if (name == "twoheaded")
    {
        // Two heads sharing the collar connector (hydra-style spread).
        applied = setHeadCount(2);
        if (activeHeadPart_ != "head_dragon")
        {
            activeHeadPart_ = "head_dragon";
            for (const char *headName : {"head", "head2", "head3"})
            {
                const InstanceId head = instanceByName(headName);
                if (head != engine::character::invalidInstance)
                {
                    character_->replacePart(character_->self, characterHandle_, head, "head_dragon", &error);
                }
            }
        }
    }
    else if (name == "threewings")
    {
        applied = setWingPairs(3);
    }
    else if (name == "fourarms")
    {
        applied = setArmPairs(3);
    }
    else if (name == "wingedtail")
    {
        applied = setWingPairs(1) && setTail(true);
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
    else if (name == "longtail")
    {
        // Tail length range end on the tailed humanoid.
        applied = setPlan(0) && setTail(true) && setParameter("tail.length", 1.50f)
               && setParameter("tail.thickness", 1.65f);
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
        // Strip every optional appendage.
        applied = setWingPairs(0) && setHeadCount(1) && setArmPairs(1) && setTail(false);
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
        return false;
    }
    std::cout << "demo: character drive preset '" << name << "' applied\n";
    applyParameterHeights();
    return evaluateCharacter(true);
}

// ---- the design document's first implementation test -----------------------------

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

    // 2. Every head variant swaps on the live assembly (downloaded meshes).
    CharacterError error = CharacterError::None;
    bool headsOk = true;
    for (const char *headId : {"head_human", "head_dragon"})
    {
        const InstanceId head = instanceByName("head");
        headsOk = headsOk && head != engine::character::invalidInstance
            && character_->replacePart(character_->self, characterHandle_, head, headId, &error);
    }
    check("head variants swap", headsOk);
    activeHeadPart_ = "head_human";
    {
        const InstanceId head = instanceByName("head");
        character_->replacePart(character_->self, characterHandle_, head, "head_human", &error);
    }

    // 3. The dragon head grafts onto the human collar (its own neck buries
    //    inside the collar overlap).
    {
        const InstanceId head = instanceByName("head");
        const bool swapped = character_->replacePart(character_->self, characterHandle_, head, "head_dragon", &error);
        activeHeadPart_ = "head_dragon";
        check("dragon head grafts on humanoid", swapped && evaluateCharacter(true) && issues_.empty());
        const InstanceId back = instanceByName("head");
        character_->replacePart(character_->self, characterHandle_, back, "head_human", &error);
        activeHeadPart_ = "head_human";
    }

    // 4. Head count 1..3 attaches and removes extra skulls on one connector.
    bool heads3Ok = setHeadCount(3) && evaluateCharacter(true) && issues_.empty();
    check("three heads attach", heads3Ok
        && instanceByName("head2") != engine::character::invalidInstance
        && instanceByName("head3") != engine::character::invalidInstance);
    check("head count back to one", setHeadCount(1)
        && instanceByName("head2") == engine::character::invalidInstance
        && instanceByName("head3") == engine::character::invalidInstance);

    // 5. Wing pairs 0..3 stack on the wither connectors.
    check("three wing pairs attach", setWingPairs(3) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("wing3_left") != engine::character::invalidInstance);
    check("wing pairs back to zero", setWingPairs(0)
        && instanceByName("wing1_left") == engine::character::invalidInstance);

    // 6. Arm pairs 1..3 along the torso.
    check("three arm pairs attach", setArmPairs(3) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("arm3_left") != engine::character::invalidInstance);
    check("arm pairs back to one", setArmPairs(1)
        && instanceByName("arm2_left") == engine::character::invalidInstance);

    // 7. The reptile tail attaches at the tailbone socket.
    check("reptile tail attaches", setTail(true) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("tail") != engine::character::invalidInstance);
    check("tail removes", setTail(false) && instanceByName("tail") == engine::character::invalidInstance);

    // 8. The centaur plan rebuilds (quadruped hindquarters + torso + arms).
    check("centaur plan builds", setPlan(1) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("body") != engine::character::invalidInstance
        && instanceByName("chest") != engine::character::invalidInstance
        && instanceByName("leg_left") == engine::character::invalidInstance);
    check("centaur species swap", setQuadSpecies(2) && evaluateCharacter(true) && issues_.empty());

    // 9. The naga plan rebuilds (torso on the serpent lower body).
    check("naga plan builds", setPlan(2) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("naga_body") != engine::character::invalidInstance
        && instanceByName("leg_left") == engine::character::invalidInstance);

    // 10. The quadruped plan rebuilds as the WHOLE animal (own head) and
    //     swaps species live.
    check("quadruped plan builds", setPlan(3) && evaluateCharacter(true) && issues_.empty()
        && instanceByName("body") != engine::character::invalidInstance);
    check("quadruped species swap", setQuadSpecies(1) && evaluateCharacter(true) && issues_.empty());
    check("wing pairs on quadruped", setWingPairs(2) && evaluateCharacter(true) && issues_.empty());
    setWingPairs(0);

    // 11. Parameter range ends keep every plan valid (extremes re-validated).
    {
        setPlan(0);
        CharacterError rangeError = CharacterError::None;
        character_->setParameter(character_->self, characterHandle_, "body.height", 1.35f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "body.width", 1.45f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "body.muscle_mass", 1.55f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "head.size", 1.35f, &rangeError);
        character_->setParameter(character_->self, characterHandle_, "limbs.length", 1.35f, &rangeError);
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

    // 12. Recipes round-trip on the new structure.
    {
        setWingPairs(1);
        setTail(true);
        const bool saved = character_->saveRecipe(character_->self, characterHandle_, kRecipeFileName, &error);
        bool loaded = false;
        if (saved)
        {
            loaded = character_->loadRecipe(character_->self, characterHandle_, kRecipeFileName, &error);
        }
        check("recipe save + load", saved && loaded);
        setWingPairs(0);
        setTail(false);
    }

    // 13. Animation drivers run without changing the structure.
    updateAnimationDrivers(1.0 / 60.0, 1.0);
    evaluateCharacter(true);
    check("animation drivers keep assembly valid", issues_.empty());

    std::printf("character-test: %u passed / %u failed / %u skipped\n", passed, failed, skipped);
    resetToDefault();
    return failed == 0;
}


} // namespace demo
