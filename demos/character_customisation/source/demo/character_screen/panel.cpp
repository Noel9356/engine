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

void CharacterScreen::uiText(const char *text) const
{
    ui_->text(ui_->self, uiContext_, text);
}

void CharacterScreen::uiTextColored(float r, float g, float b, const char *text) const
{
    ui_->textColored(ui_->self, uiContext_, r, g, b, 1.0f, text);
}

void CharacterScreen::uiHeading(const char *text) const
{
    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(0.98f, 0.83f, 0.42f, text);
    ui_->separator(ui_->self, uiContext_);
}

bool CharacterScreen::uiButton(const char *label, float width) const
{
    return ui_->button(ui_->self, uiContext_, label, width, 0.0f);
}

bool CharacterScreen::uiSlider(const char *label, float *value, float minimum, float maximum) const
{
    return ui_->sliderFloat(ui_->self, uiContext_, label, value, minimum, maximum, "%.2f");
}

bool CharacterScreen::uiCheckbox(const char *label, bool *value) const
{
    return ui_->checkbox(ui_->self, uiContext_, label, value);
}

void CharacterScreen::setStatusMessage(std::string message)
{
    statusMessage_ = std::move(message);
    statusSeconds_ = 5.0f;
}

// ---- panel helpers ------------------------------------------------------------

// Draws one row of radio-style option buttons. Every button's label carries
// a "##group" suffix so the ImGui id is unique per group; selection marks
// sit before the "##" so ids stay stable.
void CharacterScreen::uiOptionRow(const char *const *options, int optionCount, const char *idSuffix,
    int activeIndex, float width, const std::function<void(int)> &onPick)
{
    for (int index = 0; index < optionCount; ++index)
    {
        char label[96];
        std::snprintf(label, sizeof(label), "%s%s##%s", activeIndex == index ? "* " : "", options[index], idSuffix);
        if (uiButton(label, width))
        {
            onPick(index);
        }
        if (index == optionCount - 1 || (index + 1) % 4 == 0)
        {
            ui_->spacing(ui_->self, uiContext_);
        }
        else
        {
            ui_->sameLine(ui_->self, uiContext_);
        }
    }
}

// One selectable part row with its preview thumbnail on the left (the image
// comes from the engine's own --part-previews render of that part). Parts
// without a thumbnail fall back to a plain selectable row.
bool CharacterScreen::uiPartSelector(const std::vector<std::string> &partIds, const std::string &activePartId,
    const std::function<void(const std::string &)> &onPick)
{
    bool picked = false;
    for (const std::string &partId : partIds)
    {
        const PartRenderInfo *info = content_.renderInfoForPart(partId);
        if (info == nullptr)
        {
            continue;
        }
        const engine::renderer::TextureHandle preview = content_.previewTextureForPart(partId);
        bool selected = partId == activePartId;
        char label[96];
        std::snprintf(label, sizeof(label), "%s##%s", info->displayName.c_str(), partId.c_str());
        const bool hasPreview = preview != engine::renderer::invalidTexture;
        if (hasPreview)
        {
            ui_->image(ui_->self, uiContext_, static_cast<engine::ui::UiTextureHandle>(preview), 56.0f, 56.0f,
                0.0f, 0.0f, 1.0f, 1.0f);
            ui_->sameLine(ui_->self, uiContext_);
        }
        if (ui_->selectable(ui_->self, uiContext_, label, &selected, 0.0f, 0.0f))
        {
            onPick(partId);
            picked = true;
        }
    }
    return picked;
}

// One labelled slider row driven by the cached parameter registry. Returns
// true when the value changed (the caller then applies plan-specific fixes).
bool CharacterScreen::uiParameterSlider(const char *parameterId)
{
    const ParameterDefinition *definition = parameterDefinition(parameterId);
    if (definition == nullptr)
    {
        return false;
    }
    float value = definition->defaultV != 0.0f ? definition->defaultV : 1.0f;
    character_->getParameter(character_->self, characterHandle_, parameterId, &value);
    const float before = value;
    if (uiSlider(definition->displayName, &value, definition->minimum, definition->maximum)
        && std::fabs(value - before) > 1.0e-6f)
    {
        CharacterError error = CharacterError::None;
        character_->setParameter(character_->self, characterHandle_, parameterId, value, &error);
        return true;
    }
    return false;
}

void CharacterScreen::drawPanel(double deltaSeconds)
{
    const float delta = static_cast<float>(std::max(deltaSeconds, 1.0e-4));
    framesPerSecond_  = glm::mix(framesPerSecond_, 1.0f / delta, 0.06f);
    if (statusSeconds_ > 0.0f)
    {
        statusSeconds_ = std::max(0.0f, statusSeconds_ - delta);
    }

    const UiWindowFlags panelFlags
        = static_cast<UiWindowFlags>(static_cast<std::uint32_t>(UiWindowFlags::NoCollapse)
                                     | static_cast<std::uint32_t>(UiWindowFlags::NoSavedSettings));

    if (panelFirstPlacement_)
    {
        std::uint32_t width  = 0;
        std::uint32_t height = 0;
        window_->getSize(window_->self, windowHandle_, &width, &height);
        ui_->setNextWindowPosition(ui_->self, uiContext_, static_cast<float>(width) - 384.0f, 16.0f, 0.0f, 0.0f);
        ui_->setNextWindowSize(ui_->self, uiContext_, 364.0f, std::max(static_cast<float>(height) - 32.0f, 380.0f));
        panelFirstPlacement_ = false;
    }

    if (!panelVisible_)
    {
        // Capture mode: the panel window is skipped entirely (begin/end
        // pairing included - the stats overlay runs its own window and the
        // frame is submitted by submitOverlay).
        uiWantsMouse_ = false;
        return;
    }
    if (!ui_->beginWindow(ui_->self, uiContext_, "Character Studio", nullptr, panelFlags))
    {
        ui_->endWindow(ui_->self, uiContext_);
        return;
    }

    static const char *const kPlanNames[3] = {"Humanoid Biped", "Taur", "Quadruped"};
    static const char *const kTabs[5] = {"Plan", "Body", "Parts", "Motion", "File"};

    // ---- header ---------------------------------------------------------------
    uiTextColored(0.60f, 0.75f, 0.98f, kPlanNames[std::clamp(activePlan_, 0, 2)]);
    ui_->sameLine(ui_->self, uiContext_);
    if (issues_.empty())
    {
        uiTextColored(0.55f, 0.92f, 0.55f, "assembly valid");
    }
    else
    {
        char badge[64];
        std::snprintf(badge, sizeof(badge), "validation: %u issue(s)", static_cast<unsigned>(issues_.size()));
        uiTextColored(0.98f, 0.45f, 0.40f, badge);
    }
    ui_->separator(ui_->self, uiContext_);

    // ---- tab row ---------------------------------------------------------------
    std::int32_t tab = activePanelTab_;
    for (int index = 0; index < 5; ++index)
    {
        char label[32];
        std::snprintf(label, sizeof(label), "%s##tab%d", kTabs[index], index);
        ui_->radioButton(ui_->self, uiContext_, label, &tab, index);
        if (index < 4)
        {
            ui_->sameLine(ui_->self, uiContext_);
        }
    }
    if (tab != activePanelTab_)
    {
        activePanelTab_ = static_cast<int>(tab);
    }
    ui_->separator(ui_->self, uiContext_);

    switch (activePanelTab_)
    {
    case 0:
        drawPlanTab();
        break;
    case 1:
        drawBodyTab();
        break;
    case 2:
        drawPartsTab();
        break;
    case 3:
        drawMotionTab();
        break;
    default:
        drawFileTab();
        break;
    }

    ui_->endWindow(ui_->self, uiContext_);

    // Remember whether the UI wants the mouse next frame (gates orbit drag).
    UiCaptureState capture{};
    if (ui_->captureState(ui_->self, uiContext_, &capture))
    {
        uiWantsMouse_ = capture.mouse;
    }
}

// ---- tab: plan + species -------------------------------------------------------

void CharacterScreen::drawPlanTab()
{
    uiHeading("Body Plan");
    // One radio button per plan; stable ids via the "##plan" suffix. The
    // taur plan generalises the centaur: a humanoid upper body on the lower
    // body of any of the species below.
    static const char *const kPlanChoices[3] = {"Humanoid", "Taur", "Quadruped"};
    std::int32_t             plan = activePlan_;
    for (int index = 0; index < 3; ++index)
    {
        char label[48];
        std::snprintf(label, sizeof(label), "%s##plan%d", kPlanChoices[index], index);
        ui_->radioButton(ui_->self, uiContext_, label, &plan, index);
        if (index % 2 == 1)
        {
            ui_->spacing(ui_->self, uiContext_);
        }
        else
        {
            ui_->sameLine(ui_->self, uiContext_);
        }
    }
    if (plan != activePlan_)
    {
        const int requested = static_cast<int>(plan);
        if (setPlan(requested))
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "body.plan",
                static_cast<float>(requested), &error);
            setStatusMessage(std::string("Body plan: ") + kPlanChoices[requested]);
        }
        else
        {
            setStatusMessage("Body plan rebuild failed");
        }
    }

    // Species picker for the taur and quadruped plans (a combo - the
    // species list overflows a button grid).
    if (activePlan_ == 1 || activePlan_ == 2)
    {
        uiHeading("Species");
        uiText(activePlan_ == 1 ? "The lower body (taur hindquarters):"
                                : "The whole animal (own head, own tail):");
        const char *speciesNames[metrics::kSpeciesCount];
        for (int index = 0; index < metrics::kSpeciesCount; ++index)
        {
            speciesNames[index] = metrics::kSpecies[index].display;
        }
        char speciesLabel[64];
        std::snprintf(speciesLabel, sizeof(speciesLabel), "%s##species",
            speciesNames[std::clamp(quadSpecies_, 0, metrics::kSpeciesCount - 1)]);
        const std::int32_t before = quadSpecies_;
        std::int32_t       species = before;
        if (ui_->combo(ui_->self, uiContext_, speciesLabel, &species, speciesNames,
                static_cast<std::uint32_t>(metrics::kSpeciesCount))
            && species != before)
        {
            if (setQuadSpecies(static_cast<int>(species)))
            {
                setStatusMessage(std::string("Species: ") + speciesNames[species]);
            }
            else
            {
                setStatusMessage("Species swap rejected");
            }
        }
        if (activePlan_ == 1)
        {
            if (uiParameterSlider("taur.scale"))
            {
                applyParameterHeights();
            }
        }
    }
    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(0.62f, 0.66f, 0.72f,
        "Tip: every plan keeps head, ear, tail, wing and outfit options.");
}

// ---- tab: body parameters --------------------------------------------------------

void CharacterScreen::drawBodyTab()
{
    // Every slider shown here acts on the active plan; the quadruped plan's
    // body is one whole-animal mesh, so the humanoid torso and head sliders
    // would have nothing to act on and are hidden there.
    if (activePlan_ != 2)
    {
        uiHeading("Body");
        if (uiParameterSlider("body.height"))
        {
            applyParameterHeights();
        }
        if (uiParameterSlider("body.width"))
        {
        }
        if (uiParameterSlider("body.muscle_mass"))
        {
        }
        if (activePlan_ == 0)
        {
            // Limb sliders only do work on the humanoid plan (the taur has
            // no humanoid legs).
            if (uiParameterSlider("limbs.length"))
            {
                applyParameterHeights();
            }
            uiParameterSlider("limbs.thickness");
        }
        uiHeading("Head");
        uiParameterSlider("head.size");
        // Muzzle and eye spacing are the unified head settings: they deform
        // every species head (the same base head with different settings is
        // what separates, say, a cat look from a wolf look on top of the
        // ear style).
        const bool headCarriesMorphs = activeHeadPart_ != "head_human" && activeHeadPart_ != "head_dragon";
        if (headCarriesMorphs)
        {
            if (uiParameterSlider("muzzle.length"))
            {
                refreshDeformations();
            }
            if (uiParameterSlider("eyes.width"))
            {
                refreshDeformations();
            }
        }
    }

    uiHeading("Size");
    if (activePlan_ == 1 || activePlan_ == 2)
    {
        // The lower-body scale drives the animal mesh and every anchor
        // riding it (the taur chest mount, the ground line).
        if (uiParameterSlider("taur.scale"))
        {
            applyParameterHeights();
        }
    }

    // Skin tone palette (industry pattern: swatches, not a raw color picker).
    // The value is the tone index; drawCharacter maps it to a material tint
    // on the skin parts (bare torso, bare limbs, human head). The quadruped
    // plan has no humanoid skin on show, so the palette hides there.
    if (activePlan_ != 2)
    {
        uiHeading("Skin Tone");
        static const char *const toneNames[5] = {"Default", "Fair", "Tan", "Bronze", "Umber"};
        float       tone        = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "skin.tone", &tone);
        const int currentTone = static_cast<int>(tone + 0.5f);
        uiOptionRow(toneNames, 5, "skin", currentTone, 62.0f, [this](int candidate)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(
                character_->self, characterHandle_, "skin.tone", static_cast<float>(candidate), &error);
        });
    }

    // Fur coat palette for the downloaded animal grafts (species heads,
    // ears, tails, and the animal bodies).
    uiHeading("Fur Coat");
    {
        static const char *const coatNames[6] = {"Natural", "Rust", "Snow", "Charcoal", "Cream", "Silver"};
        float       coat       = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "fur.coat", &coat);
        const int currentCoat = static_cast<int>(coat + 0.5f);
        uiOptionRow(coatNames, 6, "fur", currentCoat, 52.0f, [this](int candidate)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(
                character_->self, characterHandle_, "fur.coat", static_cast<float>(candidate), &error);
            refreshDeformations();
        });
    }

    // Fur markings: the fandom's socks/gloves, tail-tip, and muzzle
    // patterns, re-derived on the affected meshes at change time.
    uiHeading("Markings");
    {
        static const char *const markingNames[4] = {"None", "Socks", "Tipped", "Full"};
        float       markings = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "fur.markings", &markings);
        const int currentMarking = static_cast<int>(markings + 0.5f);
        uiOptionRow(markingNames, 4, "mark", currentMarking, 62.0f, [this](int candidate)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(
                character_->self, characterHandle_, "fur.markings", static_cast<float>(candidate), &error);
            refreshDeformations();
        });
    }
}

// ---- tab: parts ---------------------------------------------------------------------

void CharacterScreen::drawPartsTab()
{
    // Head styles with preview thumbnails: every registered slot:head part.
    uiHeading("Head");
    uiPartSelector(content_.partsInSlot("head"), activeHeadPart_, [this](const std::string &partId)
    {
        if (setHeadPart(partId))
        {
            const PartRenderInfo *info = content_.renderInfoForPart(partId);
            setStatusMessage(std::string("Head set to ") + (info ? info->displayName : partId));
        }
        else
        {
            setStatusMessage("Head replacement rejected");
        }
    });

    // Ear styles: separate parts of the head (swap the pair on the head's
    // ear sockets). Only species heads carry ear sockets.
    if (headHasEarSockets())
    {
        uiHeading("Ears");
        std::vector<std::string> earParts = content_.partsInSlot("ears");
        // deduplicate the mirrored halves (left ids only)
        std::vector<std::string> earStyles;
        for (const std::string &partId : earParts)
        {
            if (partId.size() > 5 && partId.substr(partId.size() - 5) == "_left")
            {
                earStyles.push_back(partId);
            }
        }
        uiPartSelector(earStyles, activeEarPart_, [this](const std::string &partId)
        {
            if (setEarParts(partId))
            {
                const PartRenderInfo *info = content_.renderInfoForPart(partId);
                setStatusMessage(std::string("Ears set to ") + (info ? info->displayName : partId));
            }
            else
            {
                setStatusMessage("Ear swap rejected");
            }
        });
    }

    // Tail: toggle + style thumbnails. On the humanoid plan the tail rides
    // the torso tailbone; on the taur plan the lower body's own tail socket
    // (so the tail option affects the lower body); the quadruped plan keeps
    // its own tail.
    if (activePlan_ != 2)
    {
        uiHeading("Tail");
        bool tail = tailPresent_;
        if (uiCheckbox("Attach tail##tailtoggle", &tail))
        {
            setStatusMessage(setTail(tail) ? (tail ? "Tail attached" : "Tail removed") : "Tail op failed");
        }
        if (tailPresent_)
        {
            uiPartSelector(content_.partsInSlot("tail"), activeTailPart_, [this](const std::string &partId)
            {
                const PartRenderInfo *info = content_.renderInfoForPart(partId);
                setStatusMessage(setTailPart(partId) ? std::string("Tail: ") + (info ? info->displayName : partId)
                                                    : "Tail swap rejected");
            });
            uiParameterSlider("tail.length");
            uiParameterSlider("tail.thickness");
        }
    }

    // Wings: exactly one pair (toggle).
    uiHeading("Wings");
    if (activePlan_ == 2)
    {
        uiTextColored(0.62f, 0.66f, 0.72f, "The quadruped plan has no wing sockets.");
    }
    else
    {
        bool wings = wingsPresent_;
        if (uiCheckbox("Attach wing pair##wingtoggle", &wings))
        {
            setStatusMessage(setWings(wings) ? (wings ? "Wing pair attached" : "Wings removed")
                                             : "Wing attach failed");
        }
        if (wingsPresent_)
        {
            uiParameterSlider("wings.span");
        }
    }

    // Outfit: the master toggle swaps the bare and ranger variants of the
    // torso, arms, and legs; the pieces below add or remove overlays.
    uiHeading("Outfit");
    bool clothed = clothed_;
    if (uiCheckbox("Wear ranger outfit##outfittoggle", &clothed))
    {
        setStatusMessage(setOutfit(clothed) ? (clothed ? "Outfit on" : "Outfit off") : "Outfit swap failed");
    }
    if (clothed_)
    {
        bool belt = beltPresent_;
        if (uiCheckbox("Belt##belttoggle", &belt))
        {
            beltPresent_ = belt;
            setStatusMessage(setClothingPiece("belt", belt) ? (belt ? "Belt on" : "Belt removed")
                                                            : "Belt toggle failed");
        }
        bool bracers = bracersPresent_;
        if (uiCheckbox("Bracers##bracertoggle", &bracers))
        {
            bracersPresent_ = bracers;
            const bool okLeft  = setClothingPiece("bracer_left", bracers);
            const bool okRight = setClothingPiece("bracer_right", bracers);
            setStatusMessage(okLeft && okRight ? (bracers ? "Bracers on" : "Bracers removed")
                                               : "Bracer toggle failed");
        }
        bool pauldrons = pauldronsPresent_;
        if (uiCheckbox("Pauldrons##pauldrontoggle", &pauldrons))
        {
            pauldronsPresent_ = pauldrons;
            const bool okLeft  = setClothingPiece("pauldron_left", pauldrons);
            const bool okRight = setClothingPiece("pauldron_right", pauldrons);
            setStatusMessage(okLeft && okRight ? (pauldrons ? "Pauldrons on" : "Pauldrons removed")
                                               : "Pauldron toggle failed");
        }
    }
    bool hood = hoodPresent_;
    if (uiCheckbox("Hood##hoodtoggle", &hood))
    {
        hoodPresent_ = hood;
        setStatusMessage(setClothingPiece("hood", hood) ? (hood ? "Hood on" : "Hood removed")
                                                        : "Hood toggle failed");
    }

    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(0.62f, 0.66f, 0.72f,
        "Tip: hold the middle mouse button and drag (or use the wheel) to scroll this panel.");
}

// ---- tab: motion -----------------------------------------------------------------

void CharacterScreen::drawMotionTab()
{
    uiHeading("Motion");
    bool tailAnimation = animateTail_;
    if (uiCheckbox("Animate tail##animtail", &tailAnimation))
    {
        animateTail_ = tailAnimation;
    }
    {
        float sway = tailSwaySpeed_;
        if (uiSlider("Tail sway speed", &sway, 0.0f, 2.0f))
        {
            tailSwaySpeed_ = sway;
        }
    }
    bool wingAnimation = animateWings_;
    if (uiCheckbox("Flap wings##animwings", &wingAnimation))
    {
        animateWings_ = wingAnimation;
    }
    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(0.62f, 0.66f, 0.72f,
        "The drivers edit connection offsets each frame; the assembly re-evaluates on top of them.");
}

// ---- tab: presets, status, validation ----------------------------------------------

void CharacterScreen::drawFileTab()
{
    uiHeading("Recipes");
    if (uiButton("Save Recipe##recipe-save", 104.0f))
    {
        CharacterError error = CharacterError::None;
        if (character_->saveRecipe(character_->self, characterHandle_, kRecipeFileName, &error))
        {
            setStatusMessage(std::string("Recipe saved to ") + kRecipeFileName);
        }
        else
        {
            setStatusMessage(std::string("Save failed: ") + character_->lastErrorMessage(character_->self));
        }
    }
    ui_->sameLine(ui_->self, uiContext_);
    if (uiButton("Load Recipe##recipe-load", 104.0f))
    {
        CharacterError error = CharacterError::None;
        if (character_->loadRecipe(character_->self, characterHandle_, kRecipeFileName, &error))
        {
            applyParameterHeights();
            evaluateCharacter(true);
            setStatusMessage(std::string("Recipe loaded from ") + kRecipeFileName);
        }
        else
        {
            setStatusMessage(std::string("Load failed: ") + character_->lastErrorMessage(character_->self));
        }
    }
    ui_->sameLine(ui_->self, uiContext_);
    if (uiButton("Reset##recipe-reset", 84.0f))
    {
        resetToDefault();
    }

    uiHeading("Status");
    uiTextColored(0.75f, 0.82f, 0.95f, statusMessage_.c_str());

    uiHeading("Validation");
    if (issues_.empty())
    {
        uiTextColored(0.55f, 0.92f, 0.55f, "No issues - the assembly is consistent.");
    }
    else
    {
        const std::size_t shown = std::min<std::size_t>(issues_.size(), 8);
        for (std::size_t index = 0; index < shown; ++index)
        {
            const ValidationIssue &issue = issues_[index];
            char line[320];
            std::snprintf(
                line, sizeof(line), "%u %s: %s", static_cast<unsigned>(issue.code), issue.subject, issue.message);
            uiTextColored(0.98f, 0.55f, 0.45f, line);
        }
    }
}

void CharacterScreen::drawStatsOverlay()
{
    const UiWindowFlags flags = static_cast<UiWindowFlags>(static_cast<std::uint32_t>(UiWindowFlags::NoTitleBar)
                                                           | static_cast<std::uint32_t>(UiWindowFlags::NoResize)
                                                           | static_cast<std::uint32_t>(UiWindowFlags::NoMove)
                                                           | static_cast<std::uint32_t>(UiWindowFlags::AlwaysAutoResize)
                                                           | static_cast<std::uint32_t>(UiWindowFlags::NoSavedSettings)
                                                           | static_cast<std::uint32_t>(UiWindowFlags::NoBringToFront));
    ui_->setNextWindowPosition(ui_->self, uiContext_, 12.0f, 12.0f, 0.0f, 0.0f);
    if (!ui_->beginWindow(ui_->self, uiContext_, "##character_stats", nullptr, flags))
    {
        ui_->endWindow(ui_->self, uiContext_);
        return;
    }
    engine::character::CharacterStatistics statistics{};
    char                                   line[192];
    if (character_->statistics(character_->self, characterHandle_, &statistics))
    {
        std::snprintf(line,
            sizeof(line),
            "%.0f fps  |  parts %u  joints %u  connections %u  triangles %u",
            static_cast<double>(framesPerSecond_),
            static_cast<unsigned>(statistics.partCount),
            static_cast<unsigned>(statistics.jointCount),
            static_cast<unsigned>(statistics.connectionCount),
            static_cast<unsigned>(statistics.triangleCount));
        uiText(line);
    }
    ui_->textColored(
        ui_->self, uiContext_, 0.80f, 0.85f, 0.95f, 0.75f,
        "drag: orbit   middle-drag: scroll   space/ctrl: zoom   arrows: orbit");
    ui_->endWindow(ui_->self, uiContext_);
}

void CharacterScreen::submitOverlay()
{
    UiError uiError = UiError::None;
    if (!ui_->endFrame(ui_->self, uiContext_, &uiError))
    {
        return;
    }
    const UiDrawData *drawData = ui_->drawData(ui_->self, uiContext_);
    if (drawData == nullptr || drawData->vertexCount == 0 || drawData->indexCount == 0 || drawData->commandCount == 0)
    {
        return;
    }
    try
    {
        overlayVertices_.resize(drawData->vertexCount);
        overlayIndices_.assign(drawData->indices, drawData->indices + drawData->indexCount);
        overlayCommands_.resize(drawData->commandCount);
        for (std::uint32_t index = 0; index < drawData->vertexCount; ++index)
        {
            const engine::ui::UiVertex      &source = drawData->vertices[index];
            engine::renderer::OverlayVertex &target = overlayVertices_[index];
            target.position[0]                      = source.position[0];
            target.position[1]                      = source.position[1];
            target.uv[0]                            = source.uv[0];
            target.uv[1]                            = source.uv[1];
            target.color                            = source.color;
        }
        for (std::uint32_t index = 0; index < drawData->commandCount; ++index)
        {
            const UiDrawCommand                  &source = drawData->commands[index];
            engine::renderer::OverlayDrawCommand &target = overlayCommands_[index];
            target.indexOffset                           = source.indexOffset;
            target.vertexOffset                          = source.vertexOffset;
            target.elementCount                          = source.elementCount;
            std::memcpy(target.clipRectangle, source.clipRectangle, sizeof(target.clipRectangle));
            target.texture          = source.texture <= std::numeric_limits<engine::renderer::TextureHandle>::max()
                                        ? static_cast<engine::renderer::TextureHandle>(source.texture)
                                        : engine::renderer::invalidTexture;
            target.resetRenderState = source.resetRenderState;
        }
        const engine::renderer::OverlayDrawData overlay{
            .vertices         = overlayVertices_.data(),
            .vertexCount      = static_cast<std::uint32_t>(overlayVertices_.size()),
            .indices          = overlayIndices_.data(),
            .indexCount       = static_cast<std::uint32_t>(overlayIndices_.size()),
            .commands         = overlayCommands_.data(),
            .commandCount     = static_cast<std::uint32_t>(overlayCommands_.size()),
            .displayPosition  = {drawData->displayPosition[0], drawData->displayPosition[1]},
            .displaySize      = {drawData->displaySize[0], drawData->displaySize[1]},
            .framebufferScale = {drawData->framebufferScale[0], drawData->framebufferScale[1]},
        };
        render_->drawOverlay(render_->self, rendererHandle_, &overlay);
    }
    catch (...)
    {
        overlayVertices_.clear();
        overlayIndices_.clear();
        overlayCommands_.clear();
    }
}

} // namespace demo
