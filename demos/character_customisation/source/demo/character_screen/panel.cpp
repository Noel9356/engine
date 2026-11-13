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
// a "##group" suffix so the ImGui id is unique per group (the rev-9 panel
// reused bare "1"/"2"/"3" labels across the Heads/Wings/Arms sections, so
// several buttons shared one id and fired together). The active option is
// also text-marked; because the marker sits BEFORE the "##", the id stays
// stable whether or not the item is selected.
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

    static const char *const kPlanNames[4] = {"Humanoid Biped", "Centaur", "Naga", "Quadruped"};
    static const char *const kTabs[5] = {"Plan", "Body", "Parts", "Motion", "File"};

    // ---- header ---------------------------------------------------------------
    uiTextColored(0.60f, 0.75f, 0.98f, kPlanNames[std::clamp(activePlan_, 0, 3)]);
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
    // Tabs are radio buttons bound to activePanelTab_ with "##tab" id
    // suffixes; the row wraps after every tab so it never overflows.
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
    // One radio button per plan; stable ids via the "##plan" suffix.
    static const char *const kPlanChoices[4] = {"Humanoid", "Centaur", "Naga", "Quadruped"};
    std::int32_t             plan = activePlan_;
    for (int index = 0; index < 4; ++index)
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

    // Species picker for the centaur and quadruped plans (a combo - 11
    // species overflow a button grid).
    if (activePlan_ == 1 || activePlan_ == 3)
    {
        uiHeading("Species");
        uiText(activePlan_ == 1 ? "The hindquarters + graft band below the torso:"
                                : "The whole animal (own head, own tail):");
        char speciesLabel[64];
        std::snprintf(speciesLabel, sizeof(speciesLabel), "%s##species", kQuadSpeciesNames[std::clamp(quadSpecies_,
            0, kQuadSpeciesCount - 1)]);
        const std::int32_t before = quadSpecies_;
        std::int32_t       species = before;
        if (ui_->combo(ui_->self, uiContext_, speciesLabel, &species, kQuadSpeciesNames,
                static_cast<std::uint32_t>(kQuadSpeciesCount))
            && species != before)
        {
            if (setQuadSpecies(static_cast<int>(species)))
            {
                setStatusMessage(std::string("Species: ") + kQuadSpeciesNames[species]);
            }
            else
            {
                setStatusMessage("Species swap rejected");
            }
        }
    }
    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(0.62f, 0.66f, 0.72f,
        "Tip: every plan keeps head, wings and tail options where the anatomy allows them.");
}

// ---- tab: body parameters --------------------------------------------------------

void CharacterScreen::drawBodyTab()
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
    if (uiParameterSlider("limbs.length"))
    {
        applyParameterHeights();
    }
    uiParameterSlider("limbs.thickness");

    uiHeading("Head");
    uiParameterSlider("head.size");

    // Skin tone palette (industry pattern: swatches, not a raw color picker).
    // The value is the tone index; drawCharacter maps it to a material tint
    // on the skin parts (torso, human limbs, human head).
    uiHeading("Skin Tone");
    {
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

    // Fur coat palette for the downloaded animal grafts (species heads and
    // tails). The quadruped bodies keep the source pack's own coat colors -
    // the tint applies to the graft pieces only.
    uiHeading("Fur Coat (species grafts)");
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
        });
    }
}

// ---- tab: parts ---------------------------------------------------------------------

void CharacterScreen::drawPartsTab()
{
    // Head variants: every registered slot:head part, listed as selectable
    // rows (the dragon head reads oversized no more - it is rescaled to the
    // human graft band this revision).
    uiHeading("Head");
    for (const std::string &partId : content_.partsInSlot("head"))
    {
        const PartRenderInfo *info = content_.renderInfoForPart(partId);
        if (info == nullptr)
        {
            continue;
        }
        char label[96];
        std::snprintf(label, sizeof(label), "%s##head", info->displayName.c_str());
        bool selected = partId == activeHeadPart_;
        if (ui_->selectable(ui_->self, uiContext_, label, &selected, 0.0f, 0.0f))
        {
            CharacterError error = CharacterError::None;
            bool swapped = true;
            for (const char *headName : {"head", "head2", "head3"})
            {
                const InstanceId head = instanceByName(headName);
                if (head != engine::character::invalidInstance)
                {
                    swapped = character_->replacePart(character_->self, characterHandle_, head, partId.c_str(), &error)
                        && swapped;
                }
            }
            if (swapped)
            {
                activeHeadPart_ = partId;
                setStatusMessage(std::string("Head set to ") + info->displayName);
                evaluateCharacter(true);
            }
            else
            {
                setStatusMessage("Head replacement rejected");
            }
        }
    }

    // Head count.
    uiText("Heads");
    static const char *const headCounts[3] = {"1", "2", "3"};
    uiOptionRow(headCounts, 3, "heads", headCount_ - 1, 40.0f, [this](int index)
    {
        setStatusMessage(setHeadCount(index + 1) ? "Head count updated" : "Head attach failed");
    });

    // Tail variants + toggle (the naga carries the serpent body instead, the
    // quadruped its own tail).
    if (activePlan_ != 2 && activePlan_ != 3)
    {
        uiHeading("Tail");
        bool tail = tailPresent_;
        if (uiCheckbox("Attach tail##tailtoggle", &tail))
        {
            setStatusMessage(setTail(tail) ? (tail ? "Tail attached" : "Tail removed") : "Tail op failed");
        }
        if (tailPresent_)
        {
            // Combo over every registered slot:tail part (species tails +
            // the spiked dragon tail).
            const std::vector<std::string> &tailParts = content_.partsInSlot("tail");
            static std::vector<const char *> tailItems;
            static std::vector<std::string> tailItemStorage;
            tailItems.clear();
            tailItemStorage.clear();
            int currentIndex = 0;
            for (std::size_t index = 0; index < tailParts.size(); ++index)
            {
                const PartRenderInfo *info = content_.renderInfoForPart(tailParts[index]);
                if (info == nullptr)
                {
                    continue;
                }
                tailItemStorage.push_back(info->displayName);
                tailItems.push_back(tailItemStorage.back().c_str());
                if (tailParts[index] == activeTailPart_)
                {
                    currentIndex = static_cast<int>(tailItems.size()) - 1;
                }
            }
            if (!tailItems.empty())
            {
                char comboLabel[64];
                std::snprintf(comboLabel, sizeof(comboLabel), "%s##tailvariant",
                    tailItems[std::clamp(currentIndex, 0, static_cast<int>(tailItems.size()) - 1)]);
                std::int32_t picked = currentIndex;
                if (ui_->combo(ui_->self, uiContext_, comboLabel, &picked, tailItems.data(),
                        static_cast<std::uint32_t>(tailItems.size()))
                    && picked != currentIndex)
                {
                    const std::string &pickedId = tailParts[static_cast<std::size_t>(picked)];
                    setStatusMessage(setTailPart(pickedId) ? std::string("Tail: ") + tailItems[picked]
                                                           : "Tail swap rejected");
                }
            }
            uiParameterSlider("tail.length");
            uiParameterSlider("tail.thickness");
        }
    }

    // Wing pairs (0..3): stacked down the back on dedicated connectors.
    uiHeading("Wings");
    static const char *const wingCounts[4] = {"0", "1", "2", "3"};
    uiOptionRow(wingCounts, 4, "wings", wingPairs_, 40.0f, [this](int index)
    {
        setStatusMessage(setWingPairs(index) ? "Wing pairs updated" : "Wing attach failed");
    });
    uiParameterSlider("wings.span");

    // Arm pairs (1..3; quadruped keeps its own four limbs).
    uiHeading("Arm Pairs");
    const int armMax = activePlan_ == 3 ? 1 : 3;
    static const char *const armCounts[3] = {"1", "2", "3"};
    uiOptionRow(armCounts, armMax, "arms", armPairs_ - 1, 40.0f, [this](int index)
    {
        setStatusMessage(setArmPairs(index + 1) ? "Arm pairs updated" : "Arm attach failed");
    });
    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(0.62f, 0.66f, 0.72f,
        "Tip: stacked wing pairs roll outward so three pairs never merge into one wall.");
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
            // subject (96) + message (192) + code prefix must always fit.
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
        ui_->self, uiContext_, 0.80f, 0.85f, 0.95f, 0.75f, "drag: orbit   space/ctrl: zoom   arrows: orbit");
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
