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
        ui_->setNextWindowPosition(ui_->self, uiContext_, static_cast<float>(width) - 372.0f, 20.0f, 0.0f, 0.0f);
        ui_->setNextWindowSize(ui_->self, uiContext_, 352.0f, std::max(static_cast<float>(height) - 40.0f, 380.0f));
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

    // ---- header + validation -------------------------------------------------
    static const char *const kPlanNames[4] = {"Humanoid Biped", "Centaur", "Naga", "Quadruped"};
    uiTextColored(0.60f, 0.75f, 0.98f, kPlanNames[std::clamp(activePlan_, 0, 3)]);
    ui_->sameLine(ui_->self, uiContext_);
    if (issues_.empty())
    {
        uiTextColored(0.55f, 0.92f, 0.55f, "[assembly valid]");
    }
    else
    {
        char badge[64];
        std::snprintf(badge, sizeof(badge), "[validation: %u issue(s)]", static_cast<unsigned>(issues_.size()));
        uiTextColored(0.98f, 0.45f, 0.40f, badge);
    }
    ui_->separator(ui_->self, uiContext_);

    // ---- head variants ---------------------------------------------------------
    uiHeading("Head");
    std::size_t headButtons = 0;
    for (const std::string &partId : content_.partsInSlot("head"))
    {
        const PartRenderInfo *info = content_.renderInfoForPart(partId);
        if (info == nullptr)
        {
            continue;
        }
        char label[96];
        std::snprintf(label, sizeof(label), "%s%s", partId == activeHeadPart_ ? "* " : "", info->displayName.c_str());
        if (uiButton(label, 96.0f))
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
        ++headButtons;
        if (headButtons % 3 == 0)
        {
            ui_->spacing(ui_->self, uiContext_);
        }
        else
        {
            ui_->sameLine(ui_->self, uiContext_);
        }
    }
    ui_->spacing(ui_->self, uiContext_);

    // ---- body parameters (central registry, per-asset bindings) ---------------
    uiHeading("Body Parameters");
    const char *bodySliders[] = {
        "body.height", "body.width", "body.muscle_mass", "head.size",
    };
    for (const char *parameter : bodySliders)
    {
        float  value    = 1.0f;
        char   label[96] = {};
        // Slider ranges come from the cached registry snapshot (no per-frame
        // capability scans).
        const ParameterDefinition *definition = parameterDefinition(parameter);
        if (definition == nullptr)
        {
            continue;
        }
        const float minimum = definition->minimum;
        const float maximum = definition->maximum;
        std::snprintf(label, sizeof(label), "%s", definition->displayName);
        character_->getParameter(character_->self, characterHandle_, parameter, &value);
        const float before = value;
        if (uiSlider(label, &value, minimum, maximum) && std::fabs(value - before) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, parameter, value, &error);
        }
    }
    {
        float limbLength = 1.0f;
        const ParameterDefinition *lengthDefinition = parameterDefinition("limbs.length");
        const float limbMinimum = lengthDefinition != nullptr ? lengthDefinition->minimum : 0.75f;
        const float limbMaximum = lengthDefinition != nullptr ? lengthDefinition->maximum : 1.35f;
        character_->getParameter(character_->self, characterHandle_, "limbs.length", &limbLength);
        const float before = limbLength;
        if (uiSlider("Limb Length", &limbLength, limbMinimum, limbMaximum) && std::fabs(limbLength - before) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "limbs.length", limbLength, &error);
            applyParameterHeights();
        }
        float limbThickness = 1.0f;
        const ParameterDefinition *thicknessDefinition = parameterDefinition("limbs.thickness");
        const float thicknessMinimum = thicknessDefinition != nullptr ? thicknessDefinition->minimum : 0.65f;
        const float thicknessMaximum = thicknessDefinition != nullptr ? thicknessDefinition->maximum : 1.55f;
        character_->getParameter(character_->self, characterHandle_, "limbs.thickness", &limbThickness);
        const float thicknessBefore = limbThickness;
        if (uiSlider("Limb Thickness", &limbThickness, thicknessMinimum, thicknessMaximum)
            && std::fabs(limbThickness - thicknessBefore) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "limbs.thickness", limbThickness, &error);
        }
    }
    // Skin tone palette (industry pattern: swatches, not a raw color picker).
    // The value is the tone index; drawCharacter maps it to a material tint on
    // the skin-tagged parts.
    {
        const char *toneNames[] = {"Fair", "Tan", "Bronze", "Umber", "Charcoal"};
        float       tone        = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "skin.tone", &tone);
        const int currentTone = static_cast<int>(tone + 0.5f);
        for (int candidate = 0; candidate < 5; ++candidate)
        {
            char label[32];
            std::snprintf(label, sizeof(label), "%s%s", candidate == currentTone ? "* " : "", toneNames[candidate]);
            if (uiButton(label, 58.0f))
            {
                CharacterError error = CharacterError::None;
                character_->setParameter(
                    character_->self, characterHandle_, "skin.tone", static_cast<float>(candidate), &error);
            }
            if (candidate % 3 == 2 || candidate == 4)
            {
                ui_->spacing(ui_->self, uiContext_);
            }
            else
            {
                ui_->sameLine(ui_->self, uiContext_);
            }
        }
    }

    // ---- body plan ---------------------------------------------------------------
    uiHeading("Body Plan");
    {
        const char *planNames[4] = {"Humanoid", "Centaur", "Naga", "Quadruped"};
        for (int plan = 0; plan < 4; ++plan)
        {
            char label[48];
            std::snprintf(label, sizeof(label), "%s%s", activePlan_ == plan ? "* " : "", planNames[plan]);
            if (uiButton(label, 78.0f))
            {
                if (setPlan(plan))
                {
                    CharacterError error = CharacterError::None;
                    character_->setParameter(
                        character_->self, characterHandle_, "body.plan", static_cast<float>(plan), &error);
                    setStatusMessage(std::string("Body plan: ") + planNames[plan]);
                }
                else
                {
                    setStatusMessage("Body plan rebuild failed");
                }
            }
            if (plan % 4 == 3)
            {
                ui_->spacing(ui_->self, uiContext_);
            }
            else
            {
                ui_->sameLine(ui_->self, uiContext_);
            }
        }
    }
    if (activePlan_ == 3 || activePlan_ == 1)
    {
        // Quadruped species (visible on the quadruped and centaur plans).
        const char *speciesNames[4] = {"Horse", "Wolf", "Deer", "Bull"};
        for (int species = 0; species < 4; ++species)
        {
            char label[48];
            std::snprintf(label, sizeof(label), "%s%s", quadSpecies_ == species ? "* " : "", speciesNames[species]);
            if (uiButton(label, 62.0f))
            {
                if (setQuadSpecies(species))
                {
                    setStatusMessage(std::string("Species: ") + speciesNames[species]);
                }
                else
                {
                    setStatusMessage("Species swap rejected");
                }
            }
            if (species % 4 == 3)
            {
                ui_->spacing(ui_->self, uiContext_);
            }
            else
            {
                ui_->sameLine(ui_->self, uiContext_);
            }
        }
    }

    // ---- appendage counts ---------------------------------------------------------
    uiHeading("Appendages");
    {
        // Head count (1..3): extra skulls share the collar connector.
        uiText("Heads");
        for (int count = 1; count <= 3; ++count)
        {
            char label[32];
            std::snprintf(label, sizeof(label), "%s%d", headCount_ == count ? "* " : "", count);
            if (uiButton(label, 30.0f))
            {
                setStatusMessage(setHeadCount(count) ? "Head count updated" : "Head attach failed");
            }
            if (count == 3)
            {
                ui_->spacing(ui_->self, uiContext_);
            }
            else
            {
                ui_->sameLine(ui_->self, uiContext_);
            }
        }
        // Wing pairs (0..3): stacked down the back on dedicated connectors.
        uiText("Wing Pairs");
        for (int pairs = 0; pairs <= 3; ++pairs)
        {
            char label[32];
            std::snprintf(label, sizeof(label), "%s%d", wingPairs_ == pairs ? "* " : "", pairs);
            if (uiButton(label, 30.0f))
            {
                setStatusMessage(setWingPairs(pairs) ? "Wing pairs updated" : "Wing attach failed");
            }
            if (pairs == 3)
            {
                ui_->spacing(ui_->self, uiContext_);
            }
            else
            {
                ui_->sameLine(ui_->self, uiContext_);
            }
        }
        // Arm pairs (1..3; quadruped keeps its own four limbs).
        uiText("Arm Pairs");
        const int armMax = activePlan_ == 3 ? 1 : 3;
        for (int pairs = 1; pairs <= armMax; ++pairs)
        {
            char label[32];
            std::snprintf(label, sizeof(label), "%s%d", armPairs_ == pairs ? "* " : "", pairs);
            if (uiButton(label, 30.0f))
            {
                setStatusMessage(setArmPairs(pairs) ? "Arm pairs updated" : "Arm attach failed");
            }
            if (pairs == armMax)
            {
                ui_->spacing(ui_->self, uiContext_);
            }
            else
            {
                ui_->sameLine(ui_->self, uiContext_);
            }
        }
        // Reptile tail toggle (the naga carries the serpent body instead).
        if (activePlan_ != 2 && activePlan_ != 3)
        {
            bool tail = tailPresent_;
            if (uiCheckbox("Reptile Tail", &tail))
            {
                setStatusMessage(setTail(tail) ? (tail ? "Tail attached" : "Tail removed") : "Tail op failed");
            }
        }
    }
    {
        const ParameterDefinition *spanDefinition = parameterDefinition("wings.span");
        const float spanMinimum = spanDefinition != nullptr ? spanDefinition->minimum : 0.55f;
        const float spanMaximum = spanDefinition != nullptr ? spanDefinition->maximum : 1.45f;
        float wingSpan = 1.0f;
        character_->getParameter(character_->self, characterHandle_, "wings.span", &wingSpan);
        const float before = wingSpan;
        if (uiSlider("Wingspan", &wingSpan, spanMinimum, spanMaximum) && std::fabs(wingSpan - before) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "wings.span", wingSpan, &error);
        }
        const ParameterDefinition *tailLengthDefinition = parameterDefinition("tail.length");
        const float tailLengthMinimum = tailLengthDefinition != nullptr ? tailLengthDefinition->minimum : 0.55f;
        const float tailLengthMaximum = tailLengthDefinition != nullptr ? tailLengthDefinition->maximum : 1.50f;
        float tailLength = 1.0f;
        character_->getParameter(character_->self, characterHandle_, "tail.length", &tailLength);
        const float tailBefore = tailLength;
        if (uiSlider("Tail Length", &tailLength, tailLengthMinimum, tailLengthMaximum)
            && std::fabs(tailLength - tailBefore) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "tail.length", tailLength, &error);
        }
        const ParameterDefinition *tailThicknessDefinition = parameterDefinition("tail.thickness");
        const float thicknessMinimum = tailThicknessDefinition != nullptr ? tailThicknessDefinition->minimum : 0.55f;
        const float thicknessMaximum = tailThicknessDefinition != nullptr ? tailThicknessDefinition->maximum : 1.65f;
        float tailThickness = 1.0f;
        character_->getParameter(character_->self, characterHandle_, "tail.thickness", &tailThickness);
        const float thicknessBefore = tailThickness;
        if (uiSlider("Tail Thickness", &tailThickness, thicknessMinimum, thicknessMaximum)
            && std::fabs(tailThickness - thicknessBefore) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "tail.thickness", tailThickness, &error);
        }
    }
    bool tailAnimation = animateTail_;
    if (uiCheckbox("Animate tail (offset drivers)", &tailAnimation))
    {
        animateTail_ = tailAnimation;
    }
    {
        float sway = tailSwaySpeed_;
        if (uiSlider("Tail Sway Speed", &sway, 0.0f, 2.0f))
        {
            tailSwaySpeed_ = sway;
        }
    }
    bool wingAnimation = animateWings_;
    if (uiCheckbox("Flap wings", &wingAnimation))
    {
        animateWings_ = wingAnimation;
    }

    // ---- recipes -----------------------------------------------------------------
    uiHeading("Presets & Recipes");
    if (uiButton("Save Recipe", 104.0f))
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
    if (uiButton("Load Recipe", 104.0f))
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
    if (uiButton("Reset", 84.0f))
    {
        resetToDefault();
    }

    // ---- status -----------------------------------------------------------------
    uiHeading("Status");
    uiTextColored(0.75f, 0.82f, 0.95f, statusMessage_.c_str());

    // ---- validation issues ---------------------------------------------------------
    if (!issues_.empty())
    {
        uiHeading("Validation Issues");
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

    ui_->endWindow(ui_->self, uiContext_);

    // Remember whether the UI wants the mouse next frame (gates orbit drag).
    UiCaptureState capture{};
    if (ui_->captureState(ui_->self, uiContext_, &capture))
    {
        uiWantsMouse_ = capture.mouse;
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
