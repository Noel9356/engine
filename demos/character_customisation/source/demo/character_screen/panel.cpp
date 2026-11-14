#include "../character_metrics.gen.hpp"
#include "../character_screen_internal.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

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
void CharacterScreen::uiOptionRow(const char *const *options, int optionCount, const char *idSuffix, int activeIndex,
    float width, const std::function<void(int)> &onPick)
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

// One selectable part row with its runtime preview thumbnail on the left
// (rendered by the engine at startup; parts without a thumbnail fall back to
// a plain selectable row). Returns true when the pick happened.
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
        const engine::renderer::TextureHandle preview  = content_.previewTextureForPart(partId);
        bool                                  selected = partId == activePartId;
        char                                  label[96];
        std::snprintf(label, sizeof(label), "%s##%s", info->displayName.c_str(), partId.c_str());
        const bool hasPreview = preview != engine::renderer::invalidTexture;
        if (hasPreview)
        {
            ui_->image(ui_->self,
                uiContext_,
                static_cast<engine::ui::UiTextureHandle>(preview),
                56.0f,
                56.0f,
                0.0f,
                0.0f,
                1.0f,
                1.0f);
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

// One preview-thumbnail card: the image on top, the label under it, a wide
// invisible selectable behind both so the whole card clicks, and a selected
// ring via the header-coloured label. Returns true when the pick happened.
bool CharacterScreen::uiPartCard(const std::string &partId, const std::string &label, bool selected, float cellWidth,
    const std::function<void()> &onPick)
{
    // The classic thumbnail card: the preview fills the cell's width, the
    // label sits on its own line underneath (a full-width selectable row, so
    // the whole card is the click target and the table keeps every row
    // aligned).
    const engine::renderer::TextureHandle preview = content_.previewTextureForPart(partId);
    const bool                             hasPreview = preview != engine::renderer::invalidTexture;
    const float                            image = cellWidth - 14.0f;
    bool                                   picked = false;
    if (hasPreview)
    {
        ui_->image(ui_->self,
            uiContext_,
            static_cast<engine::ui::UiTextureHandle>(preview),
            image,
            image,
            0.0f,
            0.0f,
            1.0f,
            1.0f);
    }
    char text[96];
    std::snprintf(text, sizeof(text), "%s##card_%s", label.c_str(), partId.c_str());
    bool selectedFlag = selected;
    if (ui_->selectable(ui_->self, uiContext_, text, &selectedFlag, 0.0f, 0.0f))
    {
        onPick();
        picked = true;
    }
    if (ui_->isItemHovered(ui_->self, uiContext_))
    {
        ui_->setTooltip(ui_->self, uiContext_, label.c_str());
    }
    return picked;
}

// A grid of part cards for one slot's variants (columns per row, the label
// from each part's render info).
void CharacterScreen::uiPartCardGrid(const std::vector<std::string> &partIds, const std::string &activePartId,
    const std::function<void(const std::string &)> &onPick, int columns)
{
    if (partIds.empty())
    {
        uiTextColored(0.62f, 0.66f, 0.72f, "Nothing available on this plan.");
        return;
    }
    // A real table grid: every card sits in its own cell, so the rows stay
    // aligned whatever the label lengths and the thumbnails share one size.
    const float cell = 138.0f;
    if (!ui_->beginTable(ui_->self,
            uiContext_,
            "cards",
            static_cast<std::uint32_t>(columns),
            static_cast<UiTableFlags>(0),
            0.0f,
            0.0f,
            cell))
    {
        return;
    }
    int index = 0;
    for (const std::string &partId : partIds)
    {
        const PartRenderInfo *info = content_.renderInfoForPart(partId);
        if (info == nullptr)
        {
            continue;
        }
        if (index % columns == 0)
        {
            ui_->tableNextRow(ui_->self, uiContext_, static_cast<UiTableRowFlags>(0), 0.0f);
        }
        ui_->tableNextColumn(ui_->self, uiContext_);
        // The category is already the heading/chip above the grid: the card
        // labels drop their slot suffix so they fit the cell in one line.
        std::string name = info->displayName;
        for (const char *suffix : {" Head", " Ears", " Neck", " Tail", " Leg", " Paw Arm", " (taur)"})
        {
            const std::size_t at = name.rfind(suffix);
            if (at != std::string::npos && at + std::strlen(suffix) == name.size())
            {
                name.erase(at);
                break;
            }
        }
        uiPartCard(partId, name, partId == activePartId, cell, [&, partId]()
            {
                onPick(partId);
            });
        ++index;
    }
    ui_->endTable(ui_->self, uiContext_);
}

// A chip row: one small button per category, the active one highlighted with
// the selection prefix. Returns the (possibly new) active index.
int CharacterScreen::uiCategoryChips(const std::vector<const char *> &labels, int activeIndex)
{
    // Radio chips: the filled dot marks the active category (a plain button
    // row left the selected chip indistinguishable).
    int         picked = activeIndex;
    std::int32_t value  = activeIndex;
    for (int index = 0; index < static_cast<int>(labels.size()); ++index)
    {
        char label[64];
        std::snprintf(label, sizeof(label), "%s##chip%d", labels[index], index);
        ui_->radioButton(ui_->self, uiContext_, label, &value, index);
        if (index == static_cast<int>(labels.size()) - 1)
        {
            ui_->spacing(ui_->self, uiContext_);
        }
        else
        {
            ui_->sameLine(ui_->self, uiContext_);
        }
    }
    picked = value;
    return picked;
}

// One labelled slider row driven by the cached parameter registry. The value
// readout draws beside the track (not inside it, where it collided with the
// grab), and hovering shows the range tooltip. Returns true when the value
// changed (the caller then applies plan-specific fixes).
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
    // The value rides IN the label (ImGui reserves the label's width, so the
    // track shrinks to fit and nothing clips at the window's right edge);
    // " " hides the in-track readout that collided with the grab.
    char label[128];
    std::snprintf(label,
        sizeof(label),
        "%s  %.2f##%s",
        definition->displayName,
        static_cast<double>(value),
        parameterId);
    if (ui_->sliderFloat(ui_->self, uiContext_, label, &value, definition->minimum, definition->maximum, " ")
        && std::fabs(value - before) > 1.0e-6f)
    {
        CharacterError error = CharacterError::None;
        character_->setParameter(character_->self, characterHandle_, parameterId, value, &error);
    }
    if (ui_->isItemHovered(ui_->self, uiContext_))
    {
        char tooltip[128];
        std::snprintf(tooltip, sizeof(tooltip), "%s: %.2f (range %.2f .. %.2f)",
            definition->displayName,
            static_cast<double>(value),
            static_cast<double>(definition->minimum),
            static_cast<double>(definition->maximum));
        ui_->setTooltip(ui_->self, uiContext_, tooltip);
    }
    return std::fabs(value - before) > 1.0e-6f;
}

// One synthetic preview-texture card (the coat swatches and marking modes
// rendered by the engine at startup). Returns true when the pick happened.
bool CharacterScreen::uiPreviewTextureCard(const char *key, const char *label, bool selected, float size,
    const std::function<void()> &onPick)
{
    const engine::renderer::TextureHandle preview = content_.previewTextureForPart(key);
    if (preview == engine::renderer::invalidTexture)
    {
        return false;
    }
    ui_->image(ui_->self,
        uiContext_,
        static_cast<engine::ui::UiTextureHandle>(preview),
        size,
        size,
        0.0f,
        0.0f,
        1.0f,
        1.0f);
    const bool hovered = ui_->isItemHovered(ui_->self, uiContext_);
    char text[96];
    std::snprintf(text, sizeof(text), "%s##pv_%s", label, key);
    bool selectedFlag = selected;
    bool picked       = false;
    if (ui_->selectable(ui_->self, uiContext_, text, &selectedFlag, 0.0f, 0.0f))
    {
        onPick();
        picked = true;
    }
    if (hovered || ui_->isItemHovered(ui_->self, uiContext_))
    {
        ui_->setTooltip(ui_->self, uiContext_, label);
    }
    return picked;
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
        ui_->setNextWindowPosition(ui_->self, uiContext_, static_cast<float>(width) - 492.0f, 16.0f, 0.0f, 0.0f);
        ui_->setNextWindowSize(ui_->self, uiContext_, 472.0f, std::max(static_cast<float>(height) - 32.0f, 420.0f));
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

    // Studio style: thicker slider grabs and rounded frames read clearly at
    // capture size, a visible scrollbar keeps the long part lists usable,
    // and the amber accent matches the panel's headings.
    ui_->pushStyleVariable(ui_->self, uiContext_, engine::ui::UiStyleVariable::GrabMinSize, 14.0f);
    ui_->pushStyleVariable(ui_->self, uiContext_, engine::ui::UiStyleVariable::GrabRounding, 7.0f);
    ui_->pushStyleVariable(ui_->self, uiContext_, engine::ui::UiStyleVariable::FrameRounding, 4.0f);
    ui_->pushStyleVariable(ui_->self, uiContext_, engine::ui::UiStyleVariable::ScrollbarSize, 15.0f);
    ui_->pushStyleColor(ui_->self, uiContext_, engine::ui::UiColor::SliderGrab, 0.98f, 0.78f, 0.36f, 0.92f);
    ui_->pushStyleColor(ui_->self, uiContext_, engine::ui::UiColor::SliderGrabActive, 1.00f, 0.86f, 0.50f, 1.00f);
    ui_->pushStyleColor(ui_->self, uiContext_, engine::ui::UiColor::CheckMark, 0.98f, 0.83f, 0.42f, 1.00f);
    ui_->pushStyleColor(ui_->self, uiContext_, engine::ui::UiColor::FrameBackground, 0.20f, 0.22f, 0.26f, 1.00f);
    if (!ui_->beginWindow(ui_->self, uiContext_, "Character Studio", nullptr, panelFlags))
    {
        ui_->endWindow(ui_->self, uiContext_);
        ui_->popStyleVariable(ui_->self, uiContext_, 4);
        ui_->popStyleColor(ui_->self, uiContext_, 4);
        return;
    }

    static const char *const kPlanNames[3] = {"Humanoid Biped", "Taur", "Quadruped"};
    static const char *const kTabs[5]      = {"Species", "Body", "Parts", "Motion", "File"};

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
    ui_->separator(ui_->self, uiContext_);

    switch (activePanelTab_)
    {
    case 0:
        drawSpeciesTab();
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
    ui_->popStyleVariable(ui_->self, uiContext_, 4);
    ui_->popStyleColor(ui_->self, uiContext_, 4);

    // Remember whether the UI wants the mouse next frame (gates orbit drag).
    UiCaptureState capture{};
    if (ui_->captureState(ui_->self, uiContext_, &capture))
    {
        uiWantsMouse_ = capture.mouse;
    }
}

// ---- tab: species + looks -------------------------------------------------------

void CharacterScreen::drawSpeciesTab()
{
    // The fandom flow: pick the species first (every downstream part follows
    // it by default), then the body plan it wears. The species cards show
    // the rendered heads; head-only species (the rabbit) still drive the
    // head chain on every plan.
    uiHeading("Species");
    {
        if (!ui_->beginTable(ui_->self,
            uiContext_,
            "species_cards",
            3u,
            static_cast<UiTableFlags>(0),
            0.0f,
            0.0f,
            138.0f))
        {
            return;
        }
        int index = 0;
        for (int species = 0; species < metrics::kSpeciesCount; ++species)
        {
            const metrics::SpeciesMetrics &entry   = metrics::kSpecies[species];
            const std::string              partId  = std::string("head_") + entry.id;
            const bool                     hasBody = activePlan_ == 0 || speciesHasBodyForPlan(species);
            const std::string              label   = hasBody ? entry.display : std::string(entry.display) + " *";
            if (index % 3 == 0)
            {
                ui_->tableNextRow(ui_->self, uiContext_, static_cast<UiTableRowFlags>(0), 0.0f);
            }
            ui_->tableNextColumn(ui_->self, uiContext_);
            uiPartCard(partId,
                label,
                species == quadSpecies_,
                138.0f,
                [&, species, entry]()
                {
                    if (setQuadSpecies(species))
                    {
                        setHeadPart(std::string("head_") + entry.id);
                        if (entry.hasEarSockets)
                        {
                            setEarParts(std::string("ear_") + entry.id + "_left");
                        }
                        if (entry.hasTail && activePlan_ != 2)
                        {
                            setTailPart(std::string("tail_") + entry.id);
                            setTail(true);
                        }
                        setStatusMessage(std::string("Species: ") + entry.display);
                    }
                    else
                    {
                        setStatusMessage("Species swap rejected");
                    }
                });
            ++index;
        }
        ui_->endTable(ui_->self, uiContext_);
    }
    uiTextColored(0.62f, 0.66f, 0.72f, "* head-only species keep the current lower body on taur/quadruped plans.");

    uiHeading("Body Plan");
    // One radio button per plan; stable ids via the "##plan" suffix. The
    // taur plan generalises the centaur: a humanoid upper body on the lower
    // body of any of the species above.
    static const char *const kPlanChoices[3] = {"Humanoid", "Taur", "Quadruped"};
    std::int32_t             plan            = activePlan_;
    for (int index = 0; index < 3; ++index)
    {
        char label[48];
        std::snprintf(label, sizeof(label), "%s##plan%d", kPlanChoices[index], index);
        ui_->radioButton(ui_->self, uiContext_, label, &plan, index);
        if (index % 3 == 2)
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
            character_->setParameter(
                character_->self, characterHandle_, "body.plan", static_cast<float>(requested), &error);
            setStatusMessage(std::string("Body plan: ") + kPlanChoices[requested]);
        }
        else
        {
            setStatusMessage("Body plan rebuild failed");
        }
    }

    // Species looks: the unified-head settings (the same base heads carry
    // the muzzle.length and eyes.width morphs, so a look is a parameter
    // combination on any canid head - short muzzle and wide eyes reads as a
    // cat, a long muzzle as a wolf).
    uiHeading("Species Looks");
    uiTextColored(0.62f, 0.66f, 0.72f, "Muzzle and eye settings on the base heads:");
    struct Look
    {
        const char *name;
        const char *head;
        const char *ears;
        const char *tail;
        float       muzzle;
        float       eyes;
    };
    static const Look looks[] = {
        {"Cat", "head_shibainu", "ear_shibainu_left", "tail_shibainu", 0.72f, 1.22f},
        {"Fox", "head_fox", "ear_fox_left", "tail_fox", 1.00f, 1.00f},
        {"Fennec", "head_fox", "ear_fox_left", "tail_fox", 0.85f, 1.26f},
        {"Wolf", "head_wolf", "ear_wolf_left", "tail_wolf", 1.22f, 0.94f},
        {"Jackal", "head_wolf", "ear_wolf_left", "tail_wolf", 1.40f, 1.00f},
        {"Akita", "head_shibainu", "ear_shibainu_left", "tail_shibainu", 0.92f, 1.04f},
    };
    for (std::size_t index = 0; index < std::size(looks); ++index)
    {
        const Look &look = looks[index];
        char        label[48];
        std::snprintf(label, sizeof(label), "%s##look%zu", look.name, index);
        if (uiButton(label, 72.0f))
        {
            CharacterError error = CharacterError::None;
            if (activePlan_ == 2)
            {
                setPlan(0);
                character_->setParameter(character_->self, characterHandle_, "body.plan", 0.0f, &error);
            }
            setHeadPart(look.head);
            setEarParts(look.ears);
            setTailPart(look.tail);
            setTail(true);
            character_->setParameter(character_->self, characterHandle_, "muzzle.length", look.muzzle, &error);
            character_->setParameter(character_->self, characterHandle_, "eyes.width", look.eyes, &error);
            character_->setParameter(character_->self, characterHandle_, "fur.coat", 0.0f, &error);
            character_->setParameter(character_->self, characterHandle_, "body.coat", 1.0f, &error);
            refreshDeformations();
            setStatusMessage(std::string("Look: ") + look.name);
        }
        if (index % 5 == 4)
        {
            ui_->spacing(ui_->self, uiContext_);
        }
        else
        {
            ui_->sameLine(ui_->self, uiContext_);
        }
    }

    // The dice roll: a random but always-valid character.
    uiHeading("Surprise Me");
    if (uiButton("Randomise character##randomise", 160.0f))
    {
        randomiseCharacter();
    }

    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(0.62f,
        0.66f,
        0.72f,
        "Tip: every species grafts on every plan - the Parts tab swaps heads, ears, limbs, tails, and wings freely.");
}

// ---- tab: body parameters --------------------------------------------------------

void CharacterScreen::drawBodyTab()
{
    // Every slider shown here acts on the active plan; the quadruped plan's
    // body is one whole-animal mesh, so the humanoid torso and head sliders
    // would have nothing to act on and are hidden there. The taur's chest
    // welds to the graft on the lower body, so its frame rides taur.scale
    // instead of the biped body sliders.
    if (activePlan_ == 0)
    {
        uiHeading("Frame");
        if (uiParameterSlider("body.height"))
        {
            applyParameterHeights();
        }
        uiParameterSlider("body.width");
        uiParameterSlider("body.muscle_mass");
        // Limb sliders act on the arms of every plan and the legs of the
        // biped (the taur's legs are the animal's own).
        if (uiParameterSlider("limbs.length"))
        {
            applyParameterHeights();
        }
        uiParameterSlider("limbs.thickness");
    }
    if (activePlan_ != 2)
    {
        uiHeading("Neck & Head");
        // The neck scales its column (the head rides the top socket); the
        // thickness rides the baked morph with both weld rings pinned.
        if (uiParameterSlider("neck.length"))
        {
            applyParameterHeights();
        }
        if (uiParameterSlider("neck.thickness"))
        {
            refreshDeformations();
        }
        uiParameterSlider("head.size");
        // Muzzle and eye spacing are the unified head settings: they deform
        // every species head (the same base head with different settings is
        // what separates, say, a cat look from a wolf look on top of the
        // ear style).
        if (uiParameterSlider("muzzle.length") || uiParameterSlider("eyes.width"))
        {
            refreshDeformations();
        }
        uiParameterSlider("ears.size");
    }

    uiHeading("Size");
    if (activePlan_ == 1 || activePlan_ == 2)
    {
        // The lower-body scale drives the animal mesh and every anchor
        // riding it (the taur chest mount, the ground line, the taur tail).
        if (uiParameterSlider("taur.scale"))
        {
            applyParameterHeights();
        }
    }

    // The body coat: skin tones or the fur palette on the bare torso and
    // limbs (the anthro body read - the head follows its own part logic).
    uiHeading("Body Coat");
    {
        float bodyCoat = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "body.coat", &bodyCoat);
        static const char *const coatKinds[2] = {"Skin", "Fur"};
        uiOptionRow(coatKinds,
            2,
            "bodycoat",
            static_cast<int>(bodyCoat + 0.5f),
            82.0f,
            [this](int candidate)
            {
                CharacterError error = CharacterError::None;
                character_->setParameter(
                    character_->self, characterHandle_, "body.coat", static_cast<float>(candidate), &error);
                refreshDeformations();
            });
        if (activePlan_ != 2 && bodyCoat < 0.5f)
        {
            // Skin tone palette (industry pattern: swatches, not a raw color
            // picker); the value is the tone index, drawCharacter maps it to a
            // material tint on the skin-tagged parts.
            uiText("Skin tone:");
            static const char *const toneNames[5] = {"Default", "Fair", "Tan", "Bronze", "Umber"};
            float                    tone         = 0.0f;
            character_->getParameter(character_->self, characterHandle_, "skin.tone", &tone);
            const int currentTone = static_cast<int>(tone + 0.5f);
            uiOptionRow(toneNames,
                5,
                "skin",
                currentTone,
                62.0f,
                [this](int candidate)
                {
                    CharacterError error = CharacterError::None;
                    character_->setParameter(
                        character_->self, characterHandle_, "skin.tone", static_cast<float>(candidate), &error);
                });
        }
    }

    // Fur coat palette: swatch 0 matches the active head's own colours (the
    // one-pelt read); the rest are the fixed palette. Every swatch is a
    // runtime-rendered preview of the actual torso repaint, laid out in a
    // table so every row stays aligned (a sameLine flow staggers when a
    // label outgrows its neighbours).
    uiHeading("Fur Coat");
    {
        static const char *const coatNames[8]
            = {"Match Head", "Natural", "Rust", "Snow", "Charcoal", "Cream", "Silver", "Dusk"};
        float coat = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "fur.coat", &coat);
        const int currentCoat = std::clamp(static_cast<int>(coat + 0.5f), 0, 7);
        if (ui_->beginTable(ui_->self,
                uiContext_,
                "coat_swatches",
                4,
                static_cast<UiTableFlags>(0),
                0.0f,
                0.0f,
                96.0f))
        {
            for (int candidate = 0; candidate < 8; ++candidate)
            {
                if (candidate % 4 == 0)
                {
                    ui_->tableNextRow(ui_->self, uiContext_, static_cast<UiTableRowFlags>(0), 0.0f);
                }
                ui_->tableNextColumn(ui_->self, uiContext_);
                char key[24];
                std::snprintf(key, sizeof(key), "coat_%d", candidate);
                uiPreviewTextureCard(key,
                    coatNames[candidate],
                    candidate == currentCoat,
                    74.0f,
                    [this, candidate]()
                    {
                        CharacterError error = CharacterError::None;
                        character_->setParameter(
                            character_->self, characterHandle_, "fur.coat", static_cast<float>(candidate), &error);
                        refreshDeformations();
                    });
            }
            ui_->endTable(ui_->self, uiContext_);
        }
        ui_->spacing(ui_->self, uiContext_);
    }

    // Fur markings: the fandom's socks/gloves, tail-tip, muzzle, belly,
    // stripe, and point patterns - each a runtime-rendered preview of the
    // torso repaint, re-derived on the affected meshes at change time. The
    // same table grid as the coat swatches keeps the rows aligned.
    uiHeading("Markings");
    {
        static const char *const markingNames[6] = {"None", "Socks", "Tipped", "Full", "Stripes", "Points"};
        float                    markings        = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "fur.markings", &markings);
        const int currentMarking = std::clamp(static_cast<int>(markings + 0.5f), 0, 5);
        if (ui_->beginTable(ui_->self,
                uiContext_,
                "marking_swatches",
                3,
                static_cast<UiTableFlags>(0),
                0.0f,
                0.0f,
                96.0f))
        {
            for (int candidate = 0; candidate < 6; ++candidate)
            {
                if (candidate % 3 == 0)
                {
                    ui_->tableNextRow(ui_->self, uiContext_, static_cast<UiTableRowFlags>(0), 0.0f);
                }
                ui_->tableNextColumn(ui_->self, uiContext_);
                char key[28];
                std::snprintf(key, sizeof(key), "marking_%d", candidate);
                uiPreviewTextureCard(key,
                    markingNames[candidate],
                    candidate == currentMarking,
                    74.0f,
                    [this, candidate]()
                    {
                        CharacterError error = CharacterError::None;
                        character_->setParameter(
                            character_->self, characterHandle_, "fur.markings", static_cast<float>(candidate), &error);
                        refreshDeformations();
                    });
            }
            ui_->endTable(ui_->self, uiContext_);
        }
        ui_->spacing(ui_->self, uiContext_);
        uiTextColored(0.62f, 0.66f, 0.72f, "Stripes and Points use the species' own accent colour.");
    }
}

// ---- tab: parts ---------------------------------------------------------------------

void CharacterScreen::drawPartsTab()
{
    // Category chips first: one click switches the grid below instead of a
    // single endless scroll, so every category sits one click away.
    static const char *const kCategories[7] = {"Head", "Ears", "Neck", "Tail", "Wings", "Arms", "Legs"};
    partsCategory_ = uiCategoryChips({kCategories, kCategories + 7}, partsCategory_);

    switch (partsCategory_)
    {
    case 0:
    {
        // Head styles with preview thumbnails: every registered slot:head part.
        uiHeading("Head");
        uiPartCardGrid(content_.partsInSlot("head"),
            activeHeadPart_,
            [this](const std::string &partId)
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
            },
            4);
        break;
    }
    case 1:
    {
        // Ear styles: separate parts of the head (swap the pair on the head's
        // ear sockets). Only species heads carry ear sockets.
        uiHeading("Ears");
        if (!headHasEarSockets())
        {
            uiTextColored(0.62f, 0.66f, 0.72f, "This head carries its own ears (or none).");
            break;
        }
        std::vector<std::string> earStyles;
        for (const std::string &partId : content_.partsInSlot("ears"))
        {
            if (partId.size() > 5 && partId.substr(partId.size() - 5) == "_left")
            {
                earStyles.push_back(partId);
            }
        }
        uiPartCardGrid(earStyles,
            activeEarPart_,
            [this](const std::string &partId)
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
            },
            4);
        uiParameterSlider("ears.size");
        break;
    }
    case 2:
    {
        // Neck styles: the neck spans the collar socket to the skull-base weld.
        // It follows the head's species automatically; this grid overrides (a
        // mismatched pair leaves a step at the atlas ring, so the follow is
        // the recommended path).
        uiHeading("Neck");
        if (activePlan_ == 2)
        {
            uiTextColored(0.62f, 0.66f, 0.72f, "The quadruped plan carries its own neck.");
            break;
        }
        uiPartCardGrid(content_.partsInSlot("neck"),
            activeNeckPart_,
            [this](const std::string &partId)
            {
                if (setNeckPart(partId))
                {
                    const PartRenderInfo *info = content_.renderInfoForPart(partId);
                    setStatusMessage(std::string("Neck set to ") + (info ? info->displayName : partId));
                }
                else
                {
                    setStatusMessage("Neck replacement rejected");
                }
            },
            4);
        break;
    }
    case 3:
    {
        // Tail: toggle + style thumbnails. On the humanoid plan the tail rides
        // the sacrum-coccyx segment's tailbone; on the taur plan the lower
        // body's own tail socket (so the tail option affects the lower body);
        // the quadruped plan keeps its own tail.
        uiHeading("Tail");
        if (activePlan_ == 2)
        {
            uiTextColored(0.62f, 0.66f, 0.72f, "The quadruped plan carries its own tail.");
            break;
        }
        bool tail = tailPresent_;
        if (uiCheckbox("Attach tail##tailtoggle", &tail))
        {
            setStatusMessage(setTail(tail) ? (tail ? "Tail attached" : "Tail removed") : "Tail op failed");
        }
        if (tailPresent_)
        {
            uiPartCardGrid(content_.partsInSlot("tail"),
                activeTailPart_,
                [this](const std::string &partId)
                {
                    const PartRenderInfo *info = content_.renderInfoForPart(partId);
                    setStatusMessage(setTailPart(partId) ? std::string("Tail: ") + (info ? info->displayName : partId)
                                                         : "Tail swap rejected");
                },
                4);
            uiParameterSlider("tail.length");
            uiParameterSlider("tail.thickness");
        }
        break;
    }
    case 4:
    {
        // Wings: exactly one pair (toggle).
        uiHeading("Wings");
        if (activePlan_ == 2)
        {
            uiTextColored(0.62f, 0.66f, 0.72f, "The quadruped plan has no wing sockets.");
            break;
        }
        bool wings = wingsPresent_;
        if (uiCheckbox("Attach wing pair##wingtoggle", &wings))
        {
            setStatusMessage(setWings(wings) ? (wings ? "Wing pair attached" : "Wings removed") : "Wing attach failed");
        }
        if (wingsPresent_)
        {
            uiParameterSlider("wings.span");
        }
        break;
    }
    case 5:
    {
        // Arms: the human bare/ranger variants plus every species' paw arm
        // (the forelegs with their own paws). Species arms carry no bracer
        // socket, so the bracers come off automatically.
        uiHeading("Arms");
        if (activePlan_ == 2)
        {
            uiTextColored(0.62f, 0.66f, 0.72f, "The quadruped plan carries its own forelegs.");
            break;
        }
        uiPartCardGrid(content_.partsInSlot("arms"),
            activeArmPart_,
            [this](const std::string &partId)
            {
                const PartRenderInfo *info = content_.renderInfoForPart(partId);
                const std::string     right = partId.substr(0, partId.size() - 5) + "_right";
                const bool            okLeft = setLimbPart("arm_left", partId);
                const bool            okRight = setLimbPart("arm_right", right);
                setStatusMessage(okLeft && okRight ? std::string("Arms: ") + (info ? info->displayName : partId)
                                                   : "Arm swap rejected");
            },
            4);
        break;
    }
    default:
    {
        // Legs: the human bare/ranger variants plus every species' own hind
        // leg - digitigrade paws, hooves, and raptor claws on the biped
        // pelvis (the Parts tab answer to "not every character wears boots").
        uiHeading("Legs");
        if (activePlan_ != 0)
        {
            uiTextColored(0.62f, 0.66f, 0.72f, "Only the humanoid plan swaps biped legs.");
            break;
        }
        uiPartCardGrid(content_.partsInSlot("legs"),
            activeLegPart_,
            [this](const std::string &partId)
            {
                const PartRenderInfo *info = content_.renderInfoForPart(partId);
                const std::string     right = partId.substr(0, partId.size() - 5) + "_right";
                const bool            okLeft = setLimbPart("leg_left", partId);
                const bool            okRight = setLimbPart("leg_right", right);
                setStatusMessage(okLeft && okRight ? std::string("Legs: ") + (info ? info->displayName : partId)
                                                   : "Leg swap rejected");
            },
            4);
        break;
    }
    }

    // Outfit: the master toggle swaps the bare and ranger variants of the
    // torso and the human limbs; the pieces below add or remove overlays.
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
            setStatusMessage(
                setClothingPiece("belt", belt) ? (belt ? "Belt on" : "Belt removed") : "Belt toggle failed");
        }
        bool bracers = bracersPresent_;
        if (uiCheckbox("Bracers##bracertoggle", &bracers))
        {
            bracersPresent_    = bracers;
            const bool okLeft  = setClothingPiece("bracer_left", bracers);
            const bool okRight = setClothingPiece("bracer_right", bracers);
            setStatusMessage(okLeft && okRight ? (bracers ? "Bracers on" : "Bracers removed") : "Bracer toggle failed");
        }
        bool pauldrons = pauldronsPresent_;
        if (uiCheckbox("Pauldrons##pauldrontoggle", &pauldrons))
        {
            pauldronsPresent_  = pauldrons;
            const bool okLeft  = setClothingPiece("pauldron_left", pauldrons);
            const bool okRight = setClothingPiece("pauldron_right", pauldrons);
            setStatusMessage(
                okLeft && okRight ? (pauldrons ? "Pauldrons on" : "Pauldrons removed") : "Pauldron toggle failed");
        }
    }
    bool hood = hoodPresent_;
    if (uiCheckbox("Hood##hoodtoggle", &hood))
    {
        hoodPresent_ = hood;
        setStatusMessage(setClothingPiece("hood", hood) ? (hood ? "Hood on" : "Hood removed") : "Hood toggle failed");
    }

    ui_->spacing(ui_->self, uiContext_);
    uiTextColored(
        0.62f, 0.66f, 0.72f, "Tip: hold the middle mouse button and drag (or use the wheel) to scroll this panel.");
}

// ---- tab: motion -----------------------------------------------------------------

void CharacterScreen::drawMotionTab()
{
    uiHeading("Drivers");
    bool tailAnimation = animateTail_;
    if (uiCheckbox("Animate tail##animtail", &tailAnimation))
    {
        animateTail_ = tailAnimation;
    }
    {
        float sway = tailSwaySpeed_;
        char swayLabel[64];
        std::snprintf(swayLabel, sizeof(swayLabel), "Tail sway speed  %.2f", static_cast<double>(sway));
        if (ui_->sliderFloat(ui_->self, uiContext_, swayLabel, &sway, 0.0f, 2.0f, " "))
        {
            tailSwaySpeed_ = sway;
        }
    }
    bool wingAnimation = animateWings_;
    if (uiCheckbox("Flap wings##animwings", &wingAnimation))
    {
        animateWings_ = wingAnimation;
    }
    {
        float flap = wingFlapSpeed_;
        char flapLabel[64];
        std::snprintf(flapLabel, sizeof(flapLabel), "Wing flap speed  %.2f", static_cast<double>(flap));
        if (ui_->sliderFloat(ui_->self, uiContext_, flapLabel, &flap, 0.0f, 2.0f, " "))
        {
            wingFlapSpeed_ = flap;
        }
    }
    bool breathing = breathe_;
    if (uiCheckbox("Idle breathing##animbreath", &breathing))
    {
        breathe_ = breathing;
    }

    uiHeading("How it works");
    uiTextColored(0.62f, 0.66f, 0.72f, "The drivers edit connection offsets each frame; the assembly");
    uiTextColored(0.62f, 0.66f, 0.72f, "re-evaluates on top of them, so the welds never move.");
    uiTextColored(0.62f, 0.66f, 0.72f, "Breathing adds a subtle neck pitch sway for the idle read.");
}

// ---- tab: presets, status, validation ----------------------------------------------

void CharacterScreen::drawFileTab()
{
    uiHeading("Recipes");
    if (uiButton("Save Recipe##recipe-save", 124.0f))
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
    if (uiButton("Load Recipe##recipe-load", 124.0f))
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
    // Reset is the destructive action: it styles red and confirms through the
    // status line (the randomise button on the Species tab is the safe dice).
    ui_->pushStyleColor(ui_->self, uiContext_, engine::ui::UiColor::Button, 0.55f, 0.22f, 0.20f, 0.90f);
    ui_->pushStyleColor(ui_->self, uiContext_, engine::ui::UiColor::ButtonHovered, 0.68f, 0.28f, 0.24f, 0.95f);
    ui_->pushStyleColor(ui_->self, uiContext_, engine::ui::UiColor::ButtonActive, 0.80f, 0.32f, 0.26f, 1.00f);
    const bool reset = uiButton("Reset##recipe-reset", 92.0f);
    ui_->popStyleColor(ui_->self, uiContext_, 3);
    if (reset)
    {
        resetToDefault();
    }
    uiTextColored(0.62f, 0.66f, 0.72f, "Recipes store the whole assembly (parts, connections, parameters).");

    uiHeading("Surprise");
    if (uiButton("Randomise character##file-randomise", 160.0f))
    {
        randomiseCharacter();
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
            char                   line[320];
            std::snprintf(
                line, sizeof(line), "%u %s: %s", static_cast<unsigned>(issue.code), issue.subject, issue.message);
            uiTextColored(0.98f, 0.55f, 0.45f, line);
        }
    }

    uiHeading("About");
    uiTextColored(0.62f, 0.66f, 0.72f, "Every part is a piece of a downloaded CC0 pack (Quaternius;");
    uiTextColored(0.62f, 0.66f, 0.72f, "see LICENSE.md in the assets). The thumbnails are rendered by");
    uiTextColored(0.62f, 0.66f, 0.72f, "the engine at startup - no pre-generated images ship.");
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
        // Two short lines: a single long line ran under the studio panel
        // on narrow windows and clipped mid-word.
        std::snprintf(line,
            sizeof(line),
            "%.0f fps  |  parts %u  joints %u",
            static_cast<double>(framesPerSecond_),
            static_cast<unsigned>(statistics.partCount),
            static_cast<unsigned>(statistics.jointCount));
        uiText(line);
        std::snprintf(line,
            sizeof(line),
            "connections %u  |  triangles %u",
            static_cast<unsigned>(statistics.connectionCount),
            static_cast<unsigned>(statistics.triangleCount));
        uiText(line);
    }
    ui_->textColored(ui_->self,
        uiContext_,
        0.80f,
        0.85f,
        0.95f,
        0.75f,
        "drag: orbit   middle-drag: scroll\nspace/ctrl: zoom   arrows: orbit");
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
