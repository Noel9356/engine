// character_screen.hpp - the character customisation screen of the demo.
//
// A complete in-engine customisation screen built on the character module:
// a 3D character viewport (orbit camera, animated appendages) plus an
// immediate-mode panel with per-part preview thumbnails for swapping parts,
// editing body parameters, toggling the outfit, checking validation
// results, and saving/loading assembly recipes.

#ifndef ENGINE_DEMO_CHARACTER_SCREEN_HPP
#define ENGINE_DEMO_CHARACTER_SCREEN_HPP

#include "character_content.hpp"
#include "character_stage.hpp"

#include <engine/camera/capabilities.hpp>
#include <engine/character/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/ui/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace demo
{

class CharacterScreen
{
public:
    // Prepares the UI context, stage geometry, content, and the default
    // assembly. assetRoot points at the demo assets directory (fonts,
    // previews).
    bool create(const engine::window::WindowCapability &windowCapability, engine::window::WindowHandle windowHandle,
        const engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle rendererHandle,
        const engine::camera::CameraCapability &cameraCapability, engine::camera::CameraHandle cameraHandle,
        const engine::ui::UiCapability &uiCapability, engine::character::CharacterCapability &characterCapability,
        const std::string &assetRoot);

    void destroy();

    // Window events forwarded from the demo's event drain (keys and scroll
    // feed the UI).
    void handleWindowEvent(const engine::window::WindowEvent &event);

    // Per-frame update + draw; called between renderer beginFrame/endFrame.
    // Runs the animation drivers, evaluates the assembly after changes, draws
    // the stage and character, then the panel, and submits the overlay.
    void frame(double deltaSeconds, double elapsedSeconds);

    // Automated assembly test operations (part swaps, plan rebuilds, outfit
    // toggles, parameter extremes, recipe round-trip). Prints a PASS/FAIL
    // report to stdout. Returns true when every executable check passed.
    bool runSelfTest();

    // Camera framing override for automated captures (--camera-yaw /
    // --camera-pitch / --orbit-distance): yaw is measured around the
    // character with 0 looking from +Z (the character's front).
    void configureCamera(float yawDegrees, float pitchDegrees, float orbitDistance, float targetY);

    // Hides the studio panel (the --hide-panel capture mode); the viewport
    // keeps rendering so screenshots show the character alone.
    void setPanelVisible(bool visible);
    void setPanelTab(int tab); // opens the studio panel on the given tab
    void setUserCameraFraming(bool user); // CLI --orbit-distance/--camera-target-y survive updates

    // Applies a named configuration (the --character-drive presets used by
    // the automated capture runs): swaps head variants, drives parameters to
    // their range ends, or strips appendages. Unknown names report and stay
    // on the default configuration. Returns false when a module call failed.
    bool applyDrivePreset(const std::string &name);

    // Renders one standalone preview frame per registered part into
    // previewDirectory (PPM files named <partId>.ppm); exportPpm writes one
    // surface export. Used by --part-previews to produce the panel
    // thumbnails.
    bool renderPartPreviews(const std::string &previewDirectory,
        const std::function<bool(const std::string &)> &exportPpm);

    [[nodiscard]] bool valid() const
    {
        return created_;
    }

private:
    // ---- subsystems -------------------------------------------------------
    bool createUiContext(const std::string &assetRoot);
    void destroyUiContext();

    // Assembly helpers.
    engine::character::InstanceId instanceByName(const std::string &name) const;
    std::string                    partIdOfInstance(const std::string &name) const;
    bool                           rebuildAssembly(); // (re)create the assembly for the active body plan
    bool                           setPlan(int plan); // 0 humanoid, 1 taur, 2 quadruped
    bool                           setQuadSpecies(int species); // index into the generated species table
    bool                           setHeadPart(const std::string &partId); // swap the head variant live
    bool                           setEarParts(const std::string &leftPartId); // ear style pair
    bool                           setWings(bool present);    // one wing pair on the withers/shoulders
    bool                           setTail(bool present);     // on the humanoid tailbone or the taur socket
    bool                           setTailPart(const std::string &partId); // swap the tail variant live
    bool                           setOutfit(bool clothed);   // swap bare/ranger torso+arms+legs variants
    bool                           setClothingPiece(const char *instanceName, bool present);
    bool                           headHasEarSockets() const;
    void                           applyParameterHeights(); // keeps the feet/hooves near the ground plane
    void                           resetToDefault();

    // Evaluation + rendering.
    // evaluateCharacter re-derives transforms; full validation runs only when
    // requested (structure/parameter changes), never per animation frame.
    bool evaluateCharacter(bool validate = false);
    void updateAnimationDrivers(double deltaSeconds, double elapsedSeconds);
    void drawCharacter();

    // Panel.
    void drawPanel(double deltaSeconds);
    void drawStatsOverlay();
    void submitOverlay();
    void cacheParameterDefinitions();
    const engine::character::ParameterDefinition *parameterDefinition(const char *parameterId) const;
    void refreshDeformations(); // morphs + markings after parameter changes

    // Panel tabs (Character Studio: Plan / Body / Parts / Motion / File).
    void drawPlanTab();
    void drawBodyTab();
    void drawPartsTab();
    void drawMotionTab();
    void drawFileTab();
    void uiOptionRow(const char *const *options, int optionCount, const char *idSuffix,
        int activeIndex, float width, const std::function<void(int)> &onPick);
    bool uiParameterSlider(const char *parameterId);
    bool uiPartSelector(const std::vector<std::string> &partIds, const std::string &activePartId,
        const std::function<void(const std::string &)> &onPick);

    // UI helpers (thin wrappers around the capability).
    void uiText(const char *text) const;
    void uiTextColored(float r, float g, float b, const char *text) const;
    void uiHeading(const char *text) const;
    bool uiButton(const char *label, float width = 0.0f) const;
    bool uiSlider(const char *label, float *value, float minimum, float maximum) const;
    bool uiCheckbox(const char *label, bool *value) const;
    void setStatusMessage(std::string message);

    const engine::window::WindowCapability   *window_         = nullptr;
    engine::window::WindowHandle              windowHandle_   = engine::window::invalidWindow;
    const engine::renderer::RenderCapability *render_         = nullptr;
    engine::renderer::RendererHandle          rendererHandle_ = engine::renderer::invalidRenderer;
    const engine::camera::CameraCapability   *camera_         = nullptr;
    engine::camera::CameraHandle              cameraHandle_   = engine::camera::invalidCamera;
    const engine::ui::UiCapability           *ui_             = nullptr;
    engine::ui::UiContextHandle               uiContext_      = engine::ui::invalidUiContext;
    engine::renderer::TextureHandle           fontTexture_    = engine::renderer::invalidTexture;
    engine::character::CharacterCapability   *character_      = nullptr;
    CharacterContent                          content_;

    engine::character::CharacterHandle characterHandle_ = engine::character::invalidCharacter;

    // Static studio backdrop is an independent renderer resource owner.
    CharacterStage stage_;

    // Input state fed to the UI and the camera.
    bool   mouseButtons_[3] = {false, false, false};
    bool   orbitDragging_   = false;
    bool   uiWantsMouse_    = false;
    double lastCursorX_     = 0.0;
    double lastCursorY_     = 0.0;
    // Middle-mouse grab scrolling (the panel scroll convention).
    bool   middleScrolling_  = false;
    double middleLastX_      = 0.0;
    double middleLastY_      = 0.0;
    bool   haveLastCursor_   = false;
    // Character animation parameters (application-side drivers).
    float  tailSwaySpeed_           = 1.0f;
    bool   animateTail_             = true;
    bool   animateWings_            = true;
    double lastManualCameraTargetY_ = 1.0;

    // Overlay submission scratch.
    std::vector<engine::renderer::OverlayVertex>      overlayVertices_;
    std::vector<std::uint32_t>                        overlayIndices_;
    std::vector<engine::renderer::OverlayDrawCommand> overlayCommands_;

    // UI state.
    std::string statusMessage_       = "Welcome - drag to orbit, edit the panel";
    float       statusSeconds_       = 0.0f;
    float       framesPerSecond_     = 0.0f;
    std::string activeHeadPart_      = "head_human";
    std::string activeEarPart_       = "";      // ear style (left part id)
    std::string activeTailPart_      = "tail_fox";
    int         activePlan_          = 0;       // body.plan index
    int         quadSpecies_         = 0;       // species index (generated tables)
    bool        wingsPresent_        = false;   // exactly one pair
    bool        tailPresent_         = false;
    bool        clothed_             = true;    // outfit variant toggle
    bool        beltPresent_         = true;
    bool        bracersPresent_      = true;
    bool        pauldronsPresent_    = true;
    bool        hoodPresent_         = false;
    bool        panelFirstPlacement_ = true;
    bool        panelVisible_        = true;
    bool        userCameraFraming_   = false;   // explicit CLI framing active
    int         activePanelTab_      = 0;       // panel tab index
    std::vector<engine::character::ValidationIssue> issues_;
    bool        created_ = false;

    // Cached parameter registry (slider ranges for the panel).
    std::vector<engine::character::ParameterDefinition> parameterDefinitions_;
};

} // namespace demo

#endif // ENGINE_DEMO_CHARACTER_SCREEN_HPP
