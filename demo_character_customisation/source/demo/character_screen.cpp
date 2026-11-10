// character_screen.cpp - the character customisation screen implementation.
//
// Per frame: feed input to the UI, run the appendage animation drivers
// (connection offset rotations - the module-supported way to animate
// attachments without a skinning path), evaluate the assembly, draw the
// stage and the independently rendered parts, then draw and submit the panel
// as a renderer overlay.

#include "character_screen.hpp"
#include "character_metrics.gen.hpp"

#include "geometry.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <utility>

namespace demo
{
namespace
{

using engine::character::CharacterError;
using engine::character::CharacterHandle;
using engine::character::InstanceId;
using engine::character::ParameterDefinition;
using engine::character::ValidationIssue;
using engine::renderer::MeshHandle;
using engine::renderer::MeshMaterial;
using engine::ui::UiCaptureState;
using engine::ui::UiContextHandle;
using engine::ui::UiDrawCommand;
using engine::ui::UiDrawData;
using engine::ui::UiError;
using engine::ui::UiFrameDescription;
using engine::ui::UiKey;
using engine::ui::UiWindowFlags;
using engine::window::Key;
using engine::window::MouseButton;
using engine::window::WindowEvent;
using engine::window::WindowEventKind;
using engine::window::WindowHandle;

constexpr const char *kRecipeFileName   = "character_recipe.toml";
constexpr const char *kFontRelativePath = "fonts/Inter-Variable.ttf";

// Parts tinted by the skin.tone palette (the humanoid skin surface; monster
// heads, tails, and wings keep their authored palettes).
bool isSkinPart(const char *partId)
{
    static const char *skinParts[] = {
        "torso_humanoid", "arm_humanoid_left", "arm_humanoid_right", "leg_humanoid_left", "leg_humanoid_right",
        "neck_transition",
    };
    if (partId == nullptr)
    {
        return false;
    }
    for (const char *candidate : skinParts)
    {
        if (std::strcmp(partId, candidate) == 0)
        {
            return true;
        }
    }
    return false;
}

UiKey mapWindowKey(Key key)
{
    switch (key)
    {
    case Key::Escape:
        return UiKey::Escape;
    case Key::Space:
        return UiKey::Space;
    case Key::Enter:
        return UiKey::Enter;
    case Key::Tab:
        return UiKey::Tab;
    case Key::Backspace:
        return UiKey::Backspace;
    case Key::Left:
        return UiKey::Left;
    case Key::Right:
        return UiKey::Right;
    case Key::Up:
        return UiKey::Up;
    case Key::Down:
        return UiKey::Down;
    case Key::W:
        return UiKey::W;
    case Key::A:
        return UiKey::A;
    case Key::S:
        return UiKey::S;
    case Key::D:
        return UiKey::D;
    case Key::Q:
        return UiKey::Q;
    case Key::E:
        return UiKey::E;
    case Key::LeftShift:
        return UiKey::LeftShift;
    case Key::LeftControl:
        return UiKey::LeftControl;
    case Key::LeftAlt:
        return UiKey::LeftAlt;
    case Key::Num0:
        return UiKey::D0;
    case Key::Num1:
        return UiKey::D1;
    case Key::Num2:
        return UiKey::D2;
    case Key::Num3:
        return UiKey::D3;
    case Key::Num4:
        return UiKey::D4;
    case Key::Num5:
        return UiKey::D5;
    case Key::Num6:
        return UiKey::D6;
    case Key::Num7:
        return UiKey::D7;
    case Key::Num8:
        return UiKey::D8;
    case Key::Num9:
        return UiKey::D9;
    default:
        return UiKey::None;
    }
}

std::uint64_t keyMaskBit(Key key)
{
    const auto value = static_cast<std::uint32_t>(key);
    return value > 0 && value < 64 ? (std::uint64_t{1} << value) : 0;
}

// Compact quaternion around one axis.
void setYawRotation(float *outQuaternion4, float angleRadians)
{
    outQuaternion4[0] = 0.0f;
    outQuaternion4[1] = std::sin(angleRadians * 0.5f);
    outQuaternion4[2] = 0.0f;
    outQuaternion4[3] = std::cos(angleRadians * 0.5f);
}

void setRollRotation(float *outQuaternion4, float angleRadians)
{
    outQuaternion4[0] = 0.0f;
    outQuaternion4[1] = 0.0f;
    outQuaternion4[2] = std::sin(angleRadians * 0.5f);
    outQuaternion4[3] = std::cos(angleRadians * 0.5f);
}

} // namespace

// ---- lifecycle ---------------------------------------------------------------

bool CharacterScreen::create(const engine::window::WindowCapability &windowCapability,
    engine::window::WindowHandle windowHandle, const engine::renderer::RenderCapability &renderCapability,
    engine::renderer::RendererHandle rendererHandle, const engine::camera::CameraCapability &cameraCapability,
    engine::camera::CameraHandle cameraHandle, const engine::ui::UiCapability &uiCapability,
    engine::character::CharacterCapability &characterCapability, const std::string &assetRoot)
{
    window_         = &windowCapability;
    windowHandle_   = windowHandle;
    render_         = &renderCapability;
    rendererHandle_ = rendererHandle;
    camera_         = &cameraCapability;
    cameraHandle_   = cameraHandle;
    ui_             = &uiCapability;
    character_      = &characterCapability;

    if (!content_.create(*character_, *render_, rendererHandle_, assetRoot))
    {
        return false;
    }
    if (!createUiContext(assetRoot))
    {
        content_.destroy(*character_, *render_, rendererHandle_);
        return false;
    }

    createStage();

    // Default character from the assembly profile.
    CharacterError        error  = CharacterError::None;
    const CharacterHandle handle = character_->createAssembly(character_->self, hybridProfileId, &error);
    if (handle == engine::character::invalidCharacter)
    {
        std::fprintf(stderr,
            "demo: the character module could not create the default assembly (%u)\n",
            static_cast<unsigned>(error));
        destroyStage();
        destroyUiContext();
        content_.destroy(*character_, *render_, rendererHandle_);
        return false;
    }
    characterHandle_ = handle;

    // Orbit camera around the character: three-quarter view from the
    // character's front-right so the face stays visible and the tail reads
    // against the sky instead of hiding behind the torso.
    configureCamera(52.0f, 10.0f, 2.6f);

    applyParameterHeights();
    cacheParameterDefinitions();
    evaluateCharacter(true);
    created_ = true;
    return true;
}

void CharacterScreen::setPanelVisible(bool visible)
{
    panelVisible_ = visible;
}

void CharacterScreen::configureCamera(float yawDegrees, float pitchDegrees, float orbitDistance)
{
    const float yawRadians   = glm::radians(yawDegrees);
    const float pitchRadians = glm::radians(std::clamp(pitchDegrees, -80.0f, 80.0f));
    const glm::vec3 target(0.0f, 1.0f, 0.0f);
    const glm::vec3 eye = target + orbitDistance
                            * glm::vec3(std::sin(yawRadians) * std::cos(pitchRadians), std::sin(pitchRadians),
                                std::cos(yawRadians) * std::cos(pitchRadians));
    const float eyeFloat[3]    = {eye.x, eye.y, eye.z};
    const float targetFloat[3] = {target.x, target.y, target.z};
    const float up[3]          = {0.0f, 1.0f, 0.0f};
    camera_->setLookAt(camera_->self, cameraHandle_, eyeFloat, targetFloat, up);
    camera_->setControlMode(camera_->self, cameraHandle_, engine::camera::CameraControlMode::Orbit);
    camera_->setOrbitDistance(camera_->self, cameraHandle_, orbitDistance);
}

void CharacterScreen::cacheParameterDefinitions()
{
    // One registry snapshot for the panel's slider ranges; the registry is
    // static after content creation, so this never needs refreshing.
    parameterDefinitions_.clear();
    const std::uint32_t total = character_->parameterCount(character_->self);
    parameterDefinitions_.reserve(total);
    for (std::uint32_t index = 0; index < total; ++index)
    {
        ParameterDefinition definition{};
        if (character_->parameterInfo(character_->self, index, &definition))
        {
            parameterDefinitions_.push_back(definition);
        }
    }
}

const engine::character::ParameterDefinition *CharacterScreen::parameterDefinition(const char *parameterId) const
{
    for (const ParameterDefinition &definition : parameterDefinitions_)
    {
        if (std::string(definition.id) == parameterId)
        {
            return &definition;
        }
    }
    return nullptr;
}

void CharacterScreen::destroy()
{
    if (character_ != nullptr && characterHandle_ != engine::character::invalidCharacter)
    {
        character_->destroyAssembly(character_->self, characterHandle_);
        characterHandle_ = engine::character::invalidCharacter;
    }
    destroyStage();
    destroyUiContext();
    if (character_ != nullptr)
    {
        content_.destroy(*character_, *render_, rendererHandle_);
    }
    created_ = false;
}

// ---- UI context (font + atlas texture) --------------------------------------

bool CharacterScreen::createUiContext(const std::string &assetRoot)
{
    if (ui_ == nullptr || ui_->createContext == nullptr)
    {
        return false;
    }
    engine::ui::UiContextDescription description{};
    description.reserveVertices   = 8192;
    description.reserveIndices    = 16384;
    UiError               error   = UiError::None;
    const UiContextHandle context = ui_->createContext(ui_->self, &description, &error);
    if (context == engine::ui::invalidUiContext)
    {
        std::fprintf(stderr, "demo: character screen UI context creation failed (%u)\n", static_cast<unsigned>(error));
        return false;
    }
    uiContext_ = context;

    // A real UI font when shipped; the ImGui default otherwise.
    const std::filesystem::path fontPath = std::filesystem::path(assetRoot) / kFontRelativePath;
    engine::ui::UiFontHandle    font     = engine::ui::defaultUiFont;
    if (ui_->addFontFromFile != nullptr && std::filesystem::exists(fontPath))
    {
        if (!ui_->addFontFromFile(ui_->self, uiContext_, fontPath.string().c_str(), 17.0f, &font))
        {
            std::fprintf(stderr,
                "demo: character screen font '%s' failed to load; using the default font\n",
                fontPath.string().c_str());
            font = engine::ui::defaultUiFont;
        }
    }

    engine::ui::UiFontAtlas atlas{};
    if (!ui_->fontAtlas(ui_->self, uiContext_, &atlas) || atlas.pixels == nullptr || atlas.width == 0
        || atlas.height == 0 || atlas.bytesPerPixel != 4)
    {
        destroyUiContext();
        return false;
    }
    const engine::renderer::TextureDescription textureDescription{
        .width          = atlas.width,
        .height         = atlas.height,
        .pixels         = atlas.pixels,
        .filter         = engine::renderer::TextureFilter::Linear,
        .address        = engine::renderer::TextureAddress::ClampEdge,
        .pixelFormat    = engine::renderer::TexturePixelFormat::RGBA8,
        .origin         = engine::renderer::TextureOrigin::TopLeft,
        .rowStrideBytes = atlas.width * atlas.bytesPerPixel,
    };
    engine::renderer::RenderError         renderError = engine::renderer::RenderError::None;
    const engine::renderer::TextureHandle fontTexture
        = render_->createTexture(render_->self, rendererHandle_, &textureDescription, &renderError);
    if (fontTexture == engine::renderer::invalidTexture)
    {
        destroyUiContext();
        return false;
    }
    if (!ui_->setFontTexture(ui_->self, uiContext_, static_cast<engine::ui::UiTextureHandle>(fontTexture)))
    {
        render_->destroyTexture(render_->self, rendererHandle_, fontTexture);
        destroyUiContext();
        return false;
    }
    if (font != engine::ui::defaultUiFont)
    {
        ui_->pushFont(ui_->self, uiContext_, font);
    }
    fontTexture_ = fontTexture;
    return true;
}

void CharacterScreen::destroyUiContext()
{
    if (ui_ != nullptr && uiContext_ != engine::ui::invalidUiContext)
    {
        ui_->destroyContext(ui_->self, uiContext_);
    }
    if (render_ != nullptr && rendererHandle_ != engine::renderer::invalidRenderer
        && fontTexture_ != engine::renderer::invalidTexture)
    {
        render_->destroyTexture(render_->self, rendererHandle_, fontTexture_);
    }
    uiContext_   = engine::ui::invalidUiContext;
    fontTexture_ = engine::renderer::invalidTexture;
}

// ---- stage ------------------------------------------------------------------

void CharacterScreen::createStage()
{
    // Studio floor: a soft dark platform; an inward-facing sky dome + sun
    // disc complete the backdrop (drawn unlit, after the character, so the
    // dome's occluded hemisphere depth-rejects). The dome's winding is
    // inverted at build time so backface culling keeps exactly the visible
    // interior.
    const MeshData floorGeometry = makeBoxMesh(
        glm::vec3(0.0f, -0.06f, 0.0f), glm::vec3(6.0f, 0.06f, 6.0f), glm::vec3(0.42f, 0.44f, 0.48f), 4.0f);
    MeshData skyGeometry = makeSphereMesh(
        glm::vec3(0.0f), 900.0f, 24, 12, glm::vec3(0.55f, 0.64f, 0.78f), glm::vec3(0.24f, 0.38f, 0.62f));
    for (std::size_t index = 0; index + 2 < skyGeometry.indices.size(); index += 3)
    {
        std::swap(skyGeometry.indices[index], skyGeometry.indices[index + 1]);
        Vertex &vertex = skyGeometry.vertices[skyGeometry.indices[index]];
        vertex.normal *= -1.0f;
    }
    const MeshData sunGeometry = makeSphereMesh(
        glm::vec3(280.0f, 350.0f, -640.0f), 16.0f, 14, 8, glm::vec3(4.0f, 3.6f, 3.0f), glm::vec3(4.0f, 3.6f, 3.0f));

    struct StageUpload
    {
        const MeshData               *geometry;
        engine::renderer::MeshHandle *target;
    };
    const StageUpload uploads[] = {
        {&floorGeometry, &floorMesh_},
        {&skyGeometry, &skyDomeMesh_},
        {&sunGeometry, &sunDiscMesh_},
    };
    for (const StageUpload &upload : uploads)
    {
        const engine::renderer::MeshDescription description = upload.geometry->description();
        engine::renderer::RenderError           error       = engine::renderer::RenderError::None;
        *upload.target = render_->createMesh(render_->self, rendererHandle_, &description, &error);
        if (*upload.target == engine::renderer::invalidMesh)
        {
            std::fprintf(stderr, "demo: character stage mesh upload failed (%u)\n", static_cast<unsigned>(error));
        }
    }
}

void CharacterScreen::destroyStage()
{
    if (render_ != nullptr && rendererHandle_ != engine::renderer::invalidRenderer)
    {
        render_->destroyMesh(render_->self, rendererHandle_, floorMesh_);
        render_->destroyMesh(render_->self, rendererHandle_, skyDomeMesh_);
        render_->destroyMesh(render_->self, rendererHandle_, sunDiscMesh_);
    }
    floorMesh_   = engine::renderer::invalidMesh;
    skyDomeMesh_ = engine::renderer::invalidMesh;
    sunDiscMesh_ = engine::renderer::invalidMesh;
}

// ---- assembly helpers ---------------------------------------------------------

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

bool CharacterScreen::wingsPresent() const
{
    return instanceByName("wing_left") != engine::character::invalidInstance;
}

bool CharacterScreen::addWings()
{
    if (wingsPresent())
    {
        return true;
    }
    CharacterError   error = CharacterError::None;
    const InstanceId torso = instanceByName("torso");
    const InstanceId left
        = character_->addPart(character_->self, characterHandle_, "wing_avian_left", "wing_left", &error);
    const InstanceId right
        = left != engine::character::invalidInstance
            ? character_->addPart(character_->self, characterHandle_, "wing_avian_right", "wing_right", &error)
            : engine::character::invalidInstance;
    if (torso == engine::character::invalidInstance || left == engine::character::invalidInstance
        || right == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    bool        connected   = character_->connect(
        character_->self, characterHandle_, torso, "wings.left", left, "wing.root", nullptr, identity, &error);
    connected
        = character_->connect(
              character_->self, characterHandle_, torso, "wings.right", right, "wing.root", nullptr, identity, &error)
       && connected;
    return connected;
}

bool CharacterScreen::removeWings()
{
    const InstanceId left  = instanceByName("wing_left");
    const InstanceId right = instanceByName("wing_right");
    if (left == engine::character::invalidInstance && right == engine::character::invalidInstance)
    {
        return true;
    }
    bool removed = true;
    if (left != engine::character::invalidInstance)
    {
        removed = character_->removePart(character_->self, characterHandle_, left) && removed;
    }
    if (right != engine::character::invalidInstance)
    {
        removed = character_->removePart(character_->self, characterHandle_, right) && removed;
    }
    return removed;
}

bool CharacterScreen::secondTailPresent() const
{
    return instanceByName("tail2") != engine::character::invalidInstance;
}

bool CharacterScreen::addTail(const char *basePartId)
{
    if (instanceByName("tail") != engine::character::invalidInstance)
    {
        return true;
    }
    const char *tipPartId = std::strcmp(basePartId, "tail_spade") == 0
        ? "tail_spade_tip"
        : std::strcmp(basePartId, "tail_club") == 0 ? "tail_club_tip" : "tail_reptile_tip";
    CharacterError   error  = CharacterError::None;
    const InstanceId torso  = instanceByName("torso");
    const InstanceId tail   = character_->addPart(character_->self, characterHandle_, basePartId, "tail", &error);
    const InstanceId tailTip = tail != engine::character::invalidInstance
        ? character_->addPart(character_->self, characterHandle_, tipPartId, "tail_tip", &error)
        : engine::character::invalidInstance;
    if (torso == engine::character::invalidInstance || tail == engine::character::invalidInstance
        || tailTip == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    bool        connected   = character_->connect(
        character_->self, characterHandle_, torso, "tail.base", tail, "tail.root", nullptr, identity, &error);
    connected = character_->connect(
            character_->self, characterHandle_, tail, "tail.link", tailTip, "tail.end", nullptr, identity, &error)
        && connected;
    return connected;
}

bool CharacterScreen::removeTail()
{
    const InstanceId tailTip = instanceByName("tail_tip");
    const InstanceId tail    = instanceByName("tail");
    if (tail == engine::character::invalidInstance)
    {
        return true;
    }
    if (tailTip != engine::character::invalidInstance)
    {
        character_->removePart(character_->self, characterHandle_, tailTip);
    }
    return character_->removePart(character_->self, characterHandle_, tail);
}

bool CharacterScreen::addSecondTail()
{
    if (secondTailPresent())
    {
        return true;
    }
    CharacterError   error  = CharacterError::None;
    const InstanceId torso  = instanceByName("torso");
    const InstanceId tail2
        = character_->addPart(character_->self, characterHandle_, activeTailVariant_.c_str(), "tail2", &error);
    const char *tail2TipId = std::strcmp(activeTailVariant_.c_str(), "tail_spade") == 0
        ? "tail_spade_tip"
        : std::strcmp(activeTailVariant_.c_str(), "tail_club") == 0 ? "tail_club_tip" : "tail_reptile_tip";
    const InstanceId tail2Tip = tail2 != engine::character::invalidInstance
        ? character_->addPart(character_->self, characterHandle_, tail2TipId, "tail2_tip", &error)
        : engine::character::invalidInstance;
    if (torso == engine::character::invalidInstance || tail2 == engine::character::invalidInstance
        || tail2Tip == engine::character::invalidInstance)
    {
        return false;
    }
    // The pair spreads symmetrically around the spine (the animation driver
    // holds the base yaws; see updateAnimationDrivers). The tiny drop keeps
    // the second tail's root ball clear of the first inside the pelvis. The
    // base yaws are also applied right here so headless self-test runs see
    // the spread without waiting for the next animation frame.
    const float offset[3] = {0.0f, -0.02f, 0.0f};
    float       yaw[4];
    setYawRotation(yaw, 0.0f);
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    bool        connected   = character_->connect(
        character_->self, characterHandle_, torso, "tail.base", tail2, "tail.root", offset, yaw, &error);
    connected = character_->connect(
            character_->self, characterHandle_, tail2, "tail.link", tail2Tip, "tail.end", nullptr, identity, &error)
        && connected;
    if (connected)
    {
        const float halfSpread = 0.55f;
        float       leftYaw[4];
        float       rightYaw[4];
        setYawRotation(leftYaw, -halfSpread);
        setYawRotation(rightYaw, halfSpread);
        const InstanceId tail    = instanceByName("tail");
        const InstanceId tailTip = instanceByName("tail_tip");
        if (tail != engine::character::invalidInstance)
        {
            character_->setConnectionOffset(character_->self, characterHandle_, tail, nullptr, leftYaw);
        }
        if (tailTip != engine::character::invalidInstance)
        {
            character_->setConnectionOffset(character_->self, characterHandle_, tailTip, nullptr, leftYaw);
        }
        character_->setConnectionOffset(character_->self, characterHandle_, tail2, nullptr, rightYaw);
        character_->setConnectionOffset(character_->self, characterHandle_, tail2Tip, nullptr, rightYaw);
    }
    return connected;
}

bool CharacterScreen::removeSecondTail()
{
    const InstanceId tail2    = instanceByName("tail2");
    const InstanceId tail2Tip = instanceByName("tail2_tip");
    if (tail2 == engine::character::invalidInstance)
    {
        return true;
    }
    if (tail2Tip != engine::character::invalidInstance)
    {
        character_->removePart(character_->self, characterHandle_, tail2Tip);
    }
    return character_->removePart(character_->self, characterHandle_, tail2);
}

void CharacterScreen::applyParameterHeights()
{
    // Keep the soles near the ground plane: limb length scales the leg mesh
    // about its hip ring (part scale, ring stays on the anchor), so the
    // assembly root rises by the scaled leg drop below the hip. The leg drop
    // comes from the generated metrics (the pipeline measures the mesh).
    float limbLength = 1.0f;
    character_->getParameter(character_->self, characterHandle_, "limbs.length", &limbLength);
    const float legScale       = 1.0f + (limbLength - 1.0f);
    const float translation[3] = {0.0f, -kCharacterMetrics.legFootY * legScale - kCharacterMetrics.hipRing.y - 0.01f,
        0.0f};
    const float identity[4]    = {0.0f, 0.0f, 0.0f, 1.0f};
    character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
}

void CharacterScreen::resetToDefault()
{
    CharacterError error = CharacterError::None;
    character_->destroyAssembly(character_->self, characterHandle_);
    characterHandle_ = character_->createAssembly(character_->self, hybridProfileId, &error);
    if (characterHandle_ == engine::character::invalidCharacter)
    {
        std::fprintf(stderr, "demo: could not recreate the default assembly (%u)\n", static_cast<unsigned>(error));
        return;
    }
    activeHeadPart_    = "head_demon";
    activeTailVariant_ = "tail_reptile";
    applyParameterHeights();
    evaluateCharacter(true);
    setStatusMessage("Character reset to the Hybrid Biped profile");
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

void CharacterScreen::syncDeformation()
{
    // Push the morph-driving parameters into the content layer whenever they
    // moved; the content reuploads only the meshes whose weights changed.
    DeformationInput input;
    character_->getParameter(character_->self, characterHandle_, "body.height", &input.bodyHeight);
    character_->getParameter(character_->self, characterHandle_, "body.width", &input.bodyWidth);
    character_->getParameter(character_->self, characterHandle_, "body.muscle_mass", &input.bodyMuscle);
    character_->getParameter(character_->self, characterHandle_, "head.size", &input.headSize);
    character_->getParameter(character_->self, characterHandle_, "head.muzzle_length", &input.headMuzzle);
    character_->getParameter(character_->self, characterHandle_, "tail.curl", &input.tailCurl);
    if (deformationValid_ && std::memcmp(&input, &deformation_, sizeof(input)) == 0)
    {
        return;
    }
    content_.applyDeformation(*character_, characterHandle_, *render_, rendererHandle_, input);
    deformation_    = input;
    deformationValid_ = true;
}

// ---- input ---------------------------------------------------------------------

void CharacterScreen::handleWindowEvent(const WindowEvent &event)
{
    if (ui_ == nullptr || uiContext_ == engine::ui::invalidUiContext)
    {
        return;
    }
    if (event.kind == WindowEventKind::KeyPressed || event.kind == WindowEventKind::KeyReleased)
    {
        const UiKey uiKey = mapWindowKey(event.key);
        if (uiKey != UiKey::None)
        {
            ui_->addKey(ui_->self, uiContext_, uiKey, event.kind == WindowEventKind::KeyPressed);
        }
    }
    else if (event.kind == WindowEventKind::MouseButtonPressed || event.kind == WindowEventKind::MouseButtonReleased)
    {
        const std::uint32_t button = static_cast<std::uint32_t>(event.mouseButton);
        if (button < 3)
        {
            mouseButtons_[button] = event.kind == WindowEventKind::MouseButtonPressed;
            ui_->addMouseButton(ui_->self, uiContext_, button, mouseButtons_[button]);
        }
    }
    else if (event.kind == WindowEventKind::FocusLost)
    {
        orbitDragging_ = false;
    }
}

// ---- per-frame -------------------------------------------------------------------

void CharacterScreen::frame(double deltaSeconds, double elapsedSeconds)
{
    if (!created_ || character_ == nullptr || characterHandle_ == engine::character::invalidCharacter)
    {
        return;
    }

    // Mouse position + button backstop poll (covers backends that only
    // expose live state, and recovers from missed events).
    double     cursorX    = 0.0;
    double     cursorY    = 0.0;
    const bool haveCursor = window_->cursorPosition(window_->self, windowHandle_, &cursorX, &cursorY)
                         && std::isfinite(cursorX) && std::isfinite(cursorY);
    if (haveCursor)
    {
        ui_->addMousePosition(ui_->self, uiContext_, static_cast<float>(cursorX), static_cast<float>(cursorY));
    }
    for (std::uint32_t button = 0; button < 3; ++button)
    {
        const bool down = window_->isMouseButtonDown(window_->self, windowHandle_, static_cast<MouseButton>(button));
        if (down != mouseButtons_[button])
        {
            mouseButtons_[button] = down;
            ui_->addMouseButton(ui_->self, uiContext_, button, down);
        }
    }

    // Animation drivers, then a single evaluation pass per frame: drivers
    // edit connection offsets, evaluate() re-derives every part transform
    // and the bounds from the attachment tree. Validation is event-driven
    // (structure/parameter changes), not per-frame; morph deformation syncs
    // only when a slider actually moved.
    updateAnimationDrivers(deltaSeconds, elapsedSeconds);
    syncDeformation();
    evaluateCharacter();

    // Camera: orbit by dragging the viewport (never the panel), zoom with
    // Space/Ctrl, arrow keys orbit too.
    engine::camera::CameraInputState cameraInput{};
    const std::uint64_t              keys = window_->keyStateMask(window_->self,
        windowHandle_,
        keyMaskBit(Key::Space) | keyMaskBit(Key::LeftControl) | keyMaskBit(Key::Left) | keyMaskBit(Key::Right)
            | keyMaskBit(Key::Up) | keyMaskBit(Key::Down));
    cameraInput.zoom                      = static_cast<float>((keys & keyMaskBit(Key::Space)) != 0)
                                          - static_cast<float>((keys & keyMaskBit(Key::LeftControl)) != 0);
    cameraInput.lookHorizontal            = static_cast<float>((keys & keyMaskBit(Key::Right)) != 0)
                                          - static_cast<float>((keys & keyMaskBit(Key::Left)) != 0);
    cameraInput.lookVertical              = static_cast<float>((keys & keyMaskBit(Key::Up)) != 0)
                                          - static_cast<float>((keys & keyMaskBit(Key::Down)) != 0);

    const bool dragAllowed = mouseButtons_[0] && !uiWantsMouse_;
    if (dragAllowed && !orbitDragging_ && haveCursor)
    {
        orbitDragging_ = true;
    }
    if (orbitDragging_ && (!mouseButtons_[0] || uiWantsMouse_))
    {
        orbitDragging_ = false;
    }
    if (orbitDragging_ && haveCursor && haveLastCursor_)
    {
        cameraInput.mouseDeltaX = static_cast<float>(std::clamp(cursorX - lastCursorX_, -200.0, 200.0));
        cameraInput.mouseDeltaY = static_cast<float>(std::clamp(cursorY - lastCursorY_, -200.0, 200.0));
    }
    lastCursorX_    = cursorX;
    lastCursorY_    = cursorY;
    haveLastCursor_ = haveCursor;

    // Keep the orbit radius proportional to the character bounds.
    float boundsCenter[3] = {0.0f, 1.0f, 0.0f};
    float boundsRadius    = 1.0f;
    if (character_->bounds(character_->self, characterHandle_, boundsCenter, &boundsRadius))
    {
        camera_->setOrbitDistance(camera_->self, cameraHandle_, std::clamp(boundsRadius * 2.4f, 1.8f, 9.0f));
    }
    camera_->update(camera_->self, cameraHandle_, &cameraInput, deltaSeconds);
    std::uint32_t framebufferWidthForCamera  = 0;
    std::uint32_t framebufferHeightForCamera = 0;
    window_->getFramebufferSize(window_->self, windowHandle_, &framebufferWidthForCamera, &framebufferHeightForCamera);
    engine::camera::ViewState viewState;
    camera_->viewState(camera_->self, cameraHandle_, framebufferWidthForCamera, framebufferHeightForCamera, &viewState);
    engine::renderer::CameraMatrices cameraMatrices;
    std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
    std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
    std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
    render_->setCamera(render_->self, rendererHandle_, &cameraMatrices);

    // Renderer frame was begun by main(): draw the 3D scene first, then the
    // overlay panel, and finish the UI frame during submission. The sky and
    // sun draw last so their occluded pixels depth-reject instead of
    // shading (the CPU rasterizer's budget is shaded pixels).
    drawFloor();
    drawCharacter();
    drawSkyAndSun();

    // UI frame.
    std::uint32_t displayWidth      = 0;
    std::uint32_t displayHeight     = 0;
    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    window_->getSize(window_->self, windowHandle_, &displayWidth, &displayHeight);
    window_->getFramebufferSize(window_->self, windowHandle_, &framebufferWidth, &framebufferHeight);
    const float framebufferScaleX
        = displayWidth != 0 ? static_cast<float>(framebufferWidth) / static_cast<float>(displayWidth) : 1.0f;
    const float framebufferScaleY
        = displayHeight != 0 ? static_cast<float>(framebufferHeight) / static_cast<float>(displayHeight) : 1.0f;
    const UiFrameDescription uiFrame{
        .displayWidth      = static_cast<float>(displayWidth),
        .displayHeight     = static_cast<float>(displayHeight),
        .framebufferScaleX = framebufferScaleX,
        .framebufferScaleY = framebufferScaleY,
        .deltaSeconds      = static_cast<float>(std::clamp(deltaSeconds, 1.0e-3, 0.25))};
    if (!ui_->beginFrame(ui_->self, uiContext_, &uiFrame, nullptr))
    {
        return;
    }
    drawPanel(deltaSeconds);
    drawStatsOverlay();
    submitOverlay();
}

void CharacterScreen::updateAnimationDrivers(double deltaSeconds, double elapsedSeconds)
{
    (void)deltaSeconds;
    // Tail sway: connection offset rotations per segment. The module
    // re-derives the attached parts' transforms from the connectors, so
    // geometry and attachment points stay synchronized. With two tails the
    // pair spreads symmetrically around the spine (the fix for the old
    // overlap: both tails used to hang on the same yaw and intersect), so
    // each segment's offset rotation rides on its base yaw, not zero.
    const float spread = secondTailPresent() ? 0.55f : 0.0f;
    const struct
    {
        const char *name;
        float       phase;
        float       amplitude;
        float       baseYaw;
    } tailSegments[] = {
        {"tail", 0.0f, 0.20f, -spread},
        {"tail_tip", 0.9f, 0.30f, -spread},
        {"tail2", 1.4f, 0.18f, spread},
        {"tail2_tip", 2.2f, 0.26f, spread},
    };
    for (const auto &segment : tailSegments)
    {
        const InstanceId instance = instanceByName(segment.name);
        if (instance == engine::character::invalidInstance)
        {
            continue;
        }
        float sway = 0.0f;
        if (animateTail_ && tailSwaySpeed_ > 0.01f)
        {
            sway = std::sin(static_cast<float>(elapsedSeconds) * 1.7f * tailSwaySpeed_ + segment.phase)
                 * segment.amplitude * tailSwaySpeed_;
        }
        float rotation[4];
        setYawRotation(rotation, segment.baseYaw + sway);
        character_->setConnectionOffset(character_->self, characterHandle_, instance, nullptr, rotation);
    }

    // Wing flap: rotate the connection offset around the forward axis; the
    // right wing's socket is yaw-mirrored, so the sign flips.
    const float      flap     = animateWings_ ? std::sin(static_cast<float>(elapsedSeconds) * 2.4f) * 0.15f : 0.0f;
    const InstanceId wingLeft = instanceByName("wing_left");
    if (wingLeft != engine::character::invalidInstance)
    {
        float rotation[4];
        setRollRotation(rotation, flap);
        character_->setConnectionOffset(character_->self, characterHandle_, wingLeft, nullptr, rotation);
    }
    const InstanceId wingRight = instanceByName("wing_right");
    if (wingRight != engine::character::invalidInstance)
    {
        float rotation[4];
        setRollRotation(rotation, -flap);
        character_->setConnectionOffset(character_->self, characterHandle_, wingRight, nullptr, rotation);
    }
}

void CharacterScreen::drawCharacter()
{
    // skin.tone indexes a five-swatch palette (see the panel); the tint is a
    // draw-time material multiplier on the skin-tagged parts, so vertex
    // colors keep their shading detail underneath.
    float       skinTone = 0.0f;
    character_->getParameter(character_->self, characterHandle_, "skin.tone", &skinTone);
    static const glm::vec3 toneTints[5] = {
        {1.00f, 1.00f, 1.00f},
        {0.92f, 0.80f, 0.68f},
        {0.82f, 0.68f, 0.55f},
        {0.66f, 0.52f, 0.42f},
        {0.42f, 0.40f, 0.42f},
    };
    const int toneIndex = std::clamp(static_cast<int>(skinTone + 0.5f), 0, 4);
    const std::uint32_t count = character_->instanceCount(character_->self, characterHandle_);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const InstanceId id     = character_->instanceId(character_->self, characterHandle_, index);
        const char      *partId = nullptr;
        if (!character_->instanceInfo(character_->self, characterHandle_, id, nullptr, &partId, nullptr))
        {
            continue;
        }
        const PartRenderInfo *renderInfo = content_.renderInfoForPart(partId);
        if (renderInfo == nullptr)
        {
            continue;
        }
        const MeshHandle mesh = content_.meshForGeometry(renderInfo->geometryId);
        if (mesh == engine::renderer::invalidMesh)
        {
            continue;
        }
        float matrix[16] = {};
        if (!character_->instanceTransform(character_->self, characterHandle_, id, matrix))
        {
            continue;
        }
        MeshMaterial material = renderInfo->material;
        if (isSkinPart(partId))
        {
            const glm::vec3 &tint = toneTints[toneIndex];
            material.baseColor[0] *= tint.r;
            material.baseColor[1] *= tint.g;
            material.baseColor[2] *= tint.b;
        }
        render_->drawMesh(render_->self, rendererHandle_, mesh, matrix, &material);
    }
}

void CharacterScreen::drawFloor()
{
    static const MeshMaterial floorMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = engine::renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 0.92f,
        .metallicRoughnessTexture = engine::renderer::invalidTexture,
        .normalTexture            = engine::renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = engine::renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = engine::renderer::invalidTexture,
        .unlit                    = false,
    };
    const glm::mat4 identity(1.0f);
    if (floorMesh_ != engine::renderer::invalidMesh)
    {
        render_->drawMesh(render_->self, rendererHandle_, floorMesh_, glm::value_ptr(identity), &floorMaterial);
    }
}

void CharacterScreen::drawSkyAndSun()
{
    static const MeshMaterial skyMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = engine::renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = engine::renderer::invalidTexture,
        .normalTexture            = engine::renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = engine::renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = engine::renderer::invalidTexture,
        .unlit                    = true,
    };
    static const MeshMaterial sunMaterial{
        .baseColor                = {4.0f, 3.6f, 3.0f},
        .baseColorTexture         = engine::renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = engine::renderer::invalidTexture,
        .normalTexture            = engine::renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = engine::renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = engine::renderer::invalidTexture,
        .unlit                    = true,
    };

    const glm::mat4 identity(1.0f);
    if (skyDomeMesh_ != engine::renderer::invalidMesh)
    {
        render_->drawMesh(render_->self, rendererHandle_, skyDomeMesh_, glm::value_ptr(identity), &skyMaterial);
    }
    if (sunDiscMesh_ != engine::renderer::invalidMesh)
    {
        render_->drawMesh(render_->self, rendererHandle_, sunDiscMesh_, glm::value_ptr(identity), &sunMaterial);
    }
}

// ---- UI ---------------------------------------------------------------------

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
    uiTextColored(0.60f, 0.75f, 0.98f, "Hybrid Biped profile");
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
            const InstanceId head = instanceByName("head");
            if (head != engine::character::invalidInstance)
            {
                CharacterError error = CharacterError::None;
                if (character_->replacePart(character_->self, characterHandle_, head, partId.c_str(), &error))
                {
                    activeHeadPart_ = partId;
                    setStatusMessage(std::string("Head replaced with ") + info->displayName);
                    evaluateCharacter(true);
                }
                else
                {
                    setStatusMessage("Head replacement rejected");
                }
            }
        }
        // three variant buttons per row keeps every slot variant on-panel
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
        "body.height", "body.width", "body.muscle_mass", "head.size", "head.muzzle_length", "neck.length",
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

    // ---- appendages ------------------------------------------------------------
    uiHeading("Appendages");
    bool wings = wingsPresent();
    if (uiCheckbox("Wings", &wings))
    {
        setStatusMessage(wings ? (addWings() ? "Wings attached at the spine connectors" : "Wing attachment failed")
                               : (removeWings() ? "Wings removed" : "Wing removal failed"));
        applyParameterHeights();
        evaluateCharacter(true);
    }
    bool tail = instanceByName("tail") != engine::character::invalidInstance;
    if (uiCheckbox("Tail", &tail))
    {
        if (tail)
        {
            setStatusMessage(addTail(activeTailVariant_.c_str()) ? "Tail attached" : "Tail attachment failed");
        }
        else
        {
            setStatusMessage(removeTail() ? "Tail removed" : "Tail removal failed");
        }
        applyParameterHeights();
        evaluateCharacter(true);
    }
    // Tail variants swap the base and the tip instances together so the chain
    // keeps one look.
    uiHeading("Tail Style");
    const struct
    {
        const char *label;
        const char *baseId;
        const char *tipId;
    } tailVariants[] = {
        {"Reptile", "tail_reptile", "tail_reptile_tip"},
        {"Spade", "tail_spade", "tail_spade_tip"},
        {"Club", "tail_club", "tail_club_tip"},
    };
    std::size_t variantIndex = 0;
    for (const auto &variant : tailVariants)
    {
        ++variantIndex;
        char label[48];
        std::snprintf(label, sizeof(label), "%s%s", activeTailVariant_ == variant.baseId ? "* " : "", variant.label);
        if (uiButton(label, 62.0f))
        {
            const InstanceId tail    = instanceByName("tail");
            const InstanceId tailTip = instanceByName("tail_tip");
            bool             swapped = tail != engine::character::invalidInstance;
            CharacterError   error   = CharacterError::None;
            if (swapped)
            {
                swapped = character_->replacePart(character_->self, characterHandle_, tail, variant.baseId, &error);
            }
            if (swapped && tailTip != engine::character::invalidInstance)
            {
                swapped = character_->replacePart(character_->self, characterHandle_, tailTip, variant.tipId, &error);
            }
            if (swapped)
            {
                activeTailVariant_ = variant.baseId;
                setStatusMessage(std::string("Tail style set to ") + variant.label);
            }
            else
            {
                setStatusMessage("Tail variant replacement rejected");
            }
            evaluateCharacter(true);
        }
        if (variantIndex % 3 == 2)
        {
            ui_->spacing(ui_->self, uiContext_);
        }
        else
        {
            ui_->sameLine(ui_->self, uiContext_);
        }
    }
    {
        const ParameterDefinition *spanDefinition = parameterDefinition("wings.span");
        const float spanMinimum = spanDefinition != nullptr ? spanDefinition->minimum : 0.40f;
        const float spanMaximum = spanDefinition != nullptr ? spanDefinition->maximum : 1.60f;
        float wingSpan = 1.0f;
        character_->getParameter(character_->self, characterHandle_, "wings.span", &wingSpan);
        const float before = wingSpan;
        if (uiSlider("Wingspan", &wingSpan, spanMinimum, spanMaximum) && std::fabs(wingSpan - before) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "wings.span", wingSpan, &error);
        }
        const ParameterDefinition *tailLengthDefinition = parameterDefinition("tail.length");
        const float tailLengthMinimum = tailLengthDefinition != nullptr ? tailLengthDefinition->minimum : 0.45f;
        const float tailLengthMaximum = tailLengthDefinition != nullptr ? tailLengthDefinition->maximum : 1.75f;
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
        const ParameterDefinition *curlDefinition = parameterDefinition("tail.curl");
        const float curlMinimum = curlDefinition != nullptr ? curlDefinition->minimum : -1.0f;
        const float curlMaximum = curlDefinition != nullptr ? curlDefinition->maximum : 1.0f;
        float tailCurl = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "tail.curl", &tailCurl);
        const float curlBefore = tailCurl;
        if (uiSlider("Tail Curl", &tailCurl, curlMinimum, curlMaximum) && std::fabs(tailCurl - curlBefore) > 1.0e-6f)
        {
            CharacterError error = CharacterError::None;
            character_->setParameter(character_->self, characterHandle_, "tail.curl", tailCurl, &error);
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
    if (uiButton(secondTailPresent() ? "Remove Second Tail" : "Add Second Tail", 220.0f))
    {
        const bool wasPresent = secondTailPresent();
        const bool ok         = wasPresent ? removeSecondTail() : addSecondTail();
        setStatusMessage(ok ? (wasPresent ? "Second tail detached" : "Second tail attached (connector capacity 2)")
                            : "Second tail operation failed");
        applyParameterHeights();
        evaluateCharacter(true);
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
    const InstanceId head    = instanceByName("head");
    bool             applied = true;

    if (name == "dragon" || name == "serpent" || name == "orc" || name == "yeti" || name == "alien"
        || name == "bluedemon")
    {
        // Head variant swap: the head slot's alternative assets.
        const char *partId = name == "dragon"     ? "head_dragon"
                           : name == "serpent"    ? "head_serpent"
                           : name == "orc"        ? "head_orc"
                           : name == "yeti"       ? "head_yeti"
                           : name == "bluedemon"  ? "head_bluedemon"
                                                  : "head_alien";
        applied            = head != engine::character::invalidInstance
                          && character_->replacePart(character_->self, characterHandle_, head, partId, &error);
        activeHeadPart_    = partId;
    }
    else if (name == "spadetail" || name == "clubtail")
    {
        // Tail variant swap: base and tip instances move together.
        const char *baseId = name == "spadetail" ? "tail_spade" : "tail_club";
        const char *tipId  = name == "spadetail" ? "tail_spade_tip" : "tail_club_tip";
        const InstanceId tail = instanceByName("tail");
        const InstanceId tailTip = instanceByName("tail_tip");
        applied               = tail != engine::character::invalidInstance
                   && character_->replacePart(character_->self, characterHandle_, tail, baseId, &error);
        if (applied && tailTip != engine::character::invalidInstance)
        {
            applied = character_->replacePart(character_->self, characterHandle_, tailTip, tipId, &error);
        }
        activeTailVariant_ = baseId;
    }
    else if (name == "curltail")
    {
        // The tail.curl parameter bends the baked tail mesh; the tail_end
        // anchor follows through the matching joint-translation binding.
        applied = setParameter("tail.curl", 1.0f);
    }
    else if (name == "tall")
    {
        // Height range ends: the neck/head attachment must track the scaled
        // torso through the parameter bindings, with the longest neck.
        applied = setParameter("body.height", 1.35f) && setParameter("limbs.length", 1.35f)
               && setParameter("neck.length", 1.3f);
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
        // Tail length range end plus a second tail spread symmetrically.
        applied = setParameter("tail.length", 1.75f) && setParameter("tail.thickness", 1.65f)
               && setParameter("tail.curl", 0.8f) && addSecondTail();
    }
    else if (name == "compact")
    {
        // Everything small at once: the opposite range end of every parameter.
        applied = setParameter("body.height", 0.75f) && setParameter("body.width", 0.70f)
               && setParameter("limbs.length", 0.75f) && setParameter("head.size", 0.70f)
               && setParameter("tail.length", 0.45f) && setParameter("wings.span", 0.40f)
               && setParameter("neck.length", 0.80f);
        applyParameterHeights();
    }
    else if (name == "noappend")
    {
        // Appendage removal on both slots.
        applied               = removeWings() && removeSecondTail();
        const InstanceId tail = instanceByName("tail");
        if (tail != engine::character::invalidInstance)
        {
            const InstanceId tailTip = instanceByName("tail_tip");
            if (tailTip != engine::character::invalidInstance)
            {
                character_->removePart(character_->self, characterHandle_, tailTip);
            }
            applied = character_->removePart(character_->self, characterHandle_, tail) && applied;
        }
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

    std::printf("character-test: hybrid character assembly test (independently created assets)\n");

    // 0. The default profile assembles and validates.
    check("default profile assembles and validates", evaluateCharacter(true) && issues_.empty());

    // 1. Replace the demon head with another head (connection survives).
    {
        const InstanceId head  = instanceByName("head");
        CharacterError   error = CharacterError::None;
        const bool replaced = head != engine::character::invalidInstance
            && character_->replacePart(character_->self, characterHandle_, head, "head_dragon", &error);
        check("replace demon head with dragon head", replaced && evaluateCharacter(true) && issues_.empty());
    }

    // 2. Change leg proportions.
    {
        CharacterError error = CharacterError::None;
        const bool set = character_->setParameter(character_->self, characterHandle_, "limbs.length", 1.35f, &error);
        applyParameterHeights();
        float boundsCenter[3] = {0.0f, 0.0f, 0.0f};
        float boundsRadius    = 0.0f;
        check("change leg proportions (limbs.length=1.35)",
            set && evaluateCharacter(true)
                && character_->bounds(character_->self, characterHandle_, boundsCenter, &boundsRadius)
                && boundsRadius > 0.8f);
    }

    // 2b. Swap through every head variant (the three revision-3 heads plus
    // the three revision-4 cuts) and both tail variants.
    {
        const InstanceId head    = instanceByName("head");
        const InstanceId tail    = instanceByName("tail");
        const InstanceId tailTip = instanceByName("tail_tip");
        CharacterError   error   = CharacterError::None;
        bool             allOk   = head != engine::character::invalidInstance;
        for (const char *partId : {"head_orc", "head_yeti", "head_alien", "head_bluedemon", "head_demon"})
        {
            allOk = allOk && character_->replacePart(character_->self, characterHandle_, head, partId, &error)
                && evaluateCharacter(true) && issues_.empty();
        }
        allOk = allOk && tail != engine::character::invalidInstance
            && character_->replacePart(character_->self, characterHandle_, tail, "tail_spade", &error)
            && tailTip != engine::character::invalidInstance
            && character_->replacePart(character_->self, characterHandle_, tailTip, "tail_spade_tip", &error)
            && evaluateCharacter(true) && issues_.empty();
        allOk = allOk && character_->replacePart(character_->self, characterHandle_, tail, "tail_club", &error)
            && character_->replacePart(character_->self, characterHandle_, tailTip, "tail_club_tip", &error)
            && evaluateCharacter(true) && issues_.empty();
        allOk = allOk && character_->replacePart(character_->self, characterHandle_, tail, "tail_reptile", &error)
            && character_->replacePart(character_->self, characterHandle_, tailTip, "tail_reptile_tip", &error)
            && evaluateCharacter(true) && issues_.empty();
        activeHeadPart_    = "head_demon";
        activeTailVariant_ = "tail_reptile";
        check("swap every head and tail variant", allOk);
    }

    // 2c. The full widened-range extreme sweep: every parameter at once must
    // stay junction-safe (the revision-4 validated ranges).
    {
        const auto setParameterSafe = [&](const char *parameter, float value)
        {
            CharacterError parameterError = CharacterError::None;
            return character_->setParameter(character_->self, characterHandle_, parameter, value, &parameterError);
        };
        CharacterError error = CharacterError::None;
        const bool set = setParameterSafe("body.height", 1.35f)
            && setParameterSafe("body.width", 1.45f)
            && setParameterSafe("body.muscle_mass", 1.55f)
            && setParameterSafe("head.size", 1.50f)
            && setParameterSafe("head.muzzle_length", 1.60f)
            && setParameterSafe("neck.length", 1.30f)
            && setParameterSafe("limbs.length", 1.35f)
            && setParameterSafe("limbs.thickness", 1.55f)
            && setParameterSafe("tail.length", 1.75f)
            && setParameterSafe("tail.thickness", 1.65f)
            && setParameterSafe("tail.curl", 1.0f)
            && setParameterSafe("wings.span", 1.60f)
            && setParameterSafe("skin.tone", 4.0f);
        applyParameterHeights();
        check("every parameter at its range end validates", set && evaluateCharacter(true) && issues_.empty());
        // restore defaults for the remaining checks
        for (const char *parameter : {"body.height", "body.width", "body.muscle_mass", "head.size",
                 "head.muzzle_length", "neck.length", "limbs.length", "limbs.thickness", "tail.length",
                 "tail.thickness", "wings.span", "skin.tone"})
        {
            character_->setParameter(character_->self, characterHandle_, parameter, 1.0f, &error);
        }
        character_->setParameter(character_->self, characterHandle_, "tail.curl", 0.0f, &error);
        applyParameterHeights();
        evaluateCharacter(true);
    }

    // 3. Add a second tail (connector capacity 2 + offset correction); the
    // pair must spread symmetrically instead of hanging on the same yaw.
    {
        const bool added = addSecondTail();
        evaluateCharacter(true);
        float tailTipMatrix[16] = {};
        float tail2TipMatrix[16] = {};
        const InstanceId tailTip    = instanceByName("tail_tip");
        const InstanceId tail2TipId = instanceByName("tail2_tip");
        const bool transformed = added && tailTip != engine::character::invalidInstance
            && tail2TipId != engine::character::invalidInstance
            && character_->instanceTransform(character_->self, characterHandle_, tailTip, tailTipMatrix)
            && character_->instanceTransform(character_->self, characterHandle_, tail2TipId, tail2TipMatrix);
        const float separation = transformed
            ? std::sqrt((tailTipMatrix[12] - tail2TipMatrix[12]) * (tailTipMatrix[12] - tail2TipMatrix[12])
                + (tailTipMatrix[13] - tail2TipMatrix[13]) * (tailTipMatrix[13] - tail2TipMatrix[13])
                + (tailTipMatrix[14] - tail2TipMatrix[14]) * (tailTipMatrix[14] - tail2TipMatrix[14]))
            : 0.0f;
        // The tips ride their base yaws (±0.55 rad) - the old bug hung both
        // tails on the same transform and they intersected.
        check("add a second tail (capacity 2) with symmetric spread",
            added && evaluateCharacter(true) && issues_.empty() && separation > 0.15f);
    }

    // 4. Remove both wings.
    {
        const bool removed = removeWings();
        check("remove both wings", removed && evaluateCharacter(true) && issues_.empty());
    }

    // 5. Change torso width.
    {
        CharacterError error = CharacterError::None;
        const bool set = character_->setParameter(character_->self, characterHandle_, "body.width", 1.45f, &error);
        check("change torso width (body.width=1.45)", set && evaluateCharacter(true) && issues_.empty());
    }

    // 6. Animate the tail independently: connection offset must move the tip.
    {
        const InstanceId tailTip    = instanceByName("tail_tip");
        float            before[16] = {};
        const bool hadTransform = tailTip != engine::character::invalidInstance
            && character_->instanceTransform(character_->self, characterHandle_, tailTip, before);
        float      yaw[4];
        setYawRotation(yaw, 0.6f);
        character_->setConnectionOffset(character_->self, characterHandle_, tailTip, nullptr, yaw);
        evaluateCharacter();
        float      after[16] = {};
        const bool moved     = hadTransform
                            && character_->instanceTransform(character_->self, characterHandle_, tailTip, after)
                            && std::memcmp(before, after, sizeof(before)) != 0;
        check("animate the tail independently (connection offset)", moved);
        setYawRotation(yaw, 0.0f);
        character_->setConnectionOffset(character_->self, characterHandle_, tailTip, nullptr, yaw);
    }

    // 7. Save and reload the configuration.
    {
        CharacterError error  = CharacterError::None;
        const bool     saved  = character_->saveRecipe(character_->self, characterHandle_, kRecipeFileName, &error);
        float          before = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "body.width", &before);
        const bool loaded
            = saved && character_->loadRecipe(character_->self, characterHandle_, kRecipeFileName, &error);
        float after = 0.0f;
        character_->getParameter(character_->self, characterHandle_, "body.width", &after);
        check("save and reload the configuration", loaded && std::fabs(before - after) < 1.0e-5f);
    }

    // 8. Connector capacity is enforced: a third tail must be rejected.
    {
        CharacterError   error = CharacterError::None;
        const InstanceId tail3
            = character_->addPart(character_->self, characterHandle_, "tail_reptile", "tail3", &error);
        const InstanceId torso = instanceByName("torso");
        const bool       rejected
            = tail3 != engine::character::invalidInstance && torso != engine::character::invalidInstance
           && !character_->connect(
               character_->self, characterHandle_, torso, "tail.base", tail3, "tail.root", nullptr, nullptr, &error);
        if (tail3 != engine::character::invalidInstance)
        {
            character_->removePart(character_->self, characterHandle_, tail3);
        }
        check("connector capacity enforced (third tail rejected)", rejected);
    }

    // 9. Incompatible attachments are rejected by validation.
    {
        CharacterError   error = CharacterError::None;
        const InstanceId leg   = instanceByName("leg_left");
        const InstanceId head  = instanceByName("head");
        const bool       rejected
            = leg != engine::character::invalidInstance && head != engine::character::invalidInstance
           && !character_->connect(
               character_->self, characterHandle_, leg, "hip.root", head, "head.root", nullptr, nullptr, &error);
        check("incompatible connection rejected (head on leg)", rejected);
    }

    // 9b. Junction contract under parameter extremes: a large head on a wide,
    // muscular torso must keep the head seated on the neck (the historic
    // "head neck wider than torso neck" defect). The head's baked neck_fit
    // morph and the torso's neck-band morph share one junction radius, so
    // the head-to-neck distance stays the authored neck length.
    {
        CharacterError error = CharacterError::None;
        const bool set = character_->setParameter(character_->self, characterHandle_, "head.size", 1.50f, &error)
            && character_->setParameter(character_->self, characterHandle_, "body.width", 1.45f, &error)
            && character_->setParameter(character_->self, characterHandle_, "body.muscle_mass", 1.55f, &error);
        const InstanceId head = instanceByName("head");
        const InstanceId neck = instanceByName("neck");
        float headMatrix[16] = {};
        float neckMatrix[16] = {};
        const bool transformed = set && head != engine::character::invalidInstance
            && neck != engine::character::invalidInstance
            && evaluateCharacter(true) && issues_.empty()
            && character_->instanceTransform(character_->self, characterHandle_, head, headMatrix)
            && character_->instanceTransform(character_->self, characterHandle_, neck, neckMatrix);
        const float separation = transformed
            ? std::sqrt((headMatrix[12] - neckMatrix[12]) * (headMatrix[12] - neckMatrix[12])
                + (headMatrix[13] - neckMatrix[13]) * (headMatrix[13] - neckMatrix[13])
                + (headMatrix[14] - neckMatrix[14]) * (headMatrix[14] - neckMatrix[14]))
            : 0.0f;
        // The neck piece spans 0.205 local units scaled by body height; the
        // head ring sits exactly at its top anchor at any junction radius.
        const float expectedSpan = 0.205f * (1.0f + (1.25f - 1.0f) * 0.0f);
        check("junction contract: big head stays seated on wide torso (extremes)",
            transformed && std::fabs(separation - expectedSpan) < 0.02f + expectedSpan * 0.05f);
        // Restore the defaults for the remaining checks.
        character_->setParameter(character_->self, characterHandle_, "head.size", 1.0f, &error);
        character_->setParameter(character_->self, characterHandle_, "body.width", 1.0f, &error);
        character_->setParameter(character_->self, characterHandle_, "body.muscle_mass", 1.0f, &error);
        evaluateCharacter(true);
    }

    // 10. Locomotion/physics: no such subsystem ships with this engine build.
    std::printf("  SKIP  run locomotion and physics (no physics module in this engine build)\n");
    ++skipped;

    std::printf("character-test: %u passed, %u failed, %u skipped\n", passed, failed, skipped);

    // Restore the pristine default character for the visible session.
    resetToDefault();
    return failed == 0;
}

} // namespace demo
