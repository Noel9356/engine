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
// heads, tails, and wings keep their authored palettes; the human head joins
// the skin group so it reads as the character's own skin).
bool isSkinPart(const char *partId)
{
    static const char *skinParts[] = {
        "torso_humanoid", "arm_humanoid_left", "arm_humanoid_right", "leg_humanoid_left", "leg_humanoid_right",
        "neck_transition", "head_human",
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

// Rest pitches of the two tail pieces are BAKED into their OBJs at
// extraction (see scripts/extract_parts_verbatim.py), so the connection
// rotations stay identity and the sway driver only adds a yaw.

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

    // Default character from the humanoid body-plan profile.
    CharacterError        error  = CharacterError::None;
    const CharacterHandle handle = character_->createAssembly(character_->self, humanoidProfileId, &error);
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
    activePlan_      = 0;
    headCount_       = 1;
    wingPairs_       = 0;
    armPairs_        = 1;
    tailPresent_     = false;

    // Orbit camera around the character: three-quarter view from the
    // character's front-right so the face stays visible and the tail reads
    // against the sky instead of hiding behind the torso.
    configureCamera(52.0f, 10.0f, 2.6f, 1.0f);

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

void CharacterScreen::setUserCameraFraming(bool user)
{
    userCameraFraming_ = user;
}

void CharacterScreen::configureCamera(float yawDegrees, float pitchDegrees, float orbitDistance, float targetY)
{
    const float yawRadians   = glm::radians(yawDegrees);
    const float pitchRadians = glm::radians(std::clamp(pitchDegrees, -80.0f, 80.0f));
    const glm::vec3 target(0.0f, targetY, 0.0f);
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

// ---- assembly helpers (body plans, counts, variants) -------------------------

bool CharacterScreen::rebuildAssembly()
{
    // (Re)create the assembly from the active body-plan profile and re-apply
    // the current counts/variants. Plan changes alter the skeleton topology,
    // so the assembly is rebuilt; counts and variants re-attach afterwards.
    CharacterError error = CharacterError::None;
    if (characterHandle_ != engine::character::invalidCharacter)
    {
        character_->destroyAssembly(character_->self, characterHandle_);
        characterHandle_ = engine::character::invalidCharacter;
    }
    characterHandle_ = character_->createAssembly(character_->self, profileIdForPlan(activePlan_), &error);
    if (characterHandle_ == engine::character::invalidCharacter)
    {
        std::fprintf(stderr, "demo: could not create the %s assembly (%u)\n", profileIdForPlan(activePlan_),
            static_cast<unsigned>(error));
        return false;
    }
    const int heads = headCount_;
    const int wings = wingPairs_;
    const int arms = armPairs_;
    const bool tail = tailPresent_;
    headCount_ = 1;
    wingPairs_ = 0;
    armPairs_ = 1;
    tailPresent_ = false;
    setHeadCount(heads);
    setWingPairs(wings);
    setArmPairs(arms);
    setTail(tail && activePlan_ != 2 && activePlan_ != 3);
    applyParameterHeights();
    evaluateCharacter(true);
    return true;
}

bool CharacterScreen::setPlan(int plan)
{
    if (plan == activePlan_ || plan < 0 || plan > 3)
    {
        return true;
    }
    activePlan_ = plan;
    // Extra arm pairs and the reptile tail do not graft onto the quadruped
    // plan; the naga carries the serpent lower body instead of a tail.
    if (activePlan_ == 3)
    {
        armPairs_ = 1;
        tailPresent_ = false;
    }
    return rebuildAssembly();
}

bool CharacterScreen::setQuadSpecies(int species)
{
    if (species == quadSpecies_ || species < 0 || species > 3)
    {
        return true;
    }
    quadSpecies_ = species;
    static const char *const kQuadBodies[4] = {
        "quad_horse_body", "quad_wolf_body", "quad_deer_body", "quad_bull_body"};
    static const char *const kQuadCentaurs[4] = {
        "quad_horse_centaur", "quad_wolf_centaur", "quad_deer_centaur", "quad_bull_centaur"};
    const char *bodyId = activePlan_ == 1 ? kQuadCentaurs[species] : kQuadBodies[species];
    const InstanceId body = instanceByName("body");
    if (body == engine::character::invalidInstance)
    {
        return false;
    }
    CharacterError error = CharacterError::None;
    const bool replaced = character_->replacePart(
        character_->self, characterHandle_, body, bodyId, &error);
    applyParameterHeights();
    evaluateCharacter(true);
    return replaced;
}

bool CharacterScreen::setHeadCount(int count)
{
    count = std::clamp(count, 1, 3);
    CharacterError error = CharacterError::None;
    bool ok = true;
    // Head 1 comes from the profile; heads 2/3 attach beside it with an
    // outward roll so the pair reads as two skulls sharing the collar.
    while (headCount_ > count)
    {
        const InstanceId extra = instanceByName(headCount_ == 3 ? "head3" : "head2");
        if (extra != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, extra) && ok;
        }
        headCount_ -= 1;
    }
    while (headCount_ < count)
    {
        headCount_ += 1;
        InstanceId parent = instanceByName(activePlan_ == 0 || activePlan_ == 2 ? "torso" : "chest");
        if (parent == engine::character::invalidInstance)
        {
            parent = instanceByName("body");
        }
        const InstanceId head = character_->addPart(character_->self, characterHandle_, activeHeadPart_.c_str(),
            headCount_ == 2 ? "head2" : "head3", &error);
        if (parent == engine::character::invalidInstance || head == engine::character::invalidInstance)
        {
            ok = false;
            break;
        }
        const float lateral = headCount_ == 2 ? -0.170f : 0.170f;
        const float offset[3] = {lateral, 0.012f, -0.010f};
        float roll[4];
        setRollRotation(roll, headCount_ == 2 ? -16.0f : 16.0f);
        ok = character_->connect(character_->self, characterHandle_, parent, "neck.base", head, "head.root",
            offset, roll, &error)
            && ok;
    }
    if (ok)
    {
        evaluateCharacter(true);
    }
    return ok;
}

bool CharacterScreen::setWingPairs(int pairs)
{
    pairs = std::clamp(pairs, 0, 3);
    CharacterError error = CharacterError::None;
    (void)error;
    bool ok = true;
    while (wingPairs_ > pairs)
    {
        const std::string leftName = "wing" + std::to_string(wingPairs_) + "_left";
        const std::string rightName = "wing" + std::to_string(wingPairs_) + "_right";
        const InstanceId left = instanceByName(leftName);
        const InstanceId right = instanceByName(rightName);
        if (left != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, left) && ok;
        }
        if (right != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, right) && ok;
        }
        wingPairs_ -= 1;
    }
    while (wingPairs_ < pairs)
    {
        wingPairs_ += 1;
        ok = addWingPairToSlot(wingPairs_) && ok;
    }
    if (ok)
    {
        evaluateCharacter(true);
    }
    return ok;
}

bool CharacterScreen::addWingPairToSlot(int slot)
{
    CharacterError error = CharacterError::None;
    InstanceId parent = instanceByName(activePlan_ == 0 || activePlan_ == 2 ? "torso" : "chest");
    if (parent == engine::character::invalidInstance)
    {
        parent = instanceByName("body");
    }
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    char leftName[16];
    char rightName[16];
    std::snprintf(leftName, sizeof(leftName), "wing%d_left", slot);
    std::snprintf(rightName, sizeof(rightName), "wing%d_right", slot);
    char connector[2][16];
    std::snprintf(connector[0], sizeof(connector[0]), "wings%d.left", slot);
    std::snprintf(connector[1], sizeof(connector[1]), "wings%d.right", slot);
    const InstanceId left = character_->addPart(
        character_->self, characterHandle_, "wing_dragon_left", leftName, &error);
    const InstanceId right = left != engine::character::invalidInstance
        ? character_->addPart(character_->self, characterHandle_, "wing_dragon_right", rightName, &error)
        : engine::character::invalidInstance;
    if (left == engine::character::invalidInstance || right == engine::character::invalidInstance)
    {
        return false;
    }
    // Stacked pairs share the same wing geometry, so each slot rolls a
    // little further outward - otherwise three 1.55 m wings intersect into
    // one shark-fin wall (rev-7 frames 08/09).
    const float rollL = slot == 1 ? 0.0f : slot == 2 ? -12.0f : -22.0f;
    const float rollR = -rollL;
    float leftRoll[4];
    float rightRoll[4];
    setRollRotation(leftRoll, rollL);
    setRollRotation(rightRoll, rollR);
    bool connected = character_->connect(
        character_->self, characterHandle_, parent, connector[0], left, "wing.root", nullptr, leftRoll, &error);
    connected = character_->connect(
                   character_->self, characterHandle_, parent, connector[1], right, "wing.root", nullptr, rightRoll,
                   &error)
        && connected;
    return connected;
}

bool CharacterScreen::setArmPairs(int pairs)
{
    pairs = std::clamp(pairs, 1, 3);
    if (activePlan_ == 3)
    {
        pairs = 1; // the quadruped plan keeps the animal's own four limbs
    }
    CharacterError error = CharacterError::None;
    bool ok = true;
    while (armPairs_ > pairs)
    {
        const std::string leftName = "arm" + std::to_string(armPairs_) + "_left";
        const std::string rightName = "arm" + std::to_string(armPairs_) + "_right";
        const InstanceId left = instanceByName(leftName);
        const InstanceId right = instanceByName(rightName);
        if (left != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, left) && ok;
        }
        if (right != engine::character::invalidInstance)
        {
            ok = character_->removePart(character_->self, characterHandle_, right) && ok;
        }
        armPairs_ -= 1;
    }
    while (armPairs_ < pairs)
    {
        armPairs_ += 1;
        const int slot = armPairs_; // 2 or 3
        const InstanceId parent = instanceByName(activePlan_ == 0 || activePlan_ == 2 ? "torso" : "chest");
        if (parent == engine::character::invalidInstance)
        {
            ok = false;
            break;
        }
        const std::string leftName = "arm" + std::to_string(slot) + "_left";
        const std::string rightName = "arm" + std::to_string(slot) + "_right";
        const std::string connectorLeft = "arm" + std::to_string(slot) + ".left";
        const std::string connectorRight = "arm" + std::to_string(slot) + ".right";
        const InstanceId left = character_->addPart(
            character_->self, characterHandle_, "arm_human_left", leftName.c_str(), &error);
        const InstanceId right = left != engine::character::invalidInstance
            ? character_->addPart(character_->self, characterHandle_, "arm_human_right", rightName.c_str(), &error)
            : engine::character::invalidInstance;
        if (left == engine::character::invalidInstance || right == engine::character::invalidInstance)
        {
            ok = false;
            break;
        }
        // Extra pairs fan OUTWARD in the coronal plane. The rev-7 build
        // yawed them about the vertical axis, which only spins a hanging
        // arm in place - the pairs were invisible. Roll about the forward
        // axis actually swings the hanging arms away from the torso
        // (marilith-style fan).
        float leftPose[4];
        float rightPose[4];
        setRollRotation(leftPose, slot == 2 ? -26.0f : -42.0f);
        setRollRotation(rightPose, slot == 2 ? 26.0f : 42.0f);
        bool connected = character_->connect(character_->self, characterHandle_, parent, connectorLeft.c_str(), left,
            "shoulder.root", nullptr, leftPose, &error);
        connected = character_->connect(character_->self, characterHandle_, parent, connectorRight.c_str(), right,
            "shoulder.root", nullptr, rightPose, &error)
            && connected;
        ok = connected && ok;
    }
    if (ok)
    {
        evaluateCharacter(true);
    }
    return ok;
}

bool CharacterScreen::setTail(bool present)
{
    tailPresent_ = present;
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
    if (tail != engine::character::invalidInstance || activePlan_ == 2 || activePlan_ == 3)
    {
        return true; // already present, or the plan carries its own tail
    }
    CharacterError error = CharacterError::None;
    (void)error;
    const InstanceId parent = instanceByName(activePlan_ == 1 ? "chest" : "torso");
    if (parent == engine::character::invalidInstance)
    {
        return false;
    }
    const InstanceId tailPart = character_->addPart(
        character_->self, characterHandle_, "tail_reptile", "tail", &error);
    (void)error;
    if (tailPart == engine::character::invalidInstance)
    {
        return false;
    }
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const bool connected = character_->connect(
        character_->self, characterHandle_, parent, "tail.base", tailPart, "tail.root", nullptr, identity, &error);
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
    // sole offset below the anchor. Quadruped and centaur plans ground on the
    // animal body instead (the measured hooves' lowest point below the stub).
    const float identity[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float limbLength = 1.0f;
    character_->getParameter(character_->self, characterHandle_, "limbs.length", &limbLength);
    float height = 1.0f;
    character_->getParameter(character_->self, characterHandle_, "body.height", &height);
    if (activePlan_ == 3)
    {
        static const float kGround[4] = {kCharacterMetrics.quadHorseGroundY, kCharacterMetrics.quadWolfGroundY,
            kCharacterMetrics.quadDeerGroundY, kCharacterMetrics.quadBullGroundY};
        const float ground = kGround[std::clamp(quadSpecies_, 0, 3)] * height;
        const float translation[3] = {0.0f, -ground - 0.01f, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    if (activePlan_ == 1)
    {
        // Centaur: ground on the (headless) horse body - its hooves define
        // the lowest point below the Body-joint origin.
        const float ground = kCharacterMetrics.quadHorseGroundY * height;
        const float translation[3] = {0.0f, -ground - 0.01f, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    if (activePlan_ == 2)
    {
        // Naga: ground on the pitched serpent lower body. The serpent piece
        // is pitched -102 deg in the profile; its lowest point after that
        // rotation sits serpentGroundY below the root anchor (measured).
        const float translation[3] = {0.0f, -kCharacterMetrics.serpentGroundY * height - 0.01f, 0.0f};
        character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
        return;
    }
    const float legScale = 1.0f + (limbLength - 1.0f);
    // legFootY is the sole's offset BELOW the hip anchor (negative); the
    // root rises by its absolute value so the soles touch the ground plane.
    const float translation[3] = {0.0f, -kCharacterMetrics.legFootY * legScale - 0.01f, 0.0f};
    character_->setAssemblyTransform(character_->self, characterHandle_, translation, identity);
}

void CharacterScreen::resetToDefault()
{
    activeHeadPart_ = "head_human";
    activePlan_ = 0;
    quadSpecies_ = 0;
    headCount_ = 1;
    wingPairs_ = 0;
    armPairs_ = 1;
    tailPresent_ = false;
    CharacterError error = CharacterError::None;
    character_->setParameter(character_->self, characterHandle_, "body.plan", 0.0f, &error);
    if (!rebuildAssembly())
    {
        return;
    }
    setStatusMessage("Character reset to the humanoid profile");
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
    // (structure/parameter changes), not per-frame.
    updateAnimationDrivers(deltaSeconds, elapsedSeconds);
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

    // Keep the orbit radius proportional to the character bounds - but only
    // when the user did not request explicit framing (the CLI
    // --orbit-distance/--camera-target-y must survive, or junction
    // close-ups get re-framed away after one update).
    if (!userCameraFraming_)
    {
        float boundsCenter[3] = {0.0f, 1.0f, 0.0f};
        float boundsRadius    = 1.0f;
        if (character_->bounds(character_->self, characterHandle_, boundsCenter, &boundsRadius))
        {
            camera_->setOrbitDistance(camera_->self, cameraHandle_, std::clamp(boundsRadius * 2.4f, 1.8f, 9.0f));
        }
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
    // Tail sway: the reptile tail is one rigid piece riding its root
    // connection offset, so the sway is a yaw oscillation of that offset
    // (the module re-derives the attached part's transform from the
    // connectors). The naga lower body sways slower with the whole body.
    const float nagaSway = animateTail_ && tailSwaySpeed_ > 0.01f
        ? std::sin(static_cast<float>(elapsedSeconds) * 0.9f * tailSwaySpeed_) * 0.08f * tailSwaySpeed_
        : 0.0f;
    const struct
    {
        const char *name;
        float       phase;
        float       amplitude;
    } tails[] = {
        {"tail", 0.0f, 0.16f},
        {"naga_body", 1.1f, 0.0f}, // naga uses nagaSway below
    };
    for (const auto &entry : tails)
    {
        const InstanceId instance = instanceByName(entry.name);
        if (instance == engine::character::invalidInstance)
        {
            continue;
        }
        const float amplitude = entry.amplitude > 0.0f ? entry.amplitude : std::fabs(nagaSway);
        float sway = 0.0f;
        if (animateTail_ && tailSwaySpeed_ > 0.01f)
        {
            sway = std::sin(static_cast<float>(elapsedSeconds) * 1.7f * tailSwaySpeed_ + entry.phase) * amplitude
                 * tailSwaySpeed_;
        }
        float rotation[4];
        setYawRotation(rotation, sway);
        character_->setConnectionOffset(character_->self, characterHandle_, instance, nullptr, rotation);
    }

    // Wing flap: rotate the connection offsets around the forward axis; the
    // right wing's socket is mirrored, so the sign flips. Up to three pairs
    // flap with a phase offset per pair (lower pairs trail the upper pair).
    for (int slot = 1; slot <= wingPairs_; ++slot)
    {
        const float flap = animateWings_
            ? std::sin(static_cast<float>(elapsedSeconds) * 2.4f - 0.55f * static_cast<float>(slot - 1)) * 0.15f
            : 0.0f;
        const std::string leftName = "wing" + std::to_string(slot) + "_left";
        const std::string rightName = "wing" + std::to_string(slot) + "_right";
        const InstanceId wingLeft = instanceByName(leftName);
        if (wingLeft != engine::character::invalidInstance)
        {
            float rotation[4];
            setRollRotation(rotation, flap);
            character_->setConnectionOffset(character_->self, characterHandle_, wingLeft, nullptr, rotation);
        }
        const InstanceId wingRight = instanceByName(rightName);
        if (wingRight != engine::character::invalidInstance)
        {
            float rotation[4];
            setRollRotation(rotation, -flap);
            character_->setConnectionOffset(character_->self, characterHandle_, wingRight, nullptr, rotation);
        }
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
