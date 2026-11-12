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

    stage_.create(*render_, rendererHandle_);

    // Default character from the humanoid body-plan profile.
    CharacterError        error  = CharacterError::None;
    const CharacterHandle handle = character_->createAssembly(character_->self, humanoidProfileId, &error);
    if (handle == engine::character::invalidCharacter)
    {
        std::fprintf(stderr,
            "demo: the character module could not create the default assembly (%u)\n",
            static_cast<unsigned>(error));
        stage_.destroy();
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
    stage_.destroy();
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

// ---- assembly helpers ---------------------------------------------------------


} // namespace demo
