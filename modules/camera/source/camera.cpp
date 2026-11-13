// camera module implementation.
//
// Owns the camera store behind the "camera" capability: capacity-limited
// handle-addressed records, the [camera] configuration defaults (with
// hot-reload through ConfigurationReloadedEvent), and the optional Lua
// bridge registration. Pose math, per-record state, and matrix caching live
// in camera_record; configuration loading lives in camera_configuration.

#include <engine/camera/capabilities.hpp>
#include "camera_configuration.hpp"
#include "camera_record.hpp"
#include "lua_bridge.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstdint>
#include <new>
#include <string>
#include <unordered_map>

namespace engine
{
namespace camera
{
namespace
{

// Implements the camera capability. Not thread-safe: the camera map and the
// records (including their caches) are accessed without locks.
class CameraModule final : public ModuleInterface
{
public:
    CameraModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        // Reads [camera] defaults and subscribes for configuration hot-reload.
        context_ = &context;
        loadCameraDefaults(context, defaults_);
        try
        {
            cameras_.reserve(defaults_.maximumCameras);
        }
        catch (...)
        {
            // Capacity remains enforced even when the initial reservation fails.
        }
        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (script_runtime::canRegisterRuntimeInitializer(scriptRuntime_))
        {
            luaBridgeContext_.scriptRuntime = scriptRuntime_;
            luaBridgeContext_.camera        = &capability_;
            runtimeInitializer_              = script_runtime::tryRegisterRuntimeInitializer(
                scriptRuntime_, initializeCameraLuaRuntime, &luaBridgeContext_);
        }
        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<CameraModule, &CameraModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

    void stop() override
    {
        script_runtime::unregisterRuntimeInitializer(scriptRuntime_, runtimeInitializer_);
        {
            std::lock_guard bridgeLock(luaBridgeContext_.apiMutex);
            luaBridgeContext_.camera = nullptr;
            cameras_.clear();
        }
        if (reloadSubscription_ != 0 && context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
            reloadSubscription_ = 0;
        }
        context_ = nullptr;
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        if (capabilityName == CameraCapability::capabilityName)
        {
            return &capability_;
        }
        return nullptr;
    }

private:
    // Re-reads defaults when any changed key starts with "camera."; cameras
    // that already exist keep their current settings.
    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool changed = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("camera.", 0) == 0; });
        if (changed)
        {
            loadCameraDefaults(*context_, defaults_);
        }
    }

    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "camera",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"camera"},
            .requiredCapabilities = {},
            .optionalCapabilities = {"script_runtime"}};
    }

    CameraRecord *recordFor(CameraHandle camera) noexcept
    {
        const auto found = cameras_.find(camera);
        return found != cameras_.end() ? &found->second : nullptr;
    }

    const CameraRecord *recordFor(CameraHandle camera) const noexcept
    {
        const auto found = cameras_.find(camera);
        return found != cameras_.end() ? &found->second : nullptr;
    }

    // Builds a record from the resolved defaults plus the caller parameters;
    // ids start at 1 and the sentinel 0 is never handed out.
    CameraHandle createCameraImpl(const CameraParameters *parameters, CameraError *error)
    {
        if (cameras_.size() >= defaults_.maximumCameras || nextCameraId_ == invalidCamera)
        {
            reportError(error, CameraError::CreationFailed);
            return invalidCamera;
        }
        CameraRecord record = CameraRecord::fromParameters(parameters, defaults_);
        const CameraHandle id = nextCameraId_++;
        try
        {
            cameras_.emplace(id, std::move(record));
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, CameraError::CreationFailed);
            return invalidCamera;
        }
        reportError(error, CameraError::None);
        return id;
    }

    void destroyCameraImpl(CameraHandle camera)
    {
        cameras_.erase(camera);
    }

    void setLookAtImpl(CameraHandle camera, const float *eye, const float *target, const float *up)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || eye == nullptr || target == nullptr || up == nullptr)
        {
            return;
        }
        record->setLookAt(glm::make_vec3(eye), glm::make_vec3(target), glm::make_vec3(up));
    }

    void setControlModeImpl(CameraHandle camera, CameraControlMode mode)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        record->setControlMode(mode);
    }

    void setOrbitDistanceImpl(CameraHandle camera, float distance)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        record->setOrbitDistance(distance);
    }

    void setOrbitTargetImpl(CameraHandle camera, const float *target)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || target == nullptr)
        {
            return;
        }
        record->setOrbitTarget(glm::make_vec3(target));
    }

    void setMovementSpeedImpl(CameraHandle camera, float unitsPerSecond)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        record->setMovementSpeed(unitsPerSecond);
    }

    void setLookSpeedImpl(CameraHandle camera, float degreesPerSecond)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        record->setLookSpeed(degreesPerSecond);
    }

    void setMouseSensitivityImpl(CameraHandle camera, float degreesPerPixel)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        record->setMouseSensitivity(degreesPerPixel);
    }

    bool setControlSettingsImpl(CameraHandle camera, const CameraControlSettings *settings)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || settings == nullptr)
        {
            return false;
        }
        return record->setControlSettings(*settings);
    }

    void setProjectionImpl(CameraHandle camera, ProjectionType type, float verticalScale, float nearZ, float farZ)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        record->setProjection(type, verticalScale, nearZ, farZ);
    }

    void updateImpl(CameraHandle camera, const CameraInputState *input, double deltaSeconds)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        record->integrate(input, deltaSeconds);
    }

    void viewStateImpl(
        CameraHandle camera, std::uint32_t viewportWidth, std::uint32_t viewportHeight, ViewState *outState) const
    {
        if (outState == nullptr)
        {
            return;
        }
        const CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return;
        }
        *outState = record->viewState(viewportWidth, viewportHeight);
    }

    bool cameraPoseImpl(CameraHandle camera, CameraPose *outPose) const
    {
        if (outPose == nullptr)
        {
            return false;
        }
        const CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return false;
        }
        record->fillPose(*outPose);
        return true;
    }

    bool worldToViewportImpl(CameraHandle camera, std::uint32_t viewportWidth, std::uint32_t viewportHeight,
        const float *worldPosition, float *outX, float *outY, float *outDepth) const
    {
        if (worldPosition == nullptr || outX == nullptr || outY == nullptr || outDepth == nullptr)
        {
            return false;
        }
        const CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return false;
        }
        return record->worldToViewport(
            viewportWidth, viewportHeight, glm::make_vec3(worldPosition), *outX, *outY, *outDepth);
    }

    bool viewportToWorldRayImpl(CameraHandle camera, std::uint32_t viewportWidth, std::uint32_t viewportHeight,
        float viewportX, float viewportY, float *outOrigin, float *outDirection) const
    {
        if (outOrigin == nullptr || outDirection == nullptr)
        {
            return false;
        }
        const CameraRecord *record = recordFor(camera);
        if (record == nullptr)
        {
            return false;
        }
        glm::vec3 origin{};
        glm::vec3 direction{};
        if (!record->viewportToWorldRay(viewportWidth, viewportHeight, viewportX, viewportY, origin, direction))
        {
            return false;
        }
        outOrigin[0]    = origin.x;
        outOrigin[1]    = origin.y;
        outOrigin[2]    = origin.z;
        outDirection[0] = direction.x;
        outDirection[1] = direction.y;
        outDirection[2] = direction.z;
        return true;
    }

    static void reportError(CameraError *error, CameraError value)
    {
        if (error != nullptr)
        {
            *error = value;
        }
    }

    // Wires each capability entry to its Impl member through a forwarding
    // lambda; the void* self is this CameraModule instance.
    void assignCapabilityPointers()
    {
        capability_.self         = this;
        capability_.createCamera = [](void *self, const CameraParameters *parameters, CameraError *error)
        { return static_cast<CameraModule *>(self)->createCameraImpl(parameters, error); };
        capability_.destroyCamera
            = [](void *self, CameraHandle camera) { static_cast<CameraModule *>(self)->destroyCameraImpl(camera); };
        capability_.setLookAt
            = [](void *self, CameraHandle camera, const float eye[3], const float target[3], const float up[3])
        { static_cast<CameraModule *>(self)->setLookAtImpl(camera, eye, target, up); };
        capability_.setControlMode = [](void *self, CameraHandle camera, CameraControlMode mode)
        { static_cast<CameraModule *>(self)->setControlModeImpl(camera, mode); };
        capability_.setOrbitDistance = [](void *self, CameraHandle camera, float distance)
        { static_cast<CameraModule *>(self)->setOrbitDistanceImpl(camera, distance); };
        capability_.setOrbitTarget = [](void *self, CameraHandle camera, const float target[3])
        { static_cast<CameraModule *>(self)->setOrbitTargetImpl(camera, target); };
        capability_.setMovementSpeed = [](void *self, CameraHandle camera, float unitsPerSecond)
        { static_cast<CameraModule *>(self)->setMovementSpeedImpl(camera, unitsPerSecond); };
        capability_.setLookSpeed = [](void *self, CameraHandle camera, float degreesPerSecond)
        { static_cast<CameraModule *>(self)->setLookSpeedImpl(camera, degreesPerSecond); };
        capability_.setMouseSensitivity = [](void *self, CameraHandle camera, float degreesPerPixel)
        { static_cast<CameraModule *>(self)->setMouseSensitivityImpl(camera, degreesPerPixel); };
        capability_.setControlSettings = [](void *self, CameraHandle camera, const CameraControlSettings *settings)
        { return static_cast<CameraModule *>(self)->setControlSettingsImpl(camera, settings); };
        capability_.setProjection
            = [](void *self, CameraHandle camera, ProjectionType type, float verticalScale, float nearZ, float farZ)
        { static_cast<CameraModule *>(self)->setProjectionImpl(camera, type, verticalScale, nearZ, farZ); };
        capability_.update = [](void *self, CameraHandle camera, const CameraInputState *input, double deltaSeconds)
        { static_cast<CameraModule *>(self)->updateImpl(camera, input, deltaSeconds); };
        capability_.viewState
            = [](void *self, CameraHandle camera, std::uint32_t width, std::uint32_t height, ViewState *outState)
        { static_cast<CameraModule *>(self)->viewStateImpl(camera, width, height, outState); };
        capability_.cameraPose = [](void *self, CameraHandle camera, CameraPose *outPose)
        { return static_cast<CameraModule *>(self)->cameraPoseImpl(camera, outPose); };
        capability_.worldToViewport = [](void *self, CameraHandle camera, std::uint32_t width, std::uint32_t height,
            const float position[3], float *x, float *y, float *depth)
        {
            return static_cast<CameraModule *>(self)
                ->worldToViewportImpl(camera, width, height, position, x, y, depth);
        };
        capability_.viewportToWorldRay = [](void *self, CameraHandle camera, std::uint32_t width, std::uint32_t height,
            float x, float y, float origin[3], float direction[3])
        {
            return static_cast<CameraModule *>(self)
                ->viewportToWorldRayImpl(camera, width, height, x, y, origin, direction);
        };
    }

    ParticipantContext *context_            = nullptr;
    SubscriptionToken   reloadSubscription_ = 0;
    script_runtime::ScriptRuntimeCapability *scriptRuntime_ = nullptr;
    script_runtime::ScriptRuntimeInitializerHandle runtimeInitializer_
        = script_runtime::invalidScriptRuntimeInitializer;
    CameraLuaBridgeContext luaBridgeContext_{};

    // Defaults for new cameras; refreshed from [camera] config in
    // loadCameraDefaults (construct and hot-reload).
    CameraDefaults defaults_{};

    // Live cameras; mutable because the const query paths rebuild per-record
    // caches through CameraRecord.
    mutable std::unordered_map<CameraHandle, CameraRecord> cameras_;
    // Ids start at 1; 0 is the invalid sentinel.
    CameraHandle nextCameraId_ = 1;

    ParticipantManifest manifest_;

    CameraCapability capability_{};
};

} // namespace
} // namespace camera

using camera::CameraModule;

} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the single provided "camera" capability; no required
// capabilities, one optional (script_runtime), no declared permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"camera"};
    static const char *const optionalCapabilities[] = {"script_runtime"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "camera",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface *
        { return new engine::camera::CameraModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = optionalCapabilities,
        .optionalCapabilityCount = 1,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
