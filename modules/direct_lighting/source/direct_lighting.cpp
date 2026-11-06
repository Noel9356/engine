// direct_lighting module implementation.
//
// Keeps a fixed-size registry of validated, direction-normalized lights under
// one mutex and maintains a priority-sorted "active" snapshot for renderers.
// copyLights plus an atomic version counter let consumers detect changes
// cheaply; resolveProfile maps a render::DeviceProfile to a light budget.

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

namespace engine
{
namespace direct_lighting
{
namespace
{

// Writes an error code through an optional out-parameter.
void setError(DirectLightingError *output, DirectLightingError value)
{
    if (output != nullptr)
    {
        *output = value;
    }
}

// Whole-struct validator: every float finite, ranges hold (range > 0, inner
// <= outer <= pi/2, non-negative intensity and color), and Directional/Spot
// directions are non-degenerate (checked in double precision to resist
// squaring tiny float components).
bool validDescription(const DirectLightDescription &value)
{
    if (value.type > DirectLightType::Spot || !std::isfinite(value.intensity) || value.intensity < 0.0f
        || !std::isfinite(value.range) || value.range <= 0.0f || !std::isfinite(value.innerConeRadians)
        || !std::isfinite(value.outerConeRadians) || value.innerConeRadians < 0.0f
        || value.outerConeRadians < value.innerConeRadians || value.outerConeRadians > 1.5707964f)
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(value.position[axis]) || !std::isfinite(value.direction[axis])
            || !std::isfinite(value.color[axis]) || value.color[axis] < 0.0f)
        {
            return false;
        }
    }
    if (value.type == DirectLightType::Directional || value.type == DirectLightType::Spot)
    {
        const double lengthSquared = static_cast<double>(value.direction[0]) * value.direction[0]
                                   + static_cast<double>(value.direction[1]) * value.direction[1]
                                   + static_cast<double>(value.direction[2]) * value.direction[2];
        if (!std::isfinite(lengthSquared) || lengthSquared < 1.0e-12)
        {
            return false;
        }
    }
    return true;
}

// Normalizes direction for Directional/Spot lights; safe because
// validDescription already rejected degenerate directions.
DirectLightDescription normalizeDescription(const DirectLightDescription &source)
{
    DirectLightDescription result = source;
    if (result.type == DirectLightType::Directional || result.type == DirectLightType::Spot)
    {
        const double length = std::sqrt(static_cast<double>(result.direction[0]) * result.direction[0]
                                        + static_cast<double>(result.direction[1]) * result.direction[1]
                                        + static_cast<double>(result.direction[2]) * result.direction[2]);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            result.direction[axis] = static_cast<float>(result.direction[axis] / length);
        }
    }
    return result;
}

// Exact-value comparison used to skip snapshot churn when updateLight
// receives an unchanged description.
bool sameDescription(const DirectLightDescription &left, const DirectLightDescription &right)
{
    if (left.type != right.type || left.intensity != right.intensity || left.range != right.range
        || left.innerConeRadians != right.innerConeRadians || left.outerConeRadians != right.outerConeRadians
        || left.priority != right.priority || left.layerMask != right.layerMask || left.enabled != right.enabled)
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (left.position[axis] != right.position[axis] || left.direction[axis] != right.direction[axis]
            || left.color[axis] != right.color[axis])
        {
            return false;
        }
    }
    return true;
}

// Module singleton backing the "direct_lighting" capability. mutex_ guards
// the light arrays; version_ is atomic so consumers can watch for changes.
class DirectLightingModule final : public ModuleInterface
{
public:
    DirectLightingModule()
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
        // [direct_lighting] max_lights: 0 means "automatic" (device-dependent
        // in resolveProfile, hard max in createLight). Values above
        // maxDirectLights are clamped; negatives are ignored.
        configuredMaxLights_ = 0;
        if (const auto value = context.configurationIntegerInSection("direct_lighting", "max_lights");
            value.has_value() && *value >= 0)
        {
            configuredMaxLights_ = static_cast<std::uint32_t>(
                std::min<std::int64_t>(*value, static_cast<std::int64_t>(maxDirectLights)));
        }
    }

    void start() override
    {}

    void stop() override
    {
        // Clears the registry and snapshot; the version bump makes copyLights
        // consumers observe the (empty) change.
        std::lock_guard<std::mutex> lock(mutex_);
        lightCount_    = 0;
        snapshotCount_ = 0;
        version_.fetch_add(1u, std::memory_order_release);
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == DirectLightingCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "direct_lighting",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"direct_lighting"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self        = this;
        capability_.createLight = [](void *self, const DirectLightDescription *description, DirectLightingError *error)
        { return static_cast<DirectLightingModule *>(self)->createLightImpl(description, error); };
        capability_.updateLight = [](void                          *self,
                                      DirectLightHandle             light,
                                      const DirectLightDescription *description,
                                      DirectLightingError          *error)
        { return static_cast<DirectLightingModule *>(self)->updateLightImpl(light, description, error); };
        capability_.destroyLight = [](void *self, DirectLightHandle light)
        { return static_cast<DirectLightingModule *>(self)->destroyLightImpl(light); };
        capability_.setEnabled = [](void *self, DirectLightHandle light, bool enabled)
        { return static_cast<DirectLightingModule *>(self)->setEnabledImpl(light, enabled); };
        capability_.copyLights = [](void *self, DirectLight *output, std::uint32_t capacity, std::uint64_t *version)
        { return static_cast<DirectLightingModule *>(self)->copyLightsImpl(output, capacity, version); };
        capability_.resolveProfile = [](void *self, const render::DeviceProfile *device, DirectLightingProfile *output)
        { return static_cast<DirectLightingModule *>(self)->resolveProfileImpl(device, output); };
    }

    DirectLightHandle createLightImpl(const DirectLightDescription *description, DirectLightingError *error)
    {
        if (description == nullptr || !validDescription(*description))
        {
            setError(error, DirectLightingError::InvalidDescription);
            return invalidDirectLight;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        // configuredMaxLights_ == 0 selects the hard max.
        const std::uint32_t configuredCapacity = configuredMaxLights_ == 0 ? maxDirectLights : configuredMaxLights_;
        if (lightCount_ >= configuredCapacity)
        {
            setError(error, DirectLightingError::CapacityReached);
            return invalidDirectLight;
        }
        // Handles are never reused; also refuse when the 32-bit space is
        // exhausted (nextHandle_ wrapped to the sentinel or its max).
        if (nextHandle_ == invalidDirectLight || nextHandle_ == std::numeric_limits<DirectLightHandle>::max())
        {
            setError(error, DirectLightingError::CapacityReached);
            return invalidDirectLight;
        }
        const DirectLightHandle handle = nextHandle_++;
        lights_[lightCount_++] = DirectLight{.handle = handle, .description = normalizeDescription(*description)};
        rebuildSnapshotLocked();
        version_.fetch_add(1u, std::memory_order_release);
        setError(error, DirectLightingError::None);
        return handle;
    }

    bool updateLightImpl(
        DirectLightHandle handle, const DirectLightDescription *description, DirectLightingError *error)
    {
        if (description == nullptr || !validDescription(*description))
        {
            setError(error, DirectLightingError::InvalidDescription);
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const std::uint32_t         index = findLightLocked(handle);
        if (index == lightCount_)
        {
            setError(error, DirectLightingError::InvalidHandle);
            return false;
        }
        const DirectLightDescription normalized = normalizeDescription(*description);
        const DirectLightDescription previous   = lights_[index].description;
        if (sameDescription(previous, normalized))
        {
            setError(error, DirectLightingError::None);
            return true;
        }
        // If only inert fields changed (light still active, same priority)
        // the snapshot entry is patched in place; otherwise the snapshot must
        // be rebuilt because membership or order changed.
        const bool wasActive       = previous.enabled && previous.intensity > 0.0f
                                  && (previous.color[0] > 0.0f || previous.color[1] > 0.0f || previous.color[2] > 0.0f);
        const bool isActive        = normalized.enabled && normalized.intensity > 0.0f
                                  && (normalized.color[0] > 0.0f || normalized.color[1] > 0.0f || normalized.color[2] > 0.0f);
        lights_[index].description = normalized;
        if (wasActive != isActive || previous.priority != normalized.priority)
        {
            rebuildSnapshotLocked();
        }
        else if (isActive)
        {
            for (std::uint32_t snapshotIndex = 0; snapshotIndex < snapshotCount_; ++snapshotIndex)
            {
                if (snapshot_[snapshotIndex].handle == handle)
                {
                    snapshot_[snapshotIndex] = lights_[index];
                    break;
                }
            }
        }
        version_.fetch_add(1u, std::memory_order_release);
        setError(error, DirectLightingError::None);
        return true;
    }

    bool destroyLightImpl(DirectLightHandle handle)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::uint32_t         index = findLightLocked(handle);
        if (index == lightCount_)
        {
            return false;
        }
        // Swap-remove: the registry is unordered; the snapshot is what
        // consumers read.
        --lightCount_;
        if (index != lightCount_)
        {
            lights_[index] = lights_[lightCount_];
        }
        rebuildSnapshotLocked();
        version_.fetch_add(1u, std::memory_order_release);
        return true;
    }

    bool setEnabledImpl(DirectLightHandle handle, bool enabled)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::uint32_t         index = findLightLocked(handle);
        if (index == lightCount_)
        {
            return false;
        }
        // Toggling enabled changes snapshot membership, so rebuild.
        if (lights_[index].description.enabled != enabled)
        {
            lights_[index].description.enabled = enabled;
            rebuildSnapshotLocked();
            version_.fetch_add(1u, std::memory_order_release);
        }
        return true;
    }

    // Copies the active snapshot (priority-sorted), truncated to capacity.
    // With a null/zero-capacity buffer it still reports the current version,
    // letting callers size their buffer and detect changes. Returns the count.
    std::uint32_t copyLightsImpl(DirectLight *output, std::uint32_t capacity, std::uint64_t *outVersion) const
    {
        if (output == nullptr || capacity == 0)
        {
            if (outVersion != nullptr)
            {
                *outVersion = version_.load(std::memory_order_acquire);
            }
            return 0;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (outVersion != nullptr)
        {
            *outVersion = version_.load(std::memory_order_relaxed);
        }
        const std::uint32_t count = std::min(capacity, snapshotCount_);
        std::copy_n(snapshot_.begin(), count, output);
        return count;
    }

    // Linear scan returning lightCount_ (the one-past slot) as "not found".
    std::uint32_t findLightLocked(DirectLightHandle handle) const
    {
        for (std::uint32_t index = 0; index < lightCount_; ++index)
        {
            if (lights_[index].handle == handle)
            {
                return index;
            }
        }
        return lightCount_;
    }

    // Rebuilds the renderer-facing snapshot: only enabled lights with
    // radiance (positive intensity and a non-zero color channel), sorted by
    // descending priority then ascending handle for a stable order.
    void rebuildSnapshotLocked()
    {
        snapshotCount_ = 0;
        for (std::uint32_t index = 0; index < lightCount_; ++index)
        {
            const DirectLight            &light       = lights_[index];
            const DirectLightDescription &description = light.description;
            const bool                    hasRadiance
                = description.intensity > 0.0f
               && (description.color[0] > 0.0f || description.color[1] > 0.0f || description.color[2] > 0.0f);
            if (description.enabled && hasRadiance)
            {
                snapshot_[snapshotCount_++] = light;
            }
        }
        std::sort(snapshot_.begin(),
            snapshot_.begin() + snapshotCount_,
            [](const DirectLight &left, const DirectLight &right)
            {
                if (left.description.priority != right.description.priority)
                {
                    return left.description.priority > right.description.priority;
                }
                return left.handle < right.handle;
            });
    }

    bool resolveProfileImpl(const render::DeviceProfile *device, DirectLightingProfile *output) const
    {
        if (device == nullptr || output == nullptr)
        {
            return false;
        }
        // Vulkan light budgets by device class; without compute shaders the
        // budget is capped at 16. Non-Vulkan devices use the base 8.
        std::uint32_t automatic = 8;
        if (device->isVulkan)
        {
            switch (device->deviceClass)
            {
            case render::DeviceClass::DiscreteGpu:
                automatic = 32;
                break;
            case render::DeviceClass::IntegratedGpu:
                automatic = 24;
                break;
            case render::DeviceClass::VirtualGpu:
                automatic = 16;
                break;
            default:
                automatic = 8;
                break;
            }
            if (!device->computeShader)
            {
                automatic = std::min(automatic, 16u);
            }
        }
        // Uniform-buffer budget for the light array: 176 bytes of fixed block
        // data plus 64 bytes per light. A too-small UBO range means no lights.
        std::uint32_t hardwareLimit = maxDirectLights;
        if (device->isVulkan)
        {
            constexpr std::uint64_t fixedUniformBytes = 176;
            constexpr std::uint64_t bytesPerLight     = 64;
            hardwareLimit = device->maxUniformBufferRange < fixedUniformBytes
                              ? 0u
                              : static_cast<std::uint32_t>(std::min<std::uint64_t>(maxDirectLights,
                                    (device->maxUniformBufferRange - fixedUniformBytes) / bytesPerLight));
            automatic     = std::min(automatic, hardwareLimit);
        }
        output->maxLights = configuredMaxLights_ == 0 ? automatic : std::min(configuredMaxLights_, hardwareLimit);
        return true;
    }

    ParticipantManifest      manifest_;
    DirectLightingCapability capability_{};
    // Authoritative registry (swap-remove order) and the sorted renderer
    // view. nextHandle_ never reuses ids; version_ bumps on every mutation.
    mutable std::mutex                       mutex_;
    std::array<DirectLight, maxDirectLights> lights_{};
    std::array<DirectLight, maxDirectLights> snapshot_{};
    std::uint32_t                            lightCount_    = 0;
    std::uint32_t                            snapshotCount_ = 0;
    DirectLightHandle                        nextHandle_    = 1;
    std::atomic<std::uint64_t>               version_{1};
    std::uint32_t                            configuredMaxLights_ = 0;
};

} // namespace
} // namespace direct_lighting
} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the single provided "direct_lighting" capability; no
// required or optional capabilities, no declared permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"direct_lighting"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "direct_lighting",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::direct_lighting::DirectLightingModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = nullptr,
            .optionalCapabilityCount = 0,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
