// Per-device state registry: ingests raw InputEvents, stores per-control
// snapshots, and looks up values for the per-frame binding reads.
//
// The registry is not thread-safe; the InputActionsModule serializes every
// capability entry point with one mutex and the registry assumes the caller
// holds it.

#ifndef ENGINE_INPUT_ACTIONS_DEVICE_STATE_HPP
#define ENGINE_INPUT_ACTIONS_DEVICE_STATE_HPP

#include "configuration.hpp"
#include "internal.hpp"

#include <engine/input_actions/capabilities.hpp>

#include <unordered_map>
#include <unordered_set>

namespace engine::input_actions
{
namespace internal
{

// Per-device control snapshot. Control maps and touch contacts grow on demand
// and count toward the per-device control cap; `controlCount` tracks the sum
// cheaply so the cap check does not re-sum five map sizes per event.
// `mouseDelta`, `mouseWheel`, and per-touch deltas reset at the end of every
// advanceFrame; `mousePosition` is the most recently reported absolute cursor.
struct DeviceState
{
    std::unordered_map<std::uint32_t, bool>  keys;
    std::unordered_map<std::uint32_t, bool>  mouseButtons;
    std::unordered_map<std::uint32_t, bool>  gamepadButtons;
    std::unordered_map<std::uint32_t, float> gamepadAxes;
    std::unordered_map<std::uint32_t, TouchState> touches;
    std::uint64_t touchSequence = 0;
    std::size_t   controlCount  = 0;
    // Accumulated [x, y] since the last frame, clamped to +/-1e6.
    float mouseDelta[2]   = {0.0f, 0.0f};
    float mouseWheel[2]   = {0.0f, 0.0f};
    // Most recently reported absolute cursor position, [0, 1] when produced by
    // feedWindowEvent; raw feedEvent callers choose their own normalization.
    float mousePosition[2] = {0.0f, 0.0f};
};

class DeviceStateRegistry
{
public:
    explicit DeviceStateRegistry(const InputActionsConfiguration &config) : config_(config) {}

    // Push a raw event into device state. FocusLost clears the device instead
    // of storing anything; unknown devices are created on demand until the
    // device cap is hit. Returns false for malformed events or cap refusals.
    bool feedEvent(const InputEvent &event);

    // Drops all state for one device id (reconnect/swap).
    void reset(std::uint32_t device) { devices_.erase(device); }

    // Wipes every device (called from module stop()).
    void clear() { devices_.clear(); }

    // Wipes touch contacts on every touch-surface device; used when window
    // focus is lost so a host that misses touch-end callbacks cannot leave
    // stuck touches.
    void clearTouchDevices();

    // Resets per-frame accumulators (mouse delta, wheel, touch deltas, touch
    // pressedThisFrame) and removes touches whose lifecycle ended this frame.
    void resetFrameAccumulators();

    // Selects a touch contact for a binding read. `anyTouchContact` selects
    // the most recently updated active contact, or the latest contact when
    // none remain active. Returns nullptr when the contact id is unknown.
    const TouchState *findTouch(const DeviceState &device, std::uint32_t contactId,
        std::uint32_t *outContactId = nullptr) const;

    const DeviceState *find(std::uint32_t device) const
    {
        const auto it = devices_.find(device);
        return it == devices_.end() ? nullptr : &it->second;
    }

    std::size_t size() const noexcept { return devices_.size(); }

private:
    // Inserts or updates one control value, enforcing the per-device control
    // cap and keeping `controlCount` in sync. Returns false when the cap is
    // hit on a brand-new code.
    template <typename Map, typename Value>
    bool upsertControl(DeviceState &device, Map &controls, std::uint32_t code, Value value);

    bool handleTouch(DeviceState &device, const InputEvent &event);

    const InputActionsConfiguration            &config_;
    std::unordered_map<std::uint32_t, DeviceState> devices_;
};

} // namespace internal
} // namespace engine::input_actions

#endif // ENGINE_INPUT_ACTIONS_DEVICE_STATE_HPP
