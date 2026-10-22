#ifndef ENGINE_HOST_LEAK_CHECK_HPP
#define ENGINE_HOST_LEAK_CHECK_HPP

// The --leak-check render-QA contract, extracted from the composition root:
// the host's frame loop only wires the capture sink in and reports the
// outcome; the evaluation policy lives here.

#include "engine/capabilities/renderer_contract.hpp"
#include "engine/kernel/event_bus.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace engine
{

class ComponentContext;

// The captured frame the leak check evaluates: subscribed to the capture
// event channel, keeps the last frame's pixels (the event payload only stays
// valid for the duration of the handler).
struct LeakCapture
{
    std::vector<std::uint8_t> pixels;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

// Subscribes the sink to the FrameCapturedEvent channel; every capture
// overwrites the previous frame's pixels. The subscription lives as long as
// the bus (process lifetime for the host), so no subscription handle needs
// keeping.
void subscribeLeakCapture(EventBus& bus, LeakCapture& capture);

// Evaluates the captured frame against the [leak_test] section of the
// configuration: the configured sealed region must stay dark (a light leak
// would light it), and the optional lit region must receive light (a silently
// dead GI path would fail it). Returns true when the check passes; on failure
// returns false with a precise diagnostic (no capture, malformed region,
// luminance above/below the configured bounds).
bool evaluateLeakCheck(const ComponentContext& hostContext, const LeakCapture& capture, std::string& diagnostic);

} // namespace engine

#endif // ENGINE_HOST_LEAK_CHECK_HPP
