// Implementation of the [window] configuration parsing; see the header for
// the struct and the reader contract.

#include "window_configuration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace engine
{
namespace window
{

namespace
{

// Reads one bounded dimension (size limit): 0 - INT_MAX, 0 meaning "no
// limit"; out-of-range values warn and leave the destination untouched.
void readDimension(const ParticipantContext &context, const char *key, std::uint32_t &destination)
{
    const std::optional<std::int64_t> value = context.configurationIntegerInSection("window", key);
    if (!value.has_value())
    {
        return;
    }
    if (*value < 0 || *value > std::numeric_limits<int>::max())
    {
        std::fprintf(stderr,
            "window module: ignoring [window] %s; expected 0..%d\n",
            key,
            std::numeric_limits<int>::max());
        return;
    }
    destination = static_cast<std::uint32_t>(*value);
}

// Reads a bounded window size (>= 1); out-of-range values warn and keep the
// current default.
void readSize(const ParticipantContext &context, const char *key, std::uint32_t &destination)
{
    const std::optional<std::int64_t> value = context.configurationIntegerInSection("window", key);
    if (!value.has_value())
    {
        return;
    }
    if (*value > 0 && *value <= std::numeric_limits<std::uint32_t>::max())
    {
        destination = static_cast<std::uint32_t>(*value);
        return;
    }
    std::fprintf(stderr, "window module: ignoring [window] %s %lld; not a positive size\n", key, (long long)*value);
}

} // namespace

WindowModuleConfiguration readWindowModuleConfiguration(const ParticipantContext &context)
{
    WindowModuleConfiguration configuration;

    // Only "glfw" and "headless" are known; unknown values warn and behave
    // as "auto".
    const std::optional<std::string> backend = context.configurationStringInSection("window", "backend");
    if (backend.has_value() && *backend != "auto")
    {
        if (*backend != "glfw" && *backend != "headless")
        {
            std::fprintf(
                stderr, "window module: unknown [window] backend '%s'; using \"auto\"\n", backend->c_str());
        }
        else
        {
            configuration.backend = *backend;
        }
    }

    readSize(context, "width", configuration.width);
    readSize(context, "height", configuration.height);

    const std::optional<std::string> title = context.configurationStringInSection("window", "title");
    if (title.has_value() && !title->empty())
    {
        configuration.title = *title;
    }
    configuration.resizable = context.configurationBooleanInSection("window", "resizable");
    configuration.visible   = context.configurationBooleanInSection("window", "visible");
    configuration.decorated = context.configurationBooleanInSection("window", "decorated");
    configuration.floating  = context.configurationBooleanInSection("window", "floating");

    readDimension(context, "minimum_width", configuration.minimumWidth);
    readDimension(context, "minimum_height", configuration.minimumHeight);
    readDimension(context, "maximum_width", configuration.maximumWidth);
    readDimension(context, "maximum_height", configuration.maximumHeight);

    const std::optional<std::int64_t> queueCapacity
        = context.configurationIntegerInSection("window", "event_queue_capacity");
    if (queueCapacity.has_value() && *queueCapacity >= 0
        && static_cast<std::uint64_t>(*queueCapacity) <= std::numeric_limits<std::uint32_t>::max())
    {
        configuration.eventQueueCapacity = static_cast<std::uint32_t>(*queueCapacity);
    }
    else if (queueCapacity.has_value())
    {
        std::fprintf(stderr,
            "window module: ignoring [window] event_queue_capacity; expected 0..%u\n",
            std::numeric_limits<std::uint32_t>::max());
    }

    // A position is only used when both coordinates are valid int32s.
    const std::optional<std::int64_t> positionX = context.configurationIntegerInSection("window", "position_x");
    const std::optional<std::int64_t> positionY = context.configurationIntegerInSection("window", "position_y");
    if (positionX.has_value() && positionY.has_value() && *positionX >= std::numeric_limits<std::int32_t>::min()
        && *positionX <= std::numeric_limits<std::int32_t>::max()
        && *positionY >= std::numeric_limits<std::int32_t>::min()
        && *positionY <= std::numeric_limits<std::int32_t>::max())
    {
        configuration.positionX   = static_cast<std::int32_t>(*positionX);
        configuration.positionY   = static_cast<std::int32_t>(*positionY);
        configuration.hasPosition = true;
    }
    else if (positionX.has_value() || positionY.has_value())
    {
        std::fprintf(stderr,
            "window module: ignoring [window] position_x/position_y; both valid coordinates are required\n");
    }

    const std::optional<bool> maximized = context.configurationBooleanInSection("window", "maximized");
    if (maximized.has_value())
    {
        configuration.maximized    = *maximized;
        configuration.hasMaximized = true;
    }
    const std::optional<bool> focusOnShow = context.configurationBooleanInSection("window", "focus_on_show");
    if (focusOnShow.has_value())
    {
        configuration.focusOnShow    = *focusOnShow;
        configuration.hasFocusOnShow = true;
    }

    const std::optional<double> opacity = context.configurationFloatingInSection("window", "opacity");
    if (opacity.has_value())
    {
        if (std::isfinite(*opacity) && *opacity >= 0.0 && *opacity <= 1.0)
        {
            configuration.opacity    = static_cast<float>(*opacity);
            configuration.hasOpacity = true;
        }
        else
        {
            std::fprintf(stderr, "window module: ignoring [window] opacity; expected 0.0..1.0\n");
        }
    }

    const std::optional<bool> gamepadPolling = context.configurationBooleanInSection("window", "gamepad_polling");
    if (gamepadPolling.has_value())
    {
        configuration.gamepadPolling = *gamepadPolling;
    }
    const std::optional<double> gamepadAxisEpsilon
        = context.configurationFloatingInSection("window", "gamepad_axis_threshold");
    if (gamepadAxisEpsilon.has_value())
    {
        if (std::isfinite(*gamepadAxisEpsilon) && *gamepadAxisEpsilon >= 0.0 && *gamepadAxisEpsilon <= 1.0)
        {
            configuration.gamepadAxisEpsilon = static_cast<float>(*gamepadAxisEpsilon);
        }
        else
        {
            std::fprintf(stderr, "window module: ignoring [window] gamepad_axis_threshold; expected 0.0..1.0\n");
        }
    }

    return configuration;
}

bool touchesWindowConfiguration(const ConfigurationReloadedEvent &event)
{
    return std::any_of(event.changedKeys.begin(),
        event.changedKeys.end(),
        [](const std::string &key) { return key.rfind("window.", 0) == 0; });
}

} // namespace window
} // namespace engine
