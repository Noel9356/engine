// ParticipantContext implementation: per-participant engine services.
//
// Configuration reads scoped to the participant's own [section] plus the
// helper that derives event bus channel names from C++ type names; the
// referenced services (bus, permissions, schedulers) do the real work.

#include "engine/kernel/event_bus.hpp"
#include "engine/kernel/main_thread_task_queue.hpp"
#include "engine/kernel/profiling.hpp"
#include "engine/kernel/task_scheduler.hpp"
#include <engine/kernel/configuration.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/permission_system.hpp>

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <typeindex>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine
{

namespace
{

// Parses an Itanium mangled-name length prefix; 0 on malformed input.
std::size_t parseLengthPrefix(std::string_view digits)
{
    std::size_t value  = 0;
    const char *first  = digits.data();
    const char *last   = digits.data() + digits.size();
    const auto  result = std::from_chars(first, last, value, 10);
    if (result.ec != std::errc() || result.ptr != last)
    {
        return 0;
    }
    return value;
}

} // namespace

ParticipantContext::ParticipantContext(EventBus &eventBus, CapabilityResolver &capabilityResolver,
    PermissionSystem &permissionSystem, const ConfigurationStore &configuration, std::string participantName,
    ParticipantKind                                                             participantKind,
    const std::unordered_set<std::string, CapabilityNameHash, std::equal_to<>> *demandedCapabilities,
    MainThreadTaskQueue &mainThreadScheduler, TaskScheduler &taskScheduler, PerformanceProfiler &profiler)
    : eventBus_(eventBus),
      capabilityResolver_(capabilityResolver),
      permissionSystem_(permissionSystem),
      configuration_(configuration),
      participantName_(std::move(participantName)),
      participantKind_(participantKind),
      demandedCapabilities_(demandedCapabilities),
      mainThreadTaskQueue_(mainThreadScheduler),
      taskScheduler_(taskScheduler),
      profiler_(profiler)
{}

// Configuration reads in the participant's own [name] section.
std::optional<std::string> ParticipantContext::configurationString(const std::string &key) const
{
    return configuration_.stringValue(participantName_, key);
}

std::optional<std::int64_t> ParticipantContext::configurationInteger(const std::string &key) const
{
    return configuration_.integerValue(participantName_, key);
}

std::optional<bool> ParticipantContext::configurationBoolean(const std::string &key) const
{
    return configuration_.booleanValue(participantName_, key);
}

std::optional<double> ParticipantContext::configurationFloating(const std::string &key) const
{
    return configuration_.floatingValue(participantName_, key);
}

// Same reads in an arbitrary section; paths may index arrays with "name[0]" syntax.
std::optional<std::string> ParticipantContext::configurationStringInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.stringAtPath(section, path);
}

std::optional<std::int64_t> ParticipantContext::configurationIntegerInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.integerAtPath(section, path);
}

std::optional<bool> ParticipantContext::configurationBooleanInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.booleanAtPath(section, path);
}

std::optional<double> ParticipantContext::configurationFloatingInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.floatingAtPath(section, path);
}

std::vector<std::string> ParticipantContext::configurationStringArrayInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.stringArrayAtPath(section, path);
}

std::vector<double> ParticipantContext::configurationFloatingArrayInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.floatingArrayAtPath(section, path);
}

std::size_t ParticipantContext::configurationArraySizeInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.arraySizeAtPath(section, path);
}

std::vector<std::string> ParticipantContext::configurationTableKeysInSection(
    const std::string &section, const std::string &path) const
{
    return configuration_.tableKeysAtPath(section, path);
}

// Derives the event bus channel name from an event type: strips the Itanium
// mangling (or "struct "/"class " prefixes) down to the bare type name, e.g.
// "engine::EngineFrameEvent" -> "EngineFrameEvent".
std::string prettifyChannelName(const std::type_index &eventType)
{
    std::string                       name         = eventType.name();
    static constexpr std::string_view structPrefix = "struct ";
    static constexpr std::string_view classPrefix  = "class ";
    if (name.starts_with(structPrefix))
    {
        name.erase(0, structPrefix.size());
    }
    else if (name.starts_with(classPrefix))
    {
        name.erase(0, classPrefix.size());
    }
    if (name.size() >= 2 && name.front() == 'N' && name.back() == 'E')
    {
        const std::string inner = name.substr(1, name.size() - 2);
        std::string       last;
        std::size_t       position = 0;
        while (position < inner.size() && inner[position] >= '0' && inner[position] <= '9')
        {
            std::size_t digitsEnd = position;
            while (digitsEnd < inner.size() && inner[digitsEnd] >= '0' && inner[digitsEnd] <= '9')
            {
                ++digitsEnd;
            }
            const std::size_t length = parseLengthPrefix(inner.substr(position, digitsEnd - position));
            if (length == 0 || digitsEnd + length > inner.size())
            {
                break;
            }
            last     = inner.substr(digitsEnd, length);
            position = digitsEnd + length;

            while (position < inner.size() && !(inner[position] >= '0' && inner[position] <= '9'))
            {
                ++position;
            }
        }
        if (!last.empty())
        {
            name = last;
        }
    }
    else
    {
        std::size_t digitsEnd = 0;
        while (digitsEnd < name.size() && name[digitsEnd] >= '0' && name[digitsEnd] <= '9')
        {
            ++digitsEnd;
        }
        if (digitsEnd > 0)
        {
            const std::size_t length = parseLengthPrefix(name.substr(0, digitsEnd));
            if (length > 0 && digitsEnd + length <= name.size())
            {
                name = name.substr(digitsEnd, length);
            }
        }
    }
    static constexpr std::string_view engineNamespace = "engine::";
    if (name.starts_with(engineNamespace))
    {
        name.erase(0, engineNamespace.size());
    }
    return name;
}

} // namespace engine
