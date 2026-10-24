#include "engine/kernel/component_interface.hpp"

#include "engine/kernel/configuration.hpp"
#include "engine/kernel/permission_system.hpp"

#include <algorithm>
#include <string_view>
#include <unordered_set>

namespace engine
{

ComponentContext::ComponentContext(
    EventBus                              &eventBus,
    CapabilityResolver                    &capabilityResolver,
    PermissionSystem                      &permissionSystem,
    const ConfigurationStore              &configuration,
    std::string                            componentName,
    const std::unordered_set<std::string> *demandedCapabilities
)
    : eventBus_(eventBus), capabilityResolver_(capabilityResolver), permissionSystem_(permissionSystem),
      configuration_(configuration), componentName_(std::move(componentName)),
      demandedCapabilities_(demandedCapabilities)
{
}

std::optional<std::string> ComponentContext::configurationString(
    const std::string &key
) const
{
    return configuration_.stringValue(componentName_, key);
}

std::optional<std::int64_t> ComponentContext::configurationInteger(
    const std::string &key
) const
{
    return configuration_.integerValue(componentName_, key);
}

std::optional<bool> ComponentContext::configurationBoolean(
    const std::string &key
) const
{
    return configuration_.booleanValue(componentName_, key);
}

std::optional<double> ComponentContext::configurationFloating(
    const std::string &key
) const
{
    return configuration_.floatingValue(componentName_, key);
}

std::optional<std::string> ComponentContext::configurationStringInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.stringAtPath(section, path);
}

std::optional<std::int64_t> ComponentContext::configurationIntegerInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.integerAtPath(section, path);
}

std::optional<bool> ComponentContext::configurationBooleanInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.booleanAtPath(section, path);
}

std::optional<double> ComponentContext::configurationFloatingInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.floatingAtPath(section, path);
}

std::vector<std::string> ComponentContext::configurationStringArrayInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.stringArrayAtPath(section, path);
}

std::vector<double> ComponentContext::configurationFloatingArrayInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.floatingArrayAtPath(section, path);
}

std::size_t ComponentContext::configurationArraySizeInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.arraySizeAtPath(section, path);
}

std::vector<std::string> ComponentContext::configurationTableKeysInSection(
    const std::string &section, const std::string &path
) const
{
    return configuration_.tableKeysAtPath(section, path);
}

// Strips compiler mangling from a typeid name so channel reports read the same
// on every toolchain: GCC/Clang emit Itanium names ("17FrameBeginEvent" or the
// nested "N6engine15FrameBeginEventE"), MSVC prefixes "struct " / "class " and
// qualifies with namespaces. The common "engine::" qualification is dropped.
std::string prettifyChannelName(
    const std::type_index &eventType
)
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
        // Itanium nested form: N <len><id> <len><id> ... E - keep the last id.
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
            const std::size_t length = static_cast<std::size_t>(
                std::stoull(inner.substr(position, digitsEnd - position))
            );
            if (digitsEnd + length > inner.size())
            {
                break;
            }
            last     = inner.substr(digitsEnd, length);
            position = digitsEnd + length;
            // Source-name entries end where the next <number> or 'E' begins.
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
        // Flat mangled form: <len><name>.
        std::size_t digitsEnd = 0;
        while (digitsEnd < name.size() && name[digitsEnd] >= '0' && name[digitsEnd] <= '9')
        {
            ++digitsEnd;
        }
        if (digitsEnd > 0)
        {
            const std::size_t length = static_cast<std::size_t>(std::stoull(name.substr(0, digitsEnd)));
            if (digitsEnd + length <= name.size())
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
