// Capability name helpers: dotted names distinguish implementation-bound
// capabilities from capability families.

#ifndef ENGINE_KERNEL_CAPABILITY_NAME_HPP
#define ENGINE_KERNEL_CAPABILITY_NAME_HPP

#include <cstddef>
#include <string>

namespace engine
{

// Family capabilities (e.g. "renderer") match any implementation providing
// a dotted sub-capability; dotted names (e.g. "renderer.vulkan") select one.
inline bool isImplementationBoundCapabilityName(const std::string &capabilityName)
{
    return capabilityName.find('.') != std::string::npos;
}

// Text before the last '.' (the whole name when there is no dot).
inline std::string capabilityFamilyName(const std::string &capabilityName)
{
    const std::size_t lastDot = capabilityName.rfind('.');

    if (lastDot == std::string::npos)
    {
        return capabilityName;
    }

    return capabilityName.substr(0, lastDot);
}

// Does provided satisfy requirement? Exact match, or requirement is
// provided's family ("renderer" is satisfied by "renderer.vulkan").
inline bool capabilityRequirementSatisfiedBy(const std::string &requirement, const std::string &provided)
{
    if (requirement == provided)
    {
        return true;
    }

    const std::size_t lastDot = provided.rfind('.');

    if (lastDot == std::string::npos || lastDot != requirement.size())
    {
        return false;
    }

    return provided.compare(0, lastDot, requirement) == 0;
}

} // namespace engine

#endif
