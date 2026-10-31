#ifndef ENGINE_KERNEL_CAPABILITY_NAME_HPP
#define ENGINE_KERNEL_CAPABILITY_NAME_HPP

#include <string>

namespace engine
{

inline bool isImplementationBoundCapabilityName(const std::string &capabilityName)
{
    return capabilityName.find('.') != std::string::npos;
}

inline std::string capabilityFamilyName(const std::string &capabilityName)
{
    const std::size_t lastDot = capabilityName.rfind('.');

    if (lastDot == std::string::npos)
    {
        return capabilityName;
    }

    return capabilityName.substr(0, lastDot);
}

inline bool capabilityRequirementSatisfiedBy(
    const std::string &requirement,
    const std::string &provided
)
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
