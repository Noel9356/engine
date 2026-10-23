#ifndef ENGINE_KERNEL_CAPABILITY_NAME_HPP
#define ENGINE_KERNEL_CAPABILITY_NAME_HPP

#include <string>

namespace engine
{

// Capability name mechanism. A capability name is either a family name
// ("example" - implementation agnostic) or a family member name
// ("example.impl" - one implementation of that family, named after the
// implementation). The dot separates the two; a member's family is everything
// before its LAST dot, so members may nest one level deep without ambiguity.
//
// This is string mechanism only: the kernel never interprets what any family
// or member means. The semantics of a capability live entirely in the
// providing module's published header and its consumers.

// Whether the name is bound to one implementation ("family.member").
inline bool isImplementationBoundCapabilityName(
    const std::string &capabilityName
)
{
    return capabilityName.find('.') != std::string::npos;
}

// The family a capability name belongs to: the name itself for family names,
// everything before the last dot for member names.
inline std::string capabilityFamilyName(
    const std::string &capabilityName
)
{
    const std::size_t lastDot = capabilityName.rfind('.');
    if (lastDot == std::string::npos)
    {
        return capabilityName;
    }
    return capabilityName.substr(0, lastDot);
}

// Whether a provided member name satisfies a consumer's requirement. A
// requirement matches exactly ("example.impl" needs exactly
// "example.impl"), and a family requirement ("example") is satisfied by
// any member of that family ("example.impl") or by an exact provider of
// the family name itself.
inline bool capabilityRequirementSatisfiedBy(
    const std::string &requirement, const std::string &provided
)
{
    if (requirement == provided)
    {
        return true;
    }
    return !isImplementationBoundCapabilityName(requirement) && capabilityFamilyName(provided) == requirement;
}

} // namespace engine

#endif // ENGINE_KERNEL_CAPABILITY_NAME_HPP
