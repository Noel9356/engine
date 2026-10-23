#ifndef ENGINE_KERNEL_PERMISSION_SYSTEM_HPP
#define ENGINE_KERNEL_PERMISSION_SYSTEM_HPP

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace engine
{

// The permission name that guards capability acquisition, derived from the
// capability name. One definition because two sites build it (the kernel's
// derived-grant pass and ComponentContext's acquisition check); a typo in
// either would silently break the contract.
inline constexpr const char *capabilityPermissionPrefix = "capability.";

// Permission identifiers follow the pattern "<domain>.<action>":
//   capability.<name>    acquire the named capability contract
//   filesystem.write     create or overwrite files
// A permission is granted when the component declares it or derives it from
// a required capability (requiring capability "x" and being authorized to
// acquire "x" are the same statement, so the kernel derives the grant), and
// the runtime configuration does not deny it - a denial wins over every
// grant. Modules are not exempt: the rule is uniform, which keeps the
// startup report honest about who may touch what.
//
// Containers are unordered on purpose: isGranted sits on the capability
// acquisition path, which components call during construct and (for lazy
// acquisitions) at frame time; the checks must stay constant-cost hashes.
class PermissionSystem final
{
public:
    // Declares the permission set of one component. Called once per component
    // before construction, from the manifest.
    void declarePermissions(
        const std::string &componentName, const std::vector<std::string> &permissions
    )
    {
        std::unordered_set<std::string> &permissionSet = declaredPermissions[componentName];
        permissionSet.insert(permissions.begin(), permissions.end());
    }

    // Marks a component as trusted: the composition root (the host executable)
    // drives the frame loop and may acquire every available capability.
    void grantAllPermissions(
        const std::string &componentName
    )
    {
        trustedComponents.insert(componentName);
    }

    // Denies one permission for one component. Called from runtime
    // configuration before construction.
    void denyPermission(
        const std::string &componentName, const std::string &permission
    )
    {
        deniedPermissions[componentName].insert(permission);
    }

    bool isGranted(
        const std::string &componentName, const std::string &permission
    ) const
    {
        const auto deniedComponent = deniedPermissions.find(componentName);
        if (deniedComponent != deniedPermissions.end()
            && deniedComponent->second.find(permission) != deniedComponent->second.end())
        {
            return false;
        }
        if (trustedComponents.find(componentName) != trustedComponents.end())
        {
            return true;
        }
        const auto found = declaredPermissions.find(componentName);
        if (found == declaredPermissions.end())
        {
            return false;
        }
        return found->second.find(permission) != found->second.end();
    }

    // Returns every entry of requiredPermissions that is not granted, so the
    // kernel can produce one precise diagnostic per missing grant.
    std::vector<std::string> missingPermissions(
        const std::string &componentName, const std::vector<std::string> &requiredPermissions
    ) const
    {
        std::vector<std::string> missing;
        for (const std::string &permission : requiredPermissions)
        {
            if (!isGranted(componentName, permission))
            {
                missing.push_back(permission);
            }
        }
        return missing;
    }

private:
    std::unordered_map<std::string, std::unordered_set<std::string>> declaredPermissions;
    std::unordered_map<std::string, std::unordered_set<std::string>> deniedPermissions;
    std::unordered_set<std::string>                                  trustedComponents;
};

} // namespace engine

#endif // ENGINE_KERNEL_PERMISSION_SYSTEM_HPP
