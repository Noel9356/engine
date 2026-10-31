#ifndef ENGINE_KERNEL_PERMISSION_SYSTEM_HPP
#define ENGINE_KERNEL_PERMISSION_SYSTEM_HPP

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace engine
{

inline constexpr const char *capabilityPermissionPrefix = "capability.";

class PermissionSystem final
{
public:
    void declarePermissions(
        const std::string              &componentName,
        const std::vector<std::string> &permissions
    )
    {
        std::unordered_set<std::string> &permissionSet = declaredPermissions[componentName];
        permissionSet.insert(permissions.begin(), permissions.end());
    }

    void grantAllPermissions(const std::string &componentName)
    {
        trustedComponents.insert(componentName);
    }

    void denyPermission(
        const std::string &componentName,
        const std::string &permission
    )
    {
        deniedPermissions[componentName].insert(permission);
    }

    void clearDenials()
    {
        deniedPermissions.clear();
    }

    bool isGranted(
        const std::string &componentName,
        const std::string &permission
    ) const
    {
        if (!deniedPermissions.empty())
        {
            const auto deniedComponent = deniedPermissions.find(componentName);
            if (deniedComponent != deniedPermissions.end()
                && deniedComponent->second.find(permission) != deniedComponent->second.end())
            {
                return false;
            }
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

    std::vector<std::string> missingPermissions(
        const std::string              &componentName,
        const std::vector<std::string> &requiredPermissions
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

#endif
