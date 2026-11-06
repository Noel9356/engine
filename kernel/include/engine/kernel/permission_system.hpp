// Capability/permission gate for participants.
//
// Every capability acquire and manifest permission is checked against
// what the participant declared. Configuration denials always win, then
// blanket-trusted participants, then the declared permission set.

#ifndef ENGINE_KERNEL_PERMISSION_SYSTEM_HPP
#define ENGINE_KERNEL_PERMISSION_SYSTEM_HPP

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace engine
{

// Capabilities are checked as the synthetic permission "capability.<name>".
inline constexpr const char *capabilityPermissionPrefix = "capability.";

// In-memory policy, filled from manifests (declarePermissions), the
// configuration (denyPermission) and the app (grantAllPermissions).
// Not thread-safe: main-thread only.
class PermissionSystem final
{
public:
    // Adds to the set; a participant may be declared several times (module
    // descriptor pass + manifest pass).
    void declarePermissions(const std::string &participantName, const std::vector<std::string> &permissions)
    {
        std::unordered_set<std::string> &permissionSet = declaredPermissions[participantName];
        permissionSet.insert(permissions.begin(), permissions.end());
    }

    // Blanket trust (used for the application's own context).
    void grantAllPermissions(const std::string &participantName)
    {
        trustedComponents.insert(participantName);
    }

    // Configuration-driven deny; wins over declarations and blanket trust.
    void denyPermission(const std::string &participantName, const std::string &permission)
    {
        deniedPermissions[participantName].insert(permission);
    }

    void clearDenials()
    {
        deniedPermissions.clear();
    }

    // Order matters: explicit denial, then the trusted set, then the declared
    // set (an unknown participant is granted nothing).
    bool isGranted(const std::string &participantName, const std::string &permission) const
    {
        if (!deniedPermissions.empty())
        {
            const auto deniedComponent = deniedPermissions.find(participantName);
            if (deniedComponent != deniedPermissions.end()
                && deniedComponent->second.find(permission) != deniedComponent->second.end())
            {
                return false;
            }
        }
        if (trustedComponents.find(participantName) != trustedComponents.end())
        {
            return true;
        }
        const auto found = declaredPermissions.find(participantName);
        if (found == declaredPermissions.end())
        {
            return false;
        }
        return found->second.find(permission) != found->second.end();
    }

    // Subset of requiredPermissions not granted, in order (startup
    // diagnostics).
    std::vector<std::string> missingPermissions(
        const std::string &participantName, const std::vector<std::string> &requiredPermissions) const
    {
        std::vector<std::string> missing;
        for (const std::string &permission : requiredPermissions)
        {
            if (!isGranted(participantName, permission))
            {
                missing.push_back(permission);
            }
        }
        return missing;
    }

private:
    // declaredPermissions: name -> granted set; deniedPermissions: name ->
    // denied set; trustedComponents: blanket-granted names.
    std::unordered_map<std::string, std::unordered_set<std::string>> declaredPermissions;
    std::unordered_map<std::string, std::unordered_set<std::string>> deniedPermissions;
    std::unordered_set<std::string>                                  trustedComponents;
};

} // namespace engine

#endif
