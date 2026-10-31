#include "engine/kernel/filesystem_utilities.hpp"

#if defined(_WIN32)

#include <windows.h>
#elif defined(__APPLE__)

#include <mach-o/dyld.h>
#else
#include <limits.h>
#include <unistd.h>
#endif

#include <cstdint>
#include <vector>

namespace engine
{

std::filesystem::path executableDirectory()
{
    static const std::filesystem::path cached = []() {
#if defined(_WIN32)
        wchar_t     modulePath[MAX_PATH] = {};
        const DWORD written              = ::GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
        if (written == 0)
        {
            return std::filesystem::current_path();
        }
        return std::filesystem::path(modulePath).parent_path();
#elif defined(__APPLE__)
        std::uint32_t size = 0;
        if (_NSGetExecutablePath(nullptr, &size) != 0 && size > 0)
        {
            std::vector<char> executablePath(size);
            if (_NSGetExecutablePath(executablePath.data(), &size) == 0)
            {
                return std::filesystem::path(executablePath.data()).parent_path();
            }
        }
        return std::filesystem::current_path();
#else
        char          executablePath[PATH_MAX] = {};
        const ssize_t written = ::readlink("/proc/self/exe", executablePath, sizeof(executablePath) - 1);
        if (written <= 0)
        {
            return std::filesystem::current_path();
        }
        executablePath[written] = '\0';
        return std::filesystem::path(executablePath).parent_path();
#endif
    }();
    return cached;
}

} // namespace engine
