#include "engine/kernel/filesystem_utilities.hpp"

#if defined(_WIN32)
// WIN32_LEAN_AND_MEAN and NOMINMAX are defined for the whole build through
// the root CMakeLists; windows.h must not be included without them.
#include <windows.h>
#else
#include <limits.h>
#include <unistd.h>
#endif

namespace engine
{

std::filesystem::path executableDirectory()
{
#if defined(_WIN32)
    wchar_t modulePath[MAX_PATH] = {};
    const DWORD written = ::GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
    if (written == 0)
    {
        return std::filesystem::current_path();
    }
    return std::filesystem::path(modulePath).parent_path();
#else
    char executablePath[PATH_MAX] = {};
    const ssize_t written = ::readlink("/proc/self/exe", executablePath, sizeof(executablePath) - 1);
    if (written <= 0)
    {
        return std::filesystem::current_path();
    }
    executablePath[written] = '\0';
    return std::filesystem::path(executablePath).parent_path();
#endif
}

} // namespace engine
