// Directory containing the running executable (config/mods/modules paths
// resolve against it).

#ifndef ENGINE_KERNEL_EXECUTABLE_DIRECTORY_HPP
#define ENGINE_KERNEL_EXECUTABLE_DIRECTORY_HPP

#include <filesystem>

namespace engine
{

// Cached on first use; falls back to current_path() when the platform
// lookup fails.
std::filesystem::path executableDirectory();

} // namespace engine

#endif
