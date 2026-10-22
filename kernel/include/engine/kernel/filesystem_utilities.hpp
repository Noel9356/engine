#ifndef ENGINE_KERNEL_FILESYSTEM_UTILITIES_HPP
#define ENGINE_KERNEL_FILESYSTEM_UTILITIES_HPP

#include <filesystem>

namespace engine
{

// Directory that contains the running engine executable. Relative runtime
// paths (shaders, mods, configuration) resolve against this directory, which
// makes the runtime layout relocatable. Generated user output such as
// screenshots resolves against the process working directory instead, so a
// run started from the project root writes to <project root>/screenshots.
std::filesystem::path executableDirectory();

} // namespace engine

#endif // ENGINE_KERNEL_FILESYSTEM_UTILITIES_HPP
