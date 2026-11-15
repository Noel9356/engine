// Directory containing the running executable (config/plugins/modules paths
// resolve against it).

#pragma once

#include <filesystem>

namespace engine
{

// Cached on first use; falls back to current_path() when the platform
// lookup fails.
std::filesystem::path executableDirectory();

} // namespace engine

