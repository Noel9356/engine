#ifndef ENGINE_KERNEL_DYNAMIC_LIBRARY_HPP
#define ENGINE_KERNEL_DYNAMIC_LIBRARY_HPP

#include <filesystem>
#include <string>

namespace engine
{

// Minimal cross platform dynamic library loader for shared library mods.
// The library stays loaded until the DynamicLibrary instance is destroyed or
// release() is called; the kernel controls destruction order so mods always
// unload after the kernel stops them.
class DynamicLibrary final
{
public:
    DynamicLibrary() = default;
    ~DynamicLibrary();

    DynamicLibrary(const DynamicLibrary &)            = delete;
    DynamicLibrary &operator=(const DynamicLibrary &) = delete;

    // Loads the library from exactly the given path. Platform naming policy
    // (which decorated candidates to try, e.g. "lib<name>.so") lives with the
    // caller that knows the package layout, not here.
    bool load(const std::filesystem::path &libraryPath, std::string &errorMessage);

    void release();

    void *resolveSymbol(const std::string &symbolName) const;

private:
    void *handle = nullptr;
};

} // namespace engine

#endif // ENGINE_KERNEL_DYNAMIC_LIBRARY_HPP
