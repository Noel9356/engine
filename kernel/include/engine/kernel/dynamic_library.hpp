// RAII wrapper over dlopen/LoadLibrary for module and plugin libraries.
//
// The kernel holds one instance per loaded participant library so its
// entry table and code stay resident.

#ifndef ENGINE_KERNEL_DYNAMIC_LIBRARY_HPP
#define ENGINE_KERNEL_DYNAMIC_LIBRARY_HPP

#include <filesystem>
#include <string>

namespace engine
{

// At most one library per instance; load() releases any previous handle.
class DynamicLibrary final
{
public:
    DynamicLibrary() = default;
    ~DynamicLibrary();

    DynamicLibrary(const DynamicLibrary &)            = delete;
    DynamicLibrary &operator=(const DynamicLibrary &) = delete;

    // Error text comes from the OS; safe to call again on the same object.
    bool load(const std::filesystem::path &libraryPath, std::string &errorMessage);

    // Unloads (OS ref-counted); symbol pointers die with the library.
    void release();

    // nullptr for unknown symbols or a released library.
    void *resolveSymbol(const std::string &symbolName) const;

private:
    // Platform handle (HMODULE or void* from dlopen).
    void *handle = nullptr;
};

} // namespace engine

#endif
