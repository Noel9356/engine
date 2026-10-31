#ifndef ENGINE_KERNEL_DYNAMIC_LIBRARY_HPP
#define ENGINE_KERNEL_DYNAMIC_LIBRARY_HPP

#include <filesystem>
#include <string>

namespace engine
{

class DynamicLibrary final
{
public:
    DynamicLibrary() = default;
    ~DynamicLibrary();

    DynamicLibrary(const DynamicLibrary &)            = delete;
    DynamicLibrary &operator=(const DynamicLibrary &) = delete;

    bool load(
        const std::filesystem::path &libraryPath,
        std::string                 &errorMessage
    );

    void release();

    void *resolveSymbol(const std::string &symbolName) const;

private:
    void *handle = nullptr;
};

} // namespace engine

#endif
