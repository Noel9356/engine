#include <engine/kernel/dynamic_library.hpp>

#if defined(_WIN32)

#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace engine
{

DynamicLibrary::~DynamicLibrary()
{
    release();
}

bool DynamicLibrary::load(
    const std::filesystem::path &libraryPath,
    std::string                 &errorMessage
)
{
    errorMessage.clear();
    release();

#if defined(_WIN32)

    HMODULE loadedHandle = ::LoadLibraryExW(
        libraryPath.c_str(), nullptr, LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS
    );
    if (loadedHandle == nullptr)
    {
        const DWORD errorCode     = ::GetLastError();
        LPSTR       messageBuffer = nullptr;
        const DWORD messageSize   = ::FormatMessageA(
            FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
            nullptr,
            errorCode,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            reinterpret_cast<LPSTR>(&messageBuffer),
            0,
            nullptr
        );
        errorMessage = "failed to load '" + libraryPath.string() + "'";
        if (messageSize > 0 && messageBuffer != nullptr)
        {
            errorMessage += ": " + std::string(messageBuffer, messageSize);
            ::LocalFree(messageBuffer);
        }
        return false;
    }
    handle = loadedHandle;
    return true;
#else
    void *loadedHandle = ::dlopen(libraryPath.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (loadedHandle == nullptr)
    {
        const char *failureReason = ::dlerror();
        errorMessage              = "failed to load '" + libraryPath.string() + "'";
        if (failureReason != nullptr)
        {
            errorMessage += ": " + std::string(failureReason);
        }
        return false;
    }
    handle = loadedHandle;
    return true;
#endif
}

void DynamicLibrary::release()
{
    if (handle == nullptr)
    {
        return;
    }
#if defined(_WIN32)
    ::FreeLibrary(static_cast<HMODULE>(handle));
#else
    ::dlclose(handle);
#endif
    handle = nullptr;
}

void *DynamicLibrary::resolveSymbol(const std::string &symbolName) const
{
    if (handle == nullptr)
    {
        return nullptr;
    }
#if defined(_WIN32)
    return reinterpret_cast<void *>(::GetProcAddress(static_cast<HMODULE>(handle), symbolName.c_str()));
#else
    return ::dlsym(handle, symbolName.c_str());
#endif
}

} // namespace engine
