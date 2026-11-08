// Shared host-hardware queries (engine::util).
//
// Header-only, dependency-free probes used by modules that implement
// `auto_config`: when a module derives its own settings from the machine it
// runs on, it needs at least a CPU shape (logical processor count) and a
// memory shape (installed physical RAM). Both queries are safe to call from
// any thread and never throw; unknown values are reported as 0 and callers
// are expected to fall back to their built-in defaults.
//
// These are *coarse* system queries on purpose: they describe the machine,
// not the current load. Modules must not poll them per frame.

#ifndef ENGINE_SHARED_UTIL_HARDWARE_INFO_HPP
#define ENGINE_SHARED_UTIL_HARDWARE_INFO_HPP

#include <cstdint>

#if defined(_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <thread>

namespace engine::util
{

// Number of logical processors available to the process. hardware_concurrency
// may legally return 0 when the value cannot be determined; this wrapper maps
// that to 1 so callers can reason about "cores >= 1" unconditionally.
inline std::uint32_t logicalProcessorCount() noexcept
{
    const unsigned int hardware = std::thread::hardware_concurrency();
    return hardware != 0u ? static_cast<std::uint32_t>(hardware) : 1u;
}

// Installed physical RAM in bytes, or 0 when the platform cannot report it.
// Linux/UNIX reads SC_PHYS_PAGES (what the kernel booted with, not the
// current cgroup limit); Windows reads GlobalMemoryStatusEx. A 0 return means
// "unknown" and every consumer must treat it as such (fall back to defaults,
// never to "no memory").
inline std::uint64_t physicalMemoryBytes() noexcept
{
#if defined(_WIN32)
    MEMORYSTATUSEX status{};
    status.dwLength = sizeof(status);
    if (GlobalMemoryStatusEx(&status) != 0)
    {
        return static_cast<std::uint64_t>(status.ullTotalPhys);
    }
    return 0u;
#else
    const long pages    = sysconf(_SC_PHYS_PAGES);
    const long pageSize = sysconf(_SC_PAGESIZE);
    if (pages <= 0 || pageSize <= 0)
    {
        return 0u;
    }
    return static_cast<std::uint64_t>(pages) * static_cast<std::uint64_t>(pageSize);
#endif
}

} // namespace engine::util

#endif // ENGINE_SHARED_UTIL_HARDWARE_INFO_HPP
