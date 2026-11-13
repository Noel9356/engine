// File-reading helper: stat, read, and optional SHA-256 verification.
//
// On Linux, reads through an opened descriptor with O_NOFOLLOW (defensive
// against symlink replacement between canonicalization and read) and
// optional posix_fadvise hints. On other platforms, uses std::ifstream.
// The loader never touches the cache's budget counters; the module
// orchestrates reserve → load → commit through the cache's budget hooks.

#ifndef ENGINE_FILE_ASSET_STORE_ASSET_LOADER_HPP
#define ENGINE_FILE_ASSET_STORE_ASSET_LOADER_HPP

#include "asset_config.hpp"
#include "asset_record.hpp"
#include <engine/file_asset_store/capabilities.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace engine::file_asset_store
{

class AssetLoader
{
public:
    AssetLoader() = default;

    void configure(const AssetStoreConfig &config) noexcept
    {
        config_ = &config;
    }

    // Stats the file and returns its size, or sets error (FileNotFound on
    // stat failure, FileTooLarge when above the configured cap or the
    // addressable/streamsize range).
    std::pair<std::uint64_t, AssetError> stat(const std::filesystem::path &path) const noexcept;

    // Reads the file into a freshly allocated buffer. Returns the buffer
    // (empty on failure with error set) and reports the byte count via
    // *outSize. Pre-conditions: size has already passed the maximum-file
    // cap and (for cooked records) the expected-size check.
    std::vector<std::uint8_t> read(const std::filesystem::path &path, std::uint64_t expectedSize, AssetError *error) const;

    // Verifies the SHA-256 of the loaded bytes against the expected hex
    // digest. Empty expectedSha256 disables verification and returns true.
    bool verifyHash(const std::vector<std::uint8_t> &bytes, const std::string &expectedSha256) const;

private:
    const AssetStoreConfig *config_ = nullptr;
};

} // namespace engine::file_asset_store

#endif
