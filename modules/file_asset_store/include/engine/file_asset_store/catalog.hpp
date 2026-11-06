// Asset catalog primitives shared by the module and offline cook tooling.
//
// Provides canonical logical-path normalization, a strict binary catalog
// format (read/write), content-addressed object paths, and the directory cook
// that fills an object store from a source tree. All functions report
// failure by returning false with a message in `error`.

#ifndef ENGINE_FILE_ASSET_STORE_CATALOG_HPP
#define ENGINE_FILE_ASSET_STORE_CATALOG_HPP

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace engine::file_asset_store
{

// One catalog row: canonical logical path, lowercase hex SHA-256 of the
// content, and the object size in bytes.
struct AssetCatalogEntry
{
    std::string   logicalPath;
    std::string   sha256;
    std::uint64_t sizeBytes = 0;
};

// Cook-run counters: per-file totals and the deduplicated object store
// totals; byte fields are bytes, count fields are files/objects.
struct AssetCookStatistics
{
    std::uint64_t filesProcessed = 0;
    std::uint64_t uniqueObjects  = 0;
    std::uint64_t sourceBytes    = 0;
    std::uint64_t storedBytes    = 0;
};

// Canonicalizes a logical asset key: backslashes become '/', empty and "."
// segments are dropped, and "..", absolute paths, ':', NUL, or results over
// 64,000 bytes are rejected. Returns false unless output is the canonical
// form of a safe relative path.
bool normalizeAssetKey(const std::string &input, std::string &output);
// Parses a binary catalog (magic + version + count + entries). On any
// problem clears outEntries, sets error, and returns false.
bool readAssetCatalog(
    const std::filesystem::path &path, std::vector<AssetCatalogEntry> &outEntries, std::string &error);
// Serializes entries sorted by logical path (deterministic output) after
// validating each one; installs the file atomically via temp file + rename.
bool writeAssetCatalog(
    const std::filesystem::path &path, const std::vector<AssetCatalogEntry> &entries, std::string &error);
// Content-addressed layout: <objectDirectory>/<first 2 digest chars>/<full
// lowercase digest>. Returns an empty path for an invalid digest.
std::filesystem::path assetObjectPath(const std::filesystem::path &objectDirectory, const std::string &sha256);
// Cooks every regular file under sourceDirectory into the object store
// (deduplicated by SHA-256) and writes the catalog; output directory and
// catalog file must live outside the source tree. Symlinks are skipped.
bool cookAssetDirectory(const std::filesystem::path &sourceDirectory, const std::filesystem::path &outputDirectory,
    const std::filesystem::path &catalogPath, AssetCookStatistics &outStatistics, std::string &error);

} // namespace engine::file_asset_store

#endif
