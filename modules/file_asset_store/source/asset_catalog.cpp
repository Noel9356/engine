// Asset catalog and cook pipeline implementation.
//
// Catalog files use a compact little-endian binary format guarded by magic,
// version, entry-count, and per-entry canonical-key checks. Cooked objects
// are stored content-addressed by SHA-256 and installed with a temporary
// file plus rename so readers never observe partially written data.

#include <engine/file_asset_store/catalog.hpp>
#include <engine/kernel/sha256.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <new>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine::file_asset_store
{
namespace
{
// Every catalog file starts with this magic ("ENGINEAC").
constexpr std::array<char, 8> catalogMagic = {'E', 'N', 'G', 'I', 'N', 'E', 'A', 'C'};
// Bump when the on-disk layout changes; readers reject other versions.
constexpr std::uint32_t catalogVersion = 1;
// Size caps protecting readers/writers from absurd allocations.
constexpr std::uint64_t maximumCatalogBytes = 256ull * 1024ull * 1024ull;
constexpr std::uint32_t maximumEntries      = 4'000'000;
constexpr std::uint32_t maximumPathBytes    = 64'000;
// Process-wide sequence that keeps temporary names unique across threads.
std::atomic<std::uint64_t> temporaryFileCounter{0};

// Collision-proof temporary name: timestamp + thread id + atomic sequence.
std::filesystem::path makeTemporaryPath(const std::filesystem::path &path)
{
    const auto timestamp = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto threadId  = static_cast<std::uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    const auto sequence  = temporaryFileCounter.fetch_add(1, std::memory_order_relaxed);
    return path.string() + ".tmp." + std::to_string(timestamp) + "." + std::to_string(threadId) + "."
         + std::to_string(sequence);
}

// Append helpers emitting little-endian bytes.
void appendU32(std::vector<std::uint8_t> &bytes, std::uint32_t value)
{
    for (unsigned shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffu));
}

void appendU64(std::vector<std::uint8_t> &bytes, std::uint64_t value)
{
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffu));
}

// Bounds-checked little-endian reads; false when not enough bytes remain.
bool readU32(const std::vector<std::uint8_t> &bytes, std::size_t &offset, std::uint32_t &out)
{
    if (offset > bytes.size() || bytes.size() - offset < 4)
        return false;
    out = 0;
    for (unsigned shift = 0; shift < 32; shift += 8)
        out |= static_cast<std::uint32_t>(bytes[offset++]) << shift;
    return true;
}

bool readU64(const std::vector<std::uint8_t> &bytes, std::size_t &offset, std::uint64_t &out)
{
    if (offset > bytes.size() || bytes.size() - offset < 8)
        return false;
    out = 0;
    for (unsigned shift = 0; shift < 64; shift += 8)
        out |= static_cast<std::uint64_t>(bytes[offset++]) << shift;
    return true;
}

// Lexical containment test; a relative path starting with ".." (or an
// unrelated absolute path) counts as outside the root.
bool pathIsWithin(const std::filesystem::path &path, const std::filesystem::path &root)
{
    const std::filesystem::path relative = path.lexically_relative(root);
    if (relative.empty())
        return path == root;
    if (relative.is_absolute())
        return false;
    const auto first = relative.begin();
    return first != relative.end() && *first != "..";
}

// Installs one content-addressed object. Reuses an existing object when its
// size and SHA-256 already match; otherwise copies the source to a temporary
// file, re-verifies size and hash (guarding against concurrent source
// edits), then renames it into place (retrying once after removing a stale
// destination). `created` reports whether a new object was installed.
bool ensureObject(const std::filesystem::path &source, const std::filesystem::path &objectPath,
    const std::string &digest, std::uint64_t size, bool &created, std::string &error)
{
    error.clear();
    std::error_code filesystemError;
    if (std::filesystem::is_regular_file(objectPath, filesystemError) && !filesystemError
        && std::filesystem::file_size(objectPath, filesystemError) == size && !filesystemError)
    {
        const std::string existingDigest = engine::Sha256::fileHexDigest(objectPath, error);
        if (!existingDigest.empty() && engine::sha256HexDigestsEqual(existingDigest, digest))
        {
            created = false;
            return true;
        }
    }

    filesystemError.clear();
    std::filesystem::create_directories(objectPath.parent_path(), filesystemError);
    if (filesystemError)
    {
        error = "cannot create object directory: " + filesystemError.message();
        return false;
    }

    const std::filesystem::path temporaryPath = makeTemporaryPath(objectPath);
    filesystemError.clear();
    std::filesystem::copy_file(
        source, temporaryPath, std::filesystem::copy_options::overwrite_existing, filesystemError);
    if (filesystemError)
    {
        error = "cannot copy asset object: " + filesystemError.message();
        return false;
    }

    const std::uintmax_t copiedSize = std::filesystem::file_size(temporaryPath, filesystemError);
    if (filesystemError || copiedSize != size)
    {
        std::filesystem::remove(temporaryPath, filesystemError);
        error = "source changed size or object copy failed size verification";
        return false;
    }
    const std::string copiedDigest = engine::Sha256::fileHexDigest(temporaryPath, error);
    if (copiedDigest.empty() || !engine::sha256HexDigestsEqual(copiedDigest, digest))
    {
        std::filesystem::remove(temporaryPath, filesystemError);
        error = "source changed or object copy failed hash verification";
        return false;
    }

    filesystemError.clear();
    std::filesystem::rename(temporaryPath, objectPath, filesystemError);
    if (filesystemError)
    {
        std::error_code removeError;
        std::filesystem::remove(objectPath, removeError);
        filesystemError.clear();
        std::filesystem::rename(temporaryPath, objectPath, filesystemError);
    }
    if (filesystemError)
    {
        error = "cannot install asset object: " + filesystemError.message();
        std::filesystem::remove(temporaryPath, filesystemError);
        return false;
    }
    created = true;
    return true;
}

} // namespace

bool normalizeAssetKey(const std::string &input, std::string &output)
{
    output.clear();
    if (input.empty() || input.size() > maximumPathBytes)
        return false;

    std::string portable = input;
    std::replace(portable.begin(), portable.end(), '\\', '/');
    const std::filesystem::path path(portable);
    if (path.is_absolute() || path.has_root_name() || path.has_root_directory())
        return false;

    // Drops empty and "." segments; each remaining segment is validated below.
    std::vector<std::string> segments;
    for (const auto &part : path)
    {
        const std::string segment = part.generic_string();
        if (segment.empty() || segment == ".")
            continue;
        if (segment == ".." || segment.find('\0') != std::string::npos || segment.find(':') != std::string::npos)
            return false;
        segments.push_back(segment);
    }
    if (segments.empty())
        return false;
    for (const std::string &segment : segments)
    {
        if (!output.empty())
            output.push_back('/');
        output += segment;
    }
    return output.size() <= maximumPathBytes;
}

std::filesystem::path assetObjectPath(const std::filesystem::path &objectDirectory, const std::string &sha256)
{
    if (!engine::isValidSha256HexDigest(sha256))
        return {};
    const std::string digest = [&]
    {
        std::string value = sha256;
        std::transform(value.begin(),
            value.end(),
            value.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value;
    }();
    return objectDirectory / digest.substr(0, 2) / digest;
}

bool readAssetCatalog(const std::filesystem::path &path, std::vector<AssetCatalogEntry> &outEntries, std::string &error)
{
    // On-disk layout: 8-byte magic, u32 version, u32 entry count, then per
    // entry: u32 pathSize, u64 sizeBytes, 32 raw SHA-256 bytes, pathSize key
    // bytes. Keys must already be canonical and unique; trailing data fails.
    error.clear();
    outEntries.clear();
    std::error_code      filesystemError;
    const std::uintmax_t fileSize = std::filesystem::file_size(path, filesystemError);
    if (filesystemError || fileSize < catalogMagic.size() + 8u || fileSize > maximumCatalogBytes)
    {
        error = filesystemError ? "cannot stat asset catalog: " + filesystemError.message()
                                : "asset catalog has an invalid size";
        return false;
    }
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        error = "cannot open asset catalog";
        return false;
    }
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(fileSize));
    stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (stream.gcount() != static_cast<std::streamsize>(bytes.size()) || stream.peek() != std::char_traits<char>::eof())
    {
        error = "cannot read complete asset catalog";
        return false;
    }
    if (!std::equal(catalogMagic.begin(), catalogMagic.end(), bytes.begin()))
    {
        error = "asset catalog magic does not match";
        return false;
    }
    std::size_t   offset  = catalogMagic.size();
    std::uint32_t version = 0;
    std::uint32_t count   = 0;
    if (!readU32(bytes, offset, version) || !readU32(bytes, offset, count) || version != catalogVersion
        || count > maximumEntries)
    {
        error = "asset catalog version or entry count is invalid";
        return false;
    }

    try
    {
        outEntries.reserve(count);
        std::unordered_set<std::string> keys;
        keys.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index)
        {
            std::uint32_t pathSize  = 0;
            std::uint64_t sizeBytes = 0;
            if (!readU32(bytes, offset, pathSize) || !readU64(bytes, offset, sizeBytes) || pathSize == 0
                || pathSize > maximumPathBytes || offset > bytes.size()
                || bytes.size() - offset < engine::Sha256::digestSize + pathSize)
            {
                error = "asset catalog entry is truncated or invalid";
                outEntries.clear();
                return false;
            }
            std::array<std::uint8_t, engine::Sha256::digestSize> digestBytes{};
            std::copy_n(bytes.data() + offset, digestBytes.size(), digestBytes.data());
            offset += digestBytes.size();
            std::string digest;
            digest.reserve(engine::Sha256::hexDigestSize);
            // Render the raw digest bytes as lowercase hex.
            static constexpr char hex[] = "0123456789abcdef";
            for (const std::uint8_t byte : digestBytes)
            {
                digest.push_back(hex[byte >> 4u]);
                digest.push_back(hex[byte & 0x0fu]);
            }
            std::string key(reinterpret_cast<const char *>(bytes.data() + offset), pathSize);
            offset += pathSize;
            std::string normalized;
            if (!normalizeAssetKey(key, normalized) || normalized != key || !keys.insert(key).second)
            {
                error = "asset catalog contains a non-canonical or duplicate key";
                outEntries.clear();
                return false;
            }
            outEntries.push_back({std::move(key), std::move(digest), sizeBytes});
        }
        if (offset != bytes.size())
        {
            error = "asset catalog has trailing data";
            outEntries.clear();
            return false;
        }
        return true;
    }
    catch (const std::bad_alloc &)
    {
        outEntries.clear();
        error = "insufficient memory to load asset catalog";
        return false;
    }
}

bool writeAssetCatalog(
    const std::filesystem::path &path, const std::vector<AssetCatalogEntry> &entries, std::string &error)
{
    error.clear();
    if (entries.size() > maximumEntries)
    {
        error = "asset catalog contains too many entries";
        return false;
    }
    try
    {
        // Sorting by key makes output byte-identical for equal input sets.
        std::vector<AssetCatalogEntry> sorted = entries;
        std::sort(sorted.begin(),
            sorted.end(),
            [](const AssetCatalogEntry &left, const AssetCatalogEntry &right)
            { return left.logicalPath < right.logicalPath; });
        std::vector<std::uint8_t> bytes;
        bytes.reserve(catalogMagic.size() + 8u + sorted.size() * 64u);
        bytes.insert(bytes.end(), catalogMagic.begin(), catalogMagic.end());
        appendU32(bytes, catalogVersion);
        appendU32(bytes, static_cast<std::uint32_t>(sorted.size()));
        std::string previous;
        for (const AssetCatalogEntry &entry : sorted)
        {
            std::string normalized;
            if (!normalizeAssetKey(entry.logicalPath, normalized) || normalized != entry.logicalPath
                || entry.logicalPath == previous || !engine::isValidSha256HexDigest(entry.sha256)
                || entry.logicalPath.size() > maximumPathBytes)
            {
                error = "asset catalog contains an invalid or duplicate entry";
                return false;
            }
            previous                     = entry.logicalPath;
            const std::uint32_t pathSize = static_cast<std::uint32_t>(entry.logicalPath.size());
            appendU32(bytes, pathSize);
            appendU64(bytes, entry.sizeBytes);
            std::array<std::uint8_t, engine::Sha256::digestSize> digest{};
            // Pack the hex digest text back into 32 raw bytes.
            for (std::size_t index = 0; index < digest.size(); ++index)
            {
                const auto nibble = [](char value) -> std::uint8_t
                {
                    if (value >= '0' && value <= '9')
                        return static_cast<std::uint8_t>(value - '0');
                    if (value >= 'a' && value <= 'f')
                        return static_cast<std::uint8_t>(value - 'a' + 10);
                    if (value >= 'A' && value <= 'F')
                        return static_cast<std::uint8_t>(value - 'A' + 10);
                    return 0;
                };
                digest[index] = static_cast<std::uint8_t>(
                    (nibble(entry.sha256[index * 2]) << 4u) | nibble(entry.sha256[index * 2 + 1]));
            }
            bytes.insert(bytes.end(), digest.begin(), digest.end());
            bytes.insert(bytes.end(), entry.logicalPath.begin(), entry.logicalPath.end());
        }
        if (bytes.size() > maximumCatalogBytes)
        {
            error = "asset catalog exceeds the runtime catalog size limit";
            return false;
        }
        std::error_code filesystemError;
        if (!path.parent_path().empty())
            std::filesystem::create_directories(path.parent_path(), filesystemError);
        if (filesystemError)
        {
            error = "cannot create catalog directory: " + filesystemError.message();
            return false;
        }
        const std::filesystem::path temporaryPath = makeTemporaryPath(path);
        // Write the whole file first, then rename it into place so concurrent
        // readers see either the old or the new catalog, never a partial one.
        {
            std::ofstream stream(temporaryPath, std::ios::binary | std::ios::trunc);
            if (!stream)
            {
                error = "cannot create temporary asset catalog";
                return false;
            }
            stream.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            stream.flush();
            if (!stream)
            {
                stream.close();
                std::filesystem::remove(temporaryPath, filesystemError);
                error = "cannot write complete asset catalog";
                return false;
            }
        }
        filesystemError.clear();
        std::filesystem::rename(temporaryPath, path, filesystemError);
        if (filesystemError)
        {
            std::error_code removeError;
            std::filesystem::remove(path, removeError);
            filesystemError.clear();
            std::filesystem::rename(temporaryPath, path, filesystemError);
        }
        if (filesystemError)
        {
            error = "cannot install asset catalog: " + filesystemError.message();
            std::filesystem::remove(temporaryPath, filesystemError);
            return false;
        }
        return true;
    }
    catch (const std::bad_alloc &)
    {
        error = "insufficient memory to write asset catalog";
        return false;
    }
}

bool cookAssetDirectory(const std::filesystem::path &sourceDirectory, const std::filesystem::path &outputDirectory,
    const std::filesystem::path &catalogPath, AssetCookStatistics &outStatistics, std::string &error)
{
    outStatistics = {};
    error.clear();
    // Refuse to cook in place: object store and catalog must both live
    // outside the source tree, checked before and after resolving symlinks.
    std::error_code filesystemError;
    const auto      source = std::filesystem::canonical(sourceDirectory, filesystemError);
    if (filesystemError || !std::filesystem::is_directory(source))
    {
        error = "source directory does not exist or is not a directory";
        return false;
    }
    const auto outputAbsolute = std::filesystem::absolute(outputDirectory, filesystemError).lexically_normal();
    if (filesystemError)
    {
        error = "cannot resolve output directory: " + filesystemError.message();
        return false;
    }
    const auto outputResolved = std::filesystem::weakly_canonical(outputAbsolute, filesystemError);
    if (filesystemError || pathIsWithin(outputResolved, source))
    {
        error = "output directory must be outside the source directory";
        return false;
    }
    std::filesystem::create_directories(outputResolved, filesystemError);
    if (filesystemError)
    {
        error = "cannot create output directory: " + filesystemError.message();
        return false;
    }
    const auto output = std::filesystem::canonical(outputResolved, filesystemError);
    if (filesystemError || pathIsWithin(output, source))
    {
        error = "resolved output directory must be outside the source directory";
        return false;
    }

    std::vector<std::filesystem::path> sources;
    try
    {
        // Walk the tree manually (not range-for) so enumeration errors are
        // visible and symlinks can be skipped without recursing into them.
        std::filesystem::recursive_directory_iterator iterator(
            source, std::filesystem::directory_options::none, filesystemError);
        if (filesystemError)
        {
            error = "cannot enumerate source directory: " + filesystemError.message();
            return false;
        }
        const std::filesystem::recursive_directory_iterator end;
        for (; iterator != end; iterator.increment(filesystemError))
        {
            if (filesystemError)
            {
                error = "cannot enumerate source directory: " + filesystemError.message();
                return false;
            }
            const auto status = iterator->symlink_status(filesystemError);
            if (filesystemError)
            {
                error = "cannot inspect source asset: " + filesystemError.message();
                return false;
            }
            if (std::filesystem::is_symlink(status))
            {
                iterator.disable_recursion_pending();
                continue;
            }
            if (std::filesystem::is_regular_file(status))
                sources.push_back(iterator->path());
        }
        if (filesystemError)
        {
            error = "cannot enumerate source directory: " + filesystemError.message();
            return false;
        }
        std::sort(sources.begin(), sources.end());
        std::vector<AssetCatalogEntry> entries;
        entries.reserve(sources.size());
        // Digests already installed as objects; deduplicates the store.
        std::unordered_set<std::string> uniqueDigests;
        uniqueDigests.reserve(sources.size());
        for (const auto &sourcePath : sources)
        {
            const std::string relative = sourcePath.lexically_relative(source).generic_string();
            std::string       key;
            if (!normalizeAssetKey(relative, key))
            {
                error = "source contains an invalid logical asset path: " + relative;
                return false;
            }
            const std::uintmax_t size = std::filesystem::file_size(sourcePath, filesystemError);
            if (filesystemError || size > std::numeric_limits<std::uint64_t>::max())
            {
                error = "cannot determine asset size: " + sourcePath.string();
                return false;
            }
            const std::string digest = engine::Sha256::fileHexDigest(sourcePath, error);
            if (digest.empty())
            {
                error = "cannot hash asset " + sourcePath.string() + ": " + error;
                return false;
            }
            bool created = false;
            if (!ensureObject(sourcePath,
                    assetObjectPath(output, digest),
                    digest,
                    static_cast<std::uint64_t>(size),
                    created,
                    error))
                return false;
            if (outStatistics.sourceBytes > std::numeric_limits<std::uint64_t>::max() - size)
            {
                error = "total source asset size overflows the catalog statistics range";
                return false;
            }
            ++outStatistics.filesProcessed;
            outStatistics.sourceBytes += static_cast<std::uint64_t>(size);
            if (uniqueDigests.insert(digest).second)
            {
                if (outStatistics.storedBytes > std::numeric_limits<std::uint64_t>::max() - size)
                {
                    error = "total cooked asset size overflows the catalog statistics range";
                    return false;
                }
                ++outStatistics.uniqueObjects;
                outStatistics.storedBytes += static_cast<std::uint64_t>(size);
            }
            entries.push_back({std::move(key), digest, static_cast<std::uint64_t>(size)});
        }

        // Default catalog location is <output>/catalog.eac; a relative
        // catalogPath is resolved inside the output directory.
        const auto resolvedCatalog = catalogPath.empty()
                                       ? output / "catalog.eac"
                                       : (catalogPath.is_absolute() ? catalogPath : output / catalogPath);
        const auto catalogParent   = std::filesystem::weakly_canonical(resolvedCatalog.parent_path(), filesystemError);
        if (filesystemError || resolvedCatalog.filename().empty()
            || pathIsWithin(catalogParent / resolvedCatalog.filename(), source))
        {
            error = "catalog file must be outside the source directory";
            return false;
        }
        if (!writeAssetCatalog(catalogParent / resolvedCatalog.filename(), entries, error))
            return false;
        return true;
    }
    catch (const std::filesystem::filesystem_error &exception)
    {
        error = exception.what();
        return false;
    }
    catch (const std::bad_alloc &)
    {
        error = "insufficient memory while cooking assets";
        return false;
    }
}

} // namespace engine::file_asset_store
