// AssetLoader implementation: see asset_loader.hpp for the contract.

#include "asset_loader.hpp"

#include <engine/kernel/sha256.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace engine::file_asset_store
{

namespace
{

#if defined(__linux__)
// Issue posix_fadvise for the given fd; silently ignores failures (the
// hint is advisory and does not affect correctness).
void adviseFd(int fd, int advice) noexcept
{
    if (fd >= 0)
        ::posix_fadvise(fd, 0, 0, advice);
}
#endif

} // namespace

std::pair<std::uint64_t, AssetError> AssetLoader::stat(const std::filesystem::path &path) const noexcept
{
    const std::uint64_t maxFileBytes = config_ != nullptr ? config_->maximumFileBytes : 512ull * 1024ull * 1024ull;
    std::error_code      filesystemError;
    const std::uintmax_t fileSize = std::filesystem::file_size(path, filesystemError);
    if (filesystemError)
        return {0, AssetError::FileNotFound};
    if (fileSize > maxFileBytes
        || fileSize > static_cast<std::uintmax_t>(std::numeric_limits<std::size_t>::max())
        || fileSize > static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max()))
        return {0, AssetError::FileTooLarge};
    return {static_cast<std::uint64_t>(fileSize), AssetError::None};
}

std::vector<std::uint8_t> AssetLoader::read(
    const std::filesystem::path &path, std::uint64_t expectedSize, AssetError *error) const
{
    const bool readaheadHint = config_ != nullptr && config_->readaheadHint;
    const bool dropAfterRead = config_ != nullptr && config_->dropAfterRead;

    std::vector<std::uint8_t> loaded;
    try
    {
        loaded.resize(static_cast<std::size_t>(expectedSize));
    }
    catch (const std::bad_alloc &)
    {
        if (error != nullptr)
            *error = AssetError::CapacityExceeded;
        return {};
    }

#if defined(__linux__)
    // Read from an opened descriptor instead of mapping the file. O_NOFOLLOW
    // guards against the final path component being replaced by a symlink
    // between canonicalization and open (intermediate directories are still
    // followed, matching the existing behaviour). This handles empty files
    // and reports concurrent truncation as an ordinary read failure rather
    // than risking a fault while touching mapped pages.
    {
        const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if (fd < 0)
        {
            if (error != nullptr)
                *error = AssetError::FileNotFound;
            return {};
        }
        if (readaheadHint)
            adviseFd(fd, POSIX_FADV_WILLNEED);

        struct stat fileStatus{};
        if (::fstat(fd, &fileStatus) != 0 || fileStatus.st_size < 0
            || static_cast<std::uintmax_t>(fileStatus.st_size) != expectedSize)
        {
            ::close(fd);
            if (error != nullptr)
                *error = AssetError::ReadFailed;
            return {};
        }

        std::size_t offset     = 0;
        bool        readFailed = false;
        while (offset < loaded.size())
        {
            const std::size_t remaining = loaded.size() - offset;
            const std::size_t chunkSize = std::min<std::size_t>(
                remaining, static_cast<std::size_t>(std::numeric_limits<ssize_t>::max()));
            const ssize_t count = ::read(fd, loaded.data() + offset, chunkSize);
            if (count < 0 && errno == EINTR)
                continue;
            if (count <= 0)
            {
                readFailed = true;
                break;
            }
            offset += static_cast<std::size_t>(count);
        }
        if (!readFailed)
        {
            // Verify the file did not grow past the expected size between
            // fstat and the final read.
            std::uint8_t extraByte = 0;
            ssize_t     count      = 0;
            do
            {
                count = ::read(fd, &extraByte, 1u);
            } while (count < 0 && errno == EINTR);
            readFailed = count != 0;
        }
        if (dropAfterRead)
            adviseFd(fd, POSIX_FADV_DONTNEED);
        ::close(fd);
        if (readFailed)
        {
            if (error != nullptr)
                *error = AssetError::ReadFailed;
            return {};
        }
    }
#else
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        if (error != nullptr)
            *error = AssetError::FileNotFound;
        return {};
    }
    if (!loaded.empty())
    {
        stream.read(reinterpret_cast<char *>(loaded.data()), static_cast<std::streamsize>(loaded.size()));
        if (stream.gcount() != static_cast<std::streamsize>(loaded.size()))
        {
            if (error != nullptr)
                *error = AssetError::ReadFailed;
            return {};
        }
    }
    if (stream.peek() != std::char_traits<char>::eof())
    {
        if (error != nullptr)
            *error = AssetError::ReadFailed;
        return {};
    }
#endif
    if (error != nullptr)
        *error = AssetError::None;
    return loaded;
}

bool AssetLoader::verifyHash(const std::vector<std::uint8_t> &bytes, const std::string &expectedSha256) const
{
    if (expectedSha256.empty())
        return true;
    const std::string digest = engine::Sha256::hexDigest(bytes.data(), bytes.size());
    return engine::sha256HexDigestsEqual(digest, expectedSha256);
}

} // namespace engine::file_asset_store
