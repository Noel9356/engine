// Implementation of the shared DEMO_* environment controls; see
// demo_environment.hpp.
#include "demo_environment.hpp"

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace demo::environment
{
namespace
{

bool affirmative(std::string value)
{
    for (char &character : value)
    {
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    }
    return value == "1" || value == "true" || value == "on" || value == "yes";
}

bool negative(std::string value)
{
    for (char &character : value)
    {
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    }
    return value == "0" || value == "false" || value == "off" || value == "no";
}

std::string trim(const std::string &value)
{
    const std::size_t first = value.find_first_not_of(" \t");
    if (first == std::string::npos)
    {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t");
    return value.substr(first, last - first + 1);
}

} // namespace

bool readFlag(const char *name, bool &outValue, std::string &outError)
{
    const char *raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    const std::string value = trim(raw);
    if (affirmative(value))
    {
        outValue = true;
        return true;
    }
    if (negative(value))
    {
        outValue = false;
        return true;
    }
    outError = std::string("invalid ") + name + " '" + value + "'; expected 1/true/on/yes or 0/false/off/no";
    return false;
}

bool readUint64(const char *name, std::uint64_t &outValue, std::uint64_t minimum, std::uint64_t maximum,
    std::string &outError)
{
    const char *raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    const std::string value = trim(raw);
    std::size_t       parsed = 0;
    unsigned long long number = 0;
    try
    {
        number = std::stoull(value, &parsed);
    }
    catch (const std::exception &)
    {
        parsed = 0;
    }
    if (parsed != value.size() || number < minimum || number > maximum)
    {
        std::ostringstream range;
        range << minimum << ".." << maximum;
        outError = std::string("invalid ") + name + " '" + value + "'; expected an integer in " + range.str();
        return false;
    }
    outValue = static_cast<std::uint64_t>(number);
    return true;
}

bool readDouble(const char *name, double &outValue, double minimum, double maximum, std::string &outError)
{
    const char *raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    const std::string value = trim(raw);
    std::size_t       parsed = 0;
    double            number = 0.0;
    try
    {
        number = std::stod(value, &parsed);
    }
    catch (const std::exception &)
    {
        parsed = 0;
    }
    if (parsed != value.size() || number < minimum || number > maximum || number != number)
    {
        std::ostringstream range;
        range << minimum << ".." << maximum;
        outError = std::string("invalid ") + name + " '" + value + "'; expected a number in " + range.str();
        return false;
    }
    outValue = number;
    return true;
}

bool readConfigurationOverrides(std::vector<std::string> &outOverrides, std::string &outError)
{
    const char *raw = std::getenv("DEMO_SET");
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    std::istringstream stream(raw);
    std::string        assignment;
    while (std::getline(stream, assignment, ';'))
    {
        assignment = trim(assignment);
        if (assignment.empty())
        {
            continue;
        }
        const std::size_t split = assignment.find('=');
        if (split == std::string::npos || split == 0 || split + 1 == assignment.size())
        {
            outError = "invalid DEMO_SET entry '" + assignment + "'; expected semicolon-separated key=value pairs";
            return false;
        }
        outOverrides.push_back(assignment);
    }
    return true;
}

bool readProfilePaths(std::vector<std::string> &outPaths, std::string &outError)
{
    const char *raw = std::getenv("DEMO_PROFILES");
    if (raw == nullptr || *raw == '\0')
    {
        return true;
    }
    std::istringstream stream(raw);
    std::string        path;
    while (std::getline(stream, path, ';'))
    {
        path = trim(path);
        if (path.empty())
        {
            continue;
        }
        outPaths.push_back(path);
    }
    (void)outError;
    return true;
}

std::string timestampedScreenshotPath(const std::string &prefix, std::uint64_t index)
{
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm           local{};
#if defined(_WIN32)
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    std::ostringstream name;
    name << prefix << "_" << std::put_time(&local, "%Y%m%d_%H%M%S") << "_" << index << ".png";
    return name.str();
}

} // namespace demo::environment
