// Application command-line options and parsing entry points.
//
// Engine options are separated from application arguments; the result
// converts to ApplicationHostOptions via makeApplicationHostOptions.

#ifndef ENGINE_APPLICATION_COMMAND_LINE_HPP
#define ENGINE_APPLICATION_COMMAND_LINE_HPP

#include <engine/application/application_host.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace engine
{

struct ApplicationCommandLineOptions
{
    std::filesystem::path              configurationFile;
    std::filesystem::path              sceneFile;
    std::vector<std::filesystem::path> gameModsDirectories;
    std::vector<std::string>           configurationOverrides;
    // nullopt = no explicit set; an empty list (--no-mods) = run without mods.
    std::optional<std::vector<std::string>> activeGameMods;
    std::vector<std::string>                applicationArguments;
    bool                                    validateOnly  = false;
    bool                                    helpRequested = false;
    std::filesystem::path                   configurationProfileFile;
    std::vector<std::filesystem::path>      configurationProfileFiles;
};

// argv-style parse; false with a reason in "error" on malformed input.
bool parseApplicationCommandLine(
    int argumentCount, char *arguments[], ApplicationCommandLineOptions &outOptions, std::string &error);

ApplicationHostOptions makeApplicationHostOptions(
    const ApplicationCommandLineOptions &commandLine, std::vector<std::string> requiredCapabilities = {});

std::string applicationCommandLineUsage(const std::string &executableName);

} // namespace engine

#endif
