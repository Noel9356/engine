// Command-line parsing for engine applications.
//
// Splits engine options (--config/--profile/--scene/--plugin-dir/--set/
// --active-plugin/--no-plugins/--validate) from application arguments (after
// "--" or unrecognized tokens) and builds ApplicationHostOptions.

#include "engine/application/application_host.hpp"
#include <engine/application/command_line.hpp>

#include <string>
#include <utility>
#include <vector>

namespace engine
{

bool parseApplicationCommandLine(
    int argumentCount, char *arguments[], ApplicationCommandLineOptions &outOptions, std::string &error)
{
    outOptions = {};
    error.clear();
    if (argumentCount < 0 || (argumentCount > 0 && arguments == nullptr))
    {
        error = "invalid argument array";
        return false;
    }

    // Value options need a following token that is not another "--..." switch.
    const auto consumeValue = [&](int &index, const char *option, std::string &value)
    {
        if (index + 1 >= argumentCount || arguments[index + 1] == nullptr || arguments[index + 1][0] == '\0'
            || (arguments[index + 1][0] == '-' && arguments[index + 1][1] == '-'))
        {
            error = std::string("missing value for ") + option;
            return false;
        }
        value = arguments[++index];
        return true;
    };

    for (int index = 1; index < argumentCount; ++index)
    {
        if (arguments[index] == nullptr)
        {
            error = "null argument at index " + std::to_string(index);
            return false;
        }
        const std::string argument = arguments[index];
        // Everything after "--" belongs to the application verbatim.
        if (argument == "--")
        {
            for (++index; index < argumentCount; ++index)
            {
                if (arguments[index] != nullptr)
                {
                    outOptions.applicationArguments.emplace_back(arguments[index]);
                }
            }
            break;
        }
        if (argument == "--help" || argument == "-h")
        {
            outOptions.helpRequested = true;
        }
        else if (argument == "--validate")
        {
            outOptions.validateOnly = true;
            // --no-plugins and --active-plugin are two spellings of the active plugin set; they conflict.
        }
        else if (argument == "--no-plugins")
        {
            if (outOptions.activePlugins.has_value() && !outOptions.activePlugins->empty())
            {
                error = "--no-plugins cannot be combined with --active-plugin";
                return false;
            }
            outOptions.activePlugins = std::vector<std::string>{};
        }
        else if (argument == "--config" || argument == "--profile" || argument == "--scene" || argument == "--plugin-dir"
                 || argument == "--set" || argument == "--active-plugin")
        {
            std::string value;
            if (!consumeValue(index, argument.c_str(), value))
            {
                return false;
            }
            if (argument == "--config")
            {
                outOptions.configurationFile = value;
            }
            else if (argument == "--profile")
            {
                if (outOptions.configurationProfileFile.empty())
                    outOptions.configurationProfileFile = value;
                outOptions.configurationProfileFiles.emplace_back(std::move(value));
            }
            else if (argument == "--scene")
            {
                outOptions.sceneFile = value;
            }
            else if (argument == "--plugin-dir")
            {
                outOptions.pluginsDirectories.emplace_back(value);
            }
            else if (argument == "--active-plugin")
            {
                if (outOptions.activePlugins.has_value() && outOptions.activePlugins->empty())
                {
                    error = "--active-plugin cannot be combined with --no-plugins";
                    return false;
                }
                if (!outOptions.activePlugins.has_value())
                {
                    outOptions.activePlugins.emplace();
                }
                outOptions.activePlugins->push_back(std::move(value));
            }
            else
            {
                // --set requires "key=value"; staged as a ConfigurationStore override.
                if (value.find('=') == std::string::npos || value.front() == '=')
                {
                    error = "--set expects SECTION.KEY=VALUE";
                    return false;
                }
                outOptions.configurationOverrides.push_back(std::move(value));
            }
        }
        else
        {
            outOptions.applicationArguments.push_back(argument);
        }
    }
    return true;
}

// Pure projection of the parsed options; requiredCapabilities comes from the app,
// not from the command line.
ApplicationHostOptions makeApplicationHostOptions(
    const ApplicationCommandLineOptions &commandLine, std::vector<std::string> requiredCapabilities)
{
    ApplicationHostOptions options;
    options.configurationFile             = commandLine.configurationFile;
    options.configurationProfileFile      = commandLine.configurationProfileFile;
    options.configurationProfileFiles     = commandLine.configurationProfileFiles;
    options.sceneFile                     = commandLine.sceneFile;
    options.additionalPluginsDirectories = commandLine.pluginsDirectories;
    options.configurationOverrides        = commandLine.configurationOverrides;
    options.activePlugins                = commandLine.activePlugins;
    options.requiredCapabilities          = std::move(requiredCapabilities);
    return options;
}

std::string applicationCommandLineUsage(const std::string &executableName)
{
    return "Usage: " + executableName
         + " [engine options] [application options]\n"
           "Engine options:\n"
           "  --config <path>       Select the base engine/application TOML configuration.\n"
           "  --profile <path>      Apply a tuning profile over the base configuration; may be repeated (later "
           "profiles win).\n"
           "  --scene <path>        Select a scene configuration.\n"
           "  --plugin-dir <path>      Add a plugin search directory; may be repeated.\n"
           "  --active-plugin <id>     Activate only this plugin; may be repeated.\n"
           "  --no-plugins             Disable all plugins.\n"
           "  --set <key=value>     Override a configuration value; may be repeated.\n"
           "  --validate            Validate configuration and the participant graph.\n"
           "  --help, -h            Show this help.\n"
           "  --                    Pass all remaining arguments to the application.\n";
}

} // namespace engine
