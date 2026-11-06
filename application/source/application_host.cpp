// ApplicationHost implementation: kernel lifecycle with exception safety.
//
// configureKernel maps ApplicationHostOptions onto the kernel's staged
// setters; validate() runs a throwaway kernel without native code;
// start()/stop() own the real kernel plus the application context.

#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/participant_interface.hpp"
#include <engine/application/application_host.hpp>
#include <engine/kernel/executable_directory.hpp>

#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace engine
{
namespace
{

// Relative option paths anchor to the cwd; absolute paths pass through.
std::filesystem::path resolveApplicationPath(const std::filesystem::path &path)
{
    if (path.empty() || path.is_absolute())
    {
        return path;
    }
    return std::filesystem::absolute(path).lexically_normal();
}

// config/engine.toml next to the cwd wins over the one beside the executable.
std::filesystem::path defaultConfigurationPath()
{
    const std::filesystem::path workingDirectoryPath = std::filesystem::current_path() / "config" / "engine.toml";
    if (std::filesystem::exists(workingDirectoryPath))
    {
        return workingDirectoryPath;
    }
    return executableDirectory() / "config" / "engine.toml";
}

} // namespace

ApplicationHost::~ApplicationHost()
{
    stop();
}

// Translates options into kernel setters; paths are resolved at configure time so a
// later chdir cannot shift their meaning.
bool ApplicationHost::configureKernel(EngineKernel &kernel, const ApplicationHostOptions &options, std::string &report)
{
    const std::filesystem::path configurationFile = options.configurationFile.empty()
                                                      ? defaultConfigurationPath()
                                                      : resolveApplicationPath(options.configurationFile);
    kernel.setConfigurationFile(configurationFile);
    if (!options.configurationProfileFiles.empty())
    {
        for (const std::filesystem::path &profile : options.configurationProfileFiles)
        {
            if (profile.empty())
            {
                report = "application host: a configuration profile path is empty\n";
                return false;
            }
            kernel.addConfigurationProfileFile(resolveApplicationPath(profile));
        }
    }
    else if (!options.configurationProfileFile.empty())
    {
        kernel.setConfigurationProfileFile(resolveApplicationPath(options.configurationProfileFile));
    }

    if (!options.sceneFile.empty())
    {
        kernel.setSceneFile(resolveApplicationPath(options.sceneFile));
    }
    if (!options.gameModsDirectory.empty())
    {
        kernel.setGameModsDirectory(resolveApplicationPath(options.gameModsDirectory));
    }
    for (const std::filesystem::path &directory : options.additionalGameModsDirectories)
    {
        if (directory.empty())
        {
            report = "application host: an additional game-mod directory is empty\n";
            return false;
        }
        kernel.addGameModsDirectory(resolveApplicationPath(directory));
    }

    kernel.setApplicationRequirements(options.requiredCapabilities);
    if (options.activeGameMods.has_value())
    {
        kernel.setActiveGameMods(*options.activeGameMods);
    }
    kernel.setTrustConsentHandler(options.trustConsentHandler);

    for (const std::string &assignment : options.configurationOverrides)
    {
        std::string error;
        if (!kernel.addCommandLineOverride(assignment, error))
        {
            report = "application host: invalid configuration override '" + assignment + "': " + error + "\n";
            return false;
        }
    }
    return true;
}

// Dry run on a throwaway kernel: full configuration and graph validation, with the
// participant graph dump appended to the report.
bool ApplicationHost::validate(const ApplicationHostOptions &options, std::string &report) const
{
    report.clear();
    if (kernel_ != nullptr)
    {
        report = "application host: stop the running host before validating another configuration\n";
        return false;
    }

    try
    {
        EngineKernel candidate;
        if (!configureKernel(candidate, options, report))
        {
            return false;
        }

        const bool        valid = candidate.validate(report);
        const std::string graph = candidate.participantGraphReport();
        if (!graph.empty())
        {
            if (!report.empty() && report.back() != '\n')
            {
                report.push_back('\n');
            }
            report += graph;
        }
        return valid;
    }
    catch (const std::exception &exception)
    {
        report += "application host: validation exception: ";
        report += exception.what();
        report.push_back('\n');
    }
    catch (...)
    {
        report += "application host: validation failed with an unknown exception\n";
    }
    return false;
}

// Constructs and starts the real kernel, then creates the application context; on
// any exception the half-built kernel is destroyed and the host stays uninitialized.
bool ApplicationHost::start(const ApplicationHostOptions &options, std::string &report)
{
    report.clear();
    if (kernel_ != nullptr)
    {
        report = "application host: the host is already initialized\n";
        return false;
    }

    std::unique_ptr<EngineKernel> candidate;
    try
    {
        candidate = std::make_unique<EngineKernel>();
        if (!configureKernel(*candidate, options, report))
        {
            return false;
        }
        if (!candidate->construct(report))
        {
            return false;
        }
        if (!report.empty() && report.back() != '\n')
        {
            report.push_back('\n');
        }
        if (!candidate->startupWarnings().empty())
        {
            report += candidate->startupWarnings();
        }

        candidate->start();
        auto applicationContext = candidate->createAppContext();
        kernel_                 = std::move(candidate);
        context_                = std::move(applicationContext);
        return true;
    }
    catch (const std::exception &exception)
    {
        report += "application host: startup exception: ";
        report += exception.what();
        report.push_back('\n');
    }
    catch (...)
    {
        report += "application host: startup failed with an unknown exception\n";
    }
    return false;
}

// Context first (participants may still use it), then the kernel; never throws.
void ApplicationHost::stop() noexcept
{
    context_.reset();
    if (kernel_ != nullptr)
    {
        try
        {
            kernel_->stop();
        }
        catch (...)
        {}
        kernel_.reset();
    }
}

bool ApplicationHost::isStarted() const noexcept
{
    return kernel_ != nullptr && kernel_->isStarted();
}

EngineKernel &ApplicationHost::kernel()
{
    if (kernel_ == nullptr)
    {
        throw std::logic_error("application host: kernel is not initialized");
    }
    return *kernel_;
}

const EngineKernel &ApplicationHost::kernel() const
{
    if (kernel_ == nullptr)
    {
        throw std::logic_error("application host: kernel is not initialized");
    }
    return *kernel_;
}

ParticipantContext &ApplicationHost::context()
{
    if (context_ == nullptr)
    {
        throw std::logic_error("application host: application context is not initialized");
    }
    return *context_;
}

const ParticipantContext &ApplicationHost::context() const
{
    if (context_ == nullptr)
    {
        throw std::logic_error("application host: application context is not initialized");
    }
    return *context_;
}

} // namespace engine
