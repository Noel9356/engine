// ApplicationHost implementation: kernel lifecycle with exception safety.
//
// configureKernel maps ApplicationHostOptions onto the kernel's staged
// setters; validate() runs a throwaway kernel without native code;
// start()/stop() own the real kernel plus the application context.

#include <engine/application/application_host.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/kernel/participant_interface.hpp>

#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace engine
{

// The application's own installed configuration wins: config/engine.toml
// beside the executable over one in the working directory. The working
// directory depends on how the application was launched and can sit inside an
// unrelated tree (a repository root whose configuration belongs to other
// applications) - silently swapping trust bindings, plugin directories, and
// module settings with the launch directory's copies breaks the application,
// so a divergent working-directory copy earns a loud note on stderr while the
// executable's own configuration stays in charge.
std::filesystem::path defaultEngineConfigurationPath()
{
    const std::filesystem::path besideExecutable = executableDirectory() / "config" / "engine.toml";
    if (std::filesystem::exists(besideExecutable))
    {
        const std::filesystem::path workingDirectoryPath = std::filesystem::current_path() / "config" / "engine.toml";
        if (std::error_code error; std::filesystem::exists(workingDirectoryPath)
            && !std::filesystem::equivalent(workingDirectoryPath, besideExecutable, error) && !error)
        {
            std::fprintf(stderr,
                "engine: using the configuration beside the executable (%s), not the one from the working "
                "directory (%s); an application runs on its own installed configuration\n",
                besideExecutable.string().c_str(), workingDirectoryPath.string().c_str());
        }
        return besideExecutable;
    }
    const std::filesystem::path workingDirectoryPath = std::filesystem::current_path() / "config" / "engine.toml";
    if (std::filesystem::exists(workingDirectoryPath))
    {
        return workingDirectoryPath;
    }
    return besideExecutable;
}
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
                                                      ? defaultEngineConfigurationPath()
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
        // configureKernel only ever runs on a freshly constructed kernel
        // (validate() and start() each build their own candidate), so
        // appending reproduces the retired single-profile setter exactly.
        kernel.addConfigurationProfileFile(resolveApplicationPath(options.configurationProfileFile));
    }

    if (!options.sceneFile.empty())
    {
        kernel.setSceneFile(resolveApplicationPath(options.sceneFile));
    }
    if (!options.pluginsDirectory.empty())
    {
        kernel.setPluginsDirectory(resolveApplicationPath(options.pluginsDirectory));
    }
    for (const std::filesystem::path &directory : options.additionalPluginsDirectories)
    {
        if (directory.empty())
        {
            report = "application host: an additional plugin directory is empty\n";
            return false;
        }
        kernel.addPluginsDirectory(resolveApplicationPath(directory));
    }

    kernel.setApplicationRequirements(options.requiredCapabilities);
    if (options.activePlugins.has_value())
    {
        kernel.setActivePlugins(*options.activePlugins);
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
