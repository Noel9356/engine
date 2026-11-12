// ApplicationHost: owns a constructed EngineKernel and the application's
// ParticipantContext.
//
// ApplicationHostOptions mirrors the kernel's staged setters; validate()
// runs on a throwaway kernel and never touches the host's own one.

#ifndef ENGINE_APPLICATION_APPLICATION_HOST_HPP
#define ENGINE_APPLICATION_APPLICATION_HOST_HPP

#include "engine/kernel/participant_interface.hpp"
#include <engine/kernel/engine_kernel.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace engine
{

struct ApplicationHostOptions
{
    std::filesystem::path              configurationFile;
    std::filesystem::path              sceneFile;
    std::filesystem::path              pluginsDirectory;
    std::vector<std::filesystem::path> additionalPluginsDirectories;

    std::vector<std::string> requiredCapabilities;
    // nullopt lets the configuration decide; an explicit list filters discovered plugins
    // (an empty list means none).
    std::optional<std::vector<std::string>> activePlugins;
    // Staged "key=value" TOML overrides, applied after profiles.
    std::vector<std::string> configurationOverrides;

    TrustConsentHandler trustConsentHandler;

    // Legacy single-profile slot; ignored when configurationProfileFiles is non-empty.
    std::filesystem::path              configurationProfileFile;
    std::vector<std::filesystem::path> configurationProfileFiles;
};

class ApplicationHost final
{
public:
    ApplicationHost() = default;
    ~ApplicationHost();

    ApplicationHost(const ApplicationHost &)            = delete;
    ApplicationHost &operator=(const ApplicationHost &) = delete;

    bool validate(const ApplicationHostOptions &options, std::string &report) const;

    bool start(const ApplicationHostOptions &options, std::string &report);

    void stop() noexcept;

    [[nodiscard]] bool isStarted() const noexcept;

    [[nodiscard]] EngineKernel             &kernel();
    [[nodiscard]] const EngineKernel       &kernel() const;
    [[nodiscard]] ParticipantContext       &context();
    [[nodiscard]] const ParticipantContext &context() const;

private:
    static bool configureKernel(EngineKernel &kernel, const ApplicationHostOptions &options, std::string &report);

    std::unique_ptr<EngineKernel>       kernel_  = nullptr;
    std::unique_ptr<ParticipantContext> context_ = nullptr;
};

} // namespace engine

#endif
