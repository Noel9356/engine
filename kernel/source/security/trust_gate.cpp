// TrustGate implementation: risk advisories and trust decisions.
//
// Advisories hash participant content (the shared library for native
// code, a deterministic bundle for script packages); evaluate() and
// decideScanConsent enforce [trust] bindings or ask the consent handler.

#include "trust_gate.hpp"
#include "engine/kernel/configuration.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/module_descriptor.hpp"
#include "engine/kernel/permission_system.hpp"
#include "engine/kernel/sha256.hpp"
#include "kernel_internals.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <engine/kernel/plugin_discovery.hpp>
#include <engine/kernel/plugin_manifest.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace engine
{

namespace
{

// A binding matches on name; version and sha256 narrow the match when present (a
// binding without sha256 is name-only trust).
bool trustBindingMatches(const TrustBinding &binding, const ParticipantTrustAdvisory &advisory)
{
    if (binding.name != advisory.name)
    {
        return false;
    }
    if (!binding.version.empty() && binding.version != advisory.version)
    {
        return false;
    }
    if (!binding.sha256.empty() && (advisory.sha256.empty() || !sha256HexDigestsEqual(binding.sha256, advisory.sha256)))
    {
        return false;
    }
    return true;
}

// Bindings with a malformed sha256 degrade to name-only matching, with a warning.
bool anyTrustBindingMatches(const std::vector<TrustBinding> &bindings, const ParticipantTrustAdvisory &advisory,
    std::vector<std::string> &warnings)
{
    for (const TrustBinding &binding : bindings)
    {
        if (!binding.sha256.empty() && !isValidSha256HexDigest(binding.sha256))
        {
            warnings.push_back("the [trust] entry for '" + binding.name
                               + "' carries a malformed sha256 (must be 64 hexadecimal characters); it is treated "
                                 "as name-only trust for this run");
            continue;
        }
        if (trustBindingMatches(binding, advisory))
        {
            return true;
        }
    }
    return false;
}

// Trust hash of a native plugin: the sha256 of its shared library (first platform
// candidate that hashes).
std::string nativePluginContentHash(const PluginManifest &pluginManifest)
{
    for (const std::filesystem::path &candidate :
        nativePluginLibraryCandidates(pluginManifest.directory, pluginManifest.entry))
    {
        std::error_code statError;
        if (!std::filesystem::is_regular_file(candidate, statError) || statError)
        {
            continue;
        }
        std::string       hashError;
        const std::string digest = Sha256::fileHexDigest(candidate, hashError);
        if (!digest.empty())
        {
            return digest;
        }
    }
    return {};
}

// Trust hash of a script package: a length-prefixed digest over the manifest and
// every .lua file sorted by relative path (the version tag pins the layout, so
// any rename or reorder changes the hash).
std::string scriptPluginContentHash(const PluginManifest &pluginManifest)
{
    struct HashedFile
    {
        std::string           relativePath;
        std::filesystem::path path;
    };

    std::vector<HashedFile>                       files;
    std::error_code                               scanError;
    std::filesystem::recursive_directory_iterator iterator(
        pluginManifest.directory, std::filesystem::directory_options::skip_permission_denied, scanError);
    if (!scanError)
    {
        for (const std::filesystem::directory_entry &entry : iterator)
        {
            std::error_code statError;
            if (statError || !entry.is_regular_file(statError) || statError)
            {
                continue;
            }
            const std::string extension = entry.path().extension().string();
            if (extension != ".lua")
            {
                continue;
            }
            std::string relative
                = std::filesystem::relative(entry.path(), pluginManifest.directory, statError).string();
            if (statError)
            {
                relative = entry.path().filename().string();
            }
            files.push_back(HashedFile{.relativePath = std::move(relative), .path = entry.path()});
        }
    }
    std::sort(files.begin(),
        files.end(),
        [](const HashedFile &left, const HashedFile &right) { return left.relativePath < right.relativePath; });

    Sha256     hasher;
    const auto feed     = [&hasher](const void *data, std::size_t size) { hasher.update(data, size); };
    const auto feedText = [&feed](const std::string &text)
    {
        std::uint64_t size = static_cast<std::uint64_t>(text.size());
        feed(&size, sizeof(size));
        feed(text.data(), text.size());
    };

    feedText("engine-script-plugin-hash-v1");
    feedText("plugin.toml");
    {
        std::ifstream manifestStream(
            pluginManifest.directory / PluginManifestLoader::manifestFileName, std::ios::in | std::ios::binary);
        if (manifestStream.is_open())
        {
            std::string manifestBytes(
                (std::istreambuf_iterator<char>(manifestStream)), std::istreambuf_iterator<char>());
            feedText(manifestBytes);
        }
    }
    for (const HashedFile &file : files)
    {
        feedText(file.relativePath);
        std::ifstream scriptStream(file.path, std::ios::in | std::ios::binary);
        if (!scriptStream.is_open())
        {
            return {};
        }
        std::string scriptBytes((std::istreambuf_iterator<char>(scriptStream)), std::istreambuf_iterator<char>());
        feedText(scriptBytes);
    }

    std::uint8_t digest[Sha256::digestSize];
    hasher.finish(digest);
    std::string hex;
    hex.reserve(Sha256::hexDigestSize);
    static const char digits[] = "0123456789abcdef";
    for (const std::uint8_t byte : digest)
    {
        hex.push_back(digits[byte >> 4]);
        hex.push_back(digits[byte & 0x0F]);
    }
    return hex;
}

// Permissions outside the capability.* namespace - the ones the trust gate flags.
std::vector<std::string> environmentPermissionsOf(const std::vector<std::string> &permissions)
{
    std::vector<std::string> environmentPermissions;
    for (const std::string &permission : permissions)
    {
        if (permission.rfind(capabilityPermissionPrefix, 0) != 0)
        {
            environmentPermissions.push_back(permission);
        }
    }
    return environmentPermissions;
}

std::string joinQuoted(const std::vector<std::string> &names)
{
    std::string joined;
    for (const std::string &name : names)
    {
        if (!joined.empty())
        {
            joined += ", ";
        }
        joined += "'" + name + "'";
    }
    return joined;
}

// Records the advisory and echoes it as a startup warning.
void appendAdvisory(std::vector<ParticipantTrustAdvisory> &advisories, ParticipantTrustAdvisory advisory,
    const char *participantKindNoun, std::string &warnings)
{
    advisories.push_back(std::move(advisory));
    std::string joined;
    for (const std::string &reason : advisories.back().reasons)
    {
        if (!joined.empty())
        {
            joined += "; ";
        }
        joined += reason;
    }
    warnings += std::string("warning: ") + participantKindNoun + " '" + advisories.back().name
              + advisoryIdentitySuffix(advisories.back()) + " requests elevated trust: " + joined + ". Review this "
              + participantKindNoun + " before trusting it with sensitive data.\n";
}

} // namespace

// Advisories for plugins: native code, permissions outside capability.*, or a broad
// capability appetite produce one. The content hash comes from the library
// (native) or the script bundle.
void TrustGate::appendPluginRiskWarning(const PluginManifest &pluginManifest, bool isNativePlugin, std::string &warnings)
{
    std::vector<std::string> reasons;

    if (isNativePlugin)
    {
        reasons.emplace_back("it runs native machine code");
    }
    const std::vector<std::string> environmentPermissions = environmentPermissionsOf(pluginManifest.permissions);
    if (!environmentPermissions.empty())
    {
        reasons.push_back(
            "it requests permissions outside the engine's capability system: " + joinQuoted(environmentPermissions));
    }
    if (pluginManifest.capabilities.size() >= broadCapabilityWarningThreshold)
    {
        reasons.push_back(
            "it acquires a broad set of " + std::to_string(pluginManifest.capabilities.size()) + " capabilities");
    }
    if (reasons.empty())
    {
        return;
    }
    ParticipantTrustAdvisory advisory;
    advisory.kind    = pluginManifest.isNative() ? ParticipantKind::NativePlugin : ParticipantKind::ScriptPlugin;
    advisory.name    = pluginManifest.id;
    advisory.version = pluginManifest.version;
    advisory.sha256  = isNativePlugin ? nativePluginContentHash(pluginManifest) : scriptPluginContentHash(pluginManifest);
    advisory.reasons = reasons;
    appendAdvisory(advisories_, std::move(advisory), "plugin", warnings);
}

// Module equivalent of appendPluginRiskWarning; the hash is the library digest taken
// during the directory scan.
void TrustGate::appendModuleRiskWarning(const ModuleLibraryRecord &module, std::string &warnings)
{
    std::vector<std::string>       reasons;
    const std::vector<std::string> environmentPermissions
        = environmentPermissionsOf(module.descriptor.declaredPermissions);
    if (!environmentPermissions.empty())
    {
        reasons.push_back(
            "it requests permissions outside the engine's capability system: " + joinQuoted(environmentPermissions));
    }
    const std::size_t acquiredCapabilities
        = module.descriptor.requiredCapabilities.size() + module.descriptor.optionalCapabilities.size();
    if (acquiredCapabilities >= broadCapabilityWarningThreshold)
    {
        reasons.push_back("it acquires a broad set of " + std::to_string(acquiredCapabilities) + " capabilities");
    }
    if (reasons.empty())
    {
        return;
    }
    ParticipantTrustAdvisory advisory;
    advisory.kind    = ParticipantKind::EngineModule;
    advisory.name    = module.descriptor.name;
    advisory.version = module.descriptor.version;
    advisory.sha256  = module.contentSha256;
    advisory.reasons = reasons;
    appendAdvisory(advisories_, std::move(advisory), "module", warnings);
}

// Decides whether the run may proceed. Non-enforcing callers (validate) only get
// notes; enforcing callers get the consent handler as a last chance before an
// abort report with [trust] instructions.
bool TrustGate::evaluate(
    std::string &report, bool enforce, std::string &warnings, const ConfigurationStore &configuration) const
{
    if (advisories_.empty())
    {
        return true;
    }

    const std::vector<TrustBinding> pluginBindings    = configuration.trustedPluginBindings();
    const std::vector<TrustBinding> moduleBindings = configuration.trustedModuleBindings();
    const bool                      allPlugins        = configuration.trustAllPlugins();
    const bool                      allModules     = configuration.trustAllModules();
    std::vector<std::string>        bindingWarnings;
    const auto                      isTrusted = [&](const ParticipantTrustAdvisory &advisory)
    {
        if (advisory.kind == ParticipantKind::EngineModule)
        {
            return allModules || anyTrustBindingMatches(moduleBindings, advisory, bindingWarnings);
        }
        return allPlugins || anyTrustBindingMatches(pluginBindings, advisory, bindingWarnings);
    };

    std::vector<ParticipantTrustAdvisory> untrusted;
    for (const ParticipantTrustAdvisory &advisory : advisories_)
    {
        if (!isTrusted(advisory))
        {
            untrusted.push_back(advisory);
        }
    }
    for (const std::string &warning : bindingWarnings)
    {
        warnings += "warning: " + warning + "\n";
    }
    if (untrusted.empty())
    {
        return true;
    }

    std::string names;
    for (const ParticipantTrustAdvisory &advisory : untrusted)
    {
        if (!names.empty())
        {
            names += ", ";
        }
        names += describeComponentKind(advisory.kind) + " '" + advisory.name + "'" + advisoryIdentitySuffix(advisory);
    }
    const char *trustInstructions
        = "To proceed without being asked, trust the participants in the configuration's [trust] section:\n"
          "  trustPlugins = [\"<plugin id>\"]       (or trustAllPlugins = true)\n"
          "  trustModules = [\"<module name>\"] (or trustAllModules = true)\n"
          "Bind trust to content so replaced binaries or scripts cannot inherit it:\n"
          "  trustPlugins = [{ id = \"<plugin id>\", version = \"1.2.3\", sha256 = \"<64 hex characters>\" }]\n"
          "  trustModules = [{ name = \"<module name>\", sha256 = \"<64 hex characters>\" }]\n";

    if (!enforce)
    {
        warnings += "note: " + std::to_string(untrusted.size())
                  + " participant(s) that can be harmful are not covered by the trust configuration (" + names
                  + "); a real run requires trusting them under [trust] or agreeing to them at startup.\n";
        return true;
    }

    if (consentHandler_ != nullptr && consentHandler_(untrusted))
    {
        warnings += "agreement granted at startup: proceeding with " + names + " for this run.\n";
        return true;
    }
    if (consentHandler_ != nullptr)
    {
        report += "startup aborted: agreement was not granted for " + names + ".\n";
    }
    else
    {
        report += "startup aborted: " + std::to_string(untrusted.size()) + " participant(s) that can be harmful ("
                + names
                + ") are not trusted and no agreement could be requested (no interactive consent "
                  "surface is connected).\n";
    }
    report += trustInstructions;
    return false;
}

// Splits undescribed-module advisories into approved (trusted or consented) and
// undecided; only approved libraries get scanned.
TrustGate::ScanDecision TrustGate::decideScanConsent(const std::vector<ParticipantTrustAdvisory> &scanAdvisories,
    const ConfigurationStore &configuration, std::string &warnings) const
{
    ScanDecision                    decision;
    const bool                      trustAllModules = configuration.trustAllModules();
    const std::vector<TrustBinding> moduleBindings  = configuration.trustedModuleBindings();

    std::vector<std::string> bindingWarnings;
    for (std::size_t index = 0; index < scanAdvisories.size(); ++index)
    {
        if (trustAllModules || anyTrustBindingMatches(moduleBindings, scanAdvisories[index], bindingWarnings))
        {
            decision.approved.push_back(index);
        }
        else
        {
            decision.undecided.push_back(index);
        }
    }
    for (const std::string &warning : bindingWarnings)
    {
        warnings += "warning: " + warning + "\n";
    }

    if (!decision.undecided.empty() && consentHandler_ != nullptr)
    {
        std::vector<ParticipantTrustAdvisory> untrusted;
        untrusted.reserve(decision.undecided.size());
        for (const std::size_t index : decision.undecided)
        {
            untrusted.push_back(scanAdvisories[index]);
        }
        if (consentHandler_(untrusted))
        {
            std::vector<std::string> names;
            names.reserve(untrusted.size());
            for (const ParticipantTrustAdvisory &advisory : untrusted)
            {
                names.push_back(advisory.name);
            }
            warnings += "agreement granted at startup: scanning " + joinQuoted(names) + " for this run.\n";
            decision.approved.insert(decision.approved.end(), decision.undecided.begin(), decision.undecided.end());
            decision.undecided.clear();
        }
        else
        {
            warnings += "agreement was not granted at startup: no undescribed module was scanned.\n";
        }
    }
    return decision;
}

} // namespace engine
