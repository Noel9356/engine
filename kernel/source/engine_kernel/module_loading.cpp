// EngineKernel module loading: library scan, descriptors and plugin loading.
//
// loadModuleLibraries scans the modules directory and resolves each
// library's descriptor from the cache or a module.toml sidecar; libraries
// with neither need trust-gate approval to be scanned (that executes
// code). Also loads native plugin libraries and resolves script hosts.

#include "engine_kernel_internal.hpp"
#include "kernel_internals.hpp"
#include "trust_gate.hpp"
#include "engine/kernel/dependency_resolver.hpp"
#include "engine/kernel/dynamic_library.hpp"
#include "engine/kernel/module_descriptor.hpp"
#include "engine/kernel/permission_system.hpp"
#include "engine/kernel/sha256.hpp"
#include <engine/kernel/capability_name.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/kernel/plugin_api.hpp>
#include <engine/kernel/plugin_discovery.hpp>
#include <engine/kernel/plugin_manifest.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/module_descriptor_cache.hpp>
#include <engine/kernel/module_manifest.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace engine
{

namespace
{

// Copies the entry table's capability/permission string lists into a descriptor.
void readModuleEntryTableCapabilities(const EngineModuleEntryTable &entry, ModuleDescriptor &outDescriptor)
{
    const auto readList = [](const char *const *values, std::uint32_t count)
    {
        std::vector<std::string> list;
        if (values == nullptr)
        {
            return list;
        }
        list.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index)
        {
            if (values[index] != nullptr)
            {
                list.emplace_back(values[index]);
            }
        }
        return list;
    };
    outDescriptor.providedCapabilities = readList(entry.providedCapabilities, entry.providedCapabilityCount);
    outDescriptor.requiredCapabilities = readList(entry.requiredCapabilities, entry.requiredCapabilityCount);
    outDescriptor.optionalCapabilities = readList(entry.optionalCapabilities, entry.optionalCapabilityCount);
    outDescriptor.declaredPermissions  = readList(entry.declaredPermissions, entry.declaredPermissionCount);
}

// Loads the library, queries engineQueryModuleEntryTable and fills the descriptor's
// identity/capabilities. Returns the entry table pointer (valid only while the
// library stays loaded) so the caller can grab the factory; nullptr on failure
// with a human-readable report appended.
const EngineModuleEntryTable *loadAndQueryModuleEntryTable(
    DynamicLibrary &library, const std::filesystem::path &libraryPath, std::string &report)
{
    std::string loadError;
    if (!library.load(libraryPath, loadError))
    {
        report += "- " + loadError + "\n";
        return nullptr;
    }
    const auto query = reinterpret_cast<const EngineModuleEntryTable *(*)()>(
        library.resolveSymbol("engineQueryModuleEntryTable"));
    if (query == nullptr)
    {
        report += "- '" + libraryPath.string()
                + "' exports no engineQueryModuleEntryTable entry point; not a module.\n";
        return nullptr;
    }
    const EngineModuleEntryTable *moduleEntryTable = query();
    if (moduleEntryTable == nullptr || moduleEntryTable->abiVersion != engineModuleAbiVersion)
    {
        report += "- '" + libraryPath.string()
                + "' was built against a different module ABI (expected version "
                + std::to_string(engineModuleAbiVersion) + "); rebuild the module with this kernel.\n";
        return nullptr;
    }
    if (moduleEntryTable->moduleName == nullptr || moduleEntryTable->createModule == nullptr)
    {
        report += "- '" + libraryPath.string() + "' returns an incomplete module descriptor.\n";
        return nullptr;
    }
    return moduleEntryTable;
}

// Fills the descriptor's identity and capability/permission lists from the entry table.
void fillDescriptorFromEntryTable(const EngineModuleEntryTable &entry, ModuleDescriptor &outDescriptor)
{
    outDescriptor.name = entry.moduleName;
    if (entry.moduleVersion != nullptr)
    {
        outDescriptor.version = entry.moduleVersion;
    }
    readModuleEntryTableCapabilities(entry, outDescriptor);
}

} // namespace

// Scans the modules directory (top level plus one nesting level), hashes every
// candidate library for the trust gate and resolves its descriptor from the cache
// or a module.toml sidecar; libraries with neither stay undescribed.
void EngineKernel::loadModuleLibraries(std::string &report)
{
    moduleLibraries.clear();
    undescribedModuleLibraries_.clear();

    std::filesystem::path       modulesDirectory = configurationStore.modulesDirectory();
    const std::filesystem::path resolvedDirectory
        = modulesDirectory.is_absolute() ? modulesDirectory : executableDirectory() / modulesDirectory;

    std::vector<std::pair<std::filesystem::path, std::filesystem::path>> moduleLibraryPaths;
    const auto collectLibraries = [&moduleLibraryPaths](const std::filesystem::directory_entry &entry)
    {
        std::error_code statError;
        if (statError || !entry.is_regular_file())
        {
            return;
        }
        const std::filesystem::path libraryPath = entry.path();
        const std::string           extension   = libraryPath.extension().string();

        if (extension == ".dll" || extension == ".so")
        {
            moduleLibraryPaths.emplace_back(libraryPath, libraryPath.parent_path());
        }
    };
    std::vector<std::filesystem::path> subdirectories;
    {
        std::error_code                     scanError;
        std::filesystem::directory_iterator module{
            resolvedDirectory, std::filesystem::directory_options::skip_permission_denied, scanError};
        if (scanError)
        {
            report += "- modules directory '" + resolvedDirectory.string()
                    + "' cannot be scanned: " + scanError.message() + "\n";
            return;
        }
        for (const std::filesystem::directory_entry &entry : module)
        {
            std::error_code statError;
            if (statError)
            {
                continue;
            }
            if (entry.is_directory())
            {
                subdirectories.push_back(entry.path());
            }
            else
            {
                collectLibraries(entry);
            }
        }
    }
    for (const std::filesystem::path &subdirectory : subdirectories)
    {
        std::error_code                     statError;
        std::filesystem::directory_iterator nested{
            subdirectory, std::filesystem::directory_options::skip_permission_denied, statError};
        if (statError)
        {
            continue;
        }
        for (const std::filesystem::directory_entry &entry : nested)
        {
            collectLibraries(entry);
        }
    }

    std::sort(moduleLibraryPaths.begin(), moduleLibraryPaths.end());

    std::map<std::string, std::size_t> librariesPerDirectory;
    for (const auto &[libraryPath, directory] : moduleLibraryPaths)
    {
        ++librariesPerDirectory[directory.string()];
    }

    // A directory's module.toml, parsed lazily once per directory (attempted flags
    // parse failures so they are reported only once).
    struct DirectorySidecar
    {
        bool           attempted = false;
        bool           present   = false;
        ModuleManifest manifest;
    };
    std::map<std::string, DirectorySidecar> sidecarsByDirectory;
    const auto sidecarFor = [&](const std::filesystem::path &directory) -> DirectorySidecar &
    {
        auto [iterator, inserted] = sidecarsByDirectory.try_emplace(directory.string());
        if (inserted)
        {
            iterator->second.attempted               = true;
            const std::filesystem::path manifestPath = directory / ModuleManifestLoader::manifestFileName;
            std::error_code             existsError;
            if (std::filesystem::exists(manifestPath, existsError) && !existsError)
            {
                std::string parseError;
                if (ModuleManifestLoader{}.loadFromFile(manifestPath, iterator->second.manifest, parseError))
                {
                    iterator->second.present = true;
                }
                else
                {
                    report += "- " + parseError + "\n";
                }
            }
        }
        return iterator->second;
    };

    // Descriptor cache (see ModuleDescriptorCache): once scanned, a library's descriptor
    // survives here without a sidecar until size or mtime change.
    ModuleDescriptorCache descriptorCache;
    descriptorCache.setCacheFile(executableDirectory() / moduleDescriptorCacheFileName);
    descriptorCache.load();
    bool cacheChanged = false;

    for (const auto &[libraryPath, libraryDirectory] : moduleLibraryPaths)
    {
        std::error_code      statError;
        const std::uintmax_t sizeBytes = std::filesystem::file_size(libraryPath, statError);
        if (statError)
        {
            report += "- '" + libraryPath.string() + "' cannot be stat-ed: " + statError.message() + "\n";
            continue;
        }
        const std::uintmax_t modificationStamp = static_cast<std::uintmax_t>(
            std::filesystem::last_write_time(libraryPath, statError).time_since_epoch().count());
        if (statError)
        {
            report += "- '" + libraryPath.string() + "' cannot be stat-ed: " + statError.message() + "\n";
            continue;
        }

        // Fast path: when the file-system stamps (size + mtime) match a cached
        // entry, the cached descriptor and the cached content hash are reused
        // directly. The hash is recomputed only on a cache miss, which avoids
        // re-hashing every module library on every startup of an unchanged
        // build - the cache lives next to the executable, so trusting size+mtime
        // as the validity signal is as strong as trusting the executable itself.
        if (const ModuleDescriptorCacheEntry *remembered
            = descriptorCache.findByStamps(libraryPath, sizeBytes, modificationStamp))
        {
            ModuleLibraryRecord record;
            record.descriptor    = remembered->descriptor;
            record.contentSha256 = remembered->contentSha256;
            record.libraryPath   = libraryPath;

            // Two libraries claiming one module name would alias; the later (sorted) path is ignored.
            if (std::find_if(moduleLibraries.begin(),
                    moduleLibraries.end(),
                    [&record](const ModuleLibraryRecord &existing)
                    { return existing.descriptor.name == record.descriptor.name; })
                != moduleLibraries.end())
            {
                report += "- module '" + record.descriptor.name
                        + "' is provided by more than one module library; '" + libraryPath.string()
                        + "' is ignored.\n";
                continue;
            }

            moduleLibraries.push_back(std::move(record));
            continue;
        }

        std::string       hashError;
        const std::string contentSha256 = Sha256::fileHexDigest(libraryPath, hashError);
        if (contentSha256.empty())
        {
            report += "- '" + libraryPath.string() + "' cannot be hashed for the trust gate: " + hashError + "\n";
            continue;
        }

        ModuleLibraryRecord record;
        {
            const DirectorySidecar &sidecar            = sidecarFor(libraryDirectory);
            bool                    describedBySidecar = false;
            if (sidecar.present)
            {
                // A sidecar describes either the one library it names or, when it names none,
                // every single-library directory it sits in.
                const bool namesThisLibrary
                    = !sidecar.manifest.library.empty() && sidecar.manifest.library == libraryPath.filename().string();
                const bool coversDirectory
                    = sidecar.manifest.library.empty() && librariesPerDirectory[libraryDirectory.string()] == 1;
                describedBySidecar = namesThisLibrary || coversDirectory;
            }
            if (!describedBySidecar)
            {
                undescribedModuleLibraries_.push_back(UndescribedModuleLibrary{
                    .libraryPath       = libraryPath,
                    .sizeBytes         = sizeBytes,
                    .modificationStamp = modificationStamp,
                    .contentSha256     = contentSha256});
                continue;
            }

            record.descriptor    = sidecar.manifest.descriptor;
            record.contentSha256 = contentSha256;
            record.libraryPath   = libraryPath;

            descriptorCache.record(libraryPath,
                ModuleDescriptorCacheEntry{
                    .sizeBytes         = sizeBytes,
                    .modificationStamp = modificationStamp,
                    .contentSha256     = contentSha256,
                    .descriptor        = record.descriptor});
            cacheChanged = true;
        }

        // Two libraries claiming one module name would alias; the later (sorted) path is ignored.
        if (std::find_if(moduleLibraries.begin(),
                moduleLibraries.end(),
                [&record](const ModuleLibraryRecord &existing)
                { return existing.descriptor.name == record.descriptor.name; })
            != moduleLibraries.end())
        {
            report += "- module '" + record.descriptor.name + "' is provided by more than one module library; '"
                    + libraryPath.string() + "' is ignored.\n";
            continue;
        }

        moduleLibraries.push_back(std::move(record));
    }

    if (cacheChanged)
    {
        descriptorCache.save();
    }
}

// Trust gate for libraries without a data descriptor: identifying them executes
// their code, so each needs [trust] or consent-handler approval before
// scanUndescribedModuleLibraryDescriptor queries its entry table.
void EngineKernel::resolveUndescribedModuleLibraries(std::string &report, bool enforceTrust)
{
    if (undescribedModuleLibraries_.empty())
    {
        return;
    }

    std::vector<ParticipantTrustAdvisory> scanAdvisories;
    scanAdvisories.reserve(undescribedModuleLibraries_.size());
    for (const UndescribedModuleLibrary &candidate : undescribedModuleLibraries_)
    {
        scanAdvisories.push_back(ParticipantTrustAdvisory{
            .kind    = ParticipantKind::EngineModule,
            .name    = candidate.libraryPath.filename().string(),
            .sha256  = candidate.contentSha256,
            .reasons = {"the kernel has no data descriptor for it (no "
                        "module.toml beside the library and no cached "
                        "descriptor); identifying this module requires "
                        "executing its code"}});
    }

    // validate never executes module code; undescribed libraries stay unloaded with a note.
    if (!enforceTrust)
    {
        for (const ParticipantTrustAdvisory &advisory : scanAdvisories)
        {
            startupWarnings_ += "note: module '" + advisory.name + advisoryIdentitySuffix(advisory)
                              + "' was not scanned: validate never executes module code and this module has no "
                                "data descriptor; construct will ask for a trust decision, or ship a module.toml "
                                "next to the library.\n";
        }
        undescribedModuleLibraries_.clear();
        return;
    }

    const TrustGate::ScanDecision decision
        = trustGate_->decideScanConsent(scanAdvisories, configurationStore, startupWarnings_);
    const std::vector<std::size_t> &approvedIndexes  = decision.approved;
    const std::vector<std::size_t> &undecidedIndexes = decision.undecided;

    ModuleDescriptorCache descriptorCache;
    descriptorCache.setCacheFile(executableDirectory() / moduleDescriptorCacheFileName);
    descriptorCache.load();
    bool cacheChanged = false;

    for (const std::size_t index : approvedIndexes)
    {
        ModuleLibraryRecord        record;
        ModuleDescriptorCacheEntry cacheEntry;
        if (!scanUndescribedModuleLibraryDescriptor(undescribedModuleLibraries_[index], record, cacheEntry, report))
        {
            continue;
        }
        if (std::find_if(moduleLibraries.begin(),
                moduleLibraries.end(),
                [&record](const ModuleLibraryRecord &existing)
                { return existing.descriptor.name == record.descriptor.name; })
            != moduleLibraries.end())
        {
            report += "- module '" + record.descriptor.name + "' is provided by more than one module library; '"
                    + record.libraryPath.string() + "' is ignored.\n";
            continue;
        }
        descriptorCache.record(record.libraryPath, cacheEntry);
        cacheChanged = true;
        moduleLibraries.push_back(std::move(record));
    }
    for (const std::size_t index : undecidedIndexes)
    {
        const ParticipantTrustAdvisory &advisory = scanAdvisories[index];
        startupWarnings_ += "note: module '" + advisory.name + advisoryIdentitySuffix(advisory)
                          + "' was not scanned: identifying it would execute its code and no trust decision covers "
                            "it. Ship a module.toml beside the library, or trust it under [trust] (or agree at "
                            "startup); until then the module stays unloaded.\n";
    }
    if (cacheChanged)
    {
        descriptorCache.save();
    }
    undescribedModuleLibraries_.clear();
}

// Loads the library, queries engineQueryModuleEntryTable and fills the record plus
// a cache entry, then releases the library again (the real load happens later).
bool EngineKernel::scanUndescribedModuleLibraryDescriptor(const UndescribedModuleLibrary &candidate,
    ModuleLibraryRecord &outRecord, ModuleDescriptorCacheEntry &outCacheEntry, std::string &report)
{
    outRecord               = ModuleLibraryRecord{};
    outRecord.libraryPath   = candidate.libraryPath;
    outRecord.contentSha256 = candidate.contentSha256;

    auto library = std::make_unique<DynamicLibrary>();
    const EngineModuleEntryTable *moduleEntryTable
        = loadAndQueryModuleEntryTable(*library, candidate.libraryPath, report);
    if (moduleEntryTable == nullptr)
    {
        return false;
    }
    outRecord.createModule = moduleEntryTable->createModule;
    fillDescriptorFromEntryTable(*moduleEntryTable, outRecord.descriptor);

    library->release();

    outCacheEntry.sizeBytes         = candidate.sizeBytes;
    outCacheEntry.modificationStamp = candidate.modificationStamp;
    outCacheEntry.contentSha256     = outRecord.contentSha256;
    outCacheEntry.descriptor        = outRecord.descriptor;
    return true;
}

// Same discovery as the scan path, but keeps the library loaded and records its factory.
bool EngineKernel::loadModuleLibrary(ModuleLibraryRecord &module, std::string &report)
{
    auto library = std::make_unique<DynamicLibrary>();
    const EngineModuleEntryTable *moduleEntryTable
        = loadAndQueryModuleEntryTable(*library, module.libraryPath, report);
    if (moduleEntryTable == nullptr)
    {
        return false;
    }
    module.descriptor      = ModuleDescriptor{};
    fillDescriptorFromEntryTable(*moduleEntryTable, module.descriptor);
    module.createModule = moduleEntryTable->createModule;

    module.library = std::move(library);
    return true;
}

// Runs the factory of every active module and cross-checks its runtime manifest
// against the descriptor (name, version, capabilities, environment permissions)
// so the trust gate's pre-instantiation view cannot be evaded.
bool EngineKernel::instantiateActiveModules(std::string &report)
{
    bool successful = true;
    for (ModuleLibraryRecord &module : moduleLibraries)
    {
        if (!module.active)
        {
            continue;
        }

        if (!module.library && !loadModuleLibrary(module, report))
        {
            successful = false;
            continue;
        }
        ParticipantRecord record{
            .name = module.descriptor.name, .manifest = ParticipantManifest{.kind = ParticipantKind::EngineModule}};

        try
        {
            record.ownedModule.reset(module.createModule());
        }
        catch (const std::exception &error)
        {
            report += "- module factory for '" + module.descriptor.name + "' threw: " + error.what() + "\n";
            successful = false;
            continue;
        }
        record.moduleInstance = record.ownedModule.get();
        if (record.ownedModule == nullptr)
        {
            report += "- module factory for '" + module.descriptor.name + "' returned no instance.\n";
            successful = false;
            continue;
        }
        record.manifest = record.moduleInstance->manifest();

        if (record.manifest.name != module.descriptor.name)
        {
            report += "- module '" + module.descriptor.name + "' reports the manifest name '" + record.manifest.name
                    + "'; the descriptor name and the manifest name must match.\n";
            successful = false;
            continue;
        }
        if (!module.descriptor.version.empty() && !record.manifest.version.empty()
            && module.descriptor.version != record.manifest.version)
        {
            report += "- module '" + module.descriptor.name + "' descriptor version '" + module.descriptor.version
                    + "' differs from its manifest version '" + record.manifest.version
                    + "'; the descriptor and the manifest must agree.\n";
            successful = false;
            continue;
        }
        if (!detail::sameStringSet(record.manifest.providedCapabilities, module.descriptor.providedCapabilities))
        {
            report += "- module '" + module.descriptor.name
                    + "' declares different provided capabilities in its manifest than in its module descriptor; "
                      "rebuild the module with this kernel.\n";
            successful = false;
            continue;
        }
        if (!detail::sameStringSet(record.manifest.requiredCapabilities, module.descriptor.requiredCapabilities))
        {
            report += "- module '" + module.descriptor.name
                    + "' declares different required capabilities in its manifest than in its module descriptor; "
                      "rebuild the module with this kernel.\n";
            successful = false;
            continue;
        }
        if (!detail::sameStringSet(record.manifest.optionalCapabilities, module.descriptor.optionalCapabilities))
        {
            report += "- module '" + module.descriptor.name
                    + "' declares different optional capabilities in its manifest than in its module descriptor; "
                      "rebuild the module with this kernel.\n";
            successful = false;
            continue;
        }

        // Environment permissions = manifest permissions outside the capability.* namespace;
        // the descriptor must declare exactly this set (the trust gate only sees the
        // descriptor before instantiation).
        std::vector<std::string> manifestEnvironmentPermissions;
        for (const std::string &permission : record.manifest.requiredPermissions)
        {
            if (permission.rfind(capabilityPermissionPrefix, 0) != 0)
            {
                manifestEnvironmentPermissions.push_back(permission);
            }
        }
        if (!detail::sameStringSet(manifestEnvironmentPermissions, module.descriptor.declaredPermissions))
        {
            report += "- module '" + module.descriptor.name
                    + "' requests different environment permissions in its manifest than its module descriptor "
                      "declares; declare the environment permissions (anything outside the capability.* "
                      "namespace) in the descriptor's declaredPermissions so the trust gate sees them before "
                      "the module is instantiated.\n";
            successful = false;
            continue;
        }
        records.push_back(std::move(record));
    }

    for (ModuleLibraryRecord &module : moduleLibraries)
    {
        if (!module.active && module.library)
        {
            module.library->release();
        }
    }
    return successful;
}

// validate-only path: declares module participants straight from their descriptors
// (no factories run, no code loaded).
void EngineKernel::appendDeclaredModuleRecords()
{
    for (const ModuleLibraryRecord &module : moduleLibraries)
    {
        if (!module.active)
        {
            continue;
        }
        ParticipantRecord record{
            .name     = module.descriptor.name,
            .manifest = ParticipantManifest{
                .name                 = module.descriptor.name,
                .kind                 = ParticipantKind::EngineModule,
                .version              = module.descriptor.version.empty() ? "0.1.0" : module.descriptor.version,
                .providedCapabilities = module.descriptor.providedCapabilities,
                .requiredCapabilities = module.descriptor.requiredCapabilities,
                .optionalCapabilities = module.descriptor.optionalCapabilities,
                .requiredPermissions  = module.descriptor.declaredPermissions}};
        records.push_back(std::move(record));
    }
}

// Post-trust-gate loading of native plugins: each pending package's library is loaded
// and its compiled manifest is checked against plugin.toml ('provides' lives only in
// plugin.toml); failures remove the participant record.
void EngineKernel::loadNativePluginLibraries(std::string &report)
{
    for (const DiscoveredPluginPackage &package : pendingNativePluginPackages_)
    {
        const PluginManifest &pluginManifest = package.manifest;
        NativePluginLoad      load;
        if (!loadNativePluginEntry(package, load, report))
        {
            auto recordIt = std::find_if(records.begin(),
                records.end(),
                [&pluginManifest](const ParticipantRecord &candidate)
                {
                    return candidate.name == pluginManifest.id
                        && candidate.manifest.kind == ParticipantKind::NativePlugin;
                });
            if (recordIt != records.end())
            {
                records.erase(recordIt);
            }
            continue;
        }
        auto recordIt = std::find_if(records.begin(),
            records.end(),
            [&pluginManifest](const ParticipantRecord &candidate)
            {
                return candidate.name == pluginManifest.id
                    && candidate.manifest.kind == ParticipantKind::NativePlugin;
            });
        if (recordIt == records.end())
        {
            report
                += "- native plugin '" + pluginManifest.id + "' lost its participant record; the package is dropped.\n";
            load.entryTable->destroyPlugin(load.pluginInstance);
            continue;
        }
        ParticipantRecord        &record           = *recordIt;
        const ParticipantManifest compiledManifest = load.pluginInstance->manifest();
        if (!compiledManifest.providedCapabilities.empty()
            && !detail::sameStringSet(compiledManifest.providedCapabilities, pluginManifest.providedCapabilities))
        {
            report += "- native plugin '" + pluginManifest.id
                    + "' provides different capabilities in its compiled manifest than plugin.toml declares under "
                      "'provides'; declare the provided capabilities in plugin.toml (and only there) so the graph "
                      "and the validate dry run see them.\n";
            load.entryTable->destroyPlugin(load.pluginInstance);
            records.erase(recordIt);
            continue;
        }
        if (compiledManifest.providedCapabilities.empty() && !pluginManifest.providedCapabilities.empty())
        {
            report += "- native plugin '" + pluginManifest.id + "' declares 'provides' in plugin.toml but its compiled "
                    + "manifest provides nothing; rebuild the plugin against this engine.\n";
            load.entryTable->destroyPlugin(load.pluginInstance);
            records.erase(recordIt);
            continue;
        }
        record.library           = std::move(load.library);
        record.pluginEntryTable = load.entryTable;
        record.pluginInstance   = load.pluginInstance;
    }
    pendingNativePluginPackages_.clear();
}

// Finds the constructed module that provides a script plugin's host capability
// (exact or family match); ambiguity needs a [providers] selection, otherwise
// hosting fails.
EngineKernel::ParticipantRecord *EngineKernel::resolveScriptHost(
    const ParticipantRecord &scriptRecord, std::string &report)
{
    const std::string capabilityName = scriptRecord.scriptPlugin.hostCapability;
    const auto        providesHost   = [&capabilityName](const ParticipantRecord &record)
    {
        return record.manifest.kind == ParticipantKind::EngineModule
            && std::any_of(record.manifest.providedCapabilities.begin(),
                record.manifest.providedCapabilities.end(),
                [&capabilityName](const std::string &candidate)
                { return capabilityRequirementSatisfiedBy(capabilityName, candidate); });
    };
    std::vector<ParticipantRecord *> candidates;
    for (ParticipantRecord &record : records)
    {
        if (record.manifest.kind != ParticipantKind::EngineModule || !record.constructed)
        {
            continue;
        }
        if (providesHost(record))
        {
            candidates.push_back(&record);
        }
    }
    if (candidates.empty())
    {
        report += "- script plugin '" + scriptRecord.name + "' cannot be hosted: no active module provides the '"
                + capabilityName + "' capability it names as its host.\n";
        return nullptr;
    }
    if (candidates.size() > 1)
    {
        const std::vector<DependencyResolver::ProviderSelection> selections = configurationStore.providerSelections();
        const auto                                               selection  = std::find_if(selections.begin(),
            selections.end(),
            [&capabilityName](const DependencyResolver::ProviderSelection &candidate)
            { return candidate.capabilityName == capabilityName; });
        if (selection != selections.end())
        {
            for (ParticipantRecord *candidate : candidates)
            {
                if (candidate->name == selection->providerName)
                {
                    return candidate;
                }
            }
        }
        std::string names;
        for (const ParticipantRecord *candidate : candidates)
        {
            if (!names.empty())
            {
                names += ", ";
            }
            names += "'" + candidate->name + "'";
        }
        report += "- script plugin '" + scriptRecord.name + "' cannot be hosted: " + std::to_string(candidates.size())
                + " active modules provide the '" + capabilityName + "' capability (" + names
                + "); select one under [providers].\n";
        return nullptr;
    }
    return candidates.front();
}

} // namespace engine
