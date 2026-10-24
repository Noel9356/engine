// The composition root includes the published capability headers of the
// modules it acquires from - the providers themselves own and publish their
// capability types; there is no central contract directory.
#include "input_capability.hpp"
#include "renderer_capability.hpp"
#include "window_capability.hpp"
#include "engine/kernel/engine_kernel.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{

// The renderer-less poll interval: the loop keeps pumping events at roughly
// 60 Hz so a run without a renderer still behaves predictably.
constexpr std::chrono::milliseconds noRendererPollInterval{16};
// The assembly configuration; the executable takes no command line input -
// everything a run can vary lives in the configuration layers
// (docs/configuration.md).
constexpr const char *defaultConfigurationFile = "config/engine.toml";

} // namespace

int main()
{
    engine::EngineKernel kernel;
    kernel.setConfigurationFile(defaultConfigurationFile);

    // Module availability is runtime, not compile time: the kernel scans
    // <exe_dir>/modules for plugin binaries and reads their static
    // descriptors (engine/kernel/module_api.hpp). Nothing is registered
    // here or anywhere else, and the mods are the sole demand source: a
    // module is instantiated only when an active mod's manifest requires a
    // capability it provides, and a requirement no available module
    // satisfies fails startup with a precise diagnostic.

    std::string report;
    if (!kernel.construct(report))
    {
        std::cerr << report;
        std::cerr << kernel.startupWarnings();
        return 1;
    }
    // Mod trust advisories are part of the user-facing startup output: the
    // engine blocks what it can enforce, and the user decides what to trust.
    if (!kernel.startupWarnings().empty())
    {
        std::cout << kernel.startupWarnings();
    }
    kernel.start();

    // The frame loop belongs to the composition root, not to the kernel: the
    // kernel only mediates communication, it never owns a window or a frame.
    // The host is a consumer like any other: it acquires the capabilities the
    // active mods have pulled in, and a capability no mod demanded is simply
    // absent - the run degrades instead of assuming a backend exists.
    std::unique_ptr<engine::ComponentContext> hostContext = kernel.createHostContext();

    engine::WindowCapability *window = nullptr;
    try
    {
        window = &hostContext->acquireCapability<engine::WindowCapability>();
    }
    catch (const std::exception &)
    {
        std::cout << "window capability is not available in this run; nothing to drive\n";
    }

    engine::RendererCapability *renderer = nullptr;
    try
    {
        renderer = &hostContext->acquireCapability<engine::RendererCapability>();
    }
    catch (const std::exception &)
    {
        std::cout << "renderer capability is not available in this build; running without frames\n";
    }

    const std::vector<engine::WindowIdentifier> windowIdentifiers = window != nullptr
                                                                      ? window->windowIdentifiers()
                                                                      : std::vector<engine::WindowIdentifier>{};
    const engine::WindowIdentifier              mainWindow = windowIdentifiers.empty() ? engine::invalidWindowIdentifier
                                                                                       : windowIdentifiers.front();

    // A failure anywhere in the frame loop (a missing shader asset, a backend
    // that refuses a pipeline, a mod that throws during dispatch) must produce
    // a diagnostic and a non-zero exit instead of an uncaught exception.
    int           exitCode          = 0;
    std::uint64_t iterations        = 0;
    double        totalFrameSeconds = 0.0;
    double        worstFrameSeconds = 0.0;
    // Frame-time percentile support, only maintained when the latency report
    // is enabled ([diagnostics] latency_report): a fixed 4096-slot ring
    // (bounded memory like the bus rings) plus a lifetime sum for the mean.
    constexpr std::size_t frameSampleCapacity = 4096;
    std::vector<double>   frameSamples;
    frameSamples.reserve(frameSampleCapacity);
    if (kernel.eventBus().latencyRecordingEnabled())
    {
        frameSamples.resize(frameSampleCapacity);
    }
    std::size_t frameSampleCount   = 0;
    std::size_t frameSampleNext    = 0;
    const bool  diagnosticsEnabled = !frameSamples.empty();
    try
    {
        while (window != nullptr && mainWindow != engine::invalidWindowIdentifier
               && !window->windowCloseRequested(mainWindow))
        {
            window->pollEvents();
            if (window->windowCloseRequested(mainWindow))
            {
                break;
            }
            if (renderer != nullptr)
            {
                const auto frameStart = std::chrono::steady_clock::now();
                renderer->renderFrame();
                const double frameSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - frameStart)
                                                .count();
                totalFrameSeconds += frameSeconds;
                worstFrameSeconds = std::max(worstFrameSeconds, frameSeconds);
                if (diagnosticsEnabled)
                {
                    frameSamples[frameSampleNext] = frameSeconds;
                    frameSampleNext               = (frameSampleNext + 1) % frameSampleCapacity;
                    frameSampleCount              = std::min(frameSampleCount + 1, frameSampleCapacity);
                }
            }
            else
            {
                std::this_thread::sleep_for(noRendererPollInterval);
            }
            ++iterations;
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << "engine frame loop failed: " << error.what() << "\n";
        exitCode = 1;
    }

    if (renderer != nullptr)
    {
        renderer->waitUntilIdle();
        if (exitCode == 0 && renderer->validationErrorsObserved())
        {
            std::cerr << "engine observed graphics backend validation errors during this run\n";
            exitCode = 3;
        }
    }

    // Communication and frame latency report: OPT-IN through the
    // configuration ([diagnostics] latency_report). A normal run publishes
    // thousands of events per second and its user does not want a data dump
    // on shutdown, so both measurement (see EventBus::setLatencyRecordingEnabled)
    // and printing are disabled unless the configuration enables them. With
    // it enabled, the bus numbers are the definitive dispatch latencies
    // measured in publish(), and the frame block reports mean plus the
    // p50/p95/p99/max over the most recent ring window.
    try
    {
        if (kernel.eventBus().latencyRecordingEnabled())
        {
            const std::string latencyReport = hostContext->eventBus().latencyReport();
            if (!latencyReport.empty())
            {
                std::cout << "communication latency report (event bus, direct dispatch):\n" << latencyReport;
            }
            if (iterations > 0 && renderer != nullptr)
            {
                std::vector<double> ordered(
                    frameSamples.begin(), frameSamples.begin() + static_cast<std::ptrdiff_t>(frameSampleCount)
                );
                std::sort(ordered.begin(), ordered.end());
                const auto percentile = [&ordered](std::size_t fraction)
                {
                    const std::size_t index = std::min(ordered.size() - 1, ordered.size() * fraction / 100);
                    return ordered[index];
                };
                std::cout << "frame performance: " << iterations << " frames, mean "
                          << (totalFrameSeconds / static_cast<double>(iterations)) * 1000.0 << " ms, p50 "
                          << percentile(50) * 1000.0 << " ms, p95 " << percentile(95) * 1000.0 << " ms, p99 "
                          << percentile(99) * 1000.0 << " ms, worst " << worstFrameSeconds * 1000.0
                          << " ms (percentiles over the most recent " << frameSampleCount << " frames)\n";
            }
        }
    }
    catch (const std::exception &)
    {
        // The report is diagnostic; never turn it into a run failure.
    }

    kernel.stop();
    return exitCode;
}
