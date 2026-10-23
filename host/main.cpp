#include "engine/capabilities/input_contract.hpp"
#include "engine/capabilities/renderer_contract.hpp"
#include "engine/capabilities/screenshot_contract.hpp"
#include "engine/capabilities/window_contract.hpp"
#include "engine/kernel/capability_registry.hpp"
#include "engine/kernel/engine_kernel.hpp"
#include "leak_check.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

// The renderer-less poll interval: the loop keeps pumping events at roughly
// 60 Hz so an automated run without a renderer still terminates promptly.
constexpr std::chrono::milliseconds noRendererPollInterval{16};
// The assembly configuration the documented quick start uses; overridable
// with --config. Named once: the usage text quotes the same path.
constexpr const char *defaultConfigurationFile = "config/engine.toml";

void printUsage()
{
    std::cout << "usage:\n"
              << "  engine [options]\n"
              << "options:\n"
              << "  --config <path>       configuration file, relative to the executable directory\n"
              << "                        (default: " << defaultConfigurationFile << ")\n"
              << "  --scene <path>        world content file: replaces the world ([scene]) wholesale,\n"
              << "                        overriding both the assembly configuration's [scene] and any\n"
              << "                        mod-declared scene (the file may define only [scene]); relative\n"
              << "                        to the executable directory\n"
              << "  --set <key>=<value>   override one configuration value, the highest layer; repeatable.\n"
              << "                        The key is a dotted path (scene.sun.intensity); the value uses\n"
              << "                        TOML syntax and a bare word becomes a string, so\n"
              << "                        --set window.title=My Room needs no quotes\n"
              << "  --validate            dry run: load the layered configuration, resolve the component\n"
              << "                        graph and report problems without initializing any backend;\n"
              << "                        keys that no component declares are reported as warnings\n"
              << "  --provenance          validate and additionally print which layer provided each\n"
              << "                        effective configuration value\n"
              << "  --smoke-frames <n>    run exactly n frames (or iterations when no renderer is\n"
              << "                        present) and shut down cleanly; exit code 3 reports that\n"
              << "                        the graphics backend observed validation errors, which\n"
              << "                        turns runtime regressions into failing automated runs\n"
              << "  --leak-check          with a [leak_test] section in the configuration: evaluate\n"
              << "                        the captured frame against the configured sealed/lit\n"
              << "                        regions; exit code 4 reports a light leak or a dead GI\n"
              << "  --latency-report      print the communication latency and frame-time report on\n"
              << "                        shutdown (the same report [diagnostics] latency_report\n"
              << "                        enables; off by default so normal runs stay quiet)\n"
              << "  --help                this text\n";
}

} // namespace

int main(
    int argc, char **argv
)
{
    std::filesystem::path    configurationPath = defaultConfigurationFile;
    std::filesystem::path    scenePath;
    std::vector<std::string> overrideAssignments;
    bool                     validateOnly      = false;
    bool                     provenanceReport  = false;
    bool                     leakCheck         = false;
    bool                     latencyReportFlag = false;
    std::uint64_t            smokeFrameLimit   = 0;

    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--help")
        {
            printUsage();
            return 0;
        }
        else if (argument == "--validate")
        {
            validateOnly = true;
        }
        else if (argument == "--provenance")
        {
            // Provenance is a validation-time report: it needs the fully
            // merged layer stack and no backends.
            validateOnly     = true;
            provenanceReport = true;
        }
        else if (argument == "--scene")
        {
            if (index + 1 >= argc)
            {
                std::cerr << "--scene expects a file path\n";
                return 2;
            }
            scenePath = argv[++index];
        }
        else if (argument == "--set")
        {
            if (index + 1 >= argc)
            {
                std::cerr << "--set expects a key=value assignment\n";
                return 2;
            }
            overrideAssignments.push_back(argv[++index]);
        }
        else if (argument == "--leak-check")
        {
            leakCheck = true;
        }
        else if (argument == "--latency-report")
        {
            latencyReportFlag = true;
        }
        else if (argument == "--config" && index + 1 < argc)
        {
            configurationPath = argv[++index];
        }
        else if (argument == "--smoke-frames" && index + 1 < argc)
        {
            const std::string  framesArgument = argv[++index];
            std::size_t        consumed       = 0;
            unsigned long long parsed         = 0;
            try
            {
                parsed = std::stoull(framesArgument, &consumed);
            }
            catch (const std::exception &)
            {
                std::cerr << "--smoke-frames expects a positive number, got: " << framesArgument << "\n";
                return 2;
            }
            if (consumed != framesArgument.size() || parsed == 0)
            {
                std::cerr << "--smoke-frames expects a positive number, got: " << framesArgument << "\n";
                return 2;
            }
            smokeFrameLimit = parsed;
        }
        else
        {
            std::cerr << "unknown argument: " << argument << "\n";
            printUsage();
            return 2;
        }
    }

    engine::EngineKernel kernel;
    kernel.setConfigurationFile(configurationPath);
    if (!scenePath.empty())
    {
        kernel.setSceneFile(scenePath);
    }
    // Malformed --set assignments are usage errors: they are parsed here,
    // before any backend starts, and fail the run with exit code 2.
    for (const std::string &assignment : overrideAssignments)
    {
        std::string overrideError;
        if (!kernel.addCommandLineOverride(assignment, overrideError))
        {
            std::cerr << overrideError << "\n";
            printUsage();
            return 2;
        }
    }

    // Module availability is runtime, not compile time: the kernel scans
    // <exe_dir>/modules for plugin binaries and registers one factory per
    // plugin (engine/kernel/module_api.hpp). The composition root registers
    // nothing here; a module listed in the configuration but missing from the
    // modules directory produces the kernel's precise startup diagnostic.

    // Command line diagnostics override: enable recording before the kernel
    // loads its configuration (which applies [diagnostics] latency_report),
    // so even module constructs are measured.
    if (latencyReportFlag)
    {
        kernel.eventBus().setLatencyRecordingEnabled(true);
    }

    if (validateOnly)
    {
        std::string report;
        const bool  successful = kernel.validate(report);
        if (!report.empty())
        {
            std::cout << report;
        }
        if (provenanceReport)
        {
            std::cout << kernel.configuration().provenanceReport();
        }
        std::cout << (successful ? "engine configuration is valid" : "engine configuration is invalid") << "\n";
        return successful ? 0 : 1;
    }

    std::string report;
    if (!kernel.construct(report))
    {
        std::cerr << report;
        return 1;
    }
    kernel.start();

    // The frame loop belongs to the composition root, not to the kernel: the
    // kernel only mediates communication, it never owns a window or a frame.
    std::unique_ptr<engine::ComponentContext> hostContext = kernel.createHostContext();
    engine::WindowCapability                 &window      = hostContext->acquireCapability<engine::WindowCapability>();

    engine::RendererCapability *renderer = nullptr;
    try
    {
        renderer = &hostContext->acquireCapability<engine::RendererCapability>();
    }
    catch (const std::exception &)
    {
        std::cout << "renderer capability is not available in this build; running without frames\n";
    }

    // The smoke run queues its capture through the screenshot capability so
    // the whole production path is exercised (module queue flag, frame begin
    // hand-off, readback, PNG write), not only the renderer readback.
    engine::ScreenshotCapability *screenshot = nullptr;
    if (smokeFrameLimit > 0)
    {
        try
        {
            screenshot = &hostContext->acquireCapability<engine::ScreenshotCapability>();
        }
        catch (const std::exception &)
        {
            screenshot = nullptr;
        }
    }

    const std::vector<engine::WindowIdentifier> windowIdentifiers = window.windowIdentifiers();
    const engine::WindowIdentifier              mainWindow = windowIdentifiers.empty() ? engine::invalidWindowIdentifier
                                                                                       : windowIdentifiers.front();

    // The leak check reads the captured frame: subscribe to the capture event
    // and keep the last frame's pixels (the event payload only stays valid for
    // the duration of the handler).
    engine::LeakCapture leakCapture;
    if (leakCheck)
    {
        engine::subscribeLeakCapture(hostContext->eventBus(), leakCapture);
    }

    // A failure anywhere in the frame loop (a missing shader asset, a backend
    // that refuses a pipeline, a mod that throws during dispatch) must produce
    // a diagnostic and a non-zero exit instead of an uncaught exception.
    int           exitCode          = 0;
    std::uint64_t iterations        = 0;
    double        totalFrameSeconds = 0.0;
    double        worstFrameSeconds = 0.0;
    // Frame-time percentile support, only maintained when the latency report
    // is enabled: a fixed 4096-slot ring (bounded memory like the bus rings)
    // plus a lifetime sum for the mean.
    constexpr std::size_t frameSampleCapacity = 4096;
    std::vector<double>   frameSamples;
    frameSamples.reserve(frameSampleCapacity);
    if (latencyReportFlag || kernel.eventBus().latencyRecordingEnabled())
    {
        frameSamples.resize(frameSampleCapacity);
    }
    std::size_t frameSampleCount   = 0;
    std::size_t frameSampleNext    = 0;
    const bool  diagnosticsEnabled = !frameSamples.empty();
    try
    {
        while (mainWindow != engine::invalidWindowIdentifier && !window.windowCloseRequested(mainWindow))
        {
            window.pollEvents();
            if (window.windowCloseRequested(mainWindow))
            {
                break;
            }
            if (renderer != nullptr)
            {
                // Halfway through a bounded smoke run, queue one capture
                // through the screenshot module: this exercises the full
                // production capture path (queue flag, frame begin hand-off,
                // readback barriers, copyImageToBuffer, host mapping, PNG
                // write) in every automated run.
                if (screenshot != nullptr && iterations == smokeFrameLimit / 2)
                {
                    screenshot->requestScreenshot();
                }
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
            if (smokeFrameLimit > 0 && iterations >= smokeFrameLimit)
            {
                break;
            }
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

    // Light leak check: the [leak_test] section names a sealed region that
    // must stay dark and (optionally) a lit region that must receive light -
    // together they catch both leaks and a silently dead GI path. The
    // evaluation policy lives in leak_check.cpp; the loop reports the outcome.
    if (leakCheck && exitCode == 0)
    {
        std::string leakDiagnostic;
        if (!engine::evaluateLeakCheck(*hostContext, leakCapture, leakDiagnostic))
        {
            std::cerr << "engine leak check FAILED: " << leakDiagnostic << "\n";
            exitCode = 4;
        }
        else
        {
            std::cout << "engine leak check passed\n";
        }
    }

    // Communication and frame latency report: OPT-IN. A normal run publishes
    // thousands of events per second and its user does not want a data dump on
    // shutdown, so both measurement (see EventBus::setLatencyRecordingEnabled)
    // and printing are disabled unless [diagnostics] latency_report = true or
    // the --latency-report flag is given. With it enabled, the bus numbers are
    // the definitive dispatch latencies measured in publish(), and the frame
    // block reports mean plus the p50/p95/p99/max over the most recent ring
    // window.
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
