// Shared scaffolding for the standalone demos: the window-event drain loop,
// the headless/Vulkan screenshot export pair, the cursor-delta clamp, and the
// per-frame timing recorder. Demo-specific behavior (what a resize does,
// which statistics print, which views exist) stays in each demo; only the
// mechanics that would otherwise be copy-pasted between the demos live here.
//
// The file is compiled into every demo target directly (each demo's
// CMakeLists lists it in SOURCES), so a demo checkout remains standalone: no
// new library, no installed headers.
#pragma once

#include <engine/kernel/participant_interface.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace demo::harness
{

// Pops events until the window queue is drained (the window module hands out
// at most buffer-capacity events per call, so a busy queue needs repeated
// pops) and invokes onEvent for every popped event. Returns the total number
// of events dispatched. The buffer is borrowed, not grown: its capacity is
// the per-poll batch size (any contiguous container of WindowEvent works).
template <typename EventBuffer, typename OnEvent>
std::uint32_t drainWindowEvents(engine::window::WindowCapability &windowCapability,
    engine::window::WindowHandle windowHandle, EventBuffer &eventBuffer, const OnEvent &onEvent)
{
    std::uint32_t dispatched = 0;
    std::uint32_t eventCount = 0;
    do
    {
        eventCount = windowCapability.popEvents(windowCapability.self,
            windowHandle,
            eventBuffer.data(),
            static_cast<std::uint32_t>(eventBuffer.size()));
        for (std::uint32_t eventIndex = 0; eventIndex < eventCount; ++eventIndex)
        {
            onEvent(eventBuffer[eventIndex]);
        }
        dispatched += eventCount;
    } while (eventCount == eventBuffer.size());
    return dispatched;
}

// Exports the current frame as a PNG (the default) or, when the requested
// path carries a .ppm suffix, as a binary P6 PPM - the same pixel payload
// either way. A path with no known image suffix is completed with .png so a
// DEMO_SCREENSHOT name without a suffix writes <name>.png; the interactive
// F12 key names its own timestamped file. Headless windows export their CPU
// surface directly; Vulkan renderers schedule the export through the
// renderer's vulkan extension (written at the next frame retirement). Both
// paths are tried in that order; the return value reports success and
// writtenPath carries the file actually written (the caller owns the
// one-line log output, prefixed with its own demo name).
bool exportScreenshot(engine::ParticipantContext &context, engine::window::WindowCapability &windowCapability,
    engine::window::WindowHandle windowHandle, engine::renderer::RenderCapability &renderCapability,
    engine::renderer::RendererHandle rendererHandle, const std::string &requestedPath, std::string &writtenPath);

// Per-frame timing capture: records how long each rendered frame spends in
// its update, upload, and draw phases, then reports percentile statistics and
// an optional CSV export so frame-time tails can be analysed without an
// external profiler. Phases are marked in order; a frame with no marks is
// not recorded.
struct FrameTimingSample
{
    double totalMilliseconds = 0.0;
    double updateMilliseconds = 0.0;
    double uploadMilliseconds = 0.0;
    double drawMilliseconds = 0.0;
    double gpuMilliseconds = 0.0;
};

class FrameTimingRecorder final
{
public:
    void beginFrame();
    void markUpdateDone();
    void markUploadDone();
    void markDrawDone();
    void recordGpuMilliseconds(double milliseconds);

    [[nodiscard]] const std::vector<FrameTimingSample> &samples() const noexcept
    {
        return samples_;
    }

    // Percentile statistics over the recorded frames (nearest-rank), formatted as a multi-line report.
    [[nodiscard]] std::string summary() const;

    // Writes one CSV row per frame; returns false when the file cannot be written.
    [[nodiscard]] bool writeCsv(const std::string &path) const;

    // Installs a path that is rewritten (atomically, throttled to a fixed
    // wall-clock interval) after completed frames, so a run that is
    // interrupted mid-measurement (process kill, machine shutdown) still keeps
    // the timings of the frames it managed to render. Passing an empty string
    // disables auto-flushing. Flushing happens outside the measured phases: it
    // never inflates the update, upload, or draw times of the frame itself.
    void setAutoFlushPath(const std::string &path);

private:
    using Clock = std::chrono::steady_clock;

    static constexpr double kAutoFlushIntervalMilliseconds = 250.0;

    std::vector<FrameTimingSample> samples_;
    Clock::time_point frameStart_{};
    Clock::time_point lastMark_{};
    Clock::time_point lastAutoFlush_{};
    FrameTimingSample current_{};
    std::string autoFlushPath_;
};

// Resolves the effective software frame-rate cap for a demo's frame pacer
// from the engine.max_fps contract:
//
//   max_fps > 0  - the loop never runs faster than max_fps. The software
//                  pacer enforces this on its own; a blocking present mode
//                  (FIFO/vsync) is only trusted to stand down the pacer when
//                  the display itself already caps the loop at or below
//                  max_fps (presentedPacing && refresh <= max_fps), because
//                  vsync alone cannot run slower than the display.
//   max_fps == 0 - the loop never runs faster than the display. A blocking
//                  present mode already does exactly that; otherwise the
//                  pacer takes the monitor's refresh rate as its target
//                  (0/unknown leaves the loop uncapped).
//   headless     - never paced: capture and benchmark runs measure raw cost.
//
// The monitor rate comes from the window attributes (refresh of the monitor
// the window sits on); pass 0 when the demo did not query one.
inline double resolveFramePaceTargetHz(bool windowed, std::int64_t maxFps, bool presentedPacing,
    double monitorRefreshHz)
{
    if (!windowed)
    {
        return 0.0;
    }
    if (maxFps > 0)
    {
        if (presentedPacing && monitorRefreshHz > 0.0 && monitorRefreshHz <= static_cast<double>(maxFps))
        {
            return 0.0; // the display's own cap is at least as strict as max_fps
        }
        return static_cast<double>(maxFps);
    }
    if (presentedPacing)
    {
        return 0.0; // vsync paces the loop at the display rate already
    }
    return monitorRefreshHz > 0.0 ? monitorRefreshHz : 0.0;
}

// Installs the resolved target into a FramePacer and prints the one-line
// explanation a windowed run owes the console; returns the effective target
// (0 = uncapped). Defined here so both demos narrate the limit identically.
template <typename FramePacer>
double applyFramePaceTarget(FramePacer &pacer, bool windowed, std::int64_t maxFps, bool presentedPacing,
    double monitorRefreshHz, const char *demoPrefix)
{
    const double targetHz = resolveFramePaceTargetHz(windowed, maxFps, presentedPacing, monitorRefreshHz);
    pacer.setOptions({.targetFrameRateHz = targetHz, .spinWindowMicroseconds = targetHz > 0.0 ? 200u : 0u});
    if (!windowed)
    {
        return targetHz;
    }
    if (maxFps > 0)
    {
        if (targetHz <= 0.0)
        {
            std::printf("%s: max_fps %lld left to the display - vsync already paces the frame loop at %s\n",
                demoPrefix, static_cast<long long>(maxFps),
                monitorRefreshHz > 0.0 ? "its rate or below" : "or below its rate");
        }
        else
        {
            std::printf("%s: frame limit %lld fps (engine.max_fps)\n", demoPrefix, static_cast<long long>(maxFps));
        }
    }
    else if (targetHz > 0.0)
    {
        std::printf("%s: frame limit %.0f fps (engine.max_fps 0 - the display's refresh rate)\n",
            demoPrefix, targetHz);
    }
    else if (!presentedPacing)
    {
        std::printf("%s: frame limit uncapped (engine.max_fps 0, display rate unknown, no vsync)\n", demoPrefix);
    }
    else
    {
        std::printf("%s: frame limit left to vsync (engine.max_fps 0 - presentation paces the frame loop)\n",
            demoPrefix);
    }
    return targetHz;
}

} // namespace demo::harness
