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
// either way. A path with no known image suffix is completed with .png so
// --screenshot <name> writes <name>.png. Headless windows export their CPU
// surface directly; Vulkan renderers schedule the export through the
// renderer's vulkan extension (written at the next frame retirement). Both
// paths are tried in that order; the return value reports success and
// writtenPath carries the file actually written (the caller owns the
// one-line log output, prefixed with its own demo name).
bool exportScreenshot(engine::ParticipantContext &context, engine::window::WindowCapability &windowCapability,
    engine::window::WindowHandle windowHandle, engine::renderer::RenderCapability &renderCapability,
    engine::renderer::RendererHandle rendererHandle, const std::string &requestedPath, std::string &writtenPath);

// Clamps a cursor delta to the +/-maximumCursorDeltaPixels band the demos
// use: a window-system glitch (or a warp across screens) must not throw the
// camera to the horizon in one frame. Applied symmetrically to x and y.
void clampCursorDelta(double &deltaX, double &deltaY, double maximumCursorDeltaPixels);

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
};

class FrameTimingRecorder final
{
public:
    void beginFrame();
    void markUpdateDone();
    void markUploadDone();
    void markDrawDone();

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

} // namespace demo::harness
