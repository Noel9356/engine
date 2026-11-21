// Implementation of the shared demo scaffolding; see demo_harness.hpp.
#include "demo_harness.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <ratio>
#include <sstream>

namespace demo::harness
{
namespace
{

// Image format the requested screenshot path selects: a .ppm suffix keeps
// the legacy P6 PPM; anything else (including no suffix) is PNG, the
// default.
bool screenshotIsPpm(const std::string &requestedPath)
{
    const std::size_t lastDot = requestedPath.rfind('.');
    if (lastDot == std::string::npos || lastDot == 0u)
    {
        return false;
    }
    const std::string suffix = requestedPath.substr(lastDot);
    return suffix == ".ppm" || suffix == ".PPM";
}

// Completes a suffix-less request with .png (the default format) so the
// written file carries a name the file manager recognizes.
std::string resolveScreenshotPath(const std::string &requestedPath)
{
    if (screenshotIsPpm(requestedPath))
    {
        return requestedPath;
    }
    const std::size_t lastDot = requestedPath.rfind('.');
    if (lastDot == std::string::npos || lastDot == 0u)
    {
        return requestedPath + ".png";
    }
    return requestedPath;
}

double millisecondsBetween(std::chrono::steady_clock::time_point from, std::chrono::steady_clock::time_point to)
{
    return std::chrono::duration<double, std::milli>(to - from).count();
}

double nearestRankPercentile(const std::vector<double> &sortedValues, double percentile)
{
    if (sortedValues.empty())
    {
        return 0.0;
    }
    const double rank = std::ceil(percentile / 100.0 * static_cast<double>(sortedValues.size()));
    const std::size_t index = static_cast<std::size_t>(std::max(1.0, rank)) - 1;
    return sortedValues[std::min(index, sortedValues.size() - 1)];
}

} // namespace

bool exportScreenshot(engine::ParticipantContext &context, engine::window::WindowCapability &windowCapability,
    engine::window::WindowHandle windowHandle, engine::renderer::RenderCapability &renderCapability,
    engine::renderer::RendererHandle rendererHandle, const std::string &requestedPath, std::string &writtenPath)
{
    writtenPath = resolveScreenshotPath(requestedPath);
    const bool png = !screenshotIsPpm(requestedPath);
    bool exported = false;
    auto *headlessWindowCapability = context.tryAcquireCapability<engine::window::HeadlessWindowCapability>();
    if (headlessWindowCapability != nullptr
        && windowCapability.activeBackendName(windowCapability.self) == std::string("headless"))
    {
        engine::window::WindowError exportError = engine::window::WindowError::None;
        exported = png ? headlessWindowCapability->exportSurfacePng(
                           headlessWindowCapability->self, windowHandle, writtenPath.c_str(), &exportError)
                       : headlessWindowCapability->exportSurfacePpm(
                           headlessWindowCapability->self, windowHandle, writtenPath.c_str(), &exportError);
    }
    if (!exported)
    {
        auto *vulkanCapability = context.tryAcquireCapability<engine::renderer::VulkanRenderCapability>();
        if (vulkanCapability != nullptr
            && renderCapability.activeBackendName(renderCapability.self, rendererHandle) == std::string("vulkan"))
        {
            engine::renderer::RenderError exportError = engine::renderer::RenderError::None;
            exported = png ? vulkanCapability->exportFramePng(
                               vulkanCapability->self, rendererHandle, writtenPath.c_str(), &exportError)
                           : vulkanCapability->exportFramePpm(
                               vulkanCapability->self, rendererHandle, writtenPath.c_str(), &exportError);
        }
    }
    return exported;
}

void clampCursorDelta(double &deltaX, double &deltaY, double maximumCursorDeltaPixels)
{
    deltaX = std::clamp(deltaX, -maximumCursorDeltaPixels, maximumCursorDeltaPixels);
    deltaY = std::clamp(deltaY, -maximumCursorDeltaPixels, maximumCursorDeltaPixels);
}

void FrameTimingRecorder::setAutoFlushPath(const std::string &path)
{
    autoFlushPath_ = path;
}

void FrameTimingRecorder::beginFrame()
{
    frameStart_ = Clock::now();
    lastMark_ = frameStart_;
    current_ = {};
}

void FrameTimingRecorder::markUpdateDone()
{
    const auto now = Clock::now();
    current_.updateMilliseconds = millisecondsBetween(lastMark_, now);
    lastMark_ = now;
}

void FrameTimingRecorder::markUploadDone()
{
    const auto now = Clock::now();
    current_.uploadMilliseconds = millisecondsBetween(lastMark_, now);
    lastMark_ = now;
}

void FrameTimingRecorder::markDrawDone()
{
    const auto now = Clock::now();
    current_.drawMilliseconds = millisecondsBetween(lastMark_, now);
    current_.totalMilliseconds = millisecondsBetween(frameStart_, now);
    samples_.push_back(current_);
    // Auto-flush after the frame is complete and out of the measured phases:
    // the CSV is rewritten atomically (sibling temporary file, then rename),
    // so a run that is interrupted mid-measurement (process kill, machine
    // shutdown) still keeps the timings of the frames it managed to render
    // and a reader never observes a half-written file. Flushing is throttled
    // to a wall-clock interval so long measurement runs do not pay quadratic
    // I/O; the final writeCsv at run end always publishes the complete set.
    if (!autoFlushPath_.empty())
    {
        const auto flushElapsed = millisecondsBetween(lastAutoFlush_, now);
        if (lastAutoFlush_ == Clock::time_point{} || flushElapsed >= kAutoFlushIntervalMilliseconds)
        {
            const std::string temporaryPath = autoFlushPath_ + ".tmp";
            if (writeCsv(temporaryPath))
            {
                std::rename(temporaryPath.c_str(), autoFlushPath_.c_str());
                lastAutoFlush_ = now;
            }
        }
    }
}

std::string FrameTimingRecorder::summary() const
{
    if (samples_.empty())
    {
        return "frame timing: no frames recorded\n";
    }
    std::vector<double> totals;
    totals.reserve(samples_.size());
    double sum = 0.0;
    for (const FrameTimingSample &sample : samples_)
    {
        totals.push_back(sample.totalMilliseconds);
        sum += sample.totalMilliseconds;
    }
    std::sort(totals.begin(), totals.end());

    // The slowest frame's phase split shows which stage owns the tail.
    const auto slowest = std::max_element(samples_.begin(), samples_.end(),
        [](const FrameTimingSample &a, const FrameTimingSample &b) { return a.totalMilliseconds < b.totalMilliseconds; });

    char line[256];
    std::ostringstream report;
    std::snprintf(line, sizeof(line), "frame timing: %zu frames, mean %.3f ms\n", samples_.size(),
        sum / static_cast<double>(samples_.size()));
    report << line;
    std::snprintf(line, sizeof(line), "frame timing: p50 %.3f | p90 %.3f | p99 %.3f | p99.9 %.3f | max %.3f ms\n",
        nearestRankPercentile(totals, 50.0), nearestRankPercentile(totals, 90.0), nearestRankPercentile(totals, 99.0),
        nearestRankPercentile(totals, 99.9), totals.back());
    report << line;
    std::snprintf(line, sizeof(line), "frame timing: slowest frame - update %.3f, upload %.3f, draw %.3f ms\n",
        slowest->updateMilliseconds, slowest->uploadMilliseconds, slowest->drawMilliseconds);
    report << line;
    return report.str();
}

bool FrameTimingRecorder::writeCsv(const std::string &path) const
{
    std::ofstream file(path);
    if (!file)
    {
        return false;
    }
    file << "frame,total_ms,update_ms,upload_ms,draw_ms\n";
    for (std::size_t index = 0; index < samples_.size(); ++index)
    {
        const FrameTimingSample &sample = samples_[index];
        file << index << ',' << sample.totalMilliseconds << ',' << sample.updateMilliseconds << ','
             << sample.uploadMilliseconds << ',' << sample.drawMilliseconds << '\n';
    }
    return static_cast<bool>(file);
}

} // namespace demo::harness
