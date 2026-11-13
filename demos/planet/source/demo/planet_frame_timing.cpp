#include "planet_frame_timing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace demo::planet_demo
{
namespace
{

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

} // namespace demo::planet_demo
