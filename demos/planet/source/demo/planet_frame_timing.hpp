// Per-frame timing capture for the planet demo: records how long each rendered frame spends in planet streaming,
// chunk upload, and draw submission, then reports percentile statistics and an optional CSV export so frame-time
// tails can be analysed without an external profiler.
#ifndef ENGINE_DEMO_PLANET_FRAME_TIMING_HPP
#define ENGINE_DEMO_PLANET_FRAME_TIMING_HPP

#include <chrono>
#include <string>
#include <vector>

namespace demo::planet_demo
{

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

private:
    using Clock = std::chrono::steady_clock;

    std::vector<FrameTimingSample> samples_;
    Clock::time_point frameStart_{};
    Clock::time_point lastMark_{};
    FrameTimingSample current_{};
};

} // namespace demo::planet_demo

#endif
