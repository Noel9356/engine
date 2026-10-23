#include "leak_check.hpp"

#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/configuration.hpp"

#include <algorithm>

namespace engine
{

namespace
{

// Leak-check defaults ([leak_test] may override): a sealed region may hold at
// most this normalized luminance, a lit region must reach the minimum.
constexpr double defaultLeakTestMaxLuminance    = 0.02;
constexpr double defaultLeakTestMinLitLuminance = 0.05;
// BT.709 relative luminance weights for the 0..255 sRGB capture.
constexpr double luminanceRedWeight   = 0.2126;
constexpr double luminanceGreenWeight = 0.7152;
constexpr double luminanceBlueWeight  = 0.0722;

void captureSink(
    void *userData, const FrameCapturedEvent &event
)
{
    static_cast<LeakCapture *>(userData)->pixels.assign(
        event.pixelData, event.pixelData + std::size_t(event.width) * event.height * 4
    );
    static_cast<LeakCapture *>(userData)->width  = event.width;
    static_cast<LeakCapture *>(userData)->height = event.height;
}

} // namespace

void subscribeLeakCapture(
    EventBus &bus, LeakCapture &capture
)
{
    EventChannel<FrameCapturedEvent> &capturedChannel = bus.channel<FrameCapturedEvent>();
    capturedChannel.subscribe(&captureSink, &capture);
}

bool evaluateLeakCheck(
    const ComponentContext &hostContext, const LeakCapture &capture, std::string &diagnostic
)
{
    const double              maxSealedLuminance = hostContext
                                                       .configurationFloatingInSection(
                                                           ConfigurationStore::leakTestSectionName,
                                                           ConfigurationStore::leakTestMaxLuminanceKey
                                                       )
                                                       .value_or(defaultLeakTestMaxLuminance);
    const std::vector<double> sealedRegion       = hostContext.configurationFloatingArrayInSection(
        ConfigurationStore::leakTestSectionName, ConfigurationStore::leakTestRegionKey
    );
    bool leakDetected = false;
    diagnostic        = "no captured frame";
    if (!sealedRegion.empty())
    {
        if (capture.pixels.empty())
        {
            leakDetected = true;
            diagnostic   = "leak check requested but no frame was captured";
        }
        else if (sealedRegion.size() != 4)
        {
            leakDetected = true;
            diagnostic   = "leak_test.region must hold [x0, y0, x1, y1] in normalized coordinates";
        }
        else
        {
            const auto luminanceAt = [&capture](std::uint32_t x, std::uint32_t y)
            {
                const std::size_t offset = (std::size_t(y) * capture.width + x) * 4;
                return luminanceRedWeight * capture.pixels[offset] + luminanceGreenWeight * capture.pixels[offset + 1]
                     + luminanceBlueWeight * capture.pixels[offset + 2];
            };
            const std::uint32_t x0          = static_cast<std::uint32_t>(sealedRegion[0] * capture.width);
            const std::uint32_t y0          = static_cast<std::uint32_t>(sealedRegion[1] * capture.height);
            const std::uint32_t x1          = static_cast<std::uint32_t>(sealedRegion[2] * capture.width);
            const std::uint32_t y1          = static_cast<std::uint32_t>(sealedRegion[3] * capture.height);
            double              worstSealed = 0.0;
            for (std::uint32_t y = y0; y < y1 && !leakDetected; ++y)
            {
                for (std::uint32_t x = x0; x < x1; ++x)
                {
                    worstSealed = std::max(worstSealed, luminanceAt(x, y));
                }
            }
            if (worstSealed > maxSealedLuminance * 255.0)
            {
                leakDetected = true;
                diagnostic   = "sealed region luminance " + std::to_string(worstSealed) + "/255 exceeds "
                             + std::to_string(maxSealedLuminance * 255.0) + "/255";
            }

            // Optional lit-region probe: guards against a GI path that went
            // fully dark (which would also pass the sealed check).
            const std::vector<double> litRegion = hostContext.configurationFloatingArrayInSection(
                ConfigurationStore::leakTestSectionName, ConfigurationStore::leakTestLitRegionKey
            );
            if (!leakDetected && litRegion.size() == 4)
            {
                const double        minLitLuminance = hostContext
                                                          .configurationFloatingInSection(
                                                              ConfigurationStore::leakTestSectionName,
                                                              ConfigurationStore::leakTestMinLitLuminanceKey
                                                          )
                                                          .value_or(defaultLeakTestMinLitLuminance);
                const std::uint32_t lx0             = static_cast<std::uint32_t>(litRegion[0] * capture.width);
                const std::uint32_t ly0             = static_cast<std::uint32_t>(litRegion[1] * capture.height);
                const std::uint32_t lx1             = static_cast<std::uint32_t>(litRegion[2] * capture.width);
                const std::uint32_t ly1             = static_cast<std::uint32_t>(litRegion[3] * capture.height);
                double              bestLit         = 0.0;
                for (std::uint32_t y = ly0; y < ly1; ++y)
                {
                    for (std::uint32_t x = lx0; x < lx1; ++x)
                    {
                        bestLit = std::max(bestLit, luminanceAt(x, y));
                    }
                }
                if (bestLit < minLitLuminance * 255.0)
                {
                    leakDetected = true;
                    diagnostic   = "lit region luminance " + std::to_string(bestLit) + "/255 never reached "
                                 + std::to_string(minLitLuminance * 255.0) + "/255 (GI path produced no light)";
                }
            }
        }
    }
    return !leakDetected;
}

} // namespace engine
