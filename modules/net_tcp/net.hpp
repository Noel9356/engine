#ifndef ENGINE_MODULES_NET_NET_HPP
#define ENGINE_MODULES_NET_NET_HPP

#include "logging_capability.hpp"
#include "net_capability.hpp"
#include "renderer_capability.hpp" // FrameBeginEvent: the per-frame pump hook (the
                                   // screenshot pattern - subscribed without a graph
                                   // edge; no renderer in the run = no frames = inert)
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"
#include "net_sockets.hpp"

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace engine
{

class LoggerCapability;

// Module "net_tcp": the engine's TCP transport (family member "net.tcp").
//
// The transport is a listener plus a per-frame pump. A listener thread is
// deliberately ABSENT: sockets are polled non-blocking once per frame from
// the FrameBeginEvent handler, so every receive and every connection change
// surfaces on the main thread, in frame order, through the bus - the same
// single-thread guarantee every other event consumer relies on. Accepts,
// receives and disconnects between two pumps wait for the next pump; a debug
// transport's latency budget is a frame, and the code that buys that
// simplicity is a fraction of a threaded design.
//
// Wire format (net_sockets.hpp): one frame is
//   [uint32 LE topic length][topic][uint32 LE payload length][payload]
// with topics capped at 256 bytes and payloads at 1 MiB. A stream that
// violates the caps is corrupt; that client is disconnected with a precise
// log instead of being trusted.
//
// The listener is opt-in per configuration ([net].enabled, default false):
// a module whose plugin loads must not open a socket nobody asked for.
class NetModule final : public ModuleInterface, public NetCapability
{
public:
    NetModule();
    ~NetModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // NetCapability.
    bool startListener(std::string &errorReport) override;
    void stopListener() override;
    bool isListening() const override
    {
        return listener != net::invalidSocket;
    }

    std::uint32_t connectedClientCount() const override;
    bool broadcast(const std::string &topic, const std::string &payload, std::string &errorReport) override;
    bool sendTo(
        std::uint32_t clientId, const std::string &topic, const std::string &payload, std::string &errorReport
    ) override;
    void pump() override;

private:
    // The FrameBeginEvent handler: one non-blocking poll pass, then the
    // queued connection changes and received frames publish on the bus.
    void handleFrameBegin(const FrameBeginEvent &event);

    ComponentManifest componentManifest;
    ComponentContext *boundContext = nullptr;
    LoggerCapability *logger       = nullptr;

    EventChannel<FrameBeginEvent>         *frameBeginChannel    = nullptr;
    EventChannel<NetClientConnectedEvent> *clientConnectedChannel = nullptr;
    EventChannel<NetMessageReceivedEvent> *messageReceivedChannel = nullptr;
    EventChannel<NetClientDisconnectedEvent> *clientDisconnectedChannel = nullptr;
    EventSubscription frameBeginSubscription;

    // Configuration ([net] section, read once during construct).
    std::string bindAddress = "loopback";
    std::uint16_t port      = 8765;

    // Transport state. clients/clientIds/clientBuffers are only mutated on
    // the main thread (poll runs inside the frame handler); the mutex guards
    // the outbound queue that sendTo/broadcast fill from any capability
    // caller (still main thread today, but the contract does not depend on
    // it) against the pump's flush.
    net::SocketHandle                 listener        = net::invalidSocket;
    std::vector<net::SocketHandle>    clients;
    std::vector<std::uint32_t>        clientIds;
    std::vector<std::vector<std::uint8_t>> clientBuffers;
    std::uint32_t                     nextClientId    = 1;

    struct OutboundFrame
    {
        std::uint32_t clientId; // 0 = broadcast
        std::string   topic;
        std::string   payload;
    };
    std::mutex                 outboundMutex;
    std::vector<OutboundFrame> outboundQueue;
};

} // namespace engine

#endif // ENGINE_MODULES_NET_NET_HPP
