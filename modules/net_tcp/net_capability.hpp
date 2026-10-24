#ifndef ENGINE_MODULES_NET_NET_CAPABILITY_HPP
#define ENGINE_MODULES_NET_NET_CAPABILITY_HPP

// The published capability surface of the net module. Every consumer - other
// modules, native mods, the host - includes THIS header, from the module that
// provides the capability. Like every published capability header this stays
// implementation-, platform- and vendor-agnostic.
//
// The capability is the engine's TCP transport: a listener that accepts
// local or remote clients, length-prefixed topic/payload text frames in both
// directions, and the received traffic surfaced as bus events on the main
// thread (pump() runs inside the module's FrameBeginEvent handler, so the
// bus's single-thread contract holds without any consumer doing work).
//
// Scope honesty: this is a TELEMETRY AND CONTROL transport - scene queries,
// light tweaks, overlay state, test harness commands. It is not a realtime
// state-sync protocol: no delta compression, no prediction, no clock
// synchronization. Frames are small text messages; anything heavier deserves
// a dedicated provider behind the same family name.

#include <cstdint>
#include <string>

namespace engine
{

// Published on the bus (main thread, during pump) when a client sent a
// message. The payload is the raw UTF-8 text of the frame; interpreting it
// (JSON, TOML-ish key=value, plain words) is the consumer's business.
struct NetMessageReceivedEvent
{
    std::uint32_t clientId = 0;
    std::string   topic;
    std::string   payload;
};

// Published when a client completed the TCP handshake. The id is the
// connection's stable address for sendTo for the rest of the connection.
struct NetClientConnectedEvent
{
    std::uint32_t clientId = 0;
};

// Published when a client disconnected (remote close, error, or the listener
// shutting down). Ids are never reused within a run.
struct NetClientDisconnectedEvent
{
    std::uint32_t clientId = 0;
};

class NetCapability
{
public:
    // The implementation-agnostic family name of this capability. Consumers
    // acquire through it; the module's plugin descriptor carries the
    // implementation-bound member name ("net.tcp").
    static constexpr const char *capabilityName = "net";

    virtual ~NetCapability() = default;

    // Opens the listener socket (configuration: [net] port, bind_address -
    // the default binds loopback, because a debug transport must not expose
    // an engine to the network by accident). Idempotent: an already
    // listening transport keeps its socket. Returns false with a precise
    // message when the port cannot be bound.
    virtual bool startListener(std::string &errorReport) = 0;

    // Closes the listener and every client connection (disconnection events
    // surface through the next pump, not from inside this call).
    virtual void stopListener() = 0;

    virtual bool isListening() const = 0;

    // Connected client count (a snapshot; connections may change after the
    // call returns).
    virtual std::uint32_t connectedClientCount() const = 0;

    // Sends one text frame to every connected client. Returns false with a
    // precise message only when the transport never listened; a client that
    // died mid-send is reported through its disconnection event, not here.
    virtual bool broadcast(
        const std::string &topic, const std::string &payload, std::string &errorReport
    ) = 0;

    // Sends one text frame to one client. Returns false when the client id
    // is unknown (already gone, never connected) or the transport never
    // listened; the message names which.
    virtual bool sendTo(
        std::uint32_t clientId, const std::string &topic, const std::string &payload, std::string &errorReport
    ) = 0;

    // Drains the receive queue and publishes NetMessageReceivedEvent /
    // NetClientConnectedEvent / NetClientDisconnectedEvent on the bus. The
    // module calls this itself once per frame; consumers never do.
    virtual void pump() = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_NET_NET_CAPABILITY_HPP
