#ifndef ENGINE_MODULES_NET_NET_SOCKETS_HPP
#define ENGINE_MODULES_NET_NET_SOCKETS_HPP

// The socket seam of the net module: the smallest platform shims that keep
// the transport code platform agnostic. POSIX sockets on Linux and macOS,
// WinSock2 on Windows (MinGW included). Every function returns false with a
// precise message instead of throwing; the transport maps failures onto its
// own diagnostics.

#include <cstdint>
#include <string>
#include <vector>

namespace engine::net
{

using SocketHandle = std::int64_t; // POSIX fd or WinSock SOCKET; -1 = none

inline constexpr SocketHandle invalidSocket = -1;

// One receive result: a complete frame (topic, payload) or a connection-level
// status. Partial frames stay buffered inside the transport.
struct ReceivedFrame
{
    std::string  topic;
    std::string  payload;
    std::uint32_t clientId = 0;
};

// One connection status change surfaced by pollSockets.
struct ConnectionEvent
{
    enum class Kind
    {
        Connected,
        Disconnected
    };
    Kind          kind     = Kind::Connected;
    std::uint32_t clientId = 0;
};

// Process-wide WinSock startup/shutdown (no-ops on POSIX). Called by the
// module's start/stop; safe to nest.
bool initializeSockets(std::string &errorReport);
void shutdownSockets();

// Creates a TCP listening socket bound to bindAddress:port (bindAddress may
// be "0.0.0.0" for all interfaces; the transport defaults to loopback).
// backlog limits the simultaneously pending connections.
SocketHandle openListener(const std::string &bindAddress, std::uint16_t port, int backlog, std::string &errorReport);
void closeSocket(SocketHandle socket);

// Accepts one pending connection on a listening socket, or invalidSocket
// when none is waiting (non-blocking regardless of the caller's mode).
SocketHandle acceptClient(SocketHandle listener, std::string &errorReport);

// Sends a buffer, best effort (blocking send with a short SO_SNDTIMEO).
// Returns false on a hard error (the transport then drops the client).
bool sendBuffer(SocketHandle socket, const std::uint8_t *data, std::size_t size, std::string &errorReport);

// Polls every socket for readability and drains what arrived. New
// connections surface as ConnectionEvents; the transport assigns ids from
// nextClientId (monotonic, never reused - the caller owns the counter and
// keeps one id per client entry in clientIds, parallel to clients).
// Received frames surface in receivedFrames; clients whose connection broke
// surface as Disconnected and are closed.
bool pollSockets(
    SocketHandle listener, std::vector<SocketHandle> &clients, std::vector<std::uint32_t> &clientIds,
    std::vector<std::vector<std::uint8_t>> &clientBuffers, std::uint32_t &nextClientId,
    std::vector<ConnectionEvent> &connectionEvents, std::vector<ReceivedFrame> &receivedFrames,
    std::string &errorReport
);

} // namespace engine::net

#endif // ENGINE_MODULES_NET_NET_SOCKETS_HPP
