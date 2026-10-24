// The POSIX/WinSock shims behind net_sockets.hpp. One translation unit on
// every platform; the #ifdef surface lives here and nowhere else.
#include "net_sockets.hpp"

#include <chrono>
#include <cstring>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace engine::net
{

namespace
{

constexpr int listenerBacklogDefault = 8;

#if defined(_WIN32)
void closeSocketHandle(SocketHandle socket)
{
    ::closesocket(static_cast<SOCKET>(socket));
}

bool wouldBlock()
{
    return WSAGetLastError() == WSAEWOULDBLOCK;
}
#else
void closeSocketHandle(SocketHandle socket)
{
    ::close(static_cast<int>(socket));
}

bool wouldBlock()
{
    return errno == EAGAIN || errno == EWOULDBLOCK;
}
#endif

// Wire frame: [uint32 LE topic length][topic bytes][uint32 LE payload length]
// [payload bytes]. Length-prefixed, binary safe, trivially parseable by any
// test client; maximum sizes keep a rogue length prefix from OOMing the run.
constexpr std::uint32_t maxTopicLength   = 256;
constexpr std::uint32_t maxPayloadLength = 1024 * 1024;

void appendUint32(
    std::vector<std::uint8_t> &buffer, std::uint32_t value
)
{
    buffer.push_back(static_cast<std::uint8_t>(value & 0xFF));
    buffer.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    buffer.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    buffer.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

bool readUint32(
    const std::vector<std::uint8_t> &buffer, std::size_t offset, std::uint32_t &value
)
{
    if (offset + 4 > buffer.size())
    {
        return false;
    }
    value = static_cast<std::uint32_t>(buffer[offset]) | (static_cast<std::uint32_t>(buffer[offset + 1]) << 8)
          | (static_cast<std::uint32_t>(buffer[offset + 2]) << 16)
          | (static_cast<std::uint32_t>(buffer[offset + 3]) << 24);
    return true;
}

// Frame assembler: consumes bytes off one client's buffer, emitting complete
// frames. Returns false when the stream is corrupt (oversized lengths) - the
// transport disconnects that client.
bool extractFrames(
    std::vector<std::uint8_t> &buffer, std::uint32_t clientId, std::vector<ReceivedFrame> &out
)
{
    std::size_t cursor = 0;
    while (true)
    {
        std::uint32_t topicLength = 0;
        if (!readUint32(buffer, cursor, topicLength))
        {
            break; // need more bytes for the length prefix
        }
        if (topicLength > maxTopicLength)
        {
            return false;
        }
        std::uint32_t payloadLength = 0;
        if (!readUint32(buffer, cursor + 4, payloadLength))
        {
            break; // need more bytes for the payload length
        }
        if (payloadLength > maxPayloadLength)
        {
            return false;
        }
        const std::size_t frameSize = 8 + static_cast<std::size_t>(topicLength) + payloadLength;
        if (buffer.size() - cursor < frameSize)
        {
            break; // need more bytes for the body
        }
        ReceivedFrame frame;
        frame.clientId = clientId;
        frame.topic.assign(
            reinterpret_cast<const char *>(buffer.data()) + cursor + 8, topicLength
        );
        frame.payload.assign(
            reinterpret_cast<const char *>(buffer.data()) + cursor + 8 + topicLength, payloadLength
        );
        out.push_back(std::move(frame));
        cursor += frameSize;
    }
    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(cursor));
    return true;
}

} // namespace

bool initializeSockets(
    std::string &errorReport
)
{
#if defined(_WIN32)
    WSADATA data;
    const int started = ::WSAStartup(MAKEWORD(2, 2), &data);
    if (started != 0)
    {
        errorReport = "winsock startup failed (WSAStartup error " + std::to_string(started) + ")";
        return false;
    }
#endif
    (void)errorReport;
    return true;
}

void shutdownSockets()
{
#if defined(_WIN32)
    ::WSACleanup();
#endif
}

SocketHandle openListener(
    const std::string &bindAddress, std::uint16_t port, int backlog, std::string &errorReport
)
{
#if defined(_WIN32)
    SOCKET raw = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#else
    SocketHandle raw = ::socket(AF_INET, SOCK_STREAM, 0);
#endif
    if (raw == invalidSocket)
    {
        errorReport = "socket() failed: " + std::string(std::strerror(errno));
        return invalidSocket;
    }

    // REUSEADDR: a restart within the TIME_WAIT window must not need a
    // minute of patience - a debug transport restarts often.
    int reuse = 1;
    setsockopt(
        static_cast<int>(raw), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&reuse), sizeof(reuse)
    );

    sockaddr_in address{};
    address.sin_family      = AF_INET;
    address.sin_port        = htons(port);
    address.sin_addr.s_addr = bindAddress.empty() || bindAddress == "loopback" ? htonl(INADDR_LOOPBACK)
                              : bindAddress == "0.0.0.0"                       ? INADDR_ANY
                                                                               : inet_addr(bindAddress.c_str());
    if (address.sin_addr.s_addr == static_cast<decltype(address.sin_addr.s_addr)>(-1))
    {
        errorReport = "bind address '" + bindAddress + "' is not a valid IPv4 address";
        closeSocketHandle(raw);
        return invalidSocket;
    }
    if (::bind(static_cast<int>(raw), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0)
    {
        errorReport = "bind(" + bindAddress + ":" + std::to_string(port)
                    + ") failed: " + std::string(std::strerror(errno));
        closeSocketHandle(raw);
        return invalidSocket;
    }
    if (::listen(static_cast<int>(raw), backlog > 0 ? backlog : listenerBacklogDefault) != 0)
    {
        errorReport = "listen(" + std::to_string(port) + ") failed: " + std::string(std::strerror(errno));
        closeSocketHandle(raw);
        return invalidSocket;
    }

#if !defined(_WIN32)
    // Non-blocking accept/receive: the transport polls, it never parks the
    // main thread on a socket.
    const int flags = ::fcntl(static_cast<int>(raw), F_GETFL, 0);
    ::fcntl(static_cast<int>(raw), F_SETFL, flags | O_NONBLOCK);
#endif
    return raw;
}

void closeSocket(
    SocketHandle socket
)
{
    if (socket != invalidSocket)
    {
        closeSocketHandle(socket);
    }
}

SocketHandle acceptClient(
    SocketHandle listener, std::string &errorReport
)
{
    (void)errorReport;
#if defined(_WIN32)
    u_long nonBlocking = 1;
    SOCKET raw = ::accept(static_cast<SOCKET>(listener), nullptr, nullptr);
    if (raw != INVALID_SOCKET)
    {
        ::ioctlsocket(raw, FIONBIO, &nonBlocking);
    }
    return raw == INVALID_SOCKET ? invalidSocket : static_cast<SocketHandle>(raw);
#else
    SocketHandle raw = ::accept(static_cast<int>(listener), nullptr, nullptr);
    if (raw == invalidSocket)
    {
        return invalidSocket;
    }
    const int flags = ::fcntl(static_cast<int>(raw), F_GETFL, 0);
    ::fcntl(static_cast<int>(raw), F_SETFL, flags | O_NONBLOCK);
    // Disable Nagle: a debug transport wants its small frames out now.
    int noDelay = 1;
    ::setsockopt(
        static_cast<int>(raw), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&noDelay), sizeof(noDelay)
    );
    return raw;
#endif
}

bool sendBuffer(
    SocketHandle socket, const std::uint8_t *data, std::size_t size, std::string &errorReport
)
{
    std::size_t sent = 0;
    while (sent < size)
    {
#if defined(_WIN32)
        const int chunk = ::send(
            static_cast<SOCKET>(socket), reinterpret_cast<const char *>(data + sent),
            static_cast<int>(size - sent), 0
        );
#else
        const ssize_t chunk = ::send(
            static_cast<int>(socket), reinterpret_cast<const char *>(data + sent), size - sent, MSG_NOSIGNAL
        );
#endif
        if (chunk <= 0)
        {
            if (chunk < 0 && wouldBlock())
            {
                continue; // transient; the socket is writable within microseconds
            }
            errorReport = std::string("send failed: ") + std::strerror(errno);
            return false;
        }
        sent += static_cast<std::size_t>(chunk);
    }
    return true;
}

namespace
{

// Accepts one pending connection, appends it (socket, id, empty buffer) to
// the caller's parallel vectors and surfaces the Connected event. The ids
// come from the caller's monotonic counter, so an id is never reused within
// a run - NetClientDisconnectedEvent consumers can trust that a later id
// always means a NEW connection.
void acceptPending(
    SocketHandle listener, std::vector<SocketHandle> &clients, std::vector<std::uint32_t> &clientIds,
    std::vector<std::vector<std::uint8_t>> &clientBuffers, std::uint32_t &nextClientId,
    std::vector<ConnectionEvent> &connectionEvents
)
{
    std::string acceptError;
    const SocketHandle client = acceptClient(listener, acceptError);
    if (client == invalidSocket)
    {
        return;
    }
    clients.push_back(client);
    clientIds.push_back(nextClientId);
    clientBuffers.emplace_back();
    connectionEvents.push_back(ConnectionEvent{ConnectionEvent::Kind::Connected, nextClientId});
    ++nextClientId;
}

// One readable client: drain bytes into its buffer, emit complete frames,
// close on error or corrupt stream (the Disconnected event surfaces either
// way). Returns false when the client entry was removed and the caller must
// not advance its parallel-vector cursor.
bool drainClient(
    std::size_t index, std::vector<SocketHandle> &clients, std::vector<std::uint32_t> &clientIds,
    std::vector<std::vector<std::uint8_t>> &clientBuffers, std::vector<ConnectionEvent> &connectionEvents,
    std::vector<ReceivedFrame> &receivedFrames, bool corruptStream
)
{
    const std::uint32_t clientId = clientIds[index];
    closeSocketHandle(clients[index]);
    connectionEvents.push_back(ConnectionEvent{ConnectionEvent::Kind::Disconnected, clientId});
    clients.erase(clients.begin() + static_cast<std::ptrdiff_t>(index));
    clientIds.erase(clientIds.begin() + static_cast<std::ptrdiff_t>(index));
    clientBuffers.erase(clientBuffers.begin() + static_cast<std::ptrdiff_t>(index));
    (void)corruptStream;
    return false;
}

} // namespace

bool pollSockets(
    SocketHandle listener, std::vector<SocketHandle> &clients, std::vector<std::uint32_t> &clientIds,
    std::vector<std::vector<std::uint8_t>> &clientBuffers, std::uint32_t &nextClientId,
    std::vector<ConnectionEvent> &connectionEvents, std::vector<ReceivedFrame> &receivedFrames,
    std::string &errorReport
)
{
    if (listener == invalidSocket)
    {
        errorReport = "poll on a transport that never opened a listener";
        return false;
    }

#if defined(_WIN32)
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(static_cast<SOCKET>(listener), &readSet);
    SOCKET maxFd = static_cast<SOCKET>(listener);
    for (const SocketHandle client : clients)
    {
        FD_SET(static_cast<SOCKET>(client), &readSet);
        if (static_cast<SOCKET>(client) > maxFd)
        {
            maxFd = static_cast<SOCKET>(client);
        }
    }
    timeval timeout{};
    const int ready = ::select(static_cast<int>(maxFd) + 1, &readSet, nullptr, nullptr, &timeout);
    if (ready < 0)
    {
        errorReport = "select failed (WSAGetLastError " + std::to_string(WSAGetLastError()) + ")";
        return false;
    }
    if (ready == 0)
    {
        return true;
    }
    if (FD_ISSET(static_cast<SOCKET>(listener), &readSet))
    {
        acceptPending(listener, clients, clientIds, clientBuffers, nextClientId, connectionEvents);
    }
    for (std::size_t index = 0; index < clients.size();)
    {
        if (!FD_ISSET(static_cast<SOCKET>(clients[index]), &readSet))
        {
            ++index;
            continue;
        }
        std::uint8_t chunk[4096];
        const int received = ::recv(
            static_cast<SOCKET>(clients[index]), reinterpret_cast<char *>(chunk), sizeof(chunk), 0
        );
        if (received <= 0)
        {
            drainClient(index, clients, clientIds, clientBuffers, connectionEvents, receivedFrames, false);
            continue; // parallel vectors shrank; same index re-examined
        }
        clientBuffers[index].insert(clientBuffers[index].end(), chunk, chunk + received);
        if (!extractFrames(clientBuffers[index], clientIds[index], receivedFrames))
        {
            drainClient(index, clients, clientIds, clientBuffers, connectionEvents, receivedFrames, true);
            continue;
        }
        ++index;
    }
    return true;
#else
    std::vector<pollfd> fds;
    fds.push_back(pollfd{static_cast<int>(listener), POLLIN, 0});
    for (const SocketHandle client : clients)
    {
        fds.push_back(pollfd{static_cast<int>(client), POLLIN, 0});
    }
    const int ready = ::poll(fds.data(), static_cast<nfds_t>(fds.size()), 0);
    if (ready < 0)
    {
        if (errno == EINTR)
        {
            return true;
        }
        errorReport = std::string("poll failed: ") + std::strerror(errno);
        return false;
    }
    if (ready == 0)
    {
        return true;
    }
    if (fds[0].revents & POLLIN)
    {
        acceptPending(listener, clients, clientIds, clientBuffers, nextClientId, connectionEvents);
    }
    for (std::size_t index = 0; index < clients.size();)
    {
        if (1 + index >= fds.size())
        {
            // A client accepted during THIS poll has no fd entry yet; its
            // bytes drain on the next poll. (The select path is immune: the
            // fresh socket was not in the read set to begin with.)
            break;
        }
        const pollfd &entry = fds[1 + index];
        if ((entry.revents & POLLIN) == 0)
        {
            ++index;
            continue;
        }
        std::uint8_t chunk[4096];
        const ssize_t received = ::recv(static_cast<int>(clients[index]), chunk, sizeof(chunk), 0);
        if (received <= 0)
        {
            drainClient(index, clients, clientIds, clientBuffers, connectionEvents, receivedFrames, false);
            continue;
        }
        clientBuffers[index].insert(clientBuffers[index].end(), chunk, chunk + received);
        if (!extractFrames(clientBuffers[index], clientIds[index], receivedFrames))
        {
            drainClient(index, clients, clientIds, clientBuffers, connectionEvents, receivedFrames, true);
            continue;
        }
        ++index;
    }
    return true;
#endif
}

} // namespace engine::net
