#include "net.hpp"

#include <algorithm>
#include <stdexcept>

namespace engine
{

namespace
{

// The frame header: two uint32 LE lengths. Encoding lives here once; the
// socket layer assembles and parses the same layout.
void appendUint32(
    std::vector<std::uint8_t> &buffer, std::uint32_t value
)
{
    buffer.push_back(static_cast<std::uint8_t>(value & 0xFF));
    buffer.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    buffer.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    buffer.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

} // namespace

NetModule::NetModule()
{
    componentManifest.name                 = "net";
    componentManifest.kind                 = ComponentKind::Module;
    componentManifest.providedCapabilities = {"net.tcp"};
    componentManifest.requiredCapabilities = {"logging"};
    componentManifest.requiredPermissions  = {"capability.logging"};
    componentManifest.knownConfigKeys      = {"net.enabled", "net.port", "net.bind_address"};
}

NetModule::~NetModule()
{
    stop();
}

void NetModule::construct(
    ComponentContext &context
)
{
    boundContext = &context;
    logger       = &context.acquireCapability<LoggerCapability>();

    bindAddress = context.configurationString("bind_address").value_or(std::string("loopback"));
    port        = static_cast<std::uint16_t>(
        std::clamp<std::int64_t>(context.configurationInteger("port").value_or(8765), 1024, 65535)
    );

    EventBus                                &bus            = context.eventBus();
    frameBeginChannel                                       = &bus.channel<FrameBeginEvent>();
    clientConnectedChannel                                  = &bus.channel<NetClientConnectedEvent>();
    messageReceivedChannel                                  = &bus.channel<NetMessageReceivedEvent>();
    clientDisconnectedChannel                               = &bus.channel<NetClientDisconnectedEvent>();
    frameBeginSubscription                                  = EventSubscription(
        &bus,
        typeid(FrameBeginEvent),
        frameBeginChannel->subscribe<NetModule, &NetModule::handleFrameBegin>(this)
    );

    // Opt-in listener: [net].enabled = true starts it at start(), everything
    // else waits for an explicit startListener call.
    if (context.configurationBoolean("enabled").value_or(false) && logger != nullptr)
    {
        std::string errorReport;
        if (!startListener(errorReport))
        {
            logger->logMessage(LogLevel::Warning, "net module: [net].enabled=true but the listener failed: " + errorReport);
        }
    }
}

void NetModule::start() {}

void NetModule::stop()
{
    stopListener();
    // EventSubscription is an RAII guard: release() unsubscribes and
    // disarms, so the destructor does not double-unsubscribe.
    frameBeginSubscription.release();
    boundContext = nullptr;
}

void *NetModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "net.tcp")
    {
        return static_cast<NetCapability *>(this);
    }
    return nullptr;
}

bool NetModule::startListener(
    std::string &errorReport
)
{
    if (listener != net::invalidSocket)
    {
        return true; // idempotent: an already listening transport keeps its socket
    }
    std::string socketError;
    if (!net::initializeSockets(socketError))
    {
        errorReport = "net module: socket initialization failed: " + socketError;
        return false;
    }
    listener = net::openListener(bindAddress, port, 8, socketError);
    if (listener == net::invalidSocket)
    {
        errorReport = "net module: " + socketError;
        return false;
    }
    if (logger != nullptr)
    {
        const std::string shownAddress = bindAddress == "loopback" ? std::string("127.0.0.1") : bindAddress;
        logger->logMessage(
            LogLevel::Information,
            "net module: listening on " + shownAddress + ":" + std::to_string(port) + " (tcp, length-prefixed frames)"
        );
    }
    return true;
}

void NetModule::stopListener()
{
    if (listener == net::invalidSocket)
    {
        return;
    }
    net::closeSocket(listener);
    listener = net::invalidSocket;
    // Client connections close too; the pump surfaces their disconnection
    // events on the next frame (never from inside this call - an event
    // publish mid-stop would run handlers of a tearing-down engine).
    for (const net::SocketHandle client : clients)
    {
        net::closeSocket(client);
    }
    clients.clear();
    clientIds.clear();
    clientBuffers.clear();
    if (logger != nullptr)
    {
        logger->logMessage(LogLevel::Information, "net module: listener stopped");
    }
}

std::uint32_t NetModule::connectedClientCount() const
{
    return static_cast<std::uint32_t>(clients.size());
}

bool NetModule::broadcast(
    const std::string &topic, const std::string &payload, std::string &errorReport
)
{
    return sendTo(0, topic, payload, errorReport); // 0 = every client
}

bool NetModule::sendTo(
    std::uint32_t clientId, const std::string &topic, const std::string &payload, std::string &errorReport
)
{
    if (listener == net::invalidSocket)
    {
        errorReport = "net module: send refused - the transport never opened a listener ([net].enabled "
                      "is false and no startListener call ran)";
        return false;
    }
    if (clientId != 0 && std::find(clientIds.begin(), clientIds.end(), clientId) == clientIds.end())
    {
        errorReport = "net module: send refused - client " + std::to_string(clientId)
                    + " is not connected (ids are never reused; a disconnected client stays gone)";
        return false;
    }
    std::lock_guard<std::mutex> lock(outboundMutex);
    outboundQueue.push_back(OutboundFrame{clientId, topic, payload});
    // Frames flush inside the same frame's pump (handleFrameBegin below),
    // which is later this frame when called before it, or next frame when
    // called after - either way the ordering guarantee holds: one frame's
    // sends never interleave with another's.
    return true;
}

void NetModule::handleFrameBegin(
    const FrameBeginEvent &event
)
{
    (void)event;
    if (listener == net::invalidSocket)
    {
        return; // inert transport: no socket, no work, no cost beyond the branch
    }

    // One poll pass: accepts, receives, disconnects.
    std::vector<net::ConnectionEvent> connectionEvents;
    std::vector<net::ReceivedFrame>   receivedFrames;
    std::string                       errorReport;
    if (!net::pollSockets(
            listener, clients, clientIds, clientBuffers, nextClientId, connectionEvents, receivedFrames, errorReport
        ))
    {
        if (logger != nullptr)
        {
            logger->logMessage(LogLevel::Error, "net module: socket poll failed: " + errorReport);
        }
    }

    // Outbound first: this frame's sends leave before this frame's receives
    // are published, so a request/response client sees stable ordering.
    {
        std::lock_guard<std::mutex> lock(outboundMutex);
        for (const OutboundFrame &frame : outboundQueue)
        {
            std::vector<std::uint8_t> bytes;
            bytes.reserve(8 + frame.topic.size() + frame.payload.size());
            appendUint32(bytes, static_cast<std::uint32_t>(frame.topic.size()));
            bytes.insert(bytes.end(), frame.topic.begin(), frame.topic.end());
            appendUint32(bytes, static_cast<std::uint32_t>(frame.payload.size()));
            bytes.insert(bytes.end(), frame.payload.begin(), frame.payload.end());
            for (std::size_t index = 0; index < clients.size(); ++index)
            {
                if (frame.clientId != 0 && clientIds[index] != frame.clientId)
                {
                    continue;
                }
                std::string sendError;
                if (!net::sendBuffer(clients[index], bytes.data(), bytes.size(), sendError))
                {
                    if (logger != nullptr)
                    {
                        logger->logMessage(
                            LogLevel::Warning,
                            "net module: send to client " + std::to_string(clientIds[index]) + " failed ("
                                + sendError + "); the connection drops"
                        );
                    }
                }
            }
        }
        outboundQueue.clear();
    }

    // Connection events, then messages - a consumer reacting to a connect
    // sees it before the first message of that same client.
    for (const net::ConnectionEvent &connection : connectionEvents)
    {
        if (connection.kind == net::ConnectionEvent::Kind::Connected)
        {
            if (logger != nullptr)
            {
                logger->logMessage(
                    LogLevel::Information, "net module: client " + std::to_string(connection.clientId) + " connected"
                );
            }
            clientConnectedChannel->publish(NetClientConnectedEvent{connection.clientId});
        }
        else
        {
            if (logger != nullptr)
            {
                logger->logMessage(
                    LogLevel::Information,
                    "net module: client " + std::to_string(connection.clientId) + " disconnected"
                );
            }
            clientDisconnectedChannel->publish(NetClientDisconnectedEvent{connection.clientId});
        }
    }
    for (const net::ReceivedFrame &frame : receivedFrames)
    {
        messageReceivedChannel->publish(NetMessageReceivedEvent{frame.clientId, frame.topic, frame.payload});
    }
}

void NetModule::pump()
{
    // The module pumps from its own FrameBeginEvent handler every frame;
    // this entry point runs the same drain for callers that want to force a
    // pass (a test harness, a headless run). Drains are idempotent: bytes
    // consumed into frames leave the buffers, so a second pass in the same
    // frame publishes nothing twice.
    handleFrameBegin(FrameBeginEvent{});
}

} // namespace engine
