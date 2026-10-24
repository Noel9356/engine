# net_tcp module - the TCP transport (telemetry and control)

## What this module provides

Capability `net` (implementation-bound member `net.tcp`, published in `net_capability.hpp`): a TCP transport for text messages between the engine and tooling - test harnesses, editors, telemetry dashboards. The engine runs the listener; clients connect, send topic/payload frames and receive the same. Received traffic surfaces on the event bus, so any module or native mod consumes it like every other event.

This is a TELEMETRY AND CONTROL transport, and the README says so plainly: no delta compression, no prediction, no clock synchronization. Frames are small text messages; anything heavier deserves a dedicated provider behind the same family name.

## Wire format

One frame is length-prefixed and binary safe, trivially parseable by any language:

```
[uint32 LE topic length][topic bytes][uint32 LE payload length][payload bytes]
```

Topics cap at 256 bytes, payloads at 1 MiB. A stream that violates the caps is corrupt: that client is disconnected with a precise log instead of being trusted. Both directions use the same format.

## The listener is opt-in

```toml
[net]
enabled = false          # a loaded module must not open sockets nobody asked for
port = 8765              # clamped to 1024..65535
bind_address = "loopback"  # "loopback" (default), "0.0.0.0", or any IPv4 literal
```

The default binds loopback because a debug transport must not expose an engine to the network by accident. `enabled = true` starts the listener at startup; `startListener()` (native capability or a future script binding) starts it any time. `stopListener()` closes everything; the disconnection events surface through the next pump, never from inside the call.

## Main-thread events, one poll per frame

There is no listener thread. Sockets are polled non-blocking once per frame from the module's FrameBeginEvent handler, so connects, messages and disconnects surface on the main thread, in frame order, through the bus - the same single-thread guarantee every other event consumer relies on. Between two pumps a frame's latency is the transport's worst case; that is the budget a debug transport lives in, and the code that buys the simplicity is a fraction of a threaded design.

Events published (all on the main thread, connection events before the messages of the same client):

| Event | Meaning |
|---|---|
| `NetClientConnectedEvent` | a client completed the TCP handshake; `clientId` is its address for `sendTo` |
| `NetMessageReceivedEvent` | a frame arrived: `clientId`, `topic`, `payload` (raw UTF-8; interpretation is the consumer's) |
| `NetClientDisconnectedEvent` | remote close, error, or listener shutdown; ids are NEVER reused within a run |

Sends queue through `broadcast` / `sendTo` (client id 0 means everyone) and flush inside the same frame's pump, so one frame's sends never interleave with another's. A dead client discovered mid-send is reported through its disconnection event, not a send error; `sendTo` an unknown id returns false with a precise message.

## Native access

The transport's consumers are native: C++ mods and modules include `net_capability.hpp` and acquire the `net` family (`startListener` / `stopListener` / `broadcast` / `sendTo`); received traffic reaches every consumer through the bus events above. The Lua bridge exposes no net functions today; script-facing exposure would follow the bridge's lazy, permission-gated pattern when a use case arrives.

## Permissions

The module declares `capability.logging` only. It reads no filesystem and demands no other capability: the transport is self-contained, and its configuration lives in the `[net]` section of the assembly.

## Files

- `net_capability.hpp` - the published capability surface
- `net.hpp` / `net.cpp` - the module, the per-frame pump, the outbound queue
- `net_sockets.hpp` / `net_sockets.cpp` - the POSIX/WinSock shims and the frame assembler
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_NET_TCP`)
