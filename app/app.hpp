#ifndef ENGINE_APP_APP_HPP
#define ENGINE_APP_APP_HPP

#include "engine/kernel/engine_kernel.hpp"

namespace engine
{

// The composition root: the demo application the engine tree ships. The app
// embeds the kernel and owns the one concern the kernel deliberately does
// not - the frame loop. It is a consumer like any other: it acquires the
// capabilities the active mods have pulled in, and a capability no mod
// demanded is simply absent - the run degrades instead of assuming a backend
// exists (docs/architecture.md, "Capability demand").
//
// The kernel never names the app, and the app names no module: the process
// side of the assembly only drives the engine. It configures the kernel (the
// assembly layer, config/engine.toml), runs construct / start / the frame
// loop / stop, and turns every failure into the process exit code: 0 clean,
// 1 startup or frame failure, 3 observed graphics backend validation errors.
//
// main.cpp is the thin process entry point; everything a run does lives in
// this class, so another assembly could construct and drive it directly.
class App final
{
public:
    App() = default;
    ~App() = default;

    App(const App &)            = delete;
    App &operator=(const App &) = delete;

    // The whole run: construct, start, the frame loop, the opt-in latency
    // report, stop. Returns the process exit code.
    int run();

private:
    // The kernel the app embeds. Owned as a member so the app instance holds
    // the whole engine lifetime: the kernel destructor stops everything
    // exactly once, so App going out of scope tears the run down.
    EngineKernel kernel;
};

} // namespace engine

#endif // ENGINE_APP_APP_HPP
