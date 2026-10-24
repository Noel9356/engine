#ifndef ENGINE_MODULES_UI_UI_CAPABILITY_HPP
#define ENGINE_MODULES_UI_UI_CAPABILITY_HPP

// The published capability surface of the ui module. Every consumer - other
// modules, native mods, the app - includes THIS header, from the module that
// provides the capability. Like every published capability header this stays
// implementation-, platform- and vendor-agnostic.
//
// The capability is the debug overlay: an immediate mode UI drawn inside the
// frame through the renderer's overlay seam ("renderer.overlay"). The module
// owns the widget tree (frame statistics, scene census, camera pose, capture
// and persistence buttons); consumers of THIS capability only see the modal
// state - whether the overlay is on screen and whether it currently claims
// the pointer.
//
// Modal claim semantics: while the overlay is visible it owns the cursor.
// Scripts and the free-fly camera must not fight a debug window for pointer
// motion, so input consumers gate themselves on wantsMouse() (the scripting
// bridge does exactly that before handing deltas to a script); the module
// additionally drains the cursor and wheel accumulators every frame it is
// visible, so even a consumer that ignores this contract reads a settled
// accumulator.

#include <cstdint>

namespace engine
{

class UiCapability
{
public:
    // The implementation-agnostic family name of this capability. Consumers
    // acquire through it; the module's plugin descriptor carries the
    // implementation-bound member name ("ui.imgui").
    static constexpr const char *capabilityName = "ui";

    virtual ~UiCapability() = default;

    // Whether the overlay is currently on screen.
    virtual bool overlayVisible() const = 0;

    // Shows or hides the overlay. Hiding also releases the pointer claim and
    // restores whatever cursor capture state the run had before.
    virtual void setOverlayVisible(bool visible) = 0;

    // Flips the overlay. This is what the toggle key (config [ui].key,
    // default F1) and scripts use.
    virtual void toggleOverlay() = 0;

    // Whether the overlay claims the pointer this frame. True while the
    // overlay is visible (the modal rule above); false otherwise. Input
    // consumers consult this BEFORE consuming cursor deltas.
    virtual bool wantsMouse() const = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_UI_UI_CAPABILITY_HPP
