// Context lifetime, frame lifecycle, draw-data extraction.
//
// Implements the create/destroy/beginFrame/endFrame/drawData/captureState
// surface and the internal helpers (findContext, makeCurrent, clearDrawData,
// copyDrawData) the other ui_* TUs rely on. Validates the public
// UiContextDescription and UiFrameDescription and resolves defaults through
// the module state so contexts stay self-contained once published.

#include "ui_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace engine
{
namespace ui
{
namespace backend
{

#if defined(ENGINE_UI_BACKEND_IMGUI)

UiContextRecord *findContext(UiModuleState &state, UiContextHandle handle) noexcept
{
    const auto iterator = state.contexts.find(handle);
    if (iterator == state.contexts.end() || iterator->second->ownerThread != std::this_thread::get_id())
    {
        return nullptr;
    }
    return iterator->second.get();
}

bool inFrame(const UiContextRecord *context) noexcept
{
    return context != nullptr && context->frameActive;
}

void makeCurrent(UiContextRecord &context) noexcept
{
    ImGui::SetCurrentContext(context.native);
}

void clearDrawData(UiContextRecord &context) noexcept
{
    context.vertices.clear();
    context.indices.clear();
    context.commands.clear();
    context.drawData = {};
}

bool copyDrawData(UiContextRecord &context) noexcept
{
    const ImDrawData *source = ImGui::GetDrawData();
    if (source == nullptr || !source->Valid)
    {
        clearDrawData(context);
        return true;
    }

    // First pass: count totals so the second pass can copy without
    // reallocation. Custom-callback commands are skipped in both passes
    // (only ImDrawCallback_ResetRenderState survives).
    std::uint64_t vertexCount  = 0;
    std::uint64_t indexCount   = 0;
    std::uint64_t commandCount = 0;
    for (int listIndex = 0; listIndex < source->CmdListsCount; ++listIndex)
    {
        const ImDrawList *list = source->CmdLists[listIndex];
        vertexCount += static_cast<std::uint64_t>(list->VtxBuffer.Size);
        indexCount  += static_cast<std::uint64_t>(list->IdxBuffer.Size);
        for (const ImDrawCmd &command : list->CmdBuffer)
        {
            if (command.UserCallback == nullptr || command.UserCallback == ImDrawCallback_ResetRenderState)
            {
                ++commandCount;
            }
        }
    }

    if (vertexCount > context.description.maximumVertices || indexCount > context.description.maximumIndices
        || commandCount > context.description.maximumCommands
        || vertexCount > std::numeric_limits<std::uint32_t>::max()
        || indexCount > std::numeric_limits<std::uint32_t>::max()
        || commandCount > std::numeric_limits<std::uint32_t>::max())
    {
        clearDrawData(context);
        return false;
    }

    context.vertices.clear();
    context.indices.clear();
    context.commands.clear();
    context.vertices.reserve(static_cast<std::size_t>(vertexCount));
    context.indices.reserve(static_cast<std::size_t>(indexCount));
    context.commands.reserve(static_cast<std::size_t>(commandCount));

    std::uint32_t vertexBase = 0;
    std::uint32_t indexBase  = 0;
    for (int listIndex = 0; listIndex < source->CmdListsCount; ++listIndex)
    {
        const ImDrawList *list = source->CmdLists[listIndex];
        for (const ImDrawVert &vertex : list->VtxBuffer)
        {
            UiVertex output{};
            output.position[0] = vertex.pos.x;
            output.position[1] = vertex.pos.y;
            output.uv[0]       = vertex.uv.x;
            output.uv[1]       = vertex.uv.y;
            output.color       = vertex.col;
            context.vertices.push_back(output);
        }
        for (const ImDrawIdx index : list->IdxBuffer)
        {
            context.indices.push_back(static_cast<std::uint32_t>(index));
        }
        for (const ImDrawCmd &command : list->CmdBuffer)
        {
            if (command.UserCallback != nullptr && command.UserCallback != ImDrawCallback_ResetRenderState)
            {
                continue;
            }
            UiDrawCommand output{};
            output.indexOffset      = indexBase + command.IdxOffset;
            output.vertexOffset     = vertexBase + command.VtxOffset;
            output.elementCount     = command.ElemCount;
            output.clipRectangle[0] = command.ClipRect.x;
            output.clipRectangle[1] = command.ClipRect.y;
            output.clipRectangle[2] = command.ClipRect.z;
            output.clipRectangle[3] = command.ClipRect.w;
            output.texture          = static_cast<UiTextureHandle>(command.TextureId);
            output.resetRenderState = command.UserCallback == ImDrawCallback_ResetRenderState;
            context.commands.push_back(output);
        }
        vertexBase += static_cast<std::uint32_t>(list->VtxBuffer.Size);
        indexBase  += static_cast<std::uint32_t>(list->IdxBuffer.Size);
    }

    context.drawData.vertices            = context.vertices.empty() ? nullptr : context.vertices.data();
    context.drawData.vertexCount         = static_cast<std::uint32_t>(context.vertices.size());
    context.drawData.indices             = context.indices.empty() ? nullptr : context.indices.data();
    context.drawData.indexCount          = static_cast<std::uint32_t>(context.indices.size());
    context.drawData.commands            = context.commands.empty() ? nullptr : context.commands.data();
    context.drawData.commandCount        = static_cast<std::uint32_t>(context.commands.size());
    context.drawData.displayPosition[0]  = source->DisplayPos.x;
    context.drawData.displayPosition[1]  = source->DisplayPos.y;
    context.drawData.displaySize[0]      = source->DisplaySize.x;
    context.drawData.displaySize[1]      = source->DisplaySize.y;
    context.drawData.framebufferScale[0] = source->FramebufferScale.x;
    context.drawData.framebufferScale[1] = source->FramebufferScale.y;
    return true;
}

bool resolveContextDescription(const UiContextDescription *description, UiContextDescription &resolved,
    const UiModuleState &state) noexcept
{
    if (description != nullptr)
    {
        resolved = *description;
    }
    else
    {
        resolved = UiContextDescription{};
        resolved.initialStyle    = state.defaultStyle;
        resolved.defaultFontSize = state.defaultFontSize;
    }

    if (!std::isfinite(resolved.globalScale) || resolved.globalScale <= 0.0f)
        return false;
    if (!std::isfinite(resolved.defaultFontSize) || resolved.defaultFontSize < 0.0f)
        return false;

    if (resolved.reserveVertices == 0)
        resolved.reserveVertices = 4096;
    if (resolved.reserveIndices == 0)
        resolved.reserveIndices = 8192;
    if (resolved.reserveCommands == 0)
        resolved.reserveCommands = 256;
    if (resolved.maximumVertices == 0)
        resolved.maximumVertices = 1'048'576;
    if (resolved.maximumIndices == 0)
        resolved.maximumIndices = 2'097'152;
    if (resolved.maximumCommands == 0)
        resolved.maximumCommands = 65'536;

    if (resolved.reserveVertices > resolved.maximumVertices || resolved.reserveIndices > resolved.maximumIndices
        || resolved.reserveCommands > resolved.maximumCommands)
    {
        return false;
    }
    return true;
}

bool validFrameDescription(const UiFrameDescription &description) noexcept
{
    return std::isfinite(description.displayWidth) && std::isfinite(description.displayHeight)
        && description.displayWidth > 0.0f && description.displayHeight > 0.0f
        && std::isfinite(description.framebufferScaleX) && std::isfinite(description.framebufferScaleY)
        && description.framebufferScaleX > 0.0f && description.framebufferScaleY > 0.0f
        && std::isfinite(description.deltaSeconds) && description.deltaSeconds > 0.0f;
}

// Resolves the ini filename policy for a freshly-created context. Returns
// true when ini persistence is enabled; the resolved path (if any) is stored
// into record->iniFilenameStorage and io.IniFilename is pointed at it. When
// false, io.IniFilename is set to nullptr so ImGui does not touch the file
// system in its frame path.
static bool resolveIniPolicy(UiContextRecord &record, const UiModuleState &state) noexcept
{
    ImGuiIO &io = ImGui::GetIO();
    if (!state.saveWindowLayout)
    {
        io.IniFilename = nullptr;
        return false;
    }

    const char *contextPath = record.description.iniFilename;
    if (contextPath != nullptr && contextPath[0] == '\0')
    {
        // Explicit empty string from the caller: disable even if the
        // module-level default is enabled.
        io.IniFilename = nullptr;
        return false;
    }

    if (contextPath != nullptr)
    {
        record.iniFilenameStorage = contextPath;
    }
    else if (!state.iniFilename.empty())
    {
        record.iniFilenameStorage = state.iniFilename;
    }
    else
    {
        // save_window_layout=true and no explicit path: let ImGui pick its
        // default ("imgui.ini" next to the executable).
        io.IniFilename = nullptr;
        // Override after the fact: ImGui's default uses a static buffer that
        // we cannot safely capture, so we fall back to a fixed name stored
        // in the record to keep ownership clear.
        record.iniFilenameStorage = "imgui.ini";
    }

    io.IniFilename = record.iniFilenameStorage.c_str();
    return true;
}

// Creates a context. A null description uses the defaults; zero-valued
// reserve/maximum fields fall back to the same defaults; reserve above
// maximum is rejected. The first context fixes the owner thread for all.
UiContextHandle createContext(UiModuleState &state, const UiContextDescription *description, UiError *error)
{
    setError(error, UiError::None);

    const std::thread::id currentThread = std::this_thread::get_id();
    if (state.hasOwnerThread && state.ownerThread != currentThread)
    {
        setError(error, UiError::WrongThread);
        return invalidUiContext;
    }

    UiContextDescription resolved{};
    if (!resolveContextDescription(description, resolved, state))
    {
        setError(error, UiError::InvalidDescription);
        return invalidUiContext;
    }

    auto record         = std::make_unique<UiContextRecord>();
    record->ownerThread = currentThread;
    record->description = resolved;
    record->vertices.reserve(resolved.reserveVertices);
    record->indices.reserve(resolved.reserveIndices);
    record->commands.reserve(resolved.reserveCommands);
    record->native = ImGui::CreateContext();
    if (record->native == nullptr)
    {
        setError(error, UiError::CreationFailed);
        return invalidUiContext;
    }
    makeCurrent(*record);
    IMGUI_CHECKVERSION();
    applyStyle(resolved.initialStyle);
    ImGui::GetStyle().ScaleAllSizes(resolved.globalScale);
    ImGuiIO &io        = ImGui::GetIO();
    io.FontGlobalScale = resolved.globalScale;
    if (resolved.keyboardNavigation)
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    if (resolved.gamepadNavigation)
    {
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
        io.BackendFlags |= ImGuiBackendFlags_HasGamepad;
    }
    io.BackendPlatformName = "engine_ui_input";
    io.BackendRendererName = "engine_ui_draw_data";
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
    resolveIniPolicy(*record, state);
    if (resolved.defaultFontSize > 0.0f)
    {
        ImFontConfig fontConfig{};
        fontConfig.SizePixels = resolved.defaultFontSize;
        record->fonts.emplace(defaultUiFont, io.Fonts->AddFontDefault(&fontConfig));
    }
    else
    {
        record->fonts.emplace(defaultUiFont, io.Fonts->AddFontDefault());
    }

    // Handles start at 1; skip 0 (invalidUiContext) if the counter wraps.
    const UiContextHandle handle = state.nextContext++;
    if (handle == invalidUiContext)
    {
        setError(error, UiError::CreationFailed);
        return invalidUiContext;
    }
    try
    {
        const auto [iterator, inserted] = state.contexts.emplace(handle, std::move(record));
        (void)iterator;
        if (!inserted)
        {
            setError(error, UiError::CreationFailed);
            return invalidUiContext;
        }
    }
    catch (...)
    {
        setError(error, UiError::CreationFailed);
        return invalidUiContext;
    }
    state.ownerThread    = currentThread;
    state.hasOwnerThread = true;
    return handle;
}

void destroyContext(UiModuleState &state, UiContextHandle handle)
{
    const auto iterator = state.contexts.find(handle);
    if (iterator == state.contexts.end() || iterator->second->ownerThread != std::this_thread::get_id())
        return;
    state.contexts.erase(iterator);
    if (state.contexts.empty())
    {
        state.ownerThread    = {};
        state.hasOwnerThread = false;
    }
}

// Starts a frame: validates the description, clamps deltaSeconds to
// [1 ms, 0.25 s] for ImGui's timers, clears the previous frame's draw data,
// and calls ImGui::NewFrame().
bool beginFrame(UiModuleState &state, UiContextHandle handle, const UiFrameDescription *description, UiError *error)
{
    setError(error, UiError::None);
    if (description == nullptr || !validFrameDescription(*description))
    {
        setError(error, UiError::InvalidDescription);
        return false;
    }
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr)
    {
        setError(error, UiError::InvalidHandle);
        return false;
    }
    if (record->frameActive)
    {
        setError(error, UiError::FrameAlreadyActive);
        return false;
    }
    makeCurrent(*record);
    ImGuiIO &io                = ImGui::GetIO();
    io.DisplaySize             = ImVec2(description->displayWidth, description->displayHeight);
    io.DisplayFramebufferScale = ImVec2(description->framebufferScaleX, description->framebufferScaleY);
    io.DeltaTime               = std::clamp(description->deltaSeconds, 1.0f / 1000.0f, 0.25f);
    clearDrawData(*record);
    ImGui::NewFrame();
    record->frameActive = true;
    return true;
}

// Ends the frame: ImGui::Render() produces the draw lists, which are copied
// into the ABI buffers. CapacityExceeded (with cleared draw data) is reported
// when a configured maximum was exceeded.
bool endFrame(UiModuleState &state, UiContextHandle handle, UiError *error)
{
    setError(error, UiError::None);
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr)
    {
        setError(error, UiError::InvalidHandle);
        return false;
    }
    if (!record->frameActive)
    {
        setError(error, UiError::FrameNotActive);
        return false;
    }
    makeCurrent(*record);
    ImGui::Render();
    record->frameActive = false;
    if (!copyDrawData(*record))
    {
        setError(error, UiError::CapacityExceeded);
        return false;
    }
    return true;
}

// Draw data of the last ended frame; the pointer lives until the next
// beginFrame on the same context.
const UiDrawData *drawData(UiModuleState &state, UiContextHandle handle)
{
    UiContextRecord *record = findContext(state, handle);
    return record != nullptr ? &record->drawData : nullptr;
}

bool captureState(UiModuleState &state, UiContextHandle handle, UiCaptureState *outState)
{
    if (outState == nullptr)
        return false;
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr)
        return false;
    makeCurrent(*record);
    const ImGuiIO &io   = ImGui::GetIO();
    outState->mouse     = io.WantCaptureMouse;
    outState->keyboard  = io.WantCaptureKeyboard;
    outState->textInput = io.WantTextInput;
    return true;
}

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine
