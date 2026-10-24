// The ImGui backend glue of the ui module: one-time Vulkan initialization
// against the renderer's opaque handle bundle (renderer_overlay_capability.hpp),
// the per-frame draw-data recording into the overlay seam's command buffer and
// the engine KeyCode -> ImGuiKey mapping table.
//
// Everything here speaks Vulkan C handles: the bundle carries opaque pointers
// documented as VkInstance/VkPhysicalDevice/VkDevice/VkQueue by the
// renderer.vulkan provider. The UI library's own vulkan backend does the real
// work; this file only marshals engine state into it.
#include "ui.hpp"

#include "imgui.h"
#include "imgui_impl_vulkan.h"

#include <stdexcept>
#include <vector>

namespace engine
{

namespace
{

// The UI library's Vulkan backend calls this on any VkResult it considers
// fatal. The engine's convention is precise diagnostics, not aborts, so the
// handler logs through the module's logger where possible and throws
// otherwise (backend init failure is decided once, see
// ensureBackendInitialized).
void vulkanResultGuard(
    VkResult result
)
{
    if (result == VK_SUCCESS)
    {
        return;
    }
    throw std::runtime_error("imgui vulkan backend: VkResult " + std::to_string(static_cast<int>(result)));
}

int imguiKeyFromKeyCode(
    KeyCode keyCode
)
{
    // The engine's KeyCode values mirror the platform scan codes of the
    // window backend; the mapping is explicit because the UI library's key
    // enum is its own vocabulary.
    switch (keyCode)
    {
    case KeyCode::Space:       return ImGuiKey_Space;
    case KeyCode::Apostrophe:  return ImGuiKey_Apostrophe;
    case KeyCode::Comma:       return ImGuiKey_Comma;
    case KeyCode::Minus:       return ImGuiKey_Minus;
    case KeyCode::Period:      return ImGuiKey_Period;
    case KeyCode::Slash:       return ImGuiKey_Slash;
    case KeyCode::Semicolon:   return ImGuiKey_Semicolon;
    case KeyCode::Equal:       return ImGuiKey_Equal;
    case KeyCode::LeftBracket: return ImGuiKey_LeftBracket;
    case KeyCode::Backslash:   return ImGuiKey_Backslash;
    case KeyCode::RightBracket:return ImGuiKey_RightBracket;
    case KeyCode::GraveAccent: return ImGuiKey_GraveAccent;
    case KeyCode::Escape:      return ImGuiKey_Escape;
    case KeyCode::Enter:       return ImGuiKey_Enter;
    case KeyCode::Tab:         return ImGuiKey_Tab;
    case KeyCode::Backspace:   return ImGuiKey_Backspace;
    case KeyCode::Insert:      return ImGuiKey_Insert;
    case KeyCode::Delete:      return ImGuiKey_Delete;
    case KeyCode::Right:       return ImGuiKey_RightArrow;
    case KeyCode::Left:        return ImGuiKey_LeftArrow;
    case KeyCode::Down:        return ImGuiKey_DownArrow;
    case KeyCode::Up:          return ImGuiKey_UpArrow;
    case KeyCode::PageUp:      return ImGuiKey_PageUp;
    case KeyCode::PageDown:    return ImGuiKey_PageDown;
    case KeyCode::Home:        return ImGuiKey_Home;
    case KeyCode::End:         return ImGuiKey_End;
    case KeyCode::Pause:       return ImGuiKey_Pause;
    case KeyCode::F1:          return ImGuiKey_F1;
    case KeyCode::F2:          return ImGuiKey_F2;
    case KeyCode::F3:          return ImGuiKey_F3;
    case KeyCode::F4:          return ImGuiKey_F4;
    case KeyCode::F5:          return ImGuiKey_F5;
    case KeyCode::F6:          return ImGuiKey_F6;
    case KeyCode::F7:          return ImGuiKey_F7;
    case KeyCode::F8:          return ImGuiKey_F8;
    case KeyCode::F9:          return ImGuiKey_F9;
    case KeyCode::F10:         return ImGuiKey_F10;
    case KeyCode::F11:         return ImGuiKey_F11;
    case KeyCode::F12:         return ImGuiKey_F12;
    default:                   return ImGuiKey_None;
    }
}

} // namespace

const std::vector<UiKeyMapping> &uiKeyMap()
{
    // Built once. Letters and digits map positionally (the engine's KeyCode
    // is ASCII there); the punctuation and control keys come from the switch
    // above. ImGuiKey_None rows would be pointless, so they are skipped.
    static const std::vector<UiKeyMapping> map = []()
    {
        std::vector<UiKeyMapping> entries;
        for (std::uint32_t keyCode = static_cast<std::uint32_t>(KeyCode::Space);
             keyCode <= static_cast<std::uint32_t>(KeyCode::Menu);
             ++keyCode)
        {
            const int imguiKey = imguiKeyFromKeyCode(static_cast<KeyCode>(keyCode));
            if (imguiKey != ImGuiKey_None)
            {
                entries.push_back(UiKeyMapping{imguiKey, keyCode});
            }
        }
        // The UI library wants letter and digit keys too; the engine's
        // KeyCode is ASCII in that range, and ImGuiKey_A..Z / 0..9 are
        // contiguous enum values.
        for (std::uint32_t keyCode = static_cast<std::uint32_t>(KeyCode::A);
             keyCode <= static_cast<std::uint32_t>(KeyCode::Z);
             ++keyCode)
        {
            entries.push_back(UiKeyMapping{ImGuiKey_A + static_cast<int>(keyCode - static_cast<std::uint32_t>(KeyCode::A)), keyCode});
        }
        for (std::uint32_t keyCode = static_cast<std::uint32_t>(KeyCode::Key0);
             keyCode <= static_cast<std::uint32_t>(KeyCode::Key9);
             ++keyCode)
        {
            entries.push_back(UiKeyMapping{ImGuiKey_0 + static_cast<int>(keyCode - static_cast<std::uint32_t>(KeyCode::Key0)), keyCode});
        }
        return entries;
    }();
    return map;
}

bool UiModule::ensureBackendInitialized()
{
    if (backendFailed)
    {
        return false;
    }
    if (backendInitialized)
    {
        return true;
    }

    // The frame renderer capability: resolved lazily (permission declared in
    // the manifest, no graph edge) for the camera pose display and the
    // screenshot button.
    if (rendererCapability == nullptr && boundContext != nullptr)
    {
        try
        {
            rendererCapability = &boundContext->acquireCapability<RendererCapability>();
        }
        catch (const std::exception &)
        {
            // Decided for the run: the overlay works without the frame API,
            // those two widgets just stay absent.
            rendererCapability = nullptr;
        }
    }

    // The renderer's backend bundle: the ONLY renderer internals this module
    // touches. A bundle without a device (a run that never created a
    // swapchain) is a warning, not a failure - the overlay can come back on
    // a later frame if the renderer recovered (e.g. after a swapchain
    // recreation).
    const OverlayBackendHandles *handles = overlayHost != nullptr ? overlayHost->overlayBackendHandles() : nullptr;
    if (handles == nullptr || handles->device == nullptr)
    {
        if (!warnedNoBackend && logger != nullptr)
        {
            warnedNoBackend = true;
            logger->logMessage(
                LogLevel::Warning,
                "ui module: the overlay cannot initialize yet - the renderer has no live device "
                "(a run without a window cannot host the overlay)"
            );
        }
        return false;
    }

    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr; // a debug overlay has no layout to persist

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.ApiVersion                   = VK_API_VERSION_1_3; // the renderer's instance API version
    initInfo.Instance                     = static_cast<VkInstance>(handles->instance);
    initInfo.PhysicalDevice               = static_cast<VkPhysicalDevice>(handles->physicalDevice);
    initInfo.Device                       = static_cast<VkDevice>(handles->device);
    initInfo.QueueFamily                  = handles->queueFamilyIndex;
    initInfo.Queue                        = static_cast<VkQueue>(handles->graphicsQueue);
    initInfo.MinImageCount                = handles->minImageCount;
    initInfo.ImageCount                   = handles->imageCount;
    initInfo.MSAASamples                  = VK_SAMPLE_COUNT_1_BIT;
    initInfo.DescriptorPoolSize           = 128; // the backend allocates its own pool
    initInfo.UseDynamicRendering          = true; // the renderer records without render passes
    initInfo.CheckVkResultFn              = &vulkanResultGuard;
    VkFormat colorAttachmentFormat                    = static_cast<VkFormat>(handles->colorFormat);
    VkPipelineRenderingCreateInfoKHR renderingInfo{};
    renderingInfo.sType                               = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    renderingInfo.colorAttachmentCount                = 1;
    renderingInfo.pColorAttachmentFormats             = &colorAttachmentFormat;
    initInfo.PipelineRenderingCreateInfo              = renderingInfo;

    try
    {
        if (!ImGui_ImplVulkan_Init(&initInfo))
        {
            throw std::runtime_error("imgui vulkan backend: initialization rejected the renderer's device");
        }
        // The backend uploads the font atlas during Init (it holds the
        // graphics queue from the bundle for exactly that).
    }
    catch (const std::exception &error)
    {
        backendFailed = true;
        ImGui::DestroyContext();
        if (logger != nullptr)
        {
            logger->logMessage(LogLevel::Error, "ui module: overlay backend initialization failed: " + std::string(error.what()));
        }
        return false;
    }

    // Registered ONCE: the recorder emits the draw data of every visible
    // frame into the overlay seam. Hiding the overlay leaves the recorder
    // registered - recordOverlay early-outs then.
    overlayHost->setOverlayRecorder(this);
    backendInitialized = true;
    if (logger != nullptr)
    {
        logger->logMessage(LogLevel::Information, "ui module: overlay backend initialized");
    }
    return true;
}

void UiModule::recordOverlay(
    const OverlayFrameContext &context
)
{
    // Called by the renderer between the composite pass and the present
    // transition. A hidden overlay leaves the last draw data in place; the
    // backend would happily re-emit stale vertices, so the visibility check
    // is what keeps a hidden overlay free.
    if (!backendInitialized || !overlayVisible_)
    {
        return;
    }
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), static_cast<VkCommandBuffer>(context.commandBuffer));
}

} // namespace engine
