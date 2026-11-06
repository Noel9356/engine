-- main_menu.lua -- the main_menu game mod, hosted by the script_runtime module.
--
-- Lifecycle: the kernel hands this package to script_runtime during the
-- construct phase; script_runtime runs this file in a sandboxed Lua runtime
-- (only math, string, table, and utf8 libraries are available) and forwards
-- engine events to the on_* callbacks defined below.
--
-- Behaviour: Escape toggles the menu window, which reports frame-rate
-- statistics gathered from the frame event stream. Window drawing goes through
-- the "menu_*" bridge functions that the demo application installs into this
-- runtime after start; they wrap the ui and render capabilities on the C++ side.

local MOD_ID = engine_mod_id or "main_menu"

-- One statistics sample per full second of frames, mirroring the behaviour of
-- the original native implementation: a ring buffer of per-second FPS samples
-- with min/max/frame-weighted-average recomputed after every completed bucket.
local BUCKET_DURATION_SECONDS = 1.0
local BUCKET_CAPACITY = 60

-- Window layout, in display points.
local WINDOW_WIDTH = 250.0
local WINDOW_HEIGHT = 112.0

local visible = false

-- Completed per-second samples: history[slot] = {fps, seconds, frames}. Slots
-- 1..sample_count always hold the valid samples; next_slot is the oldest slot
-- once the buffer has wrapped.
local history = {}
local sample_count = 0
local next_slot = 1

-- The bucket currently being accumulated.
local bucket_seconds = 0.0
local bucket_frames = 0

local minimum_fps = 0.0
local maximum_fps = 0.0
local average_fps = 0.0
local statistics_available = false

local function is_usable_number(value)
    return type(value) == "number" and value == value and value ~= math.huge and value ~= -math.huge
end

-- Called once by the host right after this file loaded successfully.
function on_construct()
    print(MOD_ID .. ": loaded (Lua, engine_mod_version " .. (engine_mod_version or "?") .. ")")
end

-- EngineStartupCompletedEvent.
function on_start()
    visible = false
end

-- WindowKeyEvent: the name is a stable lowercase key name from the publisher.
function on_key_pressed(key_name)
    if key_name == "escape" then
        visible = not visible
    end
end

-- EngineFrameEvent: accumulate frames into per-second buckets and refresh the
-- statistics whenever a bucket completes. The delta is the engine's (clamped)
-- frame delta, which is what the frame clock publishes.
function on_frame(delta_seconds)
    if not is_usable_number(delta_seconds) or delta_seconds <= 0.0 then
        return
    end

    -- Before any bucket completes, seed the statistics with the first frame so
    -- the window shows a plausible number immediately.
    if not statistics_available and sample_count == 0 and bucket_frames == 0 then
        local initial_fps = 1.0 / delta_seconds
        if is_usable_number(initial_fps) and initial_fps > 0.0 then
            minimum_fps = initial_fps
            maximum_fps = initial_fps
            average_fps = initial_fps
            statistics_available = true
        end
    end

    bucket_seconds = bucket_seconds + delta_seconds
    bucket_frames = bucket_frames + 1
    if bucket_seconds < BUCKET_DURATION_SECONDS then
        return
    end

    local fps = bucket_frames / bucket_seconds
    local seconds = bucket_seconds
    local frames = bucket_frames
    bucket_seconds = 0.0
    bucket_frames = 0
    if not (is_usable_number(fps) and fps > 0.0) then
        return
    end

    history[next_slot] = {fps = fps, seconds = seconds, frames = frames}
    next_slot = next_slot % BUCKET_CAPACITY + 1
    if sample_count < BUCKET_CAPACITY then
        sample_count = sample_count + 1
    end

    local total_seconds = 0.0
    local total_frames = 0.0
    minimum_fps = math.huge
    maximum_fps = 0.0
    for index = 1, sample_count do
        local sample = history[index]
        if sample ~= nil then
            if sample.fps < minimum_fps then
                minimum_fps = sample.fps
            end
            if sample.fps > maximum_fps then
                maximum_fps = sample.fps
            end
            total_seconds = total_seconds + sample.seconds
            total_frames = total_frames + sample.frames
        end
    end
    if total_seconds > 0.0 then
        average_fps = total_frames / total_seconds
        statistics_available = true
    else
        statistics_available = false
    end
end

-- Called by the application inside the render frame, between the scene draw
-- and frame submission. The frame geometry arrives in globals that the
-- application refreshes right before the call:
--   menu_display_width, menu_display_height,
--   menu_framebuffer_scale_x, menu_framebuffer_scale_y, menu_delta_seconds.
function on_draw()
    if not visible then
        return
    end

    local display_width = menu_display_width or 0.0
    local display_height = menu_display_height or 0.0
    local scale_x = menu_framebuffer_scale_x or 1.0
    local scale_y = menu_framebuffer_scale_y or 1.0
    local delta_seconds = menu_delta_seconds or (1.0 / 60.0)
    if not (is_usable_number(display_width) and display_width > 0.0)
        or not (is_usable_number(display_height) and display_height > 0.0)
        or not (is_usable_number(scale_x) and scale_x > 0.0)
        or not (is_usable_number(scale_y) and scale_y > 0.0) then
        return
    end

    if not menu_begin_frame(display_width, display_height, scale_x, scale_y, delta_seconds) then
        return
    end

    menu_set_next_window_position(display_width * 0.5, display_height * 0.5, 0.5, 0.5)
    menu_set_next_window_size(WINDOW_WIDTH, WINDOW_HEIGHT)
    if menu_begin_window("Performance") then
        local function fps_text(label, fps)
            if not is_usable_number(fps) or fps < 0.0 then
                fps = 0.0
            end
            if fps > 99999.9 then
                fps = 99999.9
            end
            menu_text(string.format("%s: %.1f FPS", label, fps))
        end

        if statistics_available then
            fps_text("Min", minimum_fps)
            fps_text("Max", maximum_fps)
            fps_text("Avg", average_fps)
        else
            menu_text("Min: -- FPS")
            menu_text("Max: -- FPS")
            menu_text("Avg: -- FPS")
        end
    end
    menu_end_window()

    menu_end_frame()
end

-- EngineShutdownStartedEvent: drop references to per-frame state. The bridge
-- functions are about to go away with the application's UI context.
function on_shutdown()
    visible = false
    statistics_available = false
end
