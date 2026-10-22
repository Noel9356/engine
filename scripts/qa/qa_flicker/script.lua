-- qa_flicker: queue one capture per frame. The screenshot module consumes the
-- queue on the FrameBegin AFTER the one this handler sees, so captures land on
-- frames 2..N for a run of N frames - consecutive and aligned, which is what
-- the diff needs.
engine.log_information("qa_flicker loaded")
engine.subscribe_frame_begin(function(frame)
    engine.request_screenshot()
end)
