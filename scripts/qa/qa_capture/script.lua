-- qa_capture: queues a capture at load; the screenshot module hands it to the
-- renderer on the next FrameBegin, so the first rendered frame is captured.
engine.log_information("qa_capture loaded")
engine.request_screenshot()
