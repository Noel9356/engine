#!/usr/bin/env sh
# Headless smoke run: boots the assembled runtime (logging, window, input,
# renderer, mods), renders a fixed number of frames and exits cleanly. The run
# fails when the engine crashes, throws or its graphics backend reports
# validation errors, which turns runtime regressions that unit tests cannot
# see (broken pipelines, disabled features, wrong image layouts) into a
# failing test.
#
# Arguments:
#   $1  path to the engine executable; its parent directory must contain the
#       runtime layout (config/, shaders/, mods/)
#   $2  number of frames to render (default: 30)
#   $3  configuration file relative to the runtime directory
#       (default: config/engine.toml; the headless smoke test passes
#       config/smoke.toml for a fast software-rasterizer run)
#
# Display handling: with DISPLAY set the engine runs on that display; without
# one the script starts its own Xvfb and stops it afterwards, so the same
# script covers developer machines and headless containers.
#
# Vulkan discovery: on machines with a system GPU driver nothing needs to be
# set; in a headless container the caller exports the usual VK_* variables
# first (see scripts/setup_sandbox_headless_vulkan.sh for a lavapipe setup).

set -u

ENGINE_BINARY="${1:?usage: run_headless_smoke.sh <engine binary> [frames] [config]}"
SMOKE_FRAMES="${2:-30}"
SMOKE_CONFIG="${3:-config/engine.toml}"

# Absolutize before any directory change: the run itself happens inside the
# runtime directory, so a relative binary path would break there.
ENGINE_BINARY="$(cd "$(dirname "${ENGINE_BINARY}")" && pwd)/$(basename "${ENGINE_BINARY}")"

RUNTIME_DIRECTORY="$(cd "$(dirname "${ENGINE_BINARY}")" && pwd)"
SMOKE_LOG="${RUNTIME_DIRECTORY}/smoke_run.log"

if [ ! -x "${ENGINE_BINARY}" ]; then
    echo "headless smoke: '${ENGINE_BINARY}' is not executable" >&2
    exit 1
fi

# Sandbox convenience: pick up lavapipe plus the validation layer when the
# provisioning script has produced its environment file.
if [ -f /home/z/engine-local/headless-vulkan-env.sh ]; then
    . /home/z/engine-local/headless-vulkan-env.sh
fi

XVFB_PID=0
cleanup()
{
    if [ "${XVFB_PID}" -ne 0 ]; then
        kill "${XVFB_PID}" 2>/dev/null || true
        wait "${XVFB_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [ -z "${DISPLAY:-}" ]; then
    if ! command -v Xvfb >/dev/null 2>&1; then
        echo "headless smoke: no DISPLAY and no Xvfb available; cannot run" >&2
        exit 1
    fi
    DISPLAY_NUMBER=99
    while [ -e "/tmp/.X11-unix/X${DISPLAY_NUMBER}" ]; do
        DISPLAY_NUMBER=$((DISPLAY_NUMBER + 1))
    done
    Xvfb ":${DISPLAY_NUMBER}" -screen 0 1280x720x24 -nolisten tcp >/dev/null 2>&1 &
    XVFB_PID=$!
    DISPLAY=":${DISPLAY_NUMBER}"
    export DISPLAY
    # Give the server a moment to accept connections.
    sleep 1
fi

cd "${RUNTIME_DIRECTORY}"
"${ENGINE_BINARY}" --smoke-frames "${SMOKE_FRAMES}" --config "${SMOKE_CONFIG}" > "${SMOKE_LOG}" 2>&1
ENGINE_STATUS=$?
cat "${SMOKE_LOG}"

if [ "${ENGINE_STATUS}" -ne 0 ]; then
    if [ "${ENGINE_STATUS}" = "3" ]; then
        echo "headless smoke: FAILED - graphics backend reported validation errors" >&2
    else
        echo "headless smoke: FAILED - engine exited with status ${ENGINE_STATUS}" >&2
    fi
    exit 1
fi

# Belt and braces: any backend error line in the log fails the run even if a
# future code path forgets to surface it through the exit code.
if grep -q "ERROR] vulkan validation" "${SMOKE_LOG}"; then
    echo "headless smoke: FAILED - validation errors in the engine log" >&2
    exit 1
fi

echo "headless smoke: PASSED (${SMOKE_FRAMES} frames, no validation errors)"
exit 0
