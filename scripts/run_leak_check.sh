#!/usr/bin/env sh
# Light leak check: runs the engine against config/leak_test.toml (a sealed
# room with one emissive panel inside and a window opening) and requires the
# engine's own --leak-check evaluation to pass. Exit code 4 from the engine
# means a light leak was measured through the sealed wall, or the window view
# stayed dark (dead GI path); both fail this test.
#
# Vulkan discovery works like run_headless_smoke.sh: the caller exports the
# usual VK_* variables first (see scripts/setup_sandbox_headless_vulkan.sh).

set -u

ENGINE_BINARY="${1:?usage: run_leak_check.sh <engine binary>}"

ENGINE_BINARY="$(cd "$(dirname "${ENGINE_BINARY}")" && pwd)/$(basename "${ENGINE_BINARY}")"
RUNTIME_DIRECTORY="$(cd "$(dirname "${ENGINE_BINARY}")" && pwd)"
LEAK_LOG="${RUNTIME_DIRECTORY}/leak_run.log"

if [ ! -x "${ENGINE_BINARY}" ]; then
    echo "leak check: '${ENGINE_BINARY}' is not executable" >&2
    exit 1
fi

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
        echo "leak check: no DISPLAY and no Xvfb available; cannot run" >&2
        exit 1
    fi
    DISPLAY_NUMBER=99
    while [ -e "/tmp/.X11-unix/X${DISPLAY_NUMBER}" ]; do
        DISPLAY_NUMBER=$((DISPLAY_NUMBER + 1))
    done
    Xvfb ":${DISPLAY_NUMBER}" -screen 0 640x480x24 -nolisten tcp >/dev/null 2>&1 &
    XVFB_PID=$!
    DISPLAY=":${DISPLAY_NUMBER}"
    export DISPLAY
    sleep 1
fi

cd "${RUNTIME_DIRECTORY}"
"${ENGINE_BINARY}" --smoke-frames 8 --config config/leak_test.toml --leak-check > "${LEAK_LOG}" 2>&1
ENGINE_STATUS=$?
cat "${LEAK_LOG}"

case "${ENGINE_STATUS}" in
    0) echo "leak check: PASSED (no light leak, GI path alive)"; exit 0 ;;
    4) echo "leak check: FAILED - light leak measured or GI path dead" >&2; exit 1 ;;
    3) echo "leak check: FAILED - graphics backend validation errors" >&2; exit 1 ;;
    *) echo "leak check: FAILED - engine exited with status ${ENGINE_STATUS}" >&2; exit 1 ;;
esac
