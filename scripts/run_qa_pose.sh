#!/usr/bin/env sh
# QA harness: render the demo scene from one configured camera pose and save
# the capture under qa/ with a given name. Usage: run_qa_pose.sh <outname> <config>
ENGINE_DIR="/home/z/my-project/engine/build/runtime"
OUT_DIR="/home/z/my-project/qa"
NAME="$1"; CFG="$2"
mkdir -p "$OUT_DIR" screenshots
# Install the QA capture mod (kept out of the shipped mods/ so the demo does
# not auto-capture on every player run).
mkdir -p "${ENGINE_DIR}/mods"
cp -r /home/z/my-project/engine/scripts/qa/qa_capture "${ENGINE_DIR}/mods/" 2>/dev/null
cd "$ENGINE_DIR"
rm -f mods/qa_capture/script.lua.lastrun
DISPLAY=${QA_DISPLAY:-:96} "$ENGINE_DIR/engine" --smoke-frames 6 --config "$CFG" > /tmp/qa_run.log 2>&1
STATUS=$?
# capture lands in screenshots/ relative to cwd; move it
latest=$(ls -t screenshots/*.png 2>/dev/null | head -1)
if [ -n "$latest" ]; then mv "$latest" "$OUT_DIR/$NAME.png"; echo "saved $OUT_DIR/$NAME.png"; else echo "NO CAPTURE (exit=$STATUS)"; tail -20 /tmp/qa_run.log; fi
exit $STATUS
