#!/usr/bin/env bash
# Install this repo's requirements.txt - the single list of Python (PyPI) packages,
# used both for the Dev PC venv and inside the Docker image.
#
# Usage:
#   setup/install_requirements.sh --venv <dir> [--pytorch]   # Dev PC: create the venv if missing, install into it
#   setup/install_requirements.sh --docker [--pytorch]        # Dockerfile: system Python of the image
#
# Tags in requirements.txt (in the comment after a package):
#   [dev-pc]              only in the Dev PC venv (AirSim, plotting, keyboard...), never in the image
#   [pytorch]             only with --pytorch. torch/torchvision are skipped when the
#                         environment already has PyTorch (left as it is)
#   [no-build-isolation]  installed last with --no-build-isolation (airsim needs numpy and
#                         msgpack-rpc-python already installed to build)
#   [no-deps]             installed last without its dependencies (ultralytics: it would
#                         add opencv-python on top of opencv-contrib-python); list them instead
# Every pin of the file is also passed as a constraint to every pip call, so no
# dependency can move a pinned package (e.g. pull NumPy 2).
# --docker also swaps opencv-*python for its -headless wheel (no GUI libraries).
# ROS 2 packages (rclpy, sensor_msgs, ...) come from ROS 2 Jazzy (apt), never from here.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

VENV=""
DOCKER=0
PYTORCH=0
while [ $# -gt 0 ]; do
    case "$1" in
        --venv) VENV="$2"; shift 2 ;;
        --docker) DOCKER=1; shift ;;
        --pytorch) PYTORCH=1; shift ;;
        *) echo "usage: $0 (--venv <dir> | --docker) [--pytorch]" >&2; exit 2 ;;
    esac
done
if [ -z "$VENV" ] && [ "$DOCKER" = "0" ]; then
    echo "usage: $0 (--venv <dir> | --docker) [--pytorch]" >&2; exit 2
fi

if [ -n "$VENV" ]; then
    # --system-site-packages: the venv sees ROS 2's Python packages and the apt ones.
    [ -x "$VENV/bin/python" ] || python3 -m venv --system-site-packages "$VENV"
    PY="$VENV/bin/python"
    PIP=("$PY" -m pip install)
else
    PY=python3
    PIP=("$PY" -m pip install --break-system-packages --no-cache-dir)
fi

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
# requirement lines only (no blank / comment-only lines), tags kept for filtering
grep -vE '^[[:space:]]*(#|$)' requirements.txt > "$TMP/all"
pick()   { grep -F "$1" "$TMP/all" || true; }
strip()  { sed -E 's/[[:space:]]*#.*$//'; }

strip < "$TMP/all" > "$TMP/constraints.txt"
grep -vF '[pytorch]' "$TMP/all" | grep -vF '[no-build-isolation]' | grep -vF '[no-deps]' > "$TMP/main" || true
grep -F '[pytorch]' "$TMP/all" | grep -vF '[no-deps]' > "$TMP/torch" || true
pick '[no-build-isolation]' > "$TMP/late"
pick '[no-deps]' > "$TMP/nodeps"
if [ "$DOCKER" = "1" ]; then
    for f in main torch late nodeps; do
        grep -vF '[dev-pc]' "$TMP/$f" | sed -E 's/^(opencv(-contrib)?-python)==/\1-headless==/' > "$TMP/$f.d" || true
        mv "$TMP/$f.d" "$TMP/$f"
    done
fi
if [ "$PYTORCH" = "1" ]; then
    if "$PY" -c "import torch" 2>/dev/null; then
        echo "[install_requirements] PyTorch $("$PY" -c 'import torch; print(torch.__version__)') is already installed - left as it is."
        for f in torch constraints.txt; do
            grep -vE '^(torch|torchvision|torchaudio)([=<>!~ ]|$)' "$TMP/$f" > "$TMP/$f.k" || true
            mv "$TMP/$f.k" "$TMP/$f"
        done
    fi
else
    : > "$TMP/torch"
    grep -vF '[pytorch]' "$TMP/nodeps" > "$TMP/nodeps.k" || true
    mv "$TMP/nodeps.k" "$TMP/nodeps"
fi

for f in main torch; do
    if [ -s "$TMP/$f" ]; then
        strip < "$TMP/$f" > "$TMP/$f.txt"
        echo "[install_requirements] installing: $(tr '\n' ' ' < "$TMP/$f.txt")"
        "${PIP[@]}" -c "$TMP/constraints.txt" -r "$TMP/$f.txt"
    fi
done
if [ -s "$TMP/late" ]; then
    strip < "$TMP/late" > "$TMP/late.txt"
    echo "[install_requirements] installing (--no-build-isolation): $(tr '\n' ' ' < "$TMP/late.txt")"
    "${PIP[@]}" --no-build-isolation -c "$TMP/constraints.txt" -r "$TMP/late.txt"
fi
if [ -s "$TMP/nodeps" ]; then
    strip < "$TMP/nodeps" > "$TMP/nodeps.txt"
    echo "[install_requirements] installing (--no-deps): $(tr '\n' ' ' < "$TMP/nodeps.txt")"
    "${PIP[@]}" --no-deps -c "$TMP/constraints.txt" -r "$TMP/nodeps.txt"
fi
echo "[install_requirements] done."
