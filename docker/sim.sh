#!/usr/bin/env bash
# Run Gradle for the physics sim inside the Ubuntu 24.04 container.
#
# For Linux machines older than Ubuntu 24.04 (WPILib 2026 vendor libraries need
# GCC 13's libstdc++). Needs Docker and an X11 desktop.
#
#   docker/sim.sh                 build + launch the sim GUI (simulateNative)
#   docker/sim.sh build           any other gradle task(s)
#   docker/sim.sh scope           AdvantageScope (3D field); "Connect to Simulator" in it
#   docker/sim.sh shell           interactive shell in the container
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The robot code: ROBOT_PROJECT_DIR if set, else the default "Rebuilt" next to this folder
ROBOT_DIR="$(cd "${ROBOT_PROJECT_DIR:-$SIM_DIR/../Rebuilt}" && pwd)"
IMAGE=frc1229-sim
GRADLE_CACHE="$HOME/.gradle-frc-docker"
# Home folder inside the container, kept so AdvantageScope remembers its layout
CONTAINER_HOME="$HOME/.frc-sim-docker-home"
# Keep the machine usable while compiling: half the cores by default (SIM_CPUS to override)
CPUS="${SIM_CPUS:-$(( $(nproc) / 2 > 0 ? $(nproc) / 2 : 1 ))}"

docker build -q -t "$IMAGE" "$SIM_DIR/docker" > /dev/null
mkdir -p "$GRADLE_CACHE" "$CONTAINER_HOME"

# Allow local containers to open windows on this X display
xhost +local: > /dev/null 2>&1 || true

args=(
    --rm
    --user "$(id -u):$(id -g)"
    -e HOME=/home/sim
    -v "$CONTAINER_HOME":/home/sim
    -e DISPLAY="${DISPLAY:-:0}"
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -v "$GRADLE_CACHE":/gradle
    # Same path as on the host so build logs and paths line up
    -v "$SIM_DIR":"$SIM_DIR"
    -v "$ROBOT_DIR":"$ROBOT_DIR"
    -e ROBOT_PROJECT_DIR="$ROBOT_DIR"
    -w "$SIM_DIR"
    # Host networking: AdvantageScope / dashboards on the host can reach NetworkTables
    --network host
    --ipc host
    # CPU cap, and lower priority than desktop apps when competing for the cores
    --cpus "$CPUS"
    --cpu-shares 256
)
# GPU for OpenGL (sim GUI) and WebGL (AdvantageScope's 3D field)
if command -v nvidia-smi > /dev/null && docker info --format '{{json .Runtimes}}' | grep -q nvidia; then
    # NVIDIA's driver; needs the NVIDIA Container Toolkit
    args+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
elif [ -e /dev/dri ]; then
    # Intel / AMD through Mesa
    args+=(--device /dev/dri)
fi
# Controllers (joystick devices)
[ -d /dev/input ] && args+=(--device /dev/input -v /run/udev:/run/udev:ro)
INPUT_GID="$(getent group input | cut -d: -f3 || true)"
[ -n "$INPUT_GID" ] && args+=(--group-add "$INPUT_GID")
# Interactive only when run from a terminal
[ -t 0 ] && args+=(-it)

# Only one sim (and one AdvantageScope) at a time: a second robot program can't
# open the NetworkTables ports, and dashboards end up talking to the old one.
run_single() {
    local name="$1"; shift
    if [ -n "$(docker ps -q --filter "name=^${name}\$")" ]; then
        echo "Already running (container '$name'). Close it first, or: docker stop $name" >&2
        exit 1
    fi
    exec docker run --name "$name" "${args[@]}" "$IMAGE" "$@"
}

case "${1:-simulateNative}" in
    shell) exec docker run "${args[@]}" "$IMAGE" bash ;;
    # Electron's sandbox needs privileges the container doesn't have
    scope) run_single frc1229-scope advantagescope --no-sandbox ;;
    simulateNative) run_single frc1229-sim bash gradlew --console=plain --max-workers "$CPUS" simulateNative ;;
esac
exec docker run "${args[@]}" "$IMAGE" bash gradlew --console=plain --max-workers "$CPUS" "$@"
