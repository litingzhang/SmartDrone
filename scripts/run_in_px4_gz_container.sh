#!/usr/bin/env bash
set -euo pipefail

Usage()
{
    cat <<'EOF'
Usage: scripts/run_in_px4_gz_container.sh [options] -- COMMAND [ARG...]

Runs a SmartDrone/PX4/Gazebo command in the pinned Harmonic image. The
workspace is mounted at the same absolute path with host network and IPC.

Options:
  --image NAME             Container image tag.
  --software-rendering     Use Mesa llvmpipe without exposing /dev/dri.
  --hardware-rendering     Expose an available render node without probing it.

The default probes /dev/dri render nodes with surfaceless EGL and uses hardware
rendering only when the container reports a hardware renderer.
SMART_DRONE_GZ_RENDERING can override the default with auto, software, or
hardware.
EOF
}

Fail()
{
    echo "ERR $*" >&2
    exit 1
}

DiscoverRenderNodes()
{
    [[ -d "$DRI_ROOT" ]] || return 0
    find "$DRI_ROOT" -maxdepth 1 -type c -name 'renderD*' -print | sort
}

RenderNodeRuntimeStatus()
{
    local render_node="$1"
    local status_file
    status_file="$DRM_SYSFS_ROOT/$(basename "$render_node")/device/power/runtime_status"
    [[ -r "$status_file" ]] || return 1
    head -n 1 "$status_file"
}

ReportRuntimePmError()
{
    local render_node="$1"
    printf 'WARN render node %s has host runtime PM status=error under kernel %s; ' \
        "$render_node" "$(uname -r)" >&2
    printf 'hardware rendering is unavailable until the host GPU driver is recovered\n' >&2
}

HardwareRendererFromOutput()
{
    local output="$1"
    local line renderer=""
    while IFS= read -r line; do
        if [[ "$line" == *"OpenGL core profile renderer:"* ]]; then
            renderer="${line#*OpenGL core profile renderer:}"
            renderer="${renderer#${renderer%%[![:space:]]*}}"
            break
        fi
    done <<<"$output"
    [[ -n "$renderer" ]] || return 1
    case "${renderer,,}" in
        *llvmpipe*|*softpipe*|*swrast*|*software\ rasterizer*|*lavapipe*)
            return 1
            ;;
    esac
    printf '%s\n' "$renderer"
}

ProbeRenderNode()
{
    local render_node="$1"
    local device_group output renderer
    device_group="$(stat -c %g "$render_node")" || return 1
    output="$(docker run --rm --network none \
        --user "$(id -u):$(id -g)" \
        --group-add "$device_group" \
        --device "$render_node" \
        --env "EGL_PLATFORM=surfaceless" \
        --env "XDG_RUNTIME_DIR=/tmp" \
        "$IMAGE" eglinfo -B -p surfaceless 2>&1)" || return 1
    renderer="$(HardwareRendererFromOutput "$output")" || return 1
    printf '%s\t%s\n' "$render_node" "$renderer"
}

ResolveAutoRendering()
{
    local render_node probe_result runtime_status
    local runtime_pm_error_seen=0
    RENDER_REASON="no_render_node"
    while IFS= read -r render_node; do
        [[ -n "$render_node" ]] || continue
        if runtime_status="$(RenderNodeRuntimeStatus "$render_node")" && \
            [[ "$runtime_status" == "error" ]]; then
            runtime_pm_error_seen=1
            ReportRuntimePmError "$render_node"
            continue
        fi
        RENDER_REASON="egl_probe_failed_or_software"
        if probe_result="$(ProbeRenderNode "$render_node")"; then
            IFS=$'\t' read -r SELECTED_RENDER_NODE SELECTED_RENDERER \
                <<<"$probe_result"
            RENDERING="hardware"
            RENDER_REASON="hardware_egl"
            RENDER_VERIFIED=1
            return
        fi
        if runtime_status="$(RenderNodeRuntimeStatus "$render_node")" && \
            [[ "$runtime_status" == "error" ]]; then
            runtime_pm_error_seen=1
            ReportRuntimePmError "$render_node"
        fi
    done < <(DiscoverRenderNodes)
    if ((runtime_pm_error_seen)); then
        RENDER_REASON="host_runtime_pm_error"
    fi
    RENDERING="software"
    SELECTED_RENDERER="llvmpipe"
    RENDER_VERIFIED=1
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(realpath "$REPO_ROOT/..")"
IMAGE="${SMART_DRONE_PX4_GZ_IMAGE:-smartdrone-px4-gz:v1.17.0}"
RENDERING="${SMART_DRONE_GZ_RENDERING:-auto}"
DRI_ROOT="${SMART_DRONE_DRI_ROOT:-/dev/dri}"
DRM_SYSFS_ROOT="${SMART_DRONE_DRM_SYSFS_ROOT:-/sys/class/drm}"
REQUESTED_RENDERING=""
RENDER_REASON="explicit"
SELECTED_RENDER_NODE=""
SELECTED_RENDERER=""
RENDER_VERIFIED=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --image) [[ $# -ge 2 ]] || Fail "--image requires a value"; IMAGE="$2"; shift 2 ;;
        --image=*) IMAGE="${1#--image=}"; shift ;;
        --software-rendering) RENDERING="software"; shift ;;
        --hardware-rendering) RENDERING="hardware"; shift ;;
        --) shift; break ;;
        -h|--help) Usage; exit 0 ;;
        *) break ;;
    esac
done
[[ $# -gt 0 ]] || { Usage >&2; exit 2; }
docker image inspect "$IMAGE" >/dev/null 2>&1 || \
    Fail "container image is not available: $IMAGE"

REQUESTED_RENDERING="$RENDERING"
case "$RENDERING" in
    auto) ResolveAutoRendering ;;
    software) SELECTED_RENDERER="llvmpipe"; RENDER_VERIFIED=1 ;;
    hardware)
        runtime_pm_error_seen=0
        while IFS= read -r render_node; do
            [[ -n "$render_node" ]] || continue
            if runtime_status="$(RenderNodeRuntimeStatus "$render_node")" && \
                [[ "$runtime_status" == "error" ]]; then
                runtime_pm_error_seen=1
                ReportRuntimePmError "$render_node"
                continue
            fi
            SELECTED_RENDER_NODE="$render_node"
            break
        done < <(DiscoverRenderNodes)
        if [[ -z "$SELECTED_RENDER_NODE" ]] && ((runtime_pm_error_seen)); then
            Fail "hardware rendering requested with a render node in runtime PM error state"
        fi
        [[ -n "$SELECTED_RENDER_NODE" ]] || \
            Fail "hardware rendering requested but no render node is available"
        SELECTED_RENDERER="not_probed"
        ;;
    *) Fail "SMART_DRONE_GZ_RENDERING must be auto, software, or hardware" ;;
esac

printf 'SmartDrone Gazebo rendering: requested=%s selected=%s reason=%s verified=%s' \
    "$REQUESTED_RENDERING" "$RENDERING" "$RENDER_REASON" \
    "$RENDER_VERIFIED" >&2
if [[ -n "$SELECTED_RENDER_NODE" ]]; then
    printf ' render_node=%s' "$SELECTED_RENDER_NODE" >&2
fi
printf ' renderer=%s\n' "$SELECTED_RENDERER" >&2

docker_args=(
    run --rm --init --network host --ipc host
    --user "$(id -u):$(id -g)"
    --env "HOME=/tmp/smartdrone-home"
    --env "USER=${USER:-smartdrone}"
    --env "XDG_RUNTIME_DIR=/tmp/smartdrone-runtime-$(id -u)"
    --env "DISPLAY=${DISPLAY:-}"
    --env "QT_X11_NO_MITSHM=1"
    --env "PX4_AUTOPILOT_DIR=$WORKSPACE_ROOT/px4/PX4-Autopilot-v1.17.0"
    --env "SMART_DRONE_GZ_RENDERING_SELECTED=$RENDERING"
    --env "SMART_DRONE_GZ_RENDERING_REQUESTED=$REQUESTED_RENDERING"
    --env "SMART_DRONE_GZ_RENDERING_REASON=$RENDER_REASON"
    --env "SMART_DRONE_GZ_RENDERING_VERIFIED=$RENDER_VERIFIED"
    --env "SMART_DRONE_GZ_RENDER_NODE=$SELECTED_RENDER_NODE"
    --env "SMART_DRONE_GZ_RENDERER=$SELECTED_RENDERER"
    --volume "$WORKSPACE_ROOT:$WORKSPACE_ROOT"
    --workdir "$REPO_ROOT"
)

if [[ "$RENDERING" == "software" ]]; then
    watchdog_scale="${SMART_DRONE_SIM_WATCHDOG_SCALE:-5}"
    docker_args+=(
        --env "LIBGL_ALWAYS_SOFTWARE=1"
        --env "MESA_LOADER_DRIVER_OVERRIDE=llvmpipe"
        --env "EGL_PLATFORM=surfaceless"
        --env "SMART_DRONE_SIM_WATCHDOG_SCALE=$watchdog_scale"
    )
elif [[ "$RENDERING" == "hardware" ]]; then
    watchdog_scale="${SMART_DRONE_SIM_WATCHDOG_SCALE:-1}"
    docker_args+=(
        --env "SMART_DRONE_SIM_WATCHDOG_SCALE=$watchdog_scale"
        --device "$SELECTED_RENDER_NODE"
        --group-add "$(stat -c %g "$SELECTED_RENDER_NODE")"
    )
    if [[ "$REQUESTED_RENDERING" == "auto" ]]; then
        docker_args+=(--env "EGL_PLATFORM=surfaceless")
    fi
else
    watchdog_scale="${SMART_DRONE_SIM_WATCHDOG_SCALE:-1}"
    docker_args+=(--env "SMART_DRONE_SIM_WATCHDOG_SCALE=$watchdog_scale")
fi
if [[ -d /tmp/.X11-unix ]]; then
    docker_args+=(--volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi
if [[ -t 0 && -t 1 ]]; then
    docker_args+=(-it)
else
    docker_args+=(-i)
fi

container_entry='mkdir -p "$HOME" "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"; exec "$@"'
exec docker "${docker_args[@]}" "$IMAGE" bash -c "$container_entry" smartdrone "$@"
