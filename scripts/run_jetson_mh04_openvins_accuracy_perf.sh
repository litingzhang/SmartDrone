#!/usr/bin/env bash
set -euo pipefail

ROOT="${EUROC_EVAL_ROOT:-/home/nvidia/euroc_eval}"
DATA="${EUROC_MACHINE_HALL_ROOT:-/home/nvidia/euroc/machine_hall}"
BIN="${EUROC_REPLAY_BIN:-$ROOT/bin/smart_drone_offline_replay}"
SETTINGS="${EUROC_OPENVINS_SETTINGS:-$ROOT/config/openvins/estimator_config_mh04_dyn.yaml}"
EVAL="${EUROC_EVAL_SCRIPT:-$ROOT/tests/euroc/evaluate_euroc_regression.py}"
OUT="${EUROC_OUT:-$ROOT/results/mh04_openvins_accuracy_perf_$(date +%Y%m%d_%H%M%S)}"
EPG_DIR="${EUROC_EPG_DIR:-$ROOT/output/epg}"
TEGRASTATS_INTERVAL_MS="${TEGRASTATS_INTERVAL_MS:-1000}"
EUROC_RUNTIME_LIB_DIRS="${EUROC_RUNTIME_LIB_DIRS:-$ROOT/bin:/home/nvidia:/home/nvidia/vpi_root/opt/nvidia/vpi2/lib/aarch64-linux-gnu:/home/nvidia/openvins_deps/prefix/usr/lib:/home/nvidia/openvins_deps/prefix/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-11.4/targets/aarch64-linux/lib:/usr/local/cuda/targets/aarch64-linux/lib}"

mkdir -p "$OUT" "$EPG_DIR"
cd "$ROOT"
export LD_LIBRARY_PATH="${EUROC_RUNTIME_LIB_DIRS}:${LD_LIBRARY_PATH:-}"
export SMART_DRONE_EPG_APPLY_CPU_BINDING="${SMART_DRONE_EPG_APPLY_CPU_BINDING:-1}"

require_path() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "missing required path: $path" >&2
    exit 2
  fi
}

require_path "$BIN"
require_path "$SETTINGS"
require_path "$EVAL"
require_path "$ROOT/config/epg/epg_topology.dot"
require_path "$DATA/MH_04_difficult/mav0"

run_profiled_replay() {
  local run_dir="$1"
  shift
  mkdir -p "$run_dir"

  local tegrastats_pid=""
  if command -v tegrastats >/dev/null 2>&1; then
    tegrastats --interval "$TEGRASTATS_INTERVAL_MS" >"$run_dir/tegrastats.log" 2>&1 &
    tegrastats_pid="$!"
  fi

  set +e
  if [[ -x /usr/bin/time ]]; then
    /usr/bin/time -v -o "$run_dir/time.log" "$@" >"$run_dir/replay.log" 2>&1
  else
    local start_seconds="$SECONDS"
    "$@" >"$run_dir/replay.log" 2>&1
    printf "Elapsed seconds: %s\n" "$((SECONDS - start_seconds))" >"$run_dir/time.log"
  fi
  local replay_status="$?"
  set -e

  if [[ -n "$tegrastats_pid" ]]; then
    kill "$tegrastats_pid" >/dev/null 2>&1 || true
    wait "$tegrastats_pid" >/dev/null 2>&1 || true
  fi
  return "$replay_status"
}

write_system_info() {
  {
    echo "date: $(date -Is)"
    echo "host: $(hostname)"
    echo "uname: $(uname -a)"
    if command -v nvpmodel >/dev/null 2>&1; then
      echo
      nvpmodel -q 2>/dev/null || true
    fi
    if command -v jetson_clocks >/dev/null 2>&1; then
      echo
      jetson_clocks --show 2>/dev/null || true
    fi
    echo
    echo "openvins:"
    echo "  settings=$SETTINGS"
    echo "  epg_dir=$EPG_DIR"
  } >"$OUT/system_info.log"
}

evaluate_run() {
  local run_dir="$1"
  if ! python3 "$EVAL" \
      --dataset "$DATA/MH_04_difficult/mav0" \
      --estimate "$run_dir/euroc_pose.csv" \
      --out-json "$run_dir/euroc_metrics.json" \
      --require-realtime-pose \
      >"$run_dir/eval.log" 2>&1; then
    echo "evaluation failed: $run_dir" >&2
    tail -80 "$run_dir/replay.log" >&2 || true
    tail -80 "$run_dir/eval.log" >&2 || true
    exit 1
  fi
}

run_epg_openvins() {
  local label="$1"
  local run_dir="$OUT/$label"
  shift
  echo "=== MH04 OpenVINS EPG $label ==="
  run_profiled_replay "$run_dir" \
    "$BIN" \
    --dataset "$DATA/MH_04_difficult/mav0" \
    --settings "$SETTINGS" \
    --sensor-mode stereo-imu \
    --fps 20 \
    --slam-fps 20 \
    --out "$run_dir/euroc_pose.csv" \
    --summary-json "$run_dir/euroc_summary.json" \
    --slam-backend openvins \
    --feature-frontend lk_gftt_per_frame \
    --epg-profile-out "$run_dir/epg_profile.json" \
    "$@"
  evaluate_run "$run_dir"
}

write_system_info

rm -f "$EPG_DIR/optimized_slam_session_graph.json" \
      "$EPG_DIR/optimized_slam_session_graph_report.json"

run_epg_openvins static \
  --epg-optimized-out "$OUT/optimized_slam_session_graph.json" \
  --epg-solver-report-out "$OUT/optimized_slam_session_graph_report.json"

cp -f "$OUT/optimized_slam_session_graph.json" \
      "$EPG_DIR/optimized_slam_session_graph.json"
cp -f "$OUT/optimized_slam_session_graph_report.json" \
      "$EPG_DIR/optimized_slam_session_graph_report.json"

run_epg_openvins optimized

python3 - "$OUT" <<'PY'
import json
import math
import re
from pathlib import Path
import sys

out = Path(sys.argv[1])

def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))

def parse_elapsed(path):
    if not path.exists():
        return math.nan
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        r"^\s*Elapsed \(wall clock\) time.*\):\s*([0-9:.]+)\s*$",
        text,
        re.MULTILINE,
    )
    if match:
        parts = [float(part) for part in match.group(1).split(":")]
        seconds = 0.0
        for part in parts:
            seconds = seconds * 60.0 + part
        return seconds
    match = re.search(r"Elapsed seconds: ([0-9.]+)", text)
    return float(match.group(1)) if match else math.nan

def format_float(value, digits):
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"

def task_metric(profile, name, field):
    task = profile["diagnostics"]["tasks"].get(name, {})
    return float(task.get(field, math.inf)) / 1000.0

def queue_drops(profile):
    total = 0
    for queue in profile["diagnostics"]["queues"].values():
        total += int(queue.get("droppedNewest", 0))
        total += int(queue.get("overwrittenOldest", 0))
    return total

def row(label):
    root = out / label
    metrics = load_json(root / "euroc_metrics.json")
    profile = load_json(root / "epg_profile.json")
    rows = int(metrics.get("estimate_rows") or 0)
    elapsed = parse_elapsed(root / "time.log")
    replay_fps = rows / elapsed if elapsed > 0 else math.nan
    return {
        "label": label,
        "ate": float(metrics["ate_rmse_m"]),
        "rpe": float(metrics["rpe_trans_rmse_m"]),
        "matched": int(metrics["matched_pairs"]),
        "rows": rows,
        "replay_fps": replay_fps,
        "openvins_mean": task_metric(profile, "SlamOpenVinsTrackingTask", "averageLoopUs"),
        "openvins_p90": task_metric(profile, "SlamOpenVinsTrackingTask", "p90LoopUs"),
        "openvins_p99": task_metric(profile, "SlamOpenVinsTrackingTask", "p99LoopUs"),
        "openvins_max": task_metric(profile, "SlamOpenVinsTrackingTask", "maxLoopUs"),
        "acquire_mean": task_metric(profile, "SlamAcquireTask", "averageLoopUs"),
        "backend_tick_mean": task_metric(profile, "SlamBackendTickTask", "averageLoopUs"),
        "drops": queue_drops(profile),
    }

static = row("static")
optimized = row("optimized")

lines = [
    "# MH04 OpenVINS EPG Accuracy And Performance",
    "",
    f"- Result directory: `{out}`",
    "- Replay path: `SlamOpenVinsTrackingTask` inside the SLAM EPG graph.",
    "",
    "| Profile | Rows | Matched | ATE RMSE (m) | RPE RMSE (m) | Replay FPS | OpenVINS mean/p90/p99/max ms | Acquire mean ms | Backend tick mean ms | Queue drops |",
    "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
]
for item in (static, optimized):
    lines.append(
        f"| {item['label']} | {item['rows']} | {item['matched']} | "
        f"{item['ate']:.4f} | {item['rpe']:.4f} | "
        f"{format_float(item['replay_fps'], 2)} | "
        f"{item['openvins_mean']:.2f}/{item['openvins_p90']:.2f}/"
        f"{item['openvins_p99']:.2f}/{item['openvins_max']:.2f} | "
        f"{item['acquire_mean']:.2f} | {item['backend_tick_mean']:.2f} | "
        f"{item['drops']} |"
    )

lines.extend([
    "",
    f"- ATE delta optimized-static: `{optimized['ate'] - static['ate']:.6f} m`",
    f"- RPE delta optimized-static: `{optimized['rpe'] - static['rpe']:.6f} m`",
    f"- OpenVINS mean delta optimized-static: `{optimized['openvins_mean'] - static['openvins_mean']:.3f} ms`",
])

(out / "openvins_epg_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print("\n".join(lines))
PY

echo "summary: $OUT/openvins_epg_summary.md"
