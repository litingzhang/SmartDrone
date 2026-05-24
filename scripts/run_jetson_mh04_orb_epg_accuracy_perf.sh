#!/usr/bin/env bash
set -euo pipefail

ROOT="${EUROC_EVAL_ROOT:-/home/nvidia/euroc_eval}"
DATA="${EUROC_MACHINE_HALL_ROOT:-/home/nvidia/euroc/machine_hall}"
BIN="${EUROC_REPLAY_BIN:-$ROOT/bin/smart_drone_offline_replay}"
SETTINGS="${EUROC_ORB_SETTINGS:-$ROOT/config/euroc/stereo_orb_official.yaml}"
VOCAB="${EUROC_VOCAB:-$ROOT/ORBvoc.txt}"
EVAL="${EUROC_EVAL_SCRIPT:-$ROOT/tests/euroc/evaluate_euroc_regression.py}"
OUT="${EUROC_OUT:-$ROOT/results/mh04_orb_epg_accuracy_perf_$(date +%Y%m%d_%H%M%S)}"
EPG_DIR="${EUROC_EPG_DIR:-$ROOT/output/epg}"
MAX_ATE_DELTA="${EUROC_MAX_ATE_DELTA:-0.0000}"
MAX_RPE_DELTA="${EUROC_MAX_RPE_DELTA:-0.0000}"
MAX_ORB_TRACK_DELTA_MS="${EUROC_MAX_ORB_TRACK_DELTA_MS:--0.001}"
TEGRASTATS_INTERVAL_MS="${TEGRASTATS_INTERVAL_MS:-1000}"
EPG_BACKEND_TICK_MIN_INTERVAL_MS="${SMART_DRONE_EPG_BACKEND_TICK_MIN_INTERVAL_MS:-50}"
EPG_BACKEND_TICK_TRACKING_COOLDOWN_MS="${SMART_DRONE_EPG_BACKEND_TICK_TRACKING_COOLDOWN_MS:-50}"

mkdir -p "$OUT" "$EPG_DIR"
cd "$ROOT"

export LD_LIBRARY_PATH="$ROOT/bin:$ROOT/lib:/home/nvidia/opencv_cuda_orb/lib:/home/nvidia/vpi_root/opt/nvidia/vpi2/lib/aarch64-linux-gnu:/home/nvidia/vpi_root/opt/nvidia/cupva-2.3/lib/aarch64-linux-gnu:/home/nvidia:/home/nvidia/SmartDrone_cross:/home/nvidia/sd_replay_pkg_jetson/lib:/usr/local/cuda-11.4/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
export SMART_DRONE_EPG_BACKEND_TICK_MIN_INTERVAL_MS="$EPG_BACKEND_TICK_MIN_INTERVAL_MS"
export SMART_DRONE_EPG_BACKEND_TICK_TRACKING_COOLDOWN_MS="$EPG_BACKEND_TICK_TRACKING_COOLDOWN_MS"

require_path() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "missing required path: $path" >&2
    exit 2
  fi
}

require_path "$BIN"
require_path "$SETTINGS"
require_path "$VOCAB"
require_path "$EVAL"
require_path "$DATA/MH_04_difficult/mav0"

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
    echo "epg_runtime:"
    echo "  SMART_DRONE_EPG_BACKEND_TICK_MIN_INTERVAL_MS=$SMART_DRONE_EPG_BACKEND_TICK_MIN_INTERVAL_MS"
    echo "  SMART_DRONE_EPG_BACKEND_TICK_TRACKING_COOLDOWN_MS=$SMART_DRONE_EPG_BACKEND_TICK_TRACKING_COOLDOWN_MS"
  } >"$OUT/system_info.log"
}

run_profiled_replay() {
  local run_dir="$1"
  shift
  mkdir -p "$run_dir"

  local tegrastats_pid=""
  if command -v tegrastats >/dev/null 2>&1; then
    tegrastats --interval "$TEGRASTATS_INTERVAL_MS" \
      >"$run_dir/tegrastats.log" 2>&1 &
    tegrastats_pid="$!"
  fi

  local start_ns
  local end_ns
  start_ns="$(date +%s%N)"
  set +e
  "$@" >"$run_dir/replay.log" 2>&1
  local status="$?"
  set -e
  end_ns="$(date +%s%N)"

  if [[ -n "$tegrastats_pid" ]]; then
    kill "$tegrastats_pid" >/dev/null 2>&1 || true
    wait "$tegrastats_pid" >/dev/null 2>&1 || true
  fi

  python3 - "$start_ns" "$end_ns" >"$run_dir/time.log" <<'PY'
import sys
start = int(sys.argv[1])
end = int(sys.argv[2])
print(f"Elapsed seconds: {(end - start) / 1_000_000_000.0:.3f}")
PY
  return "$status"
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

run_epg_orb() {
  local label="$1"
  local run_dir="$OUT/$label"
  shift
  echo "=== MH04 ORB EPG $label ==="
  run_profiled_replay "$run_dir" \
    "$BIN" \
    --dataset "$DATA/MH_04_difficult/mav0" \
    --settings "$SETTINGS" \
    --vocab "$VOCAB" \
    --stereo-only \
    --fps 20 \
    --slam-fps 20 \
    --out "$run_dir/euroc_pose.csv" \
    --slam-backend orbslam3 \
    --feature-frontend orb \
    --orb-accel cpu \
    --epg-profile-out "$run_dir/epg_profile.json" \
    "$@"
  evaluate_run "$run_dir"
}

write_system_info

rm -f "$EPG_DIR/optimized_slam_session_graph.json" \
      "$EPG_DIR/optimized_slam_session_graph_report.json"

run_epg_orb static \
  --epg-optimized-out "$OUT/optimized_slam_session_graph.json" \
  --epg-solver-report-out "$OUT/optimized_slam_session_graph_report.json"

cp -f "$OUT/optimized_slam_session_graph.json" \
      "$EPG_DIR/optimized_slam_session_graph.json"
cp -f "$OUT/optimized_slam_session_graph_report.json" \
      "$EPG_DIR/optimized_slam_session_graph_report.json"

run_epg_orb optimized

python3 - "$OUT" "$MAX_ATE_DELTA" "$MAX_RPE_DELTA" \
  "$MAX_ORB_TRACK_DELTA_MS" <<'PY'
import json
import math
import re
import sys
from pathlib import Path

out = Path(sys.argv[1])
max_ate_delta = float(sys.argv[2])
max_rpe_delta = float(sys.argv[3])
max_orb_track_delta = float(sys.argv[4])

def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))

def parse_elapsed(path):
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"Elapsed seconds: ([0-9.]+)", text)
    return float(match.group(1)) if match else math.inf

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
    estimate_rows = int(metrics.get("estimate_rows") or 0)
    elapsed = parse_elapsed(root / "time.log")
    return {
        "label": label,
        "ate": float(metrics["ate_rmse_m"]),
        "rpe": float(metrics["rpe_trans_rmse_m"]),
        "matched": int(metrics["matched_pairs"]),
        "estimate_rows": estimate_rows,
        "elapsed": elapsed,
        "replay_fps": estimate_rows / elapsed if elapsed > 0 else 0.0,
        "orb_track_mean": task_metric(profile, "SlamOrbTrackingTask", "averageLoopUs"),
        "orb_track_p90": task_metric(profile, "SlamOrbTrackingTask", "p90LoopUs"),
        "orb_track_p99": task_metric(profile, "SlamOrbTrackingTask", "p99LoopUs"),
        "orb_track_max": task_metric(profile, "SlamOrbTrackingTask", "maxLoopUs"),
        "backend_tick_mean": task_metric(profile, "SlamBackendTickTask", "averageLoopUs"),
        "acquire_mean": task_metric(profile, "SlamAcquireTask", "averageLoopUs"),
        "drops": queue_drops(profile),
    }

static = row("static")
optimized = row("optimized")

ate_delta = optimized["ate"] - static["ate"]
rpe_delta = optimized["rpe"] - static["rpe"]
orb_delta = optimized["orb_track_mean"] - static["orb_track_mean"]
fps_delta = optimized["replay_fps"] - static["replay_fps"]

accuracy_ok = ate_delta <= max_ate_delta and rpe_delta <= max_rpe_delta
perf_ok = orb_delta <= max_orb_track_delta
lines = [
    "# MH04 ORB EPG Accuracy And Performance",
    "",
    f"- Result directory: `{out}`",
    f"- Accuracy gate: optimized ATE/RPE deltas <= `{max_ate_delta}` / `{max_rpe_delta}`.",
    f"- Performance gate: optimized ORB tracking mean delta <= `{max_orb_track_delta} ms`.",
    "- Queue drops are reported only; preserveAccuracy keeps original queue depth/drop semantics.",
    "",
    "| Profile | Rows | Matched | ATE RMSE (m) | RPE RMSE (m) | Replay FPS | ORB mean/p90/p99/max ms | Backend tick mean ms | Queue drops |",
    "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
]
for item in (static, optimized):
    lines.append(
        f"| {item['label']} | {item['estimate_rows']} | {item['matched']} | "
        f"{item['ate']:.4f} | {item['rpe']:.4f} | {item['replay_fps']:.2f} | "
        f"{item['orb_track_mean']:.2f}/{item['orb_track_p90']:.2f}/"
        f"{item['orb_track_p99']:.2f}/{item['orb_track_max']:.2f} | "
        f"{item['backend_tick_mean']:.2f} | {item['drops']} |"
    )

lines.extend([
    "",
    f"- ATE delta: `{ate_delta:.6f} m`",
    f"- RPE delta: `{rpe_delta:.6f} m`",
    f"- ORB mean delta: `{orb_delta:.3f} ms`",
    f"- Replay FPS delta: `{fps_delta:.3f}`",
    f"- Accuracy OK: `{1 if accuracy_ok else 0}`",
    f"- Performance OK: `{1 if perf_ok else 0}`",
])

(out / "orb_epg_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print("\n".join(lines))

if not (accuracy_ok and perf_ok):
    raise SystemExit(1)
PY

echo "summary: $OUT/orb_epg_summary.md"
