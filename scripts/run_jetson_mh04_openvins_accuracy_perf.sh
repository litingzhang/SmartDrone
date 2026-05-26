#!/usr/bin/env bash
set -euo pipefail

ROOT="${EUROC_EVAL_ROOT:-/home/nvidia/euroc_eval}"
DATA="${EUROC_MACHINE_HALL_ROOT:-/home/nvidia/euroc/machine_hall}"
BIN="${EUROC_REPLAY_BIN:-$ROOT/bin/smart_drone_offline_replay}"
SETTINGS="${EUROC_OPENVINS_SETTINGS:-$ROOT/config/openvins/estimator_config_mh04_dyn.yaml}"
EVAL="${EUROC_EVAL_SCRIPT:-$ROOT/tests/euroc/evaluate_euroc_regression.py}"
OUT="${EUROC_OUT:-$ROOT/results/mh04_openvins_accuracy_perf_$(date +%Y%m%d_%H%M%S)}"
TEGRASTATS_INTERVAL_MS="${TEGRASTATS_INTERVAL_MS:-1000}"
EUROC_RUNTIME_LIB_DIRS="${EUROC_RUNTIME_LIB_DIRS:-$ROOT/bin:/home/nvidia:/home/nvidia/vpi_root/opt/nvidia/vpi2/lib/aarch64-linux-gnu:/home/nvidia/openvins_deps/prefix/usr/lib:/home/nvidia/openvins_deps/prefix/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-11.4/targets/aarch64-linux/lib:/usr/local/cuda/targets/aarch64-linux/lib}"

mkdir -p "$OUT"
export LD_LIBRARY_PATH="${EUROC_RUNTIME_LIB_DIRS}:${LD_LIBRARY_PATH:-}"

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
    "$@" >"$run_dir/replay.log" 2>&1
  fi
  local replay_status="$?"
  set -e

  if [[ -n "$tegrastats_pid" ]]; then
    kill "$tegrastats_pid" >/dev/null 2>&1 || true
    wait "$tegrastats_pid" >/dev/null 2>&1 || true
  fi
  return "$replay_status"
}

RUN_DIR="$OUT/openvins/MH_04_difficult"
mkdir -p "$RUN_DIR"

run_profiled_replay "$RUN_DIR" \
  "$BIN" \
  --dataset "$DATA/MH_04_difficult/mav0" \
  --settings "$SETTINGS" \
  --sensor-mode stereo-imu \
  --fps 20 \
  --slam-fps 20 \
  --out "$RUN_DIR/euroc_pose.csv" \
  --summary-json "$RUN_DIR/euroc_summary.json" \
  --slam-backend openvins \
  --feature-frontend lk_gftt_per_frame

python3 "$EVAL" \
  --dataset "$DATA/MH_04_difficult/mav0" \
  --estimate "$RUN_DIR/euroc_pose.csv" \
  --out-json "$RUN_DIR/euroc_metrics.json" \
  >"$RUN_DIR/eval.log" 2>&1

cat >"$OUT/openvins_mh04_summary.md" <<EOF
# MH04 OpenVINS Accuracy And Performance

- Result directory: \`$OUT\`
- Replay output: \`$RUN_DIR/euroc_pose.csv\`
- Metrics: \`$RUN_DIR/euroc_metrics.json\`
- Summary: \`$RUN_DIR/euroc_summary.json\`
- Replay log: \`$RUN_DIR/replay.log\`
- Eval log: \`$RUN_DIR/eval.log\`
- Tegrastats: \`$RUN_DIR/tegrastats.log\`
- Timing: \`$RUN_DIR/time.log\`
EOF

echo "summary: $OUT/openvins_mh04_summary.md"
