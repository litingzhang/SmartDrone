#!/usr/bin/env bash
set -euo pipefail

ROOT="${EUROC_EVAL_ROOT:-/home/nvidia/euroc_eval}"
DATA="${EUROC_MACHINE_HALL_ROOT:-/home/nvidia/euroc/machine_hall}"
BIN="${EUROC_REPLAY_BIN:-$ROOT/bin/smart_drone_offline_replay}"
SETTINGS="${EUROC_SETTINGS:-$ROOT/config/euroc/stereo_orb2500.yaml}"
VOCAB="${EUROC_VOCAB:-/home/nvidia/ORBvoc.txt}"
EVAL="${EUROC_EVAL_SCRIPT:-$ROOT/tests/euroc/evaluate_euroc_regression.py}"
OUT="${EUROC_OUT:-$ROOT/results/mh_feature_modes_$(date +%Y%m%d_%H%M%S)}"
XFEAT_PYTHON="${XFEAT_PYTHON:-/usr/bin/python3}"
XFEAT_REPO="${XFEAT_REPO:-/home/nvidia/accelerated_features}"
XFEAT_WORKER="${XFEAT_WORKER:-/home/nvidia/scripts/xfeat_keypoint_worker.py}"
XFEAT_DEVICE="${XFEAT_DEVICE:-cuda}"

export LD_LIBRARY_PATH="/home/nvidia:/home/nvidia/SmartDrone_cross:/home/nvidia/sd_replay_pkg_jetson/lib:${LD_LIBRARY_PATH:-}"

read -r -a SEQUENCES <<< "${EUROC_SEQUENCES:-MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult}"
read -r -a MODES <<< "${EUROC_MODES:-lk_gftt_grid lk_gftt_per_frame lk_xfeat_seed}"

for seq in "${SEQUENCES[@]}"; do
  if [[ ! -d "$DATA/$seq/mav0" ]]; then
    echo "missing EuRoC sequence: $DATA/$seq/mav0" >&2
    exit 2
  fi
done

mkdir -p "$OUT"

for mode in "${MODES[@]}"; do
  for seq in "${SEQUENCES[@]}"; do
    seq_out="$OUT/$mode/$seq"
    mkdir -p "$seq_out"
    common=(
      "$BIN"
      --dataset "$DATA/$seq/mav0"
      --settings "$SETTINGS"
      --vocab "$VOCAB"
      --stereo-only
      --fps 20
      --slam-fps 20
      --out "$seq_out/euroc_pose.csv"
      --summary-json "$seq_out/euroc_summary.json"
    )

    echo "=== $mode $seq ==="
    case "$mode" in
      lk_gftt_grid)
        "${common[@]}" --feature-frontend lk >"$seq_out/replay.log" 2>&1
        ;;
      lk_gftt_per_frame)
        "${common[@]}" --feature-frontend lk_gftt_per_frame >"$seq_out/replay.log" 2>&1
        ;;
      lk_xfeat_seed)
        "${common[@]}" \
          --feature-frontend lk \
          --lk-xfeat-seeding \
          --xfeat-python "$XFEAT_PYTHON" \
          --xfeat-repo "$XFEAT_REPO" \
          --xfeat-worker "$XFEAT_WORKER" \
          --xfeat-device "$XFEAT_DEVICE" \
          --xfeat-top-k 1024 \
          --xfeat-max-points 768 \
          --xfeat-input-max-width 640 \
          --xfeat-input-max-height 480 \
          >"$seq_out/replay.log" 2>&1
        ;;
    esac

    if python3 "$EVAL" \
      --dataset "$DATA/$seq/mav0" \
      --estimate "$seq_out/euroc_pose.csv" \
      --out-json "$seq_out/euroc_metrics.json" \
      --max-ate-rmse 999 \
      --max-rpe-trans-rmse 999 \
      >"$seq_out/eval.log" 2>&1; then
      cat "$seq_out/eval.log"
    else
      echo "eval failed for $mode $seq" >&2
      tail -80 "$seq_out/replay.log" >&2 || true
      tail -40 "$seq_out/eval.log" >&2 || true
    fi
  done
done

python3 - "$OUT" "${SEQUENCES[*]}" "${MODES[*]}" <<'PY'
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
seqs = sys.argv[2].split()
modes = sys.argv[3].split()
print("\nmode               sequence          pairs   ATE_RMSE   ATE_MAX    RPE_RMSE")
for mode in modes:
    for seq in seqs:
        path = out / mode / seq / "euroc_metrics.json"
        if not path.exists():
            print(f"{mode:<18} {seq:<16} missing")
            continue
        metrics = json.loads(path.read_text(encoding="utf-8"))
        print(
            f"{mode:<18} {seq:<16} {metrics['matched_pairs']:>5}   "
            f"{metrics['ate_rmse_m']:>8.4f}   {metrics['ate_max_m']:>8.4f}   "
            f"{metrics['rpe_trans_rmse_m']:>8.4f}"
        )
print(f"\nOUT={out}")
PY
