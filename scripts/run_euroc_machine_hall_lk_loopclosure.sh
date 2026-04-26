#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_ROOT="${EUROC_MACHINE_HALL_ROOT:-/home/ltz/datasets/euroc/machine_hall}"
OUT_ROOT="${EUROC_LK_LOOP_OUT:-$REPO_ROOT/output/euroc_eval_lk_machine_hall_visual_loop}"
SETTINGS="${EUROC_SETTINGS:-$REPO_ROOT/config/euroc/stereo_orb2500.yaml}"
REPLAY_BIN="$REPO_ROOT/output/artifacts/host/offline-replay/smart_drone_offline_replay"
XFEAT_PYTHON="${XFEAT_PYTHON:-$REPO_ROOT/output/xfeat_replay_venv/bin/python}"
XFEAT_REPO="${XFEAT_REPO:-$REPO_ROOT/accelerated_features}"
XFEAT_WORKER="${XFEAT_WORKER:-$REPO_ROOT/scripts/xfeat_keypoint_worker.py}"
XFEAT_DEVICE="${XFEAT_DEVICE:-cpu}"
MAX_ATE_RMSE="${MAX_ATE_RMSE:-2.5}"
MAX_RPE_TRANS_RMSE="${MAX_RPE_TRANS_RMSE:-1.0}"

SEQUENCES=(
  MH_01_easy
  MH_02_easy
  MH_03_medium
  MH_04_difficult
  MH_05_difficult
)

cd "$REPO_ROOT"

for seq in "${SEQUENCES[@]}"; do
  if [[ ! -d "$DATASET_ROOT/$seq/mav0" ]]; then
    echo "missing EuRoC sequence: $DATASET_ROOT/$seq/mav0" >&2
    exit 2
  fi
done

if [[ ! -f "$SETTINGS" ]]; then
  echo "missing settings file: $SETTINGS" >&2
  exit 2
fi

./scripts/build.sh replay
mkdir -p "$OUT_ROOT"

failed=0
for seq in "${SEQUENCES[@]}"; do
  seq_out="$OUT_ROOT/$seq"
  mkdir -p "$seq_out"
  echo "=== $seq: LK + visual keyframe loop closure ==="
  "$REPLAY_BIN" \
    --dataset "$DATASET_ROOT/$seq/mav0" \
    --settings "$SETTINGS" \
    --stereo-only \
    --feature-frontend lk \
    --lk-loop-closure \
    --lk-loop-scale 1.20 \
    --lk-loop-relax 1.40 \
    --fps 20 \
    --slam-fps 20 \
    --xfeat-python "$XFEAT_PYTHON" \
    --xfeat-repo "$XFEAT_REPO" \
    --xfeat-worker "$XFEAT_WORKER" \
    --xfeat-device "$XFEAT_DEVICE" \
    --out "$seq_out/euroc_pose.csv" \
    --summary-json "$seq_out/euroc_summary.json"

  if ! python3 "$REPO_ROOT/tests/euroc/evaluate_euroc_regression.py" \
      --dataset "$DATASET_ROOT/$seq/mav0" \
      --estimate "$seq_out/euroc_pose.csv" \
      --out-json "$seq_out/euroc_metrics.json" \
      --max-ate-rmse "$MAX_ATE_RMSE" \
      --max-rpe-trans-rmse "$MAX_RPE_TRANS_RMSE"; then
    failed=1
  fi
done

python3 - "$OUT_ROOT" "${SEQUENCES[@]}" <<'PY'
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
seqs = sys.argv[2:]
print("\nsequence          pairs   ATE_RMSE   RPE_RMSE   pass")
for seq in seqs:
    metrics_path = out / seq / "euroc_metrics.json"
    if not metrics_path.exists():
        print(f"{seq:<16} missing")
        continue
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    ate = metrics["ate_rmse_m"]
    rpe = metrics["rpe_trans_rmse_m"]
    passed = ate <= metrics["threshold_max_ate_rmse_m"] and rpe <= metrics["threshold_max_rpe_trans_rmse_m"]
    print(f"{seq:<16} {metrics['matched_pairs']:>5}   {ate:>8.4f}   {rpe:>8.4f}   {passed}")
PY

exit "$failed"
