#!/usr/bin/env bash
set -euo pipefail

ROOT="${EUROC_EVAL_ROOT:-/home/nvidia/euroc_eval}"
RUN_SCRIPT="${EUROC_FEATURE_COMPARE_SCRIPT:-$ROOT/scripts/run_jetson_euroc_mh_feature_compare.sh}"
OUT="${EUROC_OUT:-$ROOT/results/mh04_splg_realtime_accuracy_sweep_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$OUT"

if [[ ! -x "$RUN_SCRIPT" ]]; then
  echo "missing executable run script: $RUN_SCRIPT" >&2
  exit 2
fi

run_profile() {
  local name="$1"
  local profile_out="$OUT/$name"
  mkdir -p "$profile_out"

  local env_args=(
    EUROC_SEQUENCES=MH_04_difficult
    EUROC_MODES=superpoint_lightglue
    EUROC_OUT="$profile_out"
    EUROC_MAX_ATE_RMSE="${EUROC_MAX_ATE_RMSE:-0.12}"
    EUROC_MAX_RPE_TRANS_RMSE="${EUROC_MAX_RPE_TRANS_RMSE:-0.05}"
    SUPERPOINT_TRT_ENGINE="${SUPERPOINT_TRT_ENGINE:-/home/nvidia/LightGlue/weights/superpoint_dense_640x409_fp16.engine}"
    SMART_DRONE_LIGHTGLUE_POINTS="${SMART_DRONE_LIGHTGLUE_POINTS:-512}"
    SMART_DRONE_SUPERPOINT_MAX_POINTS="${SMART_DRONE_SUPERPOINT_MAX_POINTS:-512}"
    SMART_DRONE_LIGHTGLUE_EVERY_N="${SMART_DRONE_LIGHTGLUE_EVERY_N:-4}"
    SMART_DRONE_LIGHTGLUE_SCORE_ORIENTATION="${SMART_DRONE_LIGHTGLUE_SCORE_ORIENTATION:-direct}"
    SMART_DRONE_SUPERPOINT_DESCRIPTOR_LIMIT="${SMART_DRONE_SUPERPOINT_DESCRIPTOR_LIMIT:-512}"
    SMART_DRONE_SUPERPOINT_DESCRIPTOR_NEAREST="${SMART_DRONE_SUPERPOINT_DESCRIPTOR_NEAREST:-1}"
    SMART_DRONE_TRT_PINNED_HOST_OUTPUT="${SMART_DRONE_TRT_PINNED_HOST_OUTPUT:-1}"
    SMART_DRONE_SUPERPOINT_PARALLEL_POST="${SMART_DRONE_SUPERPOINT_PARALLEL_POST:-1}"
    SMART_DRONE_LIGHTGLUE_MIN_SCORE="${SMART_DRONE_LIGHTGLUE_MIN_SCORE:-0.02}"
    SMART_DRONE_LIGHTGLUE_MAX_Y_DIFF_PX="${SMART_DRONE_LIGHTGLUE_MAX_Y_DIFF_PX:-1.5}"
    SMART_DRONE_LIGHTGLUE_MIN_DISPARITY_PX="${SMART_DRONE_LIGHTGLUE_MIN_DISPARITY_PX:-0.8}"
    SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_FEATURES="${SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_FEATURES:-72}"
    SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_POINTS="${SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_POINTS:-24}"
    SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO="${SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO:-0.30}"
    SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT="${SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT:-1}"
    SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS="${SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS:-1}"
    SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS="${SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS:-1}"
    SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS="${SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS:-1}"
    SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK="${SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK:-20}"
    SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE="${SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE:-1}"
    SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS="${SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS:-35}"
    SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE="${SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE:-0}"
    SMART_DRONE_REALTIME_POSE_CONTINUITY="${SMART_DRONE_REALTIME_POSE_CONTINUITY:-1}"
    SMART_DRONE_SUPERPOINT_INPUT_MAX_WIDTH="${SMART_DRONE_SUPERPOINT_INPUT_MAX_WIDTH:-640}"
    SMART_DRONE_SUPERPOINT_INPUT_MAX_HEIGHT="${SMART_DRONE_SUPERPOINT_INPUT_MAX_HEIGHT:-409}"
    SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL="${SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL:-10}"
    SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS="${SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS:-25}"
    SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE="${SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE:-0.998}"
  )

  case "$name" in
    stable_realtime)
      env_args+=(SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=0)
      ;;
    qgate_innovation)
      env_args+=(SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=1)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MODE=innovation)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MIN_INLIERS=120)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MIN_TRACKED_MAP=140)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MAX_INNOVATION_M=0.045)
      ;;
    qgate_step)
      env_args+=(SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=1)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MODE=step)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MIN_INLIERS=120)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MIN_TRACKED_MAP=140)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_GATE_MAX_STEP_M=0.050)
      ;;
    depth_0_960)
      env_args+=(SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.960)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=0)
      ;;
    depth_0_970)
      env_args+=(SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.970)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=0)
      ;;
    init_close_select)
      env_args+=(SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965)
      env_args+=(SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=0)
      env_args+=(SMART_DRONE_SP_LG_INIT_TRUST_SELECT_CLOSE_PAIRS=1)
      env_args+=(SMART_DRONE_SP_LG_INIT_TRUST_MAX_PAIRS=256)
      ;;
    *)
      echo "unknown profile: $name" >&2
      return 2
      ;;
  esac

  printf '%s\n' "${env_args[@]}" >"$profile_out/profile.env"
  echo "=== MH04 SP+LG profile: $name ==="
  env "${env_args[@]}" "$RUN_SCRIPT" 2>&1 | tee "$profile_out/sweep.log"
}

profiles=(${MH04_SPLG_SWEEP_PROFILES:-stable_realtime qgate_innovation qgate_step depth_0_960 depth_0_970 init_close_select})
for profile in "${profiles[@]}"; do
  run_profile "$profile"
done

python3 - "$OUT" "${profiles[*]}" <<'PY'
import json
import math
import sys
from pathlib import Path

out = Path(sys.argv[1])
profiles = sys.argv[2].split()
rows = []
for profile in profiles:
    seq_dir = out / profile / "superpoint_lightglue" / "MH_04_difficult"
    metrics_path = seq_dir / "euroc_metrics.json"
    summary_path = seq_dir / "euroc_summary.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8")) if metrics_path.exists() else {}
    summary = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path.exists() else {}
    frames = int(summary.get("frames_out") or metrics.get("estimate_rows") or 0)
    pose_valid = int(summary.get("pose_valid_frames") or (frames - int(metrics.get("invalid_pose_rows") or 0)))
    invalid = int(metrics.get("invalid_pose_rows") or 0)
    unusable = int(metrics.get("unusable_tracking_rows") or 0)
    stale = int(metrics.get("stale_identity_rows") or 0)
    ate = float(metrics.get("ate_rmse_m", math.inf))
    rpe = float(metrics.get("rpe_trans_rmse_m", math.inf))
    strict_ok = frames > 0 and pose_valid == frames and invalid == 0 and unusable == 0 and stale == 0
    rows.append((profile, strict_ok, frames, pose_valid, ate, rpe, summary.get("slam_total_ms_mean"), metrics, summary))

lines = [
    "# MH04 SP+LG Realtime Accuracy Sweep",
    "",
    f"- Result directory: `{out}`",
    "- Gate: strict realtime pose, all output rows must stay pose-valid.",
    "",
    "| Profile | Strict OK | Frames | Pose-valid | ATE RMSE (m) | RPE RMSE (m) | SLAM mean ms |",
    "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
]
for profile, strict_ok, frames, pose_valid, ate, rpe, slam_mean, *_ in rows:
    def fmt(value, digits=4):
        if value is None or not math.isfinite(float(value)):
            return "-"
        return f"{float(value):.{digits}f}"
    lines.append(
        f"| {profile} | {1 if strict_ok else 0} | {frames} | {pose_valid} | "
        f"{fmt(ate)} | {fmt(rpe)} | {fmt(slam_mean, 2)} |"
    )

valid = [row for row in rows if row[1] and math.isfinite(row[4])]
best = min(valid, key=lambda row: (row[4], row[5])) if valid else None
if best:
    best_profile = best[0]
    lines.extend(["", f"Best strict profile: `{best_profile}`"])
    env_src = out / best_profile / "profile.env"
    if env_src.exists():
        (out / "best_profile.env").write_text(env_src.read_text(encoding="utf-8"), encoding="utf-8")
else:
    lines.extend(["", "Best strict profile: none"])

(out / "sweep_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print("\n".join(lines))
PY

echo "sweep summary: $OUT/sweep_summary.md"
echo "best profile env: $OUT/best_profile.env"
