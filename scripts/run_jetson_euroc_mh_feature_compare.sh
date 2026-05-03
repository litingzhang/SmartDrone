#!/usr/bin/env bash
set -euo pipefail

ROOT="${EUROC_EVAL_ROOT:-/home/nvidia/euroc_eval}"
DATA="${EUROC_MACHINE_HALL_ROOT:-/home/nvidia/euroc/machine_hall}"
BIN="${EUROC_REPLAY_BIN:-$ROOT/bin/smart_drone_offline_replay}"
SETTINGS="${EUROC_SETTINGS:-$ROOT/config/euroc/stereo_orb2500.yaml}"
ORB_SETTINGS="${EUROC_ORB_SETTINGS:-$ROOT/config/euroc/stereo_orb_official.yaml}"
VOCAB="${EUROC_VOCAB:-/home/nvidia/ORBvoc.txt}"
EVAL="${EUROC_EVAL_SCRIPT:-$ROOT/tests/euroc/evaluate_euroc_regression.py}"
OUT="${EUROC_OUT:-$ROOT/results/mh_feature_modes_$(date +%Y%m%d_%H%M%S)}"
SUPERPOINT_LIGHTGLUE_REPO="${SUPERPOINT_LIGHTGLUE_REPO:-/home/nvidia/LightGlue}"
SUPERPOINT_TRT_ENGINE="${SUPERPOINT_TRT_ENGINE:-}"
if [[ -z "$SUPERPOINT_TRT_ENGINE" && -f "$SUPERPOINT_LIGHTGLUE_REPO/weights/superpoint_dense_640x480_fp16.engine" ]]; then
  SUPERPOINT_TRT_ENGINE="$SUPERPOINT_LIGHTGLUE_REPO/weights/superpoint_dense_640x480_fp16.engine"
elif [[ -z "$SUPERPOINT_TRT_ENGINE" && -f "$SUPERPOINT_LIGHTGLUE_REPO/weights/superpoint_dense_640x409_fp16.engine" ]]; then
  SUPERPOINT_TRT_ENGINE="$SUPERPOINT_LIGHTGLUE_REPO/weights/superpoint_dense_640x409_fp16.engine"
fi
FEATURE_DEVICE="${FEATURE_DEVICE:-${SUPERPOINT_DEVICE:-cuda}}"
SP_LG_POINTS="${SMART_DRONE_LIGHTGLUE_POINTS:-768}"
SP_LG_TOP_K="${SMART_DRONE_SUPERPOINT_TOP_K:-1024}"
BIN_DIR="$(cd "$(dirname "$BIN")" && pwd)"
TEGRASTATS_INTERVAL_MS="${TEGRASTATS_INTERVAL_MS:-1000}"

export LD_LIBRARY_PATH="$BIN_DIR:/home/nvidia/opencv_cuda_orb/lib:/home/nvidia/vpi_root/opt/nvidia/vpi2/lib/aarch64-linux-gnu:/home/nvidia/vpi_root/opt/nvidia/cupva-2.3/lib/aarch64-linux-gnu:/home/nvidia:/home/nvidia/SmartDrone_cross:/home/nvidia/sd_replay_pkg_jetson/lib:/usr/local/cuda-11.4/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"

SEQUENCES=( ${EUROC_SEQUENCES:-MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult} )
MODES=( ${EUROC_MODES:-orb klt_tracking superpoint_lightglue} )

for seq in "${SEQUENCES[@]}"; do
  if [[ ! -d "$DATA/$seq/mav0" ]]; then
    echo "missing EuRoC sequence: $DATA/$seq/mav0" >&2
    exit 2
  fi
done

mkdir -p "$OUT"

{
  echo "date: $(date -Is)"
  echo "host: $(hostname)"
  echo "uname: $(uname -a)"
  if command -v nvpmodel >/dev/null 2>&1; then
    echo
    echo "nvpmodel:"
    nvpmodel -q 2>/dev/null || true
  fi
  if command -v jetson_clocks >/dev/null 2>&1; then
    echo
    echo "jetson_clocks:"
    jetson_clocks --show 2>/dev/null || true
  fi
} >"$OUT/system_info.log"

run_profiled_replay() {
  local seq_out="$1"
  shift
  local tegrastats_pid=""
  if command -v tegrastats >/dev/null 2>&1; then
    tegrastats --interval "$TEGRASTATS_INTERVAL_MS" >"$seq_out/tegrastats.log" 2>&1 &
    tegrastats_pid="$!"
  fi

  set +e
  local start_ns=""
  local end_ns=""
  if [[ -x /usr/bin/time ]]; then
    /usr/bin/time -v -o "$seq_out/time.log" "$@" >"$seq_out/replay.log" 2>&1
  else
    start_ns="$(date +%s%N)"
    "$@" >"$seq_out/replay.log" 2>&1
    end_ns="$(date +%s%N)"
    python3 - "$start_ns" "$end_ns" >"$seq_out/time.log" <<'PY'
import sys
start = int(sys.argv[1])
end = int(sys.argv[2])
print(f"Elapsed seconds: {(end - start) / 1_000_000_000.0:.3f}")
PY
  fi
  local replay_status="$?"
  set -e

  if [[ -n "$tegrastats_pid" ]]; then
    kill "$tegrastats_pid" >/dev/null 2>&1 || true
    wait "$tegrastats_pid" >/dev/null 2>&1 || true
  fi
  return "$replay_status"
}

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
      orb)
        unset SMART_DRONE_ORB_ACCEL
        unset SMART_DRONE_ORB_CUDA_STEREO_REFINEMENT
        unset SMART_DRONE_ORB_VPI_REMAP
        unset SMART_DRONE_ORB_CUDA_PYRAMID
        run_profiled_replay "$seq_out" "${BIN}" \
          --dataset "$DATA/$seq/mav0" \
          --settings "$ORB_SETTINGS" \
          --vocab "$VOCAB" \
          --stereo-only \
          --fps 20 \
          --slam-fps 20 \
          --out "$seq_out/euroc_pose.csv" \
          --summary-json "$seq_out/euroc_summary.json" \
          --orb-accel cpu \
          --feature-frontend orb
        ;;
      orb_vpi_remap)
        unset SMART_DRONE_ORB_ACCEL
        export SMART_DRONE_ORB_VPI_REMAP=1
        unset SMART_DRONE_ORB_CUDA_PYRAMID
        run_profiled_replay "$seq_out" "${BIN}" \
          --dataset "$DATA/$seq/mav0" \
          --settings "$ORB_SETTINGS" \
          --vocab "$VOCAB" \
          --stereo-only \
          --fps 20 \
          --slam-fps 20 \
          --out "$seq_out/euroc_pose.csv" \
          --summary-json "$seq_out/euroc_summary.json" \
          --orb-accel vpi-remap \
          --feature-frontend orb
        ;;
      orb_cuda)
        export SMART_DRONE_ORB_ACCEL=cuda
        unset SMART_DRONE_ORB_VPI_REMAP
        unset SMART_DRONE_ORB_CUDA_PYRAMID
        run_profiled_replay "$seq_out" "${BIN}" \
          --dataset "$DATA/$seq/mav0" \
          --settings "$ORB_SETTINGS" \
          --vocab "$VOCAB" \
          --stereo-only \
          --fps 20 \
          --slam-fps 20 \
          --out "$seq_out/euroc_pose.csv" \
          --summary-json "$seq_out/euroc_summary.json" \
          --orb-accel cuda \
          --feature-frontend orb
        ;;
      lk_gftt_grid)
        run_profiled_replay "$seq_out" "${common[@]}" --feature-frontend lk
        ;;
      lk_gftt_per_frame)
        run_profiled_replay "$seq_out" "${common[@]}" --feature-frontend lk_gftt_per_frame --lk-per-frame-accel cpu
        ;;
      klt_tracking)
        run_profiled_replay "$seq_out" "${common[@]}" --feature-frontend klt_tracking --lk-per-frame-accel cpu
        ;;
      superpoint_lightglue)
        export SMART_DRONE_SUPERPOINT_LIGHTGLUE_INJECT=1
        export SMART_DRONE_FEATURE_PRECISION="${SMART_DRONE_FEATURE_PRECISION:-auto}"
        export SMART_DRONE_LIGHTGLUE_LAYERS="${SMART_DRONE_LIGHTGLUE_LAYERS:-6}"
        export SMART_DRONE_LIGHTGLUE_POINTS="$SP_LG_POINTS"
        export SMART_DRONE_LIGHTGLUE_MIN_SCORE="${SMART_DRONE_LIGHTGLUE_MIN_SCORE:-0.02}"
        export SMART_DRONE_LIGHTGLUE_MAX_Y_DIFF_PX="${SMART_DRONE_LIGHTGLUE_MAX_Y_DIFF_PX:-1.5}"
        export SMART_DRONE_LIGHTGLUE_MIN_DISPARITY_PX="${SMART_DRONE_LIGHTGLUE_MIN_DISPARITY_PX:-0.8}"
        export SMART_DRONE_EXTERNAL_STEREO_MAX_LEFT_FEATURES="${SMART_DRONE_EXTERNAL_STEREO_MAX_LEFT_FEATURES:-1200}"
        export SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL="${SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL:-10}"
        export SMART_DRONE_SUPERPOINT_INPUT_MAX_WIDTH="${SMART_DRONE_SUPERPOINT_INPUT_MAX_WIDTH:-640}"
        export SMART_DRONE_SUPERPOINT_INPUT_MAX_HEIGHT="${SMART_DRONE_SUPERPOINT_INPUT_MAX_HEIGHT:-480}"
        if [[ -n "$SUPERPOINT_TRT_ENGINE" ]]; then
          export SMART_DRONE_SUPERPOINT_TRT_ENGINE="$SUPERPOINT_TRT_ENGINE"
        fi
        run_profiled_replay "$seq_out" "${BIN}" \
          --dataset "$DATA/$seq/mav0" \
          --settings "$ORB_SETTINGS" \
          --vocab "$VOCAB" \
          --stereo-only \
          --fps 20 \
          --slam-fps 20 \
          --out "$seq_out/euroc_pose.csv" \
          --summary-json "$seq_out/euroc_summary.json" \
          --feature-frontend superpoint_lightglue \
          --superpoint-repo "$SUPERPOINT_LIGHTGLUE_REPO" \
          --superpoint-device "$FEATURE_DEVICE" \
          --superpoint-top-k "$SP_LG_TOP_K" \
          --superpoint-max-points "$SP_LG_POINTS" \
          --superpoint-input-max-width 640 \
          --superpoint-input-max-height 480
        ;;
      *)
        echo "unknown mode: $mode" >&2
        exit 2
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

python3 - "$OUT" "${SEQUENCES[*]}" "${MODES[*]}" <<'PY'
import json
import re
import sys
from pathlib import Path

out = Path(sys.argv[1])
seqs = sys.argv[2].split()
modes = sys.argv[3].split()

def load_json(path):
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))

def parse_time(path):
    data = {}
    if not path.exists():
        return data
    text = path.read_text(encoding="utf-8", errors="replace")
    elapsed = re.search(r"Elapsed \(wall clock\) time.*: ([0-9:.]+)", text)
    elapsed_seconds = re.search(r"Elapsed seconds: ([0-9.]+)", text)
    cpu = re.search(r"Percent of CPU this job got: ([0-9.]+)%", text)
    rss = re.search(r"Maximum resident set size \(kbytes\): ([0-9]+)", text)
    if elapsed_seconds:
        data["elapsed_s"] = float(elapsed_seconds.group(1))
    elif elapsed:
        parts = [float(p) for p in elapsed.group(1).split(":")]
        seconds = parts[-1]
        if len(parts) >= 2:
            seconds += 60 * parts[-2]
        if len(parts) >= 3:
            seconds += 3600 * parts[-3]
        data["elapsed_s"] = seconds
    if cpu:
        data["cpu_pct"] = float(cpu.group(1))
    if rss:
        data["max_rss_mb"] = int(rss.group(1)) / 1024.0
    return data

def tegra_summary(path):
    if not path.exists():
        return {}
    text = path.read_text(encoding="utf-8", errors="replace")
    gpu = []
    ram = []
    for line in text.splitlines():
        m = re.search(r"GR3D_FREQ\s+([0-9]+)%", line)
        if m:
            gpu.append(float(m.group(1)))
        m = re.search(r"RAM\s+([0-9]+)/([0-9]+)MB", line)
        if m:
            ram.append(float(m.group(1)))
    data = {"tegrastats_samples": len(text.splitlines())}
    if gpu:
        data["gpu_pct_mean"] = sum(gpu) / len(gpu)
        data["gpu_pct_max"] = max(gpu)
    if ram:
        data["ram_mb_mean"] = sum(ram) / len(ram)
        data["ram_mb_max"] = max(ram)
    return data

rows = []
for mode in modes:
    for seq in seqs:
        seq_out = out / mode / seq
        summary = load_json(seq_out / "euroc_summary.json")
        metrics = load_json(seq_out / "euroc_metrics.json")
        timing = parse_time(seq_out / "time.log")
        tegra = tegra_summary(seq_out / "tegrastats.log")
        frames = summary.get("frames_out")
        elapsed = timing.get("elapsed_s")
        rows.append({
            "mode": mode,
            "seq": seq,
            "frames": frames,
            "replay_fps": (frames / elapsed) if frames and elapsed else None,
            "elapsed_s": elapsed,
            "cpu_pct": timing.get("cpu_pct"),
            "max_rss_mb": timing.get("max_rss_mb"),
            "gpu_pct_mean": tegra.get("gpu_pct_mean"),
            "gpu_pct_max": tegra.get("gpu_pct_max"),
            "ram_mb_mean": tegra.get("ram_mb_mean"),
            "ram_mb_max": tegra.get("ram_mb_max"),
            "replay_acquire_ms_mean": summary.get("replay_acquire_ms_mean"),
            "replay_imu_ms_mean": summary.get("replay_imu_ms_mean"),
            "slam_total_ms_mean": summary.get("slam_total_ms_mean"),
            "input_prepare_ms_mean": summary.get("input_prepare_ms_mean"),
            "frontend_ms_mean": summary.get("frontend_ms_mean"),
            "stereo_pair_ms_mean": summary.get("stereo_pair_ms_mean"),
            "external_pack_ms_mean": summary.get("external_pack_ms_mean"),
            "mono_augment_ms_mean": summary.get("mono_augment_ms_mean"),
            "lk_rectify_ms_mean": summary.get("lk_rectify_ms_mean"),
            "lk_disparity_ms_mean": summary.get("lk_disparity_ms_mean"),
            "lk_gftt_ms_mean": summary.get("lk_gftt_ms_mean"),
            "lk_flow_ms_mean": summary.get("lk_flow_ms_mean"),
            "lk_candidate_ms_mean": summary.get("lk_candidate_ms_mean"),
            "lk_pnp_ms_mean": summary.get("lk_pnp_ms_mean"),
            "lk_update_ms_mean": summary.get("lk_update_ms_mean"),
            "superpoint_frontend_ms_mean": summary.get("superpoint_frontend_ms_mean"),
            "superpoint_match_ms_mean": summary.get("superpoint_match_ms_mean"),
            "superpoint_total_ms_mean": summary.get("superpoint_total_ms_mean"),
            "orb_track_ms_mean": summary.get("orb_track_ms_mean"),
            "orb_track_ms_max": summary.get("orb_track_ms_max"),
            "orb_extract_ms_mean": summary.get("orb_extract_ms_mean"),
            "orb_stereo_ms_mean": summary.get("orb_stereo_ms_mean"),
            "ate_rmse_m": metrics.get("ate_rmse_m"),
            "rpe_trans_rmse_m": metrics.get("rpe_trans_rmse_m"),
            "matched_pairs": metrics.get("matched_pairs"),
        })

def fmt(value, digits=2):
    if value is None:
        return "-"
    if isinstance(value, int):
        return str(value)
    return f"{value:.{digits}f}"

lines = [
    "# Jetson EuRoC Profiling Archive",
    "",
    f"- Result directory: `{out}`",
    f"- Sequences: `{' '.join(seqs)}`",
    f"- Modes: `{' '.join(modes)}`",
    f"- Raw artifacts per run: `replay.log`, `eval.log`, `time.log`, `tegrastats.log`, `euroc_summary.json`, `euroc_metrics.json`",
    "",
    "## System",
    "",
    "```text",
]
system_info = out / "system_info.log"
if system_info.exists():
    lines.extend(system_info.read_text(encoding="utf-8", errors="replace").strip().splitlines())
else:
    lines.append("system_info.log missing")
lines.extend(["```", "", "## Performance", ""])
lines.append("| Mode | Sequence | Frames | Wall (s) | Replay FPS | CPU % | Max RSS (MB) | GPU mean/max % | RAM mean/max MB | Track mean/max ms | Extract mean ms | Stereo mean ms |")
lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
for row in rows:
    lines.append(
        "| {mode} | {seq} | {frames} | {elapsed} | {fps} | {cpu} | {rss} | {gpu_mean}/{gpu_max} | {ram_mean}/{ram_max} | {track_mean}/{track_max} | {extract} | {stereo} |".format(
            mode=row["mode"],
            seq=row["seq"],
            frames=fmt(row["frames"], 0),
            elapsed=fmt(row["elapsed_s"], 2),
            fps=fmt(row["replay_fps"], 2),
            cpu=fmt(row["cpu_pct"], 1),
            rss=fmt(row["max_rss_mb"], 1),
            gpu_mean=fmt(row["gpu_pct_mean"], 1),
            gpu_max=fmt(row["gpu_pct_max"], 1),
            ram_mean=fmt(row["ram_mb_mean"], 0),
            ram_max=fmt(row["ram_mb_max"], 0),
            track_mean=fmt(row["orb_track_ms_mean"], 2),
            track_max=fmt(row["orb_track_ms_max"], 2),
            extract=fmt(row["orb_extract_ms_mean"], 2),
            stereo=fmt(row["orb_stereo_ms_mean"], 2),
        )
    )

lines.extend(["", "## Pipeline Breakdown", ""])
lines.append("| Mode | Sequence | Acquire | IMU | SLAM total | Prepare | Frontend | Stereo pair | External pack | Mono augment | ORB track | ORB extract | ORB stereo |")
lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
for row in rows:
    lines.append(
        "| {mode} | {seq} | {acquire} | {imu} | {slam} | {prepare} | {frontend} | {pair} | {pack} | {augment} | {track} | {extract} | {stereo} |".format(
            mode=row["mode"],
            seq=row["seq"],
            acquire=fmt(row["replay_acquire_ms_mean"], 2),
            imu=fmt(row["replay_imu_ms_mean"], 2),
            slam=fmt(row["slam_total_ms_mean"], 2),
            prepare=fmt(row["input_prepare_ms_mean"], 2),
            frontend=fmt(row["frontend_ms_mean"], 2),
            pair=fmt(row["stereo_pair_ms_mean"], 2),
            pack=fmt(row["external_pack_ms_mean"], 2),
            augment=fmt(row["mono_augment_ms_mean"], 2),
            track=fmt(row["orb_track_ms_mean"], 2),
            extract=fmt(row["orb_extract_ms_mean"], 2),
            stereo=fmt(row["orb_stereo_ms_mean"], 2),
        )
    )

lines.extend(["", "## KLT Breakdown", ""])
lines.append("| Mode | Sequence | Rectify | Disparity | GFTT | Flow | Candidate/depth | PnP | State update |")
lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
for row in rows:
    if row["mode"] != "klt_tracking":
        continue
    lines.append(
        "| {mode} | {seq} | {rectify} | {disp} | {gftt} | {flow} | {cand} | {pnp} | {update} |".format(
            mode=row["mode"],
            seq=row["seq"],
            rectify=fmt(row["lk_rectify_ms_mean"], 2),
            disp=fmt(row["lk_disparity_ms_mean"], 2),
            gftt=fmt(row["lk_gftt_ms_mean"], 2),
            flow=fmt(row["lk_flow_ms_mean"], 2),
            cand=fmt(row["lk_candidate_ms_mean"], 2),
            pnp=fmt(row["lk_pnp_ms_mean"], 2),
            update=fmt(row["lk_update_ms_mean"], 2),
        )
    )

lines.extend(["", "## SuperPoint LightGlue Breakdown", ""])
lines.append("| Mode | Sequence | Frontend infer | Native frontend | Stereo match | Total SP+LG path |")
lines.append("| --- | --- | ---: | ---: | ---: | ---: |")
for row in rows:
    if row["mode"] != "superpoint_lightglue":
        continue
    lines.append(
        f"| {row['mode']} | {row['seq']} | {fmt(row['frontend_ms_mean'], 2)} | {fmt(row['superpoint_frontend_ms_mean'], 2)} | {fmt(row['stereo_pair_ms_mean'], 2)} | {fmt(row['superpoint_total_ms_mean'], 2)} |"
    )

lines.extend(["", "## Accuracy", ""])
lines.append("| Mode | Sequence | Matched frames | ATE RMSE (m) | RPE RMSE (m) |")
lines.append("| --- | --- | ---: | ---: | ---: |")
for row in rows:
    lines.append(
        f"| {row['mode']} | {row['seq']} | {fmt(row['matched_pairs'], 0)} | {fmt(row['ate_rmse_m'], 4)} | {fmt(row['rpe_trans_rmse_m'], 4)} |"
    )

(out / "profile_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"profile summary: {out / 'profile_summary.md'}")
PY
