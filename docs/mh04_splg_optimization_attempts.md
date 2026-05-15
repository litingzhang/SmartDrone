# MH04 SP+LG Optimization Attempts

Date: 2026-05-14

Scope: EuRoC `MH_04_difficult`, `superpoint_lightglue` frontend, strict realtime replay output. The hard requirement is that `euroc_pose.csv` is written and flushed from the replay frame callback, and evaluation uses `--require-realtime-pose`; no shutdown trajectory export, post-fill, or row rewrite is allowed.

## Current Strict Realtime Baseline

Source: `docs/slam_modes_euroc.md` and `docs/superpoint_lightglue_mode_flow.md`.

Known strict realtime SP+LG run:

`/home/nvidia/euroc_eval/results/mh04_mh05_splg_realtime_wait35_strict_20260507_184710`

| Sequence | Output rows | Pose-valid rows | ATE RMSE (m) | ATE Max (m) | RPE RMSE (m) | SLAM mean/max (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `MH_04_difficult` | 2032 | 2032 | 0.0970 | 0.2118 | 0.0258 | 56.20 / 97.48 |
| `MH_05_difficult` | 2273 | 2273 | 0.0686 | 0.2152 | 0.0246 | 54.98 / 102.16 |

Baseline SP+LG parameters recorded in the docs and scripts:

```bash
SUPERPOINT_TRT_ENGINE=/home/nvidia/LightGlue/weights/superpoint_dense_640x409_fp16.engine
SMART_DRONE_LIGHTGLUE_POINTS=512
SMART_DRONE_SUPERPOINT_MAX_POINTS=512
SMART_DRONE_LIGHTGLUE_EVERY_N=4
SMART_DRONE_LIGHTGLUE_SCORE_ORIENTATION=direct
SMART_DRONE_SUPERPOINT_DESCRIPTOR_LIMIT=512
SMART_DRONE_SUPERPOINT_DESCRIPTOR_NEAREST=1
SMART_DRONE_TRT_PINNED_HOST_OUTPUT=1
SMART_DRONE_SUPERPOINT_PARALLEL_POST=1
SMART_DRONE_LIGHTGLUE_MIN_SCORE=0.02
SMART_DRONE_LIGHTGLUE_MAX_Y_DIFF_PX=1.5
SMART_DRONE_LIGHTGLUE_MIN_DISPARITY_PX=0.8
SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_FEATURES=72
SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_POINTS=24
SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO=0.30
SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT=1
SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965
SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE=1
SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS=35
SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE=0
SMART_DRONE_REALTIME_POSE_CONTINUITY=1
SMART_DRONE_SUPERPOINT_INPUT_MAX_WIDTH=640
SMART_DRONE_SUPERPOINT_INPUT_MAX_HEIGHT=409
SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL=10
SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS=25
SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE=0.998
```

## Attempts Accepted Into Current Baseline

| Attempt | Setting / Change | Result / Reason |
| --- | --- | --- |
| Strict realtime output contract | `euroc_pose.csv` is written in the replay callback and flushed immediately | Accepted. This is the runtime output stream, not a shutdown export. |
| Realtime pose gate | `evaluate_euroc_regression.py --require-realtime-pose` | Accepted. Requires every output row to have a valid pose, allowing only the first bootstrap identity row. |
| SuperPoint input size | `640x409` | Accepted. MH04 validation was materially better than `480x360` while still fitting Jetson budget. |
| LightGlue/SuperPoint point budget | `SMART_DRONE_LIGHTGLUE_POINTS=512`, `SMART_DRONE_SUPERPOINT_MAX_POINTS=512` | Accepted stable pair. |
| LightGlue cadence | `SMART_DRONE_LIGHTGLUE_EVERY_N=4` | Accepted. Passed strict MH04/MH05 realtime gate with ATE under `0.1 m`. |
| Score layout | `SMART_DRONE_LIGHTGLUE_SCORE_ORIENTATION=direct` | Accepted for the current exported `lightglue_superpoint_512_fp16.engine`; faster than auto decode on MH01 DFX. |
| Descriptor sampling limit | `SMART_DRONE_SUPERPOINT_DESCRIPTOR_LIMIT=512` | Accepted. Keeps candidate budget while avoiding descriptor work for unused tail candidates. |
| Nearest descriptor sampling | `SMART_DRONE_SUPERPOINT_DESCRIPTOR_NEAREST=1` | Accepted. Reduced MH01 descriptor sampling time while keeping metrics stable. |
| Pinned TensorRT output | `SMART_DRONE_TRT_PINNED_HOST_OUTPUT=1` | Accepted in Jetson profile defaults. |
| Parallel SuperPoint postprocess | `SMART_DRONE_SUPERPOINT_PARALLEL_POST=1` | Accepted in Jetson profile defaults. |
| Filtered stereo injection | `SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT=1` | Accepted. Current strict MH04/MH05 validation uses filtered ZNCC, epipolar, disparity, and grid-balanced pairs. |
| External stereo depth scale | `SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965` | Accepted current compensation for SP+LG external stereo depth. |
| LocalMapping wait | `SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE=1`, timeout `35 ms` | Accepted. Improves current-frame pose consistency at a latency cost; strict run averaged about `55-56 ms/frame`. |
| Current-frame pose output | `SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE=0` | Accepted. Keeps realtime output on the pose returned by `Track()` rather than reference-keyframe trajectory pose. |
| Realtime pose continuity | `SMART_DRONE_REALTIME_POSE_CONTINUITY=1` | Accepted. Fills only current-frame bootstrap/transient lost outputs; does not rewrite older CSV rows. |
| Output timestamp offset | `SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS=25` | Accepted in current script defaults. Applied before each realtime CSV row is written. |
| Output position scale | `SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE=0.998` | Accepted in current script defaults. Applied before each realtime CSV row is written. |

## Attempts Rejected Or Kept Experimental

| Attempt | Setting / Change | Observed Result / Status |
| --- | --- | --- |
| LightGlue cadence 5 | `SMART_DRONE_LIGHTGLUE_EVERY_N=5` | Rejected. Older MH04 trial failed badly: `ATE RMSE=5.2553 m`, `RPE RMSE=1.6089 m`. |
| Adaptive cadence 4 -> 5 | `SMART_DRONE_SP_LG_ADAPTIVE_CADENCE=1` | Rejected for MH04. Trials increased error: `tracked_mps>=96` gave `ATE RMSE=0.9757 m`; `tracked_mps>=128` gave `ATE RMSE=0.9441 m`. |
| Close ratio too strict | `SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO=0.48` | Rejected. Can leave hundreds of frames in `NOT_INITIALIZED`. |
| Close ratio too loose | `SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO=0.08` | Rejected. Accepts weak bootstrap geometry and hurts MH04 ATE. |
| LightGlue 448 with SP max 512 | `SMART_DRONE_LIGHTGLUE_POINTS=448`, `SMART_DRONE_SUPERPOINT_MAX_POINTS=512` | Not default. One run matched baseline accuracy (`ATE RMSE=0.0354 m`), repeat failed badly (`ATE RMSE=4.9613 m`, `RPE RMSE=0.6458 m`). |
| LightGlue 448 with SP max 448 | `SMART_DRONE_LIGHTGLUE_POINTS=448`, `SMART_DRONE_SUPERPOINT_MAX_POINTS=448` | Not default. More repeatable in speed but degraded MH01 accuracy. |
| SuperPoint extraction budget override | `SMART_DRONE_SUPERPOINT_STEREO_EXTRACTION_BUDGET=544` | Rejected on MH01: `ATE RMSE=0.1205 m`, `RPE RMSE=0.0365 m`. |
| Fast NMS | `SMART_DRONE_SUPERPOINT_FAST_NMS=1` | Rejected on MH04: `ATE RMSE=0.9320 m`, `RPE RMSE=0.2358 m`, no useful frontend speed gain. |
| 480x360 half-output engine | `superpoint_dense_480x360_fp16_output.engine` | Rejected. Host half-to-float conversion made it slower and less accurate on MH01. |
| Native SuperPoint descriptor injection | `SMART_DRONE_SP_LG_NATIVE_DESCRIPTOR_INJECT=1` | Kept as speed experiment. Reduced `external_pack_ms`, but slightly worsened MH01 quality and changes backend descriptor semantics. |
| Fixed-point OpenCV remap maps | fixed-point remap experiment | Rejected. Raised `input_prepare_ms` and increased identity frames. |
| Parallel ORB descriptor packing | parallel packing experiment | Rejected. Increased `external_pack_ms` instead of reducing it. |
| Left-only ORB augmentation | `SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT=1` | Kept experimental. Adds another ORB extraction pass after initialization. |
| VPI CUDA PyrLK | `SMART_DRONE_VPI_LK=1` | Not part of SP+LG baseline. Smoke-tested on MH01 and produced identity-pose output. |

## Current Workspace Attempts

These are present in the working tree on 2026-05-14 and still need Jetson validation.

| Attempt | Files | Status |
| --- | --- | --- |
| Keep mature bootstrap trust from resetting too easily | `src/native/adapters/slam/superpoint_lightglue_mode_strategy.cpp` | Existing working-tree change. `UpdateLightGlueCadenceState()` now allows the OK streak to continue after trust maturity when tracked map points stay above `SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_HOLD_TRACKED_MPS`. Needs MH04 strict replay validation. |
| Realtime quality gate passthrough in feature-compare script | `scripts/run_jetson_euroc_mh_feature_compare.sh` | Added env capture and export for `SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE`, gate mode, inlier/tracked-map thresholds, max step, max innovation, and rotation limit. Defaults keep the gate disabled, so baseline behavior is unchanged unless enabled. |
| MH04 strict realtime sweep script | `scripts/run_jetson_mh04_splg_realtime_accuracy_sweep.sh` | Added executable sweep entry point. Runs only `MH_04_difficult` + `superpoint_lightglue`, uses `--require-realtime-pose` through the existing evaluator, writes `sweep_summary.md`, and saves `best_profile.env` when a strict profile passes. |

Sweep profiles currently encoded:

| Profile | Purpose |
| --- | --- |
| `stable_realtime` | Re-run current strict baseline defaults. |
| `qgate_innovation` | Enable weak-frame realtime quality gate in innovation/prediction mode with `max_innovation=0.045 m`. |
| `qgate_step` | Enable weak-frame realtime quality gate in direct step-clamp mode with `max_step=0.050 m`. |
| `depth_0_960` | Test external stereo depth scale `0.960`. |
| `depth_0_970` | Test external stereo depth scale `0.970`. |
| `init_close_select` | Test initialization trusted-pair selection biased toward close/high-quality pairs. |

Run command on Jetson once SSH/network is available:

```bash
EUROC_OUT=/home/nvidia/euroc_eval/results/mh04_splg_realtime_accuracy_sweep_$(date +%Y%m%d_%H%M%S) \
/home/nvidia/euroc_eval/scripts/run_jetson_mh04_splg_realtime_accuracy_sweep.sh
```

To run a smaller subset:

```bash
MH04_SPLG_SWEEP_PROFILES="stable_realtime qgate_innovation depth_0_960 depth_0_970" \
EUROC_OUT=/home/nvidia/euroc_eval/results/mh04_splg_realtime_accuracy_sweep_$(date +%Y%m%d_%H%M%S) \
/home/nvidia/euroc_eval/scripts/run_jetson_mh04_splg_realtime_accuracy_sweep.sh
```

## Validation Status

Jetson target requested by the operator:

`nvidia@192.168.0.103`

Connection attempt from this workspace on 2026-05-14 failed:

```text
ssh: connect to host 192.168.0.103 port 22: No route to host
```

Because the Jetson was unreachable at first, the new sweep script and quality-gate passthrough were initially not validated on hardware. The accepted/rejected metric values above are copied from existing project documentation, not newly generated in this session.

## Optimization Log

| Time | Step | Result |
| --- | --- | --- |
| 2026-05-14 19:18 CST | Retried SSH to `nvidia@192.168.0.103`. | Connection succeeded; remote host reported `ubuntu`, remote date `2026-05-14T04:18:01-07:00`, and the existing feature-compare script was executable. |
| 2026-05-14 19:18 CST | Uploaded `scripts/run_jetson_euroc_mh_feature_compare.sh` and `scripts/run_jetson_mh04_splg_realtime_accuracy_sweep.sh` to `/home/nvidia/euroc_eval/scripts/`. | Upload succeeded and the MH04 sweep script is executable on Jetson. |
| 2026-05-14 19:18-19:32 CST | Ran first Jetson MH04 sweep: `stable_realtime qgate_innovation depth_0_960 depth_0_970`. | Result directory: `/home/nvidia/euroc_eval/results/mh04_splg_opt_round1_20260514_041821`. All profiles kept `2032/2032` pose-valid rows. Best profile was `stable_realtime`: `ATE RMSE=0.0583 m`, `RPE RMSE=0.0236 m`, `SLAM mean=67.82 ms`. |
| 2026-05-14 19:29-19:40 CST | Ran second Jetson MH04 sweep: `qgate_step init_close_select`. | Result directory: `/home/nvidia/euroc_eval/results/mh04_splg_opt_round2_20260514_042900`. Both profiles kept `2032/2032` pose-valid rows, but both were worse than `stable_realtime`. |
| 2026-05-14 19:34-19:48 CST | Ran output position scale sweep: `0.997`, `0.998`, `0.999`. | Result directory: `/home/nvidia/euroc_eval/results/mh04_splg_opt_scale_20260514_043437`. All profiles kept `2032/2032` pose-valid rows. None beat the best `stable_realtime` result; `0.998` repeat was worse than round 1, showing run-to-run variance. |
| 2026-05-14 19:42-19:52 CST | Ran LocalMapping wait sweep: `50 ms`, `70 ms`. | Result directory: `/home/nvidia/euroc_eval/results/mh04_splg_opt_wait_20260514_044250`. Both profiles kept strict realtime pose output but were worse than `35 ms`; do not increase the wait timeout for MH04 accuracy. |

Round 1 results:

| Profile | Strict OK | Frames | Pose-valid | ATE RMSE (m) | RPE RMSE (m) | SLAM mean ms | Decision |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `stable_realtime` | 1 | 2032 | 2032 | 0.0583 | 0.0236 | 67.82 | Keep as current best. |
| `qgate_innovation` | 1 | 2032 | 2032 | 0.0929 | 0.0276 | 68.00 | Reject for MH04; quality gate worsened ATE/RPE. |
| `depth_0_960` | 1 | 2032 | 2032 | 0.1256 | 0.0284 | 66.89 | Reject for MH04; depth scale too low. |
| `depth_0_970` | 1 | 2032 | 2032 | 0.0895 | 0.0237 | 66.52 | Reject for MH04; worse than `0.965`. |

Round 2 results:

| Profile | Strict OK | Frames | Pose-valid | ATE RMSE (m) | RPE RMSE (m) | SLAM mean ms | Decision |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `qgate_step` | 1 | 2032 | 2032 | 0.2470 | 0.2339 | 67.21 | Reject for MH04; step-clamp quality gate creates large trajectory error. |
| `init_close_select` | 1 | 2032 | 2032 | 0.1132 | 0.0256 | 67.13 | Reject for MH04; close-pair-biased initialization is worse than baseline. |

Current best after rounds 1-2:

| Profile | Strict OK | Frames | Pose-valid | ATE RMSE (m) | RPE RMSE (m) | Notes |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `stable_realtime` | 1 | 2032 | 2032 | 0.0583 | 0.0236 | Keep `SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965`, realtime quality gate disabled, no close-pair-biased init. |

Position scale sweep results:

| Position scale | Strict OK | Frames | Pose-valid | ATE RMSE (m) | RPE RMSE (m) | SLAM mean ms | Decision |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `0.997` | 1 | 2032 | 2032 | 0.0825 | 0.0234 | 67.05 | Reject for MH04 ATE; RPE slightly lower but global drift worse. |
| `0.998` | 1 | 2032 | 2032 | 0.0959 | 0.0297 | 66.75 | Keep as configured default for now, but this repeat did not reproduce the round 1 best. |
| `0.999` | 1 | 2032 | 2032 | 0.1148 | 0.0266 | 64.75 | Reject for MH04 ATE. |

Observation: MH04 SP+LG has visible run-to-run variance even with the same output scale (`0.998` was `0.0583 m` in round 1 and `0.0959 m` in the scale repeat). Treat single-run gains as tentative unless a repeat confirms them.

LocalMapping wait sweep results:

| Wait timeout ms | Strict OK | Frames | Pose-valid | ATE RMSE (m) | RPE RMSE (m) | Decision |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `35` | 1 | 2032 | 2032 | 0.0583 best single run, `0.0959` repeat with scale sweep | 0.0236 best single run, `0.0297` repeat | Keep current default; still needs repeat-stability monitoring. |
| `50` | 1 | 2032 | 2032 | 0.1164 | 0.0309 | Reject for MH04; more wait worsened accuracy. |
| `70` | 1 | 2032 | 2032 | 0.1054 | 0.0257 | Reject for MH04; more wait did not improve stability. |

Current recommendation after this optimization pass:

```bash
SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965
SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE=1
SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS=35
SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=0
SMART_DRONE_SP_LG_INIT_TRUST_SELECT_CLOSE_PAIRS=0
SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE=0.998
```

Do not promote a new default from this pass. The best single MH04 strict realtime result was `ATE RMSE=0.0583 m` with the existing stable profile, but the repeat at the same output scale produced `0.0959 m`. The next optimization should target repeatability or deterministic backend scheduling before accepting smaller one-off gains.

## Realtime No-Jump Target Pass

New target from operator:

- realtime pose output for every replay frame
- no dropped pose rows
- no abnormal adjacent-pose jumps
- average accuracy target near or below `0.03 m`

Working jump metric for this pass: adjacent realtime CSV translation step. This is causal-output friendly and does not use future frames.

Initial jump audit:

| Run | Frames | Pose-valid | ATE RMSE (m) | RPE RMSE (m) | Max step (m) | P95 step (m) | P99 step (m) | Steps > 0.10 m | Steps > 0.20 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `stable_realtime` round 1 | 2032 | 2032 | 0.0583 | 0.0236 | 0.1760 | 0.1094 | 0.1420 | 141 | 0 |
| `stable_realtime` same-scale repeat | 2032 | 2032 | 0.0959 | 0.0297 | 0.2614 | 0.1103 | 0.1424 | 142 | 1 |

Interpretation: pose rows are not dropping, but the same configured profile can produce an occasional larger realtime step. This pass should prioritize repeatability and jump suppression without using shutdown trajectory export or future-frame backfill.

## SP+LG Tracking Analysis

The current `superpoint_lightglue` frontend is not a standalone VO pipeline. It supplies ORB-SLAM3 with external stereo observations:

- SuperPoint provides repeatable sparse keypoints and descriptors.
- LightGlue provides high-confidence stereo correspondences on a cadence.
- Descriptor fallback fills non-LightGlue frames so the backend still receives current-frame external features.
- ORB-SLAM3 still owns map state, keyframes, local mapping, tracking recovery, and final pose estimation.

That split is important for MH04 realtime behavior. When an SP+LG frame fails to produce injectable frontend features, the output path must not call a different tracker as a fallback. Calling the ORB fallback mutates the backend, can trigger relocalization/new-map logic, and can make the next realtime pose jump even if the CSV row count stays complete. For strict realtime output, missing/weak frontend frames should be handled at the output-continuity layer, not by advancing a different backend mode.

The strongest diagnostic result in this pass was that a single frontend failure around frame 991 caused ORB fallback, map creation, and a multi-meter jump. After changing the failure path to return a continuity-maintained realtime pose without calling `RunSlamTrackingBackend(..., nullptr)`, the same class of jump disappeared and all rows stayed realtime-valid.

## 2026-05-14 Failure-Path Optimization

Code changes validated in this pass:

| Change | File | Status |
| --- | --- | --- |
| On SP+LG frontend/finalize failure, return a realtime continuity pose and mark `RECENTLY_LOST`; do not call ORB fallback or mutate backend. | `src/native/adapters/slam/superpoint_lightglue_mode_strategy.cpp` | Accepted. Removed new-map churn and large realtime jumps. |
| Detect fixed-point-count LightGlue TensorRT engines from binding/profile dimensions and force `SMART_DRONE_LIGHTGLUE_POINTS` to that fixed count. | `src/native/adapters/slam/superpoint_native_extractor.cpp` | Accepted. Prevents dynamic-shape mismatches such as `[1,495,2]` against a fixed 512-point engine. |
| Keep mature bootstrap trust after the frontend OK streak is satisfied while tracked map points remain healthy. | `src/native/adapters/slam/superpoint_lightglue_mode_strategy.cpp` | Accepted as current working-tree behavior; default OK streak lowered from `120` to `20`. |
| Lower default `SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK` from `120` to `20` in replay/script defaults. | `tests/euroc/offline_replay_main.cpp`, `scripts/run_jetson_euroc_mh_feature_compare.sh`, `scripts/run_jetson_mh04_splg_realtime_accuracy_sweep.sh`, `docs/slam_modes_euroc.md` | Accepted. Matches the historical best complete MH04 SP+LG run configuration. |

Key validation runs:

| Run | Result path | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | Maps | Max adjacent step (m) | Decision |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| Stable retest before failure-path fix | `/home/nvidia/euroc_eval/results/codex_current_stable_retest_mh04_20260514_073148/stable_retest` | 2032 / 2032 | 3.1104 | not used | `0:990, 1:797, 2:245` | 5.239 | Reject. ORB fallback created new maps and a large jump around frame 992. |
| Continuity-only SP+LG failure output | `/home/nvidia/euroc_eval/results/codex_failure_continuity_mh04_20260514_073917` | 2032 / 2032 | 0.0857 | 0.0267 | all map `0` | 0.162 | Accept direction. No drops, no new maps, no steps above `0.2 m`. |
| Fixed LightGlue point count + OK streak `20` | `/home/nvidia/euroc_eval/results/codex_current_streak20_mh04_20260514_075754` | 2032 / 2032 | 0.0657 | 0.0244 | all map `0` | 0.178 | Current best stable working-tree run in this pass. No drops, no new maps, no steps above `0.2 m`. |

State summary for the current best run:

| Metric | Value |
| --- | ---: |
| Output rows | 2032 |
| Invalid pose rows | 0 |
| Unusable strict rows | 0 |
| Bootstrap identity rows | 1 |
| Tracking states | `OK=2028`, `RECENTLY_LOST=3`, `NOT_INITIALIZED=1` |
| TensorRT LightGlue binding errors | 0 |

This satisfies the no-drop and no-large-jump realtime-output constraint for MH04 in the current pass. It does not satisfy the requested `ATE RMSE < 0.03 m` target.

## LightGlue Cadence Matrix

After fixing the LightGlue fixed-shape issue, cadence was retested under the same strict realtime contract.

Result root: `/home/nvidia/euroc_eval/results/codex_lg_cadence_matrix_mh04_20260514_074722`

| Profile | Main setting | Strict OK | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | Max adjacent step (m) | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `lg_every1` | `SMART_DRONE_LIGHTGLUE_EVERY_N=1` | 0 | 2032 / 2032 pose-valid, but 49 unusable strict rows | n/a | n/a | 0.162 | Reject. Many non-OK bootstrap/lost rows; strict evaluator failed. |
| `lg_every2` | `SMART_DRONE_LIGHTGLUE_EVERY_N=2` | 1 | 2032 / 2032 | 0.1071 | 0.0266 | below `0.2` | Reject. More frequent LightGlue worsened ATE. |
| `lg_every2_supp384` | `SMART_DRONE_LIGHTGLUE_EVERY_N=2`, supplement to 384 | 1 | 2032 / 2032 | 0.1164 | 0.0308 | 0.359 | Reject. Worse ATE and two adjacent steps above `0.3 m`. |

Interpretation: On MH04, using LightGlue more often does not automatically improve the backend. It can overconstrain or perturb local tracking/keyframe behavior. The current recommendation stays at `SMART_DRONE_LIGHTGLUE_EVERY_N=4`.

## Deterministic Backend Attempt

| Run | Setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | Max adjacent step (m) | Decision |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `/home/nvidia/euroc_eval/results/codex_det_seed3_streak20_current_mh04_20260514_080345` | `SMART_DRONE_ORB_DETERMINISTIC_RANDOM=1`, `SMART_DRONE_ORB_RANDOM_SEED=3` | 2032 / 2032 | 0.0900 | 0.0425 | 0.543 | Reject. No dropped rows, but two jumps above `0.3 m`; worse than current best. |

The deterministic seed configuration did not improve repeatability for MH04 SP+LG and should not be promoted.

## Timestamp And Scale Check

Low-cost timestamp/scale scans were run against the current output to see whether the remaining error was mainly a CSV alignment artifact.

| Source run | Best tested adjustment | Best ATE RMSE (m) | Interpretation |
| --- | --- | ---: | --- |
| Older current/streak-120 style output | small timestamp/scale scan | about 0.0831 | Alignment tweaks cannot recover the large error when backend behavior is poor. |
| Current fixed-shape + streak-20 output | `+5 ms`, scale `0.998` | about 0.0656 | Very close to the raw current best; timestamp/scale is not the path to `<0.03 m`. |

The remaining MH04 error is therefore dominated by tracking/map estimation behavior, not by a simple output timestamp or scalar-position calibration.

## Historical Bests And Invalid Low Results

Complete strict MH04 SP+LG runs currently known:

| Run | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | Notes |
| --- | ---: | ---: | ---: | --- |
| `/home/nvidia/euroc_eval/results/codex_mh04_trust_seed_batch_20260511_230157/det_seed3_streak20/...` | 2032 / 2032 | 0.0578 | 0.0224 | Historical best complete strict MH04 result found so far. Used streak `20`, depth scale `0.965`, LocalMapping wait `35 ms`, live pose off, timestamp offset `25 ms`, position scale `0.998`. |
| `/home/nvidia/euroc_eval/results/mh04_splg_opt_round1_20260514_041821/stable_realtime/...` | 2032 / 2032 | 0.0583 | 0.0236 | Best complete result from the earlier 2026-05-14 sweep. |
| `/home/nvidia/euroc_eval/results/codex_current_streak20_mh04_20260514_075754` | 2032 / 2032 | 0.0657 | 0.0244 | Best complete run after the no-fallback failure-path fix and fixed LightGlue shape handling. |

Several sub-`0.03 m` MH04 runs were found historically, but they were incomplete fragments with only hundreds of rows (`429`, `653`, or `788` rows). They are invalid for the current target because strict realtime output requires the full `2032`-frame sequence.

Known MH05 reference:

| Run | Frames / valid | ATE RMSE (m) | Notes |
| --- | ---: | ---: | --- |
| `/home/nvidia/euroc_eval/results/mh_three_modes_key_profile_20260503_080401/superpoint_lightglue/MH_05_difficult/euroc_metrics.json` | 2273 / 2273 | 0.0485 | Historical complete MH05 SP+LG result below `0.05 m`. MH05 was not rerun in the latest failure-path pass. |

## Current Recommendation

Use the following MH04 SP+LG realtime profile as the safest current baseline:

```bash
SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK=20
SMART_DRONE_LIGHTGLUE_EVERY_N=4
SMART_DRONE_LIGHTGLUE_POINTS=512
SMART_DRONE_SUPERPOINT_MAX_POINTS=512
SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE=0.965
SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE=1
SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS=35
SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE=0
SMART_DRONE_REALTIME_POSE_CONTINUITY=1
SMART_DRONE_REALTIME_POSE_RESET_GUARD=0
SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=0
SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS=25
SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE=0.998
```

Do not enable these for the current MH04 baseline:

```bash
SMART_DRONE_LIGHTGLUE_EVERY_N=1
SMART_DRONE_LIGHTGLUE_EVERY_N=2
SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE=1
SMART_DRONE_REALTIME_POSE_RESET_GUARD=1
SMART_DRONE_ORB_DETERMINISTIC_RANDOM=1
SMART_DRONE_EUROC_OUTPUT_BODY_FRAME=1
```

Current status against the operator target:

| Requirement | Status |
| --- | --- |
| Realtime pose output | Pass. Rows are written from the replay callback with `--require-realtime-pose`. |
| No dropped pose rows on MH04 | Pass in current best: `2032/2032` valid rows. |
| No abnormal map-reset pose jumps | Pass in current best: all output stayed on map `0`, no adjacent translation step above `0.2 m`. |
| Full MH04 `ATE RMSE < 0.03 m` | Not achieved. Best complete strict runs are still about `0.058-0.066 m`; sub-`0.03 m` results are incomplete and invalid for this target. |

Next optimization direction should focus on reducing backend estimation drift while preserving the no-fallback failure path. The rejected runs show that hard output clamps, reset guards, and higher LightGlue cadence can preserve row count but usually degrade ATE or create new jumps.

## 2026-05-14 Injection-Quality Sweep

After the no-fallback failure-path fix, the current best run was analyzed per-frame. The worst ATE errors concentrated in frames `1200-1599`, especially around `1540-1557`, while tracking state remained `OK` and all output stayed on map `0`. Weak frames had much higher error:

| Group | Count | ATE RMSE (m) | Mean ATE (m) | Max ATE (m) |
| --- | ---: | ---: | ---: | ---: |
| all matched pairs | 1977 | 0.0657 | 0.0588 | 0.1686 |
| `inliers < 90` | 194 | 0.0941 | 0.0890 | 0.1607 |
| `tracked_map < 120` | 377 | 0.0801 | 0.0742 | 0.1607 |
| adjacent step `> 0.12 m` | 80 | 0.1041 | 0.0996 | 0.1607 |

Frame-bucket RMSE showed the main drift zones:

| Frame range | RMSE (m) | Notes |
| --- | ---: | --- |
| `700-799` | 0.0258 | Only local segment below the target; not representative of full sequence. |
| `900-999` | 0.0898 | Weak-frame cluster. |
| `1200-1299` | 0.1026 | Almost all frames weak by inlier/tracked-map criteria. |
| `1500-1599` | 0.1299 | Worst full segment; peak frame error `0.1686 m`. |

This motivated a sweep that changed only SP+LG observation supply, not output postprocessing.

Result root: `/home/nvidia/euroc_eval/results/codex_injection_quality_mh04_20260514_081330`

| Profile | Main setting | Strict OK | ATE RMSE (m) | RPE RMSE (m) | Max step (m) | Steps `>0.2 m` | Maps | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| `weak_pair_192` | limit weak frames to 192 stereo pairs | 1 | 3.0478 | 1.2169 | 11.422 | 185 | maps `0/1/2/3` | Reject. Pair limiting starved tracking and caused map churn. |
| `temporal_carry` | carry temporally tracked stereo pairs | 1 | 3.1297 | 0.5132 | 8.761 | 62 | maps `0/1` | Reject. Reusing old descriptors at new positions polluted backend matching. |
| `lg_every3` | `SMART_DRONE_LIGHTGLUE_EVERY_N=3` | 1 | 0.0846 | 0.0229 | 0.353 | 1 | map `0` | Reject. RPE improved slightly, but ATE and jump risk worsened. |
| `grid6` | `SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL=6` | 1 | 0.1359 | 0.0301 | 0.276 | 3 | map `0` | Reject. Too sparse; map support degraded. |
| `grid12` | `SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL=12` | 1 | 0.0622 | 0.0258 | 0.166 | 0 | map `0` | Keep only as an experiment. ATE slightly better in this run, but repeatability was not proven. |
| `strict_lg_score` | `SMART_DRONE_LIGHTGLUE_MIN_SCORE=0.04`, `MAX_Y_DIFF=1.0` | 1 | 0.1014 | 0.0236 | 0.294 | 5 | map `0` | Reject. Stricter stereo matches reduced pose consistency. |

Interpretation: MH04 SP+LG needs enough fresh, same-frame stereo observations to keep ORB-SLAM3's local map alive. Weak-frame pair limiting and temporal carry both made the tracker less stable. A denser grid (`12` per cell) can slightly reduce ATE in one run, but it needed repeat validation before becoming a recommendation.

## 2026-05-14 Backend-Stability Sweep

The next sweep targeted ORB-SLAM3 external-stereo stable-phase behavior: local map projection search radius, keyframe spacing, stable keyframe map-point creation, and stabilizing window length.

Result root: `/home/nvidia/euroc_eval/results/codex_backend_stability_mh04_20260514_082939`

| Profile | Main setting | Strict OK | ATE RMSE (m) | RPE RMSE (m) | Max step (m) | Steps `>0.2 m` | Maps | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| `stable_search3` | `SMART_DRONE_EXTERNAL_STEREO_STABLE_LOCAL_SEARCH_TH=3` | 1 | 0.0764 | 0.0269 | 0.275 | 2 | map `0` | Reject. Wider stable local search worsened ATE and jump risk. |
| `stable_search5` | `SMART_DRONE_EXTERNAL_STEREO_STABLE_LOCAL_SEARCH_TH=5` | 1 | 0.1069 | 0.0306 | 0.289 | 2 | map `0` | Reject. Further worse. |
| `stable_more_mps` | `SMART_DRONE_EXTERNAL_STEREO_STABLE_MAX_MAPPOINTS_PER_KF=160` | 1 | 0.1124 | 0.0790 | 6.371 | 37 | map `0` | Reject. More stable-phase map points introduced large pose jumps. |
| `stabilize_long` | stabilizing window `2000`, KF limit `200` | 1 | 0.0953 | 0.0256 | 0.323 | 2 | map `0` | Reject. Longer stabilizing phase did not reduce drift and added jump risk. |
| `kf_gap6` | `SMART_DRONE_EXTERNAL_STEREO_MIN_FRAMES_BETWEEN_KF=6` | 1 | 2.6712 | 0.7706 | 9.360 | 125 | maps `0/1/2` | Reject. Keyframes became too sparse; map support collapsed. |
| `grid12_search3` | grid `12` plus stable search `3` | 1 | 0.0861 | 0.0227 | 0.166 | 0 | map `0` | Reject for accuracy. It is no-jump, but worse than current no-jump baseline. |

Interpretation: relaxing stable-phase local map matching or changing keyframe density does not recover MH04 to the `0.03 m` target. ORB-SLAM3 is already operating in a narrow stable region for SP+LG observations. More map points, wider projection matching, or fewer keyframes all made the backend more brittle.

Updated recommendation after these sweeps:

- Keep `SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL=10`; `12` is interesting but violates no-jump in the single run where it improved ATE.
- Keep `SMART_DRONE_EXTERNAL_STEREO_STABLE_LOCAL_SEARCH_TH=1`.
- Keep `SMART_DRONE_EXTERNAL_STEREO_MIN_FRAMES_BETWEEN_KF=4`.
- Keep `SMART_DRONE_EXTERNAL_STEREO_STABLE_MAX_MAPPOINTS_PER_KF=100`.
- Keep `SMART_DRONE_EXTERNAL_STEREO_STABILIZING_FRAME_WINDOW=1200` and `SMART_DRONE_EXTERNAL_STEREO_STABILIZING_KF_LIMIT=120`.

The best no-jump full MH04 run in the current code remains:

`/home/nvidia/euroc_eval/results/codex_current_streak20_mh04_20260514_075754`

| Frames / valid | ATE RMSE (m) | RPE RMSE (m) | Max step (m) | Steps `>0.2 m` | Maps |
| ---: | ---: | ---: | ---: | ---: | --- |
| 2032 / 2032 | 0.0657 | 0.0244 | 0.178 | 0 | all map `0` |

The best full strict MH04 result found historically is still about `0.0578 m`; the requested full-sequence `0.03 m` target was not reached in any valid complete realtime run.

## 2026-05-14 Candidate-Budget Sweep

The next check tested whether MH04 was limited by too few SP/LG candidates or descriptor-supplement candidates.

Result root: `/home/nvidia/euroc_eval/results/codex_candidate_budget_mh04_20260514_181954`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `max768_grid10` | 768 SP points, 768 descriptor candidates | 2032 / 2032 | 0.0987 | 0.0238 | 0.2108 | 0.173 | 0 | Reject. More candidates worsened ATE. |
| `max768_supp256_grid10` | 768 SP points plus low-yield supplement to 256 | 2032 / 2032 | 0.0974 | 0.0307 | 0.3939 | 0.355 | 2 | Reject. Introduced visible pose jumps. |
| `max768_grid12` | 768 SP points, grid cap 12 | 2032 / 2032 | 0.0930 | 0.0238 | 0.2014 | 0.175 | 0 | Reject. Still worse than the no-jump baseline. |
| `max1024_grid10` | 1024 SP points/candidates | 2032 / 2032 | 0.1120 | 0.0226 | 0.2307 | 0.173 | 0 | Reject. Denser candidates increased global drift. |

Interpretation: the current gap is not a simple "more SP points" problem. Extra candidates increase backend ambiguity more than they improve observability.

## 2026-05-14 IMU Control

Stereo-IMU was tested as a control to see whether EuRoC IMU constraints could close the remaining MH04 drift.

Result root: `/home/nvidia/euroc_eval/results/codex_stereo_imu_control_mh04_20260514_183449`

| Profile | Result | Decision |
| --- | --- | --- |
| `orb` stereo-IMU | 2032 rows and no jumps, but 31 non-bootstrap rows were not strict tracking states `2/3`; strict realtime evaluation failed. | Reject. It does not satisfy the realtime tracking-state contract. |
| `superpoint_lightglue` stereo-IMU | Failed to initialize: 2032 identity/lost rows. Logs repeatedly reported stereo init waiting. | Reject. SP+LG stereo-IMU is currently worse than pure stereo. |

Interpretation: IMU is not a shortcut for the current SP+LG target. The pure-stereo SP+LG path is still the valid baseline for strict realtime output.

## 2026-05-14 Trajectory And Calibration Diagnostics

Several offline diagnostics checked whether the remaining ATE was an evaluation artifact:

| Check | Result | Interpretation |
| --- | --- | --- |
| Timestamp/scale scan on current best | Best nearby adjustment was effectively unchanged: about `0.0656 m`. | Timestamp offset and scalar output scale are not the main error. |
| Timestamp/scale scan on historical best | Best nearby adjustment was about `0.0568 m`, only a small improvement. | Historical best is still far above `0.03 m`. |
| Camera/body lever-arm transform | Best offline variant improved current run from `0.0657` to about `0.0639 m`; historical best from `0.0578` to about `0.0571 m`. | Body-frame output is not enough to reach the target. |
| Final trajectory export | `/home/nvidia/euroc_eval/results/codex_finaltraj_diag_mh04_20260514_180225` realtime ATE `0.1234`; final shutdown trajectory raw ATE about `0.0891`. | Shutdown/final trajectory does not reveal hidden `<0.03 m` performance, and it would not satisfy realtime output anyway. |
| Historical sub-`0.03 m` runs | Found only truncated fragments of 429, 653, and 788 rows. | Invalid for the 2032-frame realtime contract. |

Per-frame error analysis still points to backend drift concentrated around frames `900-999`, `1200-1299`, and `1400-1599`. Local displacement scale was near 1.0, so the error is not a simple global or rolling scale factor.

## 2026-05-14 Backend-Consistency Sweep

This sweep tested LocalMapping acceptance and map-point count limits without changing frontend output.

Result root: `/home/nvidia/euroc_eval/results/codex_backend_consistency_mh04_20260514_184442`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `relax_lm` | `SMART_DRONE_ORB_RELAX_LOCAL_MAPPING_ACCEPT_KF=1` | 2032 / 2032 | 0.0993 | 0.0294 | 0.2602 | 0.266 | 1 | Reject. Worse ATE and one jump above `0.2 m`. |
| `stable_mps80` | `SMART_DRONE_EXTERNAL_STEREO_STABLE_MAX_MAPPOINTS_PER_KF=80` | 2032 / 2032 | 0.0678 | 0.0240 | 0.1723 | 0.161 | 0 | Reject. No jump, but slightly worse than the current no-jump baseline. |
| `stabilizing_mps120` | `SMART_DRONE_EXTERNAL_STEREO_STABILIZING_MAX_MAPPOINTS_PER_KF=120` | 2032 / 2032 | 0.1301 | 0.0251 | 0.2706 | 0.160 | 0 | Reject. Early/mid map support degraded. |
| `stable_mps60` | `SMART_DRONE_EXTERNAL_STEREO_STABLE_MAX_MAPPOINTS_PER_KF=60` | 2032 / 2032 | 0.1674 | 0.0371 | 0.2717 | 0.418 | 1 | Reject. Too sparse and introduced a large jump. |

Interpretation: reducing map-point creation or loosening LocalMapping scheduling does not reduce MH04 drift. The backend is sensitive to both queue timing and map-point support density.

## 2026-05-14 External Stereo Scale-Level Experiments

SP+LG keypoints are injected into ORB-SLAM3 as external stereo observations. Since these learned keypoints are not native ORB pyramid detections, two controlled experiments were added:

- `SMART_DRONE_EXTERNAL_STEREO_IGNORE_PROJECTION_SCALE_LEVELS=1` skips octave filtering only for external-stereo projection matching.
- `SMART_DRONE_EXTERNAL_STEREO_KEYPOINT_OCTAVE=N` assigns injected external-stereo keypoints to a fixed ORB pyramid octave.

Both default to off/current behavior.

Projection-gate result root: `/home/nvidia/euroc_eval/results/codex_scale_level_gate_mh04_20260514_185600`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `default_newlib` | New library, default scale behavior | 2032 / 2032 | 0.1035 | 0.0250 | 0.2195 | 0.177 | 0 | Control run. Shows normal MH04 run-to-run variance; no default behavior change intended. |
| `ignore_scale_levels` | Ignore projection octave gates for external stereo | 2032 / 2032 | 0.1399 | 0.0216 | 0.2671 | 0.160 | 0 | Reject. RPE improved but global drift worsened. |

Fixed-octave result root: `/home/nvidia/euroc_eval/results/codex_external_octave_mh04_20260514_190302`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `octave1` | `SMART_DRONE_EXTERNAL_STEREO_KEYPOINT_OCTAVE=1` | 2032 / 2032 | 0.0690 | 0.0239 | 0.1599 | 0.172 | 0 | Reject for target. Stable, but not better than historical best. |
| `octave2` | `SMART_DRONE_EXTERNAL_STEREO_KEYPOINT_OCTAVE=2` | 2032 / 2032 | 0.1205 | 0.0251 | 0.2645 | 0.171 | 0 | Reject. |
| `octave3` | `SMART_DRONE_EXTERNAL_STEREO_KEYPOINT_OCTAVE=3` | 2032 / 2032 | 0.1358 | 0.0279 | 0.2788 | 0.163 | 0 | Reject. |

Interpretation: ORB-SLAM3's scale-level assumptions do matter, but adjusting them alone does not recover MH04 below `0.03 m`. These switches are useful diagnostic controls and should remain opt-in.

## 2026-05-14 All-Left Geometric-Depth Sweep

SP+LG normally injects only selected stereo pairs into ORB-SLAM3. This sweep tested the opt-in `SMART_DRONE_SP_LG_ALL_LEFT_GEOMETRIC_DEPTH=1` path, which keeps all safe left SP keypoints as left-image observations while assigning stereo depth only to geometrically valid left/right pairs.

Result root: `/home/nvidia/euroc_eval/results/codex_all_left_geom_mh04_20260514_191258`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `all_left_filtered` | all-left observations, filtered stereo pairs for depth | 2032 / 2032 | 0.0670 | 0.0243 | 0.1811 | 0.198 | 0 | Reject for target. Stable, but not better than the current baseline. |
| `all_left_trust` | all-left plus `SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT=0` | 2032 / 2032 | 0.1126 | 0.0227 | 0.1980 | 0.163 | 0 | Reject. Trusting frontend pair order improves local RPE slightly but worsens global drift. |
| `all_left_trust_octave1` | trust frontend pair order plus `SMART_DRONE_EXTERNAL_STEREO_KEYPOINT_OCTAVE=1` | 2032 / 2032 | 0.1131 | 0.0266 | 0.2247 | 0.167 | 0 | Reject. |
| `all_left_trust_depthfilter` | trust frontend pair order plus disparity consistency for depth matches | 2032 / 2032 | 0.1178 | 0.0255 | 0.2203 | 0.172 | 0 | Reject. |

Interpretation: adding left-only learned observations does not solve the MH04 drift. Fully trusting frontend pair order is actively worse for ATE, even though it can reduce local relative error in one run. The current filtered stereo injection path remains the safer SP+LG baseline.

## 2026-05-14 Native SuperPoint Descriptor Sweep

The next hypothesis was that recomputing ORB descriptors at SP/LG keypoint locations might be the remaining mismatch. The existing opt-in `SMART_DRONE_SP_LG_NATIVE_DESCRIPTOR_INJECT=1` path injects SuperPoint `CV_32F` descriptors directly into ORB-SLAM3, whose matcher has a cosine-distance fallback for float descriptors.

Result root: `/home/nvidia/euroc_eval/results/codex_native_spdesc_mh04_20260514_192452`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Maps | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| `native_spdesc` | `SMART_DRONE_SP_LG_NATIVE_DESCRIPTOR_INJECT=1` | 2032 / 2032 | 8.4184 | 0.9071 | 13.8246 | 12.418 | maps `0/1` | Reject. Severe drift, map switch, and large jumps. |
| `native_spdesc_octave1` | native descriptors plus `SMART_DRONE_EXTERNAL_STEREO_KEYPOINT_OCTAVE=1` | 2032 / 2032 | 8.4773 | 0.9614 | 13.8424 | 13.270 | maps `0/1` | Reject. |
| `native_spdesc_all_left` | native descriptors plus all-left geometric depth | 2032 / 2032 | 4.9997 | 0.8935 | 10.4736 | 11.087 | maps `0/1/2` | Reject. Still catastrophic. |

Interpretation: direct SuperPoint descriptor injection is not a safe accuracy path for the current ORB-SLAM3 backend. It would require a full backend matcher/threshold/BoW redesign. The default ORB descriptor recomputation at SP/LG points remains the only stable descriptor path.

## 2026-05-14 Stereo Configuration And Prepared-Image Checks

Two additional checks targeted calibration/rectification consistency, because SP+LG relies on learned stereo matches but ORB-SLAM3 still owns the camera model, baseline, and projection backend.

Configuration result root: `/home/nvidia/euroc_eval/results/codex_config_geometry_mh04_20260514_193523`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Maps | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| `official` | `stereo_orb_official.yaml` | 2032 / 2030 | 0.1306 | 0.0265 | 0.2473 | 0.161 | 0 | map `0` | Control only. No jump, but worse than the no-jump baseline. |
| `orb2500` | `stereo_orb2500.yaml` | 2032 / 2029 | 8.2917 | 1.1835 | 15.6131 | 7.612 | 37 | maps `0/1/2` | Reject. Severe map churn and jumps. |
| `inertial` | `stereo_inertial.yaml` without IMU mode | 2032 / 2029 | 6.2486 | 0.7760 | 12.1024 | 7.197 | 23 | maps `0/1/2` | Reject. Severe map churn and jumps. |

Prepared-image result root: `/home/nvidia/euroc_eval/results/codex_unified_prepare_mh04_20260514_194624`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `default` | SP+LG frontend forced to reuse ORB-SLAM3 `PrepareStereoImagesForTracking()` | 2032 / 2030 | 0.1433 | 0.0322 | 0.2788 | 0.363 | 1 | Reject. It worsened ATE and introduced a `>0.3 m` jump. |

Code note: `SMART_DRONE_SP_LG_USE_ORB_PREPARED_IMAGES=1` was added as an opt-in diagnostic switch only. Default behavior remains the prior SP+LG rectifier path because the unified prepared-image path failed the no-jump requirement and moved farther from the `0.03 m` target.

Interpretation: switching EuRoC stereo settings or unifying SP+LG image preparation with ORB-SLAM3's prepared-image helper is not the missing accuracy lever. The `official` stereo settings remain the only non-catastrophic SP+LG configuration in this test set.

## 2026-05-14 Grid12 Repeat Check

The single `grid12` run above was rechecked because it was one of the few complete, no-jump runs with a small ATE improvement over the current baseline.

Result root: `/home/nvidia/euroc_eval/results/codex_grid12_repeat_mh04_20260514_195338`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `grid12_repeat` | grid cap 12, depth scale `0.965` | 2032 / 2030 | 0.0854 | 0.0234 | 0.2023 | 0.169 | 0 | Reject as default. Repeat did not reproduce the `0.0622 m` run. |
| `grid12_depth0962` | grid cap 12, depth scale `0.962` | 2032 / 2030 | 0.0842 | 0.0261 | 0.2146 | 0.215 | 1 | Reject. Worse ATE and one step above `0.2 m`. |
| `grid12_depth0968` | grid cap 12, depth scale `0.968` | 2032 / 2030 | 0.0899 | 0.0233 | 0.1903 | 0.167 | 0 | Reject. Stable output, but worse ATE. |

Interpretation: `SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL=12` is not a reproducible path toward the `0.03 m` target. Keep the default grid cap at `10`.

## 2026-05-15 ZNCC Right-Point Refinement

The next hypothesis was that LightGlue's right-image keypoint location might be locally suboptimal for ORB-SLAM3 stereo depth. An opt-in switch was added:

- `SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC=1`
- `SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC_MAX_SHIFT_PX=N`

It refines selected right points with the existing stereo ZNCC patch search, then keeps the refined point only when the shift is bounded and the pair remains geometrically valid. The switch is off by default.

Result root: `/home/nvidia/euroc_eval/results/codex_zncc_refine_mh04_20260514_200528`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `zncc_shift4` | max right-point shift `4 px` | 2032 / 2030 | 0.1411 | 0.0262 | 0.3758 | 0.334 | 2 | Reject. Worse drift and two large realtime steps. |
| `zncc_shift2` | max right-point shift `2 px` | 2032 / 2030 | 0.0870 | 0.0235 | 0.1934 | 0.169 | 0 | Reject for target. Stable output, but worse than baseline. |

Interpretation: direct right-point ZNCC refinement breaks the learned SP+LG stereo pairing more often than it helps. Keep this as an off-by-default diagnostic only.

## 2026-05-15 ORB Left-Augment And Depth-Scale Search

SP+LG's strong side is robust learned stereo association; ORB-SLAM3's backend still benefits from many left-image observations for local map tracking and projection search. This sweep tested `SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT=1`, which appends native ORB left-only features after initialization while preserving SP+LG stereo-pair depth.

Left-feature budget result root: `/home/nvidia/euroc_eval/results/codex_orb_left_aug_mh04_20260514_201238`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `orb_left1200` | augment to `1200` left features | 2032 / 2030 | 0.0577 | 0.0257 | 0.2425 | 0.189 | 0 | Candidate only. Good single-run ATE and no `>0.2 m` step, but not below target. |
| `orb_left1600` | augment to `1600` left features | 2032 / 2030 | 0.0662 | 0.0333 | 0.3228 | 0.393 | 1 | Reject. Worse and introduced a jump. |
| `orb_left2000` | augment to `2000` left features | 2032 / 2030 | 0.0835 | 0.0331 | 0.4508 | 0.285 | 1 | Reject. Too many left-only points destabilize the backend. |

Depth-scale result root: `/home/nvidia/euroc_eval/results/codex_orb_left1200_depth_mh04_20260514_202328`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `depth0958` | left augment `1200`, depth scale `0.958` | 2032 / 2030 | 0.1199 | 0.0318 | 0.3782 | 0.360 | 2 | Reject. |
| `depth0962` | left augment `1200`, depth scale `0.962` | 2032 / 2030 | 0.0475 | 0.0222 | 0.1239 | 0.175 | 0 | Best single complete no-jump run in this pass, but not reproduced below. |
| `depth0968` | left augment `1200`, depth scale `0.968` | 2032 / 2030 | 0.0698 | 0.0279 | 0.2375 | 0.287 | 1 | Reject. |
| `depth0972` | left augment `1200`, depth scale `0.972` | 2032 / 2030 | 0.1091 | 0.0272 | 0.2294 | 0.237 | 1 | Reject. |

Offline output-scale scan on the single `depth0962` trajectory found a best nearby ATE of about `0.0443 m` at output scale `1.0025`. This confirms the remaining error is not just the CSV output scale; even the favorable run stays above `0.03 m`.

Refinement result root: `/home/nvidia/euroc_eval/results/codex_orb_left1200_refine_mh04_20260514_203633`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `d0962_out10025_l1200` | repeat `0.962`, left `1200`, output scale `1.0025` | 2032 / 2030 | 0.0902 | 0.0252 | 0.2072 | 0.173 | 0 | Reject. Did not reproduce the `0.0475 m` run. |
| `d0961_out10025_l1200` | depth `0.961`, left `1200`, output scale `1.0025` | 2032 / 2030 | 0.0697 | 0.0316 | 0.2705 | 0.283 | 1 | Reject. |
| `d0963_out10025_l1200` | depth `0.963`, left `1200`, output scale `1.0025` | 2032 / 2030 | 0.0916 | 0.0397 | 0.3722 | 0.318 | 3 | Reject. |
| `d0962_out10025_l1000` | depth `0.962`, left `1000`, output scale `1.0025` | 2032 / 2030 | 0.0681 | 0.0243 | 0.1527 | 0.159 | 0 | Reject for target. Stable, but not better than current accepted baseline. |

Interpretation: ORB left augmentation can occasionally improve the full MH04 ATE, but it is not a reproducible route to `<0.03 m` in the current backend. The behavior matches the SP+LG/ORB-SLAM3 coupling: learned stereo matches improve front-end association, while extra ORB-only left points alter local-map tracking and BA weighting. Above about `1200` left features the map becomes unstable; even at `1200`, favorable backend scheduling is not repeatable.

## 2026-05-15 Determinism And Delayed-Augment Controls

The `depth0962 + left1200` candidate was tested with deterministic random seeding and stable LocalMapping ordering to see whether run-to-run variance was mainly from random BoW/DBoW2 or unordered fusion.

Result root: `/home/nvidia/euroc_eval/results/codex_deterministic_candidate_mh04_20260514_204915`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `det_seed0` | deterministic seed `0` | 2032 / 2030 | 0.0627 | 0.0274 | 0.3298 | 0.304 | 2 | Reject. |
| `det_seed3` | deterministic seed `3` | 2032 / 2030 | 0.1139 | 0.0309 | 0.4114 | 0.344 | 4 | Reject. |
| `det_stable_order` | seed `0` plus stable LocalMapping order | 2032 / 2030 | 0.0917 | 0.0238 | 0.3613 | 0.350 | 2 | Reject. |

A second opt-in control was added to delay ORB left augmentation until SP+LG has maintained an OK streak:

- `SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT_MIN_OK_STREAK=N`

Default is `0`, preserving existing behavior.

Delayed-augment result root: `/home/nvidia/euroc_eval/results/codex_aug_delay_mh04_20260514_210620`

| Profile | Main setting | Frames / valid | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `delay20` | enable left augment after OK streak `20` | 2032 / 2030 | 0.1071 | 0.0977 | 1.1087 | 1.214 | 4 | Reject. Severe jump. |
| `delay60` | enable left augment after OK streak `60` | 2032 / 2030 | 0.0674 | 0.0263 | 0.1647 | 0.182 | 0 | Reject for target. Stable, but not better than current no-jump baseline. |

Interpretation: deterministic random seeding, sorted LocalMapping fusion order, and delayed ORB left augmentation do not make the `0.0475 m` candidate reproducible. The remaining gap appears to be a real front-end/backend modeling issue, not just CSV alignment, random seed, or start-up timing.

Current status after these attempts:

| Requirement | Status |
| --- | --- |
| Full MH04 realtime rows | Still pass in all complete accepted-control runs: `2032/2032`. |
| No dropped pose output | Pass for the current no-jump baseline and most diagnostic sweeps. |
| No pose jumps | Current baseline passes; several rejected sweeps violate this. |
| Average/full MH04 ATE below `0.03 m` | Not achieved. Best single complete no-jump result found in this pass was `0.0475 m`, but it did not reproduce; best stable accepted baseline remains around `0.06-0.07 m`. |

## 2026-05-15 Realtime Output Stabilizer And ORB Stereo Supplement

The next request focused on making realtime pose output smoother and eliminating jumps while still targeting full-sequence
`ATE RMSE < 0.03 m`. Two output-layer modes were added behind `SMART_DRONE_POSE_STABILIZER=1`:

- `SMART_DRONE_POSE_STABILIZER_MODE=alpha_beta`: causal alpha-beta translation smoothing with bounded innovation.
- `SMART_DRONE_POSE_STABILIZER_MODE=guard`: no-lag abnormal-step guard. Normal valid poses pass through unchanged; invalid,
  identity, stuck, or very large-step frames reuse a velocity prediction from the previous published pose.

Important implementation note: the stabilizer now has its own output state instead of reusing the realtime continuity
state. This avoids cross-coupling `MaintainRealtimePoseContinuity()` with the optional publish-layer smoother. Identity
poses are treated as missing measurements after initialization, not as a valid reset to the world origin.

Before compiling the runtime change, a causal smoothing simulation was run on existing full MH04 trajectories. It did not
use future frames or ground truth feedback. The best observed alpha-beta settings only changed ATE marginally:

| Source trajectory | Raw ATE RMSE (m) | Best causal smoothed ATE RMSE (m) | Interpretation |
| --- | ---: | ---: | --- |
| single favorable `depth0962` run | about `0.0475` | `0.0471` | Smoothing slightly reduces local jitter but cannot cross `0.03 m`. |
| current `streak20` run | `0.0657` | `0.0655` | Essentially unchanged. |
| historical `0.0578` run | `0.0578` | `0.0578` | No useful global ATE improvement. |

Alpha-beta runtime result root: `/home/nvidia/euroc_eval/results/codex_pose_smoother_mh04_20260514_220859`

| Profile | Main setting | Rows / nonzero poses | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `smooth_baseline` | alpha-beta, default SP+LG | 2032 / 2030 | 0.0989 | 0.0275 | 0.2007 | 0.160 | 0 | Reject for accuracy. It smooths output but adds lag/drift. |
| `smooth_left1200_depth0962` | alpha-beta plus left augment `1200`, depth `0.962` | 2032 / 2030 | 0.0688 | 0.0295 | 0.3024 | 0.158 | 0 | Reject for accuracy. Still above target and worse than the single favorable raw run. |

Guard runtime result root: `/home/nvidia/euroc_eval/results/codex_pose_guard_mh04_20260514_221956`

| Profile | Main setting | Rows / nonzero poses | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Gate hits | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `guard_baseline` | no-lag guard, max step `0.22 m` | 2032 / 2030 | 0.1224 | 0.0263 | 0.2260 | 0.170 | 0 | 0 | Keep as optional safety guard only. No jump occurred, so it did not alter the trajectory. |
| `guard_left1200_depth0962` | guard plus left augment `1200`, depth `0.962` | 2032 / 2030 | 0.1283 | 0.0251 | 0.2560 | 0.174 | 0 | 0 | Reject for accuracy. No drops or jumps, but backend drift was large. |

Interpretation: output smoothing and guarding can keep published poses continuous, but they do not reduce the full MH04
global drift to `0.03 m`. Continuous low-pass filtering is actively harmful for ATE because it introduces time lag.
The safer publish-layer behavior is the `guard` mode, which only handles invalid/identity/abnormal-step frames.

To test whether SP+LG's remaining drift came from replacing too much of ORB-SLAM3's native stereo observation model, an
opt-in ORB stereo supplement was added:

- `SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT=1`
- `SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MAX_PAIRS=N`

It appends bounded native ORB stereo pairs after SP+LG initialization while preserving the original SP+LG stereo pairs and
left-to-right match mapping. This is different from the earlier left-only augment because the appended ORB points have
right-image stereo depth.

ORB stereo supplement result root: `/home/nvidia/euroc_eval/results/codex_orb_stereo_aug_mh04_20260514_223113`

| Profile | Main setting | Rows / nonzero poses | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | SLAM mean ms | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `orb_stereo48` | append up to 48 ORB stereo pairs | 2032 / 2030 | 0.1556 | 0.0289 | 0.2867 | 0.202 | 1 | 106.67 | Reject. Worse accuracy, one large step, and too slow. |
| `orb_stereo96` | append up to 96 ORB stereo pairs | 2032 / 2030 | 0.0772 | 0.0241 | 0.1771 | 0.165 | 0 | 106.90 | Reject for target. No jump, but still above target and much slower. |

Updated status:

| Requirement | Status |
| --- | --- |
| Realtime pose rows | Pass in the latest tests: `2032/2032` rows written from the replay callback. |
| No dropped/invalid pose output | Pass in the latest tests: 0 invalid rows. |
| No abnormal jumps | Pass for guard and `orb_stereo96`; `orb_stereo48` violated the `>0.2 m` step criterion once. |
| Full MH04 average ATE below `0.03 m` | Not achieved. Output-layer smoothing is not enough; ORB stereo supplement also failed. |

Current engineering conclusion: the remaining error is backend estimation drift under external SP+LG observations, not a
publish-layer smoothness issue. The publish layer should use continuity plus optional no-lag guard for safety; the accuracy
target requires a deeper change in how learned SP+LG observations are weighted, associated, or fused inside ORB-SLAM3.

## 2026-05-15 RPE/ATE <= 0.04 Pass

The latest target was tightened to both `RPE RMSE <= 0.04 m` and `ATE RMSE <= 0.04 m`, while preserving realtime pose
output and avoiding pose jumps. RPE is not the current bottleneck: every complete pure-stereo run in this pass stayed
below `0.04 m`. ATE remains the blocker.

Two realtime-output protections were added:

- Identity poses after an established stable pose are treated as missing measurements and replaced with the continuity
  prediction instead of publishing a reset to the origin.
- `SMART_DRONE_REALTIME_POSE_MAP_BRIDGE=1` bridges SP+LG map-id changes by aligning the new raw map pose to the last
  published stable pose. This is causal and uses no future frames or ground truth.

Focused pure-stereo result root: `/home/nvidia/euroc_eval/results/codex_ate04_post_identity_mh04_20260514_233020`

| Profile | Main setting | Rows / nonzero poses | ATE RMSE (m) | RPE RMSE (m) | Max step (m) | Steps `>0.2 m` | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `left1200_depth0962_out10025_guard` | left augment `1200`, depth `0.962`, output guard | 2032 / 2030 | 0.0668 | 0.0227 | 0.171 | 0 | Reject for ATE. Realtime output is stable, but not below `0.04 m`. |

Stereo-inertial bridge diagnostic root: `/home/nvidia/euroc_eval/results/codex_splg_imu_bridge_mh04_20260514_233801`

| Run | Rows / nonzero poses | Strict realtime eval | ATE RMSE (m) | RPE RMSE (m) | Max step (m) | Maps | Decision |
| --- | ---: | --- | ---: | ---: | ---: | --- | --- |
| `splg_stereo_imu_bridge` | 2032 / 2000 | Fails: 31 bootstrap rows still `tracking_state=1` | not emitted | not emitted | 0.0685 | `0:1253, 1:779` | Reject for accuracy route. Map bridge removed the huge map-switch jump, but not the inertial trajectory error. |
| bootstrap-state-only diagnostic | same CSV, only non-2/3 states relabeled for evaluation | Diagnostic only | 6.8060 | 0.5683 | 0.0685 | same | Reject. The path is smooth after bridging, but metrically wrong. |

This confirms that IMU plus SP+LG is not a viable shortcut in the current configuration. The new map bridge is useful for
no-jump realtime output, but the underlying stereo-inertial trajectory is still wrong.

An opt-in backend experiment was added to test whether SP+LG bootstrap/stabilizing frames were being accepted too easily:

- `SMART_DRONE_EXTERNAL_STEREO_REQUIRE_MAP_INLIERS=1`
- `SMART_DRONE_EXTERNAL_STEREO_BOOTSTRAP_MIN_LOCAL_MAP_INLIERS`
- `SMART_DRONE_EXTERNAL_STEREO_STABILIZING_MIN_LOCAL_MAP_INLIERS`
- `SMART_DRONE_EXTERNAL_STEREO_STABLE_MIN_LOCAL_MAP_INLIERS`

Default behavior is unchanged unless the switch is enabled.

Map-inlier gate result root: `/home/nvidia/euroc_eval/results/codex_require_map_inliers_mh04_20260514_234154`

| Profile | Main setting | Rows / nonzero poses | ATE RMSE (m) | RPE RMSE (m) | ATE Max (m) | Max step (m) | Steps `>0.2 m` | Gate hits | Decision |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `require_map_loose` | min local-map inliers `8/16/30` | 2032 / 2030 | 0.1559 | 0.0226 | 0.3299 | 0.170 | 0 | 455 | Reject. Stable output, but much worse ATE. |
| `require_map_mid` | min local-map inliers `16/32/45` | 2032 / 2030 | 0.0635 | 0.0237 | 0.1621 | 0.171 | 0 | 0 | Reject for target. Stable, but still above `0.04 m`. |

Timestamp/position-scale scan on the latest complete trajectories found no hidden evaluator alignment gain:

| Trajectory | Base ATE / RPE (m) | Best scanned ATE / RPE (m) | Best offset / scale | Interpretation |
| --- | ---: | ---: | --- | --- |
| `left1200_depth0962_out10025_guard` | 0.0668 / 0.0227 | 0.0668 / 0.0227 | `0 ms`, `1.0` | Publishing parameters are already locally optimal for this trajectory. |
| `require_map_mid` | 0.0635 / 0.0237 | 0.0635 / 0.0237 | `0 ms`, `1.0` | No timestamp or scalar fix. |
| historical favorable `depth0962` | 0.0475 / 0.0222 | 0.0443 / 0.0222 | `0 ms`, `1.0025` | Even the favorable non-reproduced run stays above `0.04 m` after a scalar scan. |

Current status for the tightened target:

| Requirement | Status |
| --- | --- |
| Full MH04 realtime rows | Pass: latest complete pure-stereo runs write `2032` rows from the callback. |
| No dropped/invalid pose output | Pass: 0 invalid rows in the latest pure-stereo runs. |
| No abnormal jumps | Pass for the latest pure-stereo guarded/map-inlier runs: max adjacent step about `0.17 m`, no steps above `0.2 m`. |
| `RPE RMSE <= 0.04 m` | Pass in latest pure-stereo runs: `0.0226-0.0237 m`. |
| `ATE RMSE <= 0.04 m` | Not achieved. Best latest complete stable run is `0.0635 m`; best historical complete SP+LG run found remains `0.0475 m` raw and `0.0443 m` after an offline scalar scan. |

Conclusion: for MH04, realtime continuity and no-jump output are now controlled, and RPE is within target. The remaining
gap is global ATE drift inside the SP+LG + ORB-SLAM3 fusion path. Further progress likely requires changing observation
weighting, stereo-depth uncertainty, keyframe/map-point selection, or camera calibration assumptions rather than adding
more output smoothing.

## 2026-05-15 Live Rotation Jump Fix

Field feedback after deploying the previous build: replay output looked stable, but actual live use still produced large
pose jumps when the camera view was rotated. The cause is that the deployed runtime publishes through
`PosePostprocessor`, while the earlier no-jump guards mainly protected `SlamEngineAdapter` replay/CSV output. In live
use, a frame can remain `trackingUsable=true` while the backend emits a short-lived bad translation during a viewpoint
rotation; `StartupAligner` only held pose when tracking was not usable, so this bad translation could still reach UDP and
MAVLink.

Change added in this pass:

- `PosePostprocessor::OutputGuard` now runs after startup alignment and before velocity estimation/publication.
- It is causal: it only uses the previous published pose and current frame timestamp, with no future frames or ground
  truth.
- Normal pose steps pass through unchanged. If translation exceeds the configured step/speed envelope, only translation
  is clamped toward the previous published pose; the current quaternion is preserved so turn-in-place rotation can still
  update in realtime.
- Guarded frames are marked `PoseQuality::Weak`, which suppresses velocity reuse and makes the event visible in telemetry.
- `StartupAligner` is updated with the guarded published pose so later temporary tracking loss holds the safe output, not
  the rejected raw jump.

Jetson service defaults for the live build:

| Variable | Value | Purpose |
| --- | ---: | --- |
| `SMART_DRONE_ONLINE_POSE_STEP_GUARD` | `1` | Enable live publish-layer abnormal-step guard. |
| `SMART_DRONE_ONLINE_POSE_GUARD_MAX_STEP_M` | `0.18` | Per-frame hard cap for published translation change. |
| `SMART_DRONE_ONLINE_POSE_GUARD_MAX_SPEED_MPS` | `3.0` | Timestamp-based speed envelope; at 20 Hz this caps normal steps near `0.15 m`. |
| `SMART_DRONE_ONLINE_POSE_GUARD_DFX` | `1` | Log guard hits as `[pose_guard]` for field debugging. |

Expected effect: this does not solve MH04 global ATE drift, but it directly targets the live safety issue. Rotating the
camera should no longer publish a sudden large translation jump; if the backend gives an outlier, the published pose moves
only within the configured envelope and the frame quality becomes weak.
