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
| `grid12` | `SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL=12` | 1 | 0.0622 | 0.0258 | 0.272 | 4 | map `0` | Keep only as an experiment. ATE slightly better than current run, but violates no-jump target. |
| `strict_lg_score` | `SMART_DRONE_LIGHTGLUE_MIN_SCORE=0.04`, `MAX_Y_DIFF=1.0` | 1 | 0.1014 | 0.0236 | 0.294 | 5 | map `0` | Reject. Stricter stereo matches reduced pose consistency. |

Interpretation: MH04 SP+LG needs enough fresh, same-frame stereo observations to keep ORB-SLAM3's local map alive. Weak-frame pair limiting and temporal carry both made the tracker less stable. A denser grid (`12` per cell) can slightly reduce ATE in one run, but it introduced adjacent steps above `0.2 m`, so it cannot be the no-jump baseline.

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
