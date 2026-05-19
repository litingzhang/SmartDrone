# DPVO TensorRT Backend

The DPVO route is intentionally a backend replacement, not another ORB-SLAM3 feature frontend. Select it with:

```bash
--slam-backend dpvo \
--dpvo-repo /home/nvidia/DPVO \
--dpvo-patch-engine /home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine \
--dpvo-update-engine /home/nvidia/DPVO/weights/dpvo_update_fp16.engine
```

Current status as of 2026-05-15:

- The SmartDrone runtime now has a backend boundary: default `klt`, `dpvo_tensorrt`, and optional legacy `orbslam3`.
- `dpvo_tensorrt` is native C++/TensorRT only. It does not start Python or PyTorch.
- The backend loads and executes the DPVO patchifier TensorRT engine in process for both left and right rectified
  images.
- The backend loads and executes the exported DPVO update TensorRT engines in process.
- The export path now also emits split update engines around official `SoftAgg`:
  `dpvo_update_preagg_fp16.engine` and `dpvo_update_postagg_fp16.engine`.
- Native C++ now owns DPVO frame memory, patch slots, forward/back patch graph edges, initialization state,
  removal-window pruning, recurrent edge state, temporal neighbors, `SoftAgg` scatter-softmax/scatter-sum,
  correlation volume construction, SE3 pose prediction/retraction, and Schur-style BA.
- The KLT/PnP pose bridge has been removed from the DPVO path. In DPVO runs the KLT disparity/flow/PnP timers are `0`.
- Stereo right-fmap matching is used only to initialize DPVO patch inverse depth. It is not a VO replacement.
- Jetson now has generated FP16 engines under `/home/nvidia/DPVO/weights/`.
- It fails closed if the engines are missing or invalid, instead of silently falling back to a different backend.
- `smart_drone.service` now starts the default KLT backend; DPVO remains selectable by runtime config.

Important implementation note: original DPVO is not a single network that directly outputs a pose. It combines:

- a patchifier network
- an update network
- CUDA correlation
- CUDA bundle adjustment
- SE3 graph/state management

Original DPVO does not use KLT for tracking. It uses patchifier features, `altcorr` correlation, `SoftAgg` update
aggregation, `fastba`, and `lietorch.SE3` state updates. The current SmartDrone DPVO backend follows that interface, but
the native correlation/BA implementation is still CPU-side and not yet numerically matched to the official CUDA kernels
well enough for the MH04/MH05 target.

Next native steps:

1. Move DPVO `altcorr` and `fastba` to CUDA C++/TensorRT-plugin style runtime code, with no Python/Torch in production.
2. Match official `corr` flattening/interpolation and `fastba` Schur solve/retraction numerically against the DPVO CUDA
   kernels.
3. Restore the official motion-probe/keyframe/edge lifecycle without the current CPU runtime cost.
4. Pass MH04/MH05 replay and live rotation tests with no pose drops or jump discontinuities before enabling DPVO in
   `smart_drone.service`.

## TensorRT Engine Export

2026-05-15 Jetson export:

```bash
/home/nvidia/euroc_eval/scripts/export_dpvo_tensorrt.sh \
  --repo /home/nvidia/DPVO \
  --weights /home/nvidia/DPVO/dpvo.pth \
  --width 640 \
  --height 400 \
  --max-edges 4096 \
  --skip-download
```

Artifact directory:

```text
/home/nvidia/euroc_eval/results/dpvo_trt_export_20260515_043023
```

Generated files:

```text
/home/nvidia/DPVO/weights/dpvo_patchifier_640x400.onnx
/home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine
/home/nvidia/DPVO/weights/dpvo_update_core_edges4096.onnx
/home/nvidia/DPVO/weights/dpvo_update_fp16.engine
/home/nvidia/DPVO/weights/dpvo_update_preagg_edges4096.onnx
/home/nvidia/DPVO/weights/dpvo_update_preagg_fp16.engine
/home/nvidia/DPVO/weights/dpvo_update_postagg_edges4096.onnx
/home/nvidia/DPVO/weights/dpvo_update_postagg_fp16.engine
```

The first update export attempt failed on TensorRT 8.5 because the ONNX contained `LayerNormalization`, which had no
registered TensorRT importer/plugin on the Jetson. `scripts/export_dpvo_tensorrt.sh` now replaces DPVO update LayerNorm
with an equivalent mean/variance expression before ONNX export. It also carries the `GatedResidual` definition locally so
offline export does not depend on importing `dpvo.blocks` or `torch_scatter`.

TensorRT results:

| Engine | Build | Throughput | GPU compute mean |
| --- | ---: | ---: | ---: |
| `dpvo_patchifier_fp16.engine` | 219.706 s | 91.36 qps | 10.9057 ms |
| `dpvo_update_fp16.engine` | 301.362 s | 470.826 qps | 2.12103 ms |
| `dpvo_update_preagg_fp16.engine` | 201.6 s | 547.713 qps | 1.82349 ms |
| `dpvo_update_postagg_fp16.engine` | 217.596 s | 823.09 qps | 1.21304 ms |

## MH04 Replay Status

### Native DPVO TensorRT/C++ Attempts

All runs below use `--slam-backend dpvo --dpvo-repo /home/nvidia/DPVO` on Jetson, with Python/Torch absent from the
SmartDrone runtime. `native_dpvo=1` means the DPVO TensorRT patchifier/update engines, native `SoftAgg`, native
correlation, and native BA/SE3 path were active. KLT/PnP timers are `0` in these runs.

| Attempt | Frames | ATE RMSE | RPE RMSE | Mean / Max SLAM Time | Result directory | Notes |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| Native C++ initial FP16 fix | 200 | 1.4089 m | 0.4334 m | 1334 ms / n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_short_20260515_063945` | First full native path; strict realtime failed on initial stale identity rows. |
| adjSE3 + zero-boundary direct BA | 200 | 1.0571 m | 0.3349 m | 1040 ms / n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_adjfix_20260515_064727` | Matched official `fastba` adjoint sign/cross convention; improved but not converged. |
| Schur BA | 200 | 1.6698 m | 0.4460 m | n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_schur_20260515_065730` | Schur form alone was worse before bootstrap/depth fixes. |
| Schur + 12 bootstrap | 60 | 0.0910 m | 0.1320 m | n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_boot12_20260515_070451` | Mirrors official `n == 8` twelve-update initialization; still weak depth scale. |
| Stereo fmap depth init, raw dot | 60 | 0.0399 m | 0.0551 m | 1109 / 8709 ms | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_stereo_depth_60_20260515_072030` | Right fmap epipolar match fixed scale enough to lower ATE, RPE still high. |
| Stereo fmap depth init, NCC + subpixel | 60 | 0.0270 m | 0.0403 m | 1103 ms / n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_stereo_depth_ncc_60_20260515_072406` | Best early result; just above 0.04 RPE target. |
| Early update from frame 2 | 60 | 0.0531 m | 0.0699 m | n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_early_update_ncc_60_20260515_072652` | Reduced identity rows but degraded trajectory; reverted. |
| Direct BA env path | 60 | 0.0536 m | 0.0702 m | n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_direct_ba_ncc_60_20260515_073205` | Worse than Schur; left as diagnostic behind `SMART_DRONE_DPVO_DIRECT_BA=1`. |
| NCC + Schur, longer segment | 200 | 0.1534 m | 0.1586 m | 1101 ms / n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_ncc_schur_200_20260515_073520` | Short-segment accuracy does not hold over 200 frames. |
| Persistent edges, cap 1024 | 60 | 0.0193 m | 0.0275 m | 1146 / 12391 ms | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_persistent_edges1024_60_20260515_074424` | Best short metric, but long segment drifted. |
| Persistent edges, cap 1024 | 200 | 0.2572 m | 0.1617 m | n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_persistent_edges1024_200_20260515_074554` | More persistent state did not solve drift. |
| Patch-motion keyframe removal | 200 | 0.1514 m | 0.1828 m | n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_keyframe_patchmotion_200_20260515_075408` | Keyframe deletion fired too aggressively; disabled by default. |
| Stereo prior weight 0.05 | 60 | 0.0593 m | 0.1098 m | n/a | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_stereo_prior005_60_20260515_080018` | Depth prior over-constrained BA; default prior weight is `0`. |
| Default verification after reverting experimental flags | 60 | 0.0321 m | 0.0417 m | 1112 / 8878 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/base` | Current clean default: rebuilt edges, NCC stereo depth init, Schur BA, 12 bootstrap. |
| NCC stricter `min=0.08,margin=0.02` | 60 | 0.0584 m | 0.0813 m | 1109 / 8697 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ncc_strict_a` | Too few stereo depth updates (`22` vs `33`); worse. |
| NCC stricter `min=0.10,margin=0.02` | 60 | 0.0265 m | 0.0454 m | 1106 / 8687 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ncc_strict_b` | ATE improved but RPE worsened; not a stable target setting. |
| BA step loosened `0.06m/0.12rad` | 60 | 0.0316 m | 0.0410 m | 1109 / 8742 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ba_loose` | Small RPE improvement; still above 0.04. |
| Accept guard disabled | 60 | 0.0321 m | 0.0417 m | 1104 / 8694 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/accept_off` | No change; accept guard was not the limiting factor on this segment. |
| BA step `0.08m/0.16rad` | 60 | 0.0316 m | 0.0410 m | 1115 / 8753 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ba_008_016` | Same as `0.06/0.12`; clamp is no longer limiting. |
| BA step `0.10m/0.20rad` | 60 | 0.0316 m | 0.0410 m | 1116 / 8705 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ba_010_020` | Same as `0.06/0.12`; no further gain. |
| BA step `0.06/0.12` + NCC `0.10/0.02` | 60 | 0.0432 m | 0.0605 m | 1103 / 8619 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ba_006_ncc010` | Depth coverage dropped to `22` updates; worse. |
| NCC loose `min=0.00,margin=0.00` | 60 | 0.0354 m | 0.0537 m | 1112 / 8852 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ncc_loose_a` | Depth updates rose to `47`; low-confidence depths hurt RPE. |
| NCC loose `min=-0.10,margin=0.00,max_disp=44` | 60 | 0.0468 m | 0.0694 m | 1114 / 8734 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ncc_loose_b` | More permissive depth was worse. |
| `PATCHES_PER_FRAME=96` | 60 | 0.0604 m | 0.0786 m | 2176 / 17363 ms | `/home/nvidia/euroc_eval/results/dpvo_sweep_20260515_082118/ppf96_default` | More patches doubled CPU time and worsened accuracy in current native solver. |
| Persistent edges, cap 4096 | 60 | 0.2328 m | 0.3743 m | 4129 / 35427 ms | `/home/nvidia/euroc_eval/results/mh04_dpvo_native_cpp_persistent4096_60_20260515_081203` | Closer to official edge density but much worse and far from realtime on CPU. |

Interpretation:

- The KLT/PnP bridge is gone; current DPVO metrics come from the native DPVO graph path.
- Stereo fmap NCC depth initialization is necessary for the current native solver; raw random DPVO depth does not
  converge to the requested MH04 accuracy.
- Pure output smoothing was tested offline on the 200-frame baseline and did not fix drift (`ATE` remained about
  `0.135 m`, `RPE` about `0.148 m`), so the jump/drift issue must be fixed in the tracker/optimizer rather than hidden
  in the published pose stream.
- The 60-frame target can be reached in some experimental settings, but 200-frame MH04 and realtime runtime are not yet
  solved. CPU correlation/BA is the main runtime blocker.
- The current best clean 60-frame setting is `SMART_DRONE_DPVO_BA_MAX_TRANS_STEP=0.06` and
  `SMART_DRONE_DPVO_BA_MAX_ROT_STEP=0.12`, with `ATE=0.0316 m` and `RPE=0.0410 m`. This is close but does not meet the
  requested `0.04` RPE target.
- The official persistent graph cannot be restored by simply retaining more edges: the 4096-edge run is slower and
  less accurate because the current runtime lacks official global BA/keyframe handling and CUDA `fastba` scaling.
- Next implementation work should prioritize native CUDA `altcorr`/`fastba` parity and official state-machine parity
  over additional smoothing or KLT-style fallbacks.

Useful DPVO TensorRT/C++ debug controls:

| Env var | Default | Purpose |
| --- | ---: | --- |
| `SMART_DRONE_DPVO_STEREO_NCC_MIN` | `0.05` | Minimum NCC score for right-fmap stereo depth initialization. |
| `SMART_DRONE_DPVO_STEREO_NCC_MARGIN` | `0.01` | Best-vs-second-best NCC margin. |
| `SMART_DRONE_DPVO_STEREO_MAX_DISP` | `36` | Max right-fmap disparity in feature-map pixels. |
| `SMART_DRONE_DPVO_BA_MAX_TRANS_STEP` | `0.03` | Per-BA-iteration SE3 translation clamp in meters; `0` disables the clamp. |
| `SMART_DRONE_DPVO_BA_MAX_ROT_STEP` | `0.08` | Per-BA-iteration SE3 rotation clamp in radians; `0` disables the clamp. |
| `SMART_DRONE_DPVO_ACCEPT_GUARD` | `1` | Revert the newest pose if a BA step exceeds the publish guard. |
| `SMART_DRONE_DPVO_ACCEPT_MAX_TRANS` | `0.08` | Newest-pose accept guard translation threshold. |
| `SMART_DRONE_DPVO_ACCEPT_MAX_ROT` | `0.16` | Newest-pose accept guard rotation threshold. |

2026-05-15 Jetson run after adding native stereo VO pose output:

```text
/home/nvidia/euroc_eval/results/mh04_dpvo_tensorrt_pose_20260515_045517
```

Result:

| Metric | Value |
| --- | ---: |
| `REPLAY_STATUS` | `0` |
| `EVAL_STATUS` | `0` |
| `frames_out` | `2032` |
| `pose_valid_frames` | `2032` |
| `tracking_ok_frames` | `2032` |
| `identity_pose_frames` | `1` |
| `ATE RMSE` | `6.2211 m` |
| `RPE RMSE` | `0.5563 m` |
| `slam_total_ms_mean/max` | `48.5 / 126.618 ms` |

This resolves the immediate "no pose output" blocker: DPVO mode now emits a realtime pose for every MH04 replay row and
passes the strict realtime-pose evaluator gate. It does not solve accuracy. The current output is produced by a native
stereo VO bridge while the DPVO TensorRT engines are loaded; it is not the final DPVO patch/correlation/BA trajectory.

2026-05-15 Jetson run after adding the DPVO runtime state shell and real TensorRT execution:

```text
/home/nvidia/euroc_eval/results/mh04_dpvo_state_machine_20260515_051248
```

Startup confirmed that the neural pieces are now executed, not just loaded:

```text
[dpvo_trt] ready patch_engine=/home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine update_engine=/home/nvidia/DPVO/weights/dpvo_update_fp16.engine input=640x400 patches=48 opt_window=7 update_warmup_ms=4.28805
[dpvo_trt] patchifier active fmap=[1x128x100x160] imap=[1x384x100x160] ms=106.551
```

Result:

| Metric | Value |
| --- | ---: |
| `frames_out` | `2032` |
| `pose_valid_frames` | `2032` |
| `tracking_ok_frames` | `2032` |
| `tracking_lost_frames` | `0` |
| `matched_pairs` | `1978` |
| `ATE RMSE` | `6.2422 m` |
| `RPE RMSE` | `0.5566 m` |
| `slam_total_ms_mean/max` | `87.0494 / 181.515 ms` |

Interpretation: this run validates the TensorRT execution and C++ DPVO state-management scaffolding. Accuracy remains at
the bridge level because the pose still comes from stereo LK/PnP. Full DPVO accuracy requires replacing that bridge with
native correlation, update aggregation, BA, and SE3 integration.

2026-05-15 Jetson run after splitting official `SoftAgg` around the update network:

```text
/home/nvidia/euroc_eval/results/mh04_dpvo_softagg_split_20260515_055303
```

Startup confirmed all four TensorRT neural pieces were loaded and warmed up:

```text
[dpvo_trt] ready patch_engine=/home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine update_engine=/home/nvidia/DPVO/weights/dpvo_update_fp16.engine update_preagg_engine=/home/nvidia/DPVO/weights/dpvo_update_preagg_fp16.engine update_postagg_engine=/home/nvidia/DPVO/weights/dpvo_update_postagg_fp16.engine input=640x400 patches=48 opt_window=7 update_warmup_ms=3.73087 preagg_warmup_ms=2.4433 postagg_warmup_ms=0.908883 fmap_flow=0
```

Result:

| Metric | Value |
| --- | ---: |
| `frames_out` | `2032` |
| `pose_valid_frames` | `2032` |
| `tracking_ok_frames` | `2032` |
| `invalid_pose_rows` | `0` |
| `matched_pairs` | `1978` |
| `ATE RMSE` | `6.3908 m` |
| `RPE RMSE` | `0.5375 m` |
| `slam_total_ms_mean/max` | `92.8003 / 147.689 ms` |

Interpretation: the exported update engine no longer lacks the official `SoftAgg` weights; the native runtime can load
the split preagg/postagg engines. Accuracy is still not converged because the split engines are not yet connected to
native scatter aggregation, correlation, BA, and SE3 retraction.

2026-05-15 diagnostic run using patchifier `fmap` as a direct matching descriptor:

```text
/home/nvidia/euroc_eval/results/mh04_dpvo_fmapcorr_20260515_053639
```

Result:

| Metric | Value |
| --- | ---: |
| `frames_out` | `2032` |
| `pose_valid_frames` | `2032` |
| `ATE RMSE` | `6.3129 m` |
| `RPE RMSE` | `0.5744 m` |
| `slam_total_ms_mean/max` | `296.595 / 541.709 ms` |

Interpretation: raw DPVO `fmap` should not be treated as a plain local descriptor for CPU patch matching. It slowed MH04
to about 5 Hz and worsened RPE. This path is now diagnostic-only behind `SMART_DRONE_DPVO_FMAP_FLOW=1`.

2026-05-15 KLT control run:

```text
/home/nvidia/euroc_eval/results/mh04_klt_control_20260515_055816
```

Strict realtime evaluation failed because `50/2032` rows had `pose_valid!=1`. Non-strict metrics were
`ATE RMSE=6.2082 m`, `RPE RMSE=0.5565 m`. This confirms KLT is not a viable DPVO substitute; it is only a temporary
pose-output bridge.

2026-05-15 Jetson run after engine export:

```bash
/home/nvidia/euroc_eval/bin/smart_drone_offline_replay_dpvo_trt \
  --dataset /home/nvidia/euroc/machine_hall/MH_04_difficult/mav0 \
  --settings /home/nvidia/euroc_eval/config/euroc/stereo_orb2500.yaml \
  --stereo-only \
  --fps 20 \
  --slam-fps 20 \
  --slam-backend dpvo \
  --dpvo-repo /home/nvidia/DPVO
```

Artifact directory:

```text
/home/nvidia/euroc_eval/results/mh04_dpvo_tensorrt_20260515_043949
```

Result: `smart_drone_offline_replay_dpvo_trt` successfully selected `slam_backend=dpvo_tensorrt` and loaded both engines:

```text
[offline_replay] slam_backend=dpvo_tensorrt feature_frontend=lk_gftt_per_frame
[offline_replay] dpvo_repo=/home/nvidia/DPVO patch_engine= update_engine=
[dpvo_trt] ready patch_engine=/home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine update_engine=/home/nvidia/DPVO/weights/dpvo_update_fp16.engine input=640x400 patches=48 opt_window=7
frames_out: 2032
pose_valid_frames: 0
tracking_ok_frames: 2032
```

No ATE/RPE metric was produced because all `2032/2032` output rows have `pose_valid!=1`. This is now past the missing
engine blocker: the remaining blocker is the native C++/CUDA DPVO state machine. The current backend deliberately refuses
to publish placeholder poses until correlation, BA, patch graph state, and SE3 pose integration are implemented.

Earlier same-day control run:

```text
/home/nvidia/euroc_eval/results/mh04_dpvo_tensorrt_20260515_040901
```

That run failed before replay with missing engines:

```text
[dpvo_trt] missing engine(s): patch='' update='' repo='/home/nvidia/DPVO'
```

2026-05-15 Jetson native CUDA kernel integration validation:

Implemented a native CUDA/NVRTC runtime inside the C++ DPVO TensorRT backend. It dynamically loads `libnvrtc` and
`libcuda`, reuses the CUDA runtime primary context, compiles kernels for Orin `compute_72`, and launches them on the
same stream used by TensorRT. This keeps the path C++/TensorRT/CUDA-only; no Python, Torch runtime, or KLT bridge is
used.

Build/deploy:

```bash
./scripts/build.sh replay --jetson-orin-nx --jobs 16
scp output/artifacts/jetson-orin-nx/offline-replay/smart_drone_offline_replay \
  nvidia@192.168.0.103:/home/nvidia/euroc_eval/bin/smart_drone_offline_replay_dpvo_trt_native
```

Smoke-only run:

```text
/home/nvidia/euroc_eval/results/dpvo_cuda_kernel_smoke_20260515_084817
```

Result:

```text
[dpvo_cuda] native CUDA kernels ready smoke_expected=1.2226 smoke_got=1.2226
[dpvo_trt] ready ... native_cuda_kernels=1 native_dpvo=1
frames_out: 5
pose_valid_frames: 5
identity_pose_frames: 5
```

Then the same runtime was wired into the real DPVO correlation path. The C++ solver now packs live edge patch-gmaps,
reprojected 3x3 patch coordinates, target feature maps, and level-4 pooled maps, launches `dpvo_corr_batch`, and feeds
the resulting `corr` tensor into the split update preagg TensorRT engine. First live batch validation compares GPU output
against the old CPU correlation implementation.

Live correlation validation run:

```text
/home/nvidia/euroc_eval/results/dpvo_cuda_corr_validate_20260515_085658
```

Result:

```text
[dpvo_cuda] native CUDA kernels ready smoke_expected=1.2226 smoke_got=1.2226
[dpvo_trt] ready ... native_cuda_kernels=1 native_dpvo=1
[dpvo_cuda] correlation batch ready edges=720 values=635040 max_abs=0 rmse=0
frames_out: 12
pose_valid_frames: 12
identity_pose_frames: 7
```

Interpretation: Jetson native CUDA kernel loading/compilation/launch is verified both in isolation and on the real DPVO
correlation tensor path. The first GPU correlation batch is bit-identical to the CPU implementation for the validated
MH04 slice. Remaining convergence work is now beyond the correlation bridge: move SoftAgg scatter/reduction and BA/SE3
linearization off the CPU and continue tuning the DPVO graph/patch/depth state against full MH04/MH05 ATE/RPE.

60-frame MH04 sanity run with CUDA correlation enabled:

```text
/home/nvidia/euroc_eval/results/dpvo_cuda_corr_mh04_60_20260515_085744
```

Result:

| Metric | Value |
| --- | ---: |
| `frames_out` | `60` |
| `pose_valid_frames` | `60` |
| `tracking_lost_frames` | `0` |
| `identity_pose_frames` | `7` |
| first live `corr` validation | `edges=720`, `values=635040`, `max_abs=0`, `rmse=0` |
| `slam_total_ms_mean/max` | `304.655 / 1613.57 ms` |
| `lk_update_ms_mean/max` | `229.056 / 1544.25 ms` |

Interpretation: CUDA correlation does not break real-time pose publication semantics in this short MH04 slice: all
frames publish a valid pose and tracking does not drop. The update time is still too high; after correlation the measured
hot path is the CPU-side graph packing, SoftAgg scatter/reduction, TensorRT host round trips, and BA/SE3 solve.

2026-05-16 Jetson native CUDA SoftAgg and device update-chain validation:

Implemented `dpvo_softagg_expand` in the same NVRTC runtime and wired it into the real DPVO update path. The first pass
kept the previous host round trips: preagg outputs were copied to CPU, CUDA SoftAgg computed `agg_kk_y`/`agg_ij_y`, and
postagg inputs were copied back to TensorRT. This validates the SoftAgg math independently.

SoftAgg validation run:

```text
/home/nvidia/euroc_eval/results/dpvo_cuda_softagg_validate_20260515_180030
```

Result:

```text
[dpvo_cuda] correlation batch ready edges=720 values=635040 max_abs=0 rmse=0
[dpvo_cuda] softagg ready edges=720 dim=384 kk_max_abs=9.53674e-07 kk_rmse=7.1073e-08 ij_max_abs=1.19209e-06 ij_rmse=1.65577e-07
frames_out: 12
pose_valid_frames: 12
identity_pose_frames: 7
slam_total_ms_mean/max: 258.574 / 1565.07 ms
lk_update_ms_mean/max: 168.588 / 1490.36 ms
```

60-frame host-roundtrip SoftAgg sanity run:

```text
/home/nvidia/euroc_eval/results/dpvo_cuda_corr_softagg_mh04_60_20260515_180051
```

Result:

| Metric | Value |
| --- | ---: |
| `frames_out` | `60` |
| `pose_valid_frames` | `60` |
| `tracking_lost_frames` | `0` |
| `identity_pose_frames` | `7` |
| `slam_total_ms_mean/max` | `296.709 / 1583.5 ms` |
| `lk_update_ms_mean/max` | `222.632 / 1507.62 ms` |

Then the path was tightened to a device update-chain: preagg writes FP32 outputs to device buffers, CUDA SoftAgg reads
those buffers and writes device `agg_kk_y`/`agg_ij_y`, and postagg consumes those device buffers directly. Only final
`updated_net`/`delta`/`weight` are copied back to host for the current C++ state update and BA path. The chain is guarded
by `SMART_DRONE_DPVO_CUDA_DEVICE_UPDATE_CHAIN`; it falls back to the host path if binding types or validation fail.

Device-chain validation run:

```text
/home/nvidia/euroc_eval/results/dpvo_cuda_device_chain_validate_20260515_180812
```

Result:

```text
[dpvo_cuda] device update chain softagg ready edges=720 dim=384 kk_max_abs=9.53674e-07 kk_rmse=7.1073e-08 ij_max_abs=1.19209e-06 ij_rmse=1.65577e-07
frames_out: 12
pose_valid_frames: 12
identity_pose_frames: 7
slam_total_ms_mean/max: 255.039 / 1532.15 ms
lk_update_ms_mean/max: 163.983 / 1457.39 ms
```

60-frame device-chain sanity run:

```text
/home/nvidia/euroc_eval/results/dpvo_cuda_device_chain_mh04_60_20260515_180835
```

Result:

| Metric | Value |
| --- | ---: |
| `frames_out` | `60` |
| `pose_valid_frames` | `60` |
| `tracking_lost_frames` | `0` |
| `identity_pose_frames` | `7` |
| first live `corr` validation | `edges=720`, `values=635040`, `max_abs=0`, `rmse=0` |
| first live device-chain SoftAgg validation | `kk_max_abs=9.53674e-07`, `ij_max_abs=1.19209e-06` |
| `slam_total_ms_mean/max` | `291.128 / 1501.57 ms` |
| `lk_update_ms_mean/max` | `215.75 / 1427.11 ms` |

Interpretation: SoftAgg is now verified on native Jetson CUDA, and the preagg -> SoftAgg -> postagg segment can remain
on-device for FP32 TensorRT bindings. The improvement is measurable but still modest because the C++ state machine still
rebuilds graph tensors on CPU every iteration and BA/SE3 remains CPU/Eigen. The next convergence/performance target is
native BA/SE3 or reducing bootstrap iterations and graph packing overhead without sacrificing pose continuity.

## 2026-05-16 Phone DPV Mode

Goal: add a phone-side DPV/DPVO mode that actually switches the runtime backend, not a placeholder frontend entry.

Implementation:

- Extended runtime TLV config with `slamBackend(u8)` at byte `[109]`: `0=orbslam3`, `1=dpvo_tensorrt`, `2=klt`.
- Android now sends 110-byte runtime config packets and includes the backend byte after the existing ORB acceleration byte.
- The phone settings spinner label changed from `Feature Frontend` to `Tracking Mode`.
- `Tracking Mode` options now include ORB, KLT Tracking, SuperPoint + LightGlue, and `DPV (DPVO)` when capabilities advertise `dpvo_tensorrt`.
- Selecting `DPV (DPVO)` sends `slam.backend=dpvo_tensorrt`, so DPVO goes through the DPVO engine instead of being represented as a feature frontend.
- Runtime config service now accepts `slam.backend`, restarts the pipeline when the backend changes, and normalizes DPVO mode away from KLT/SP+LG frontend paths.

Validation:

```text
cd src/android && ./gradlew assembleDebug
```

Result:

```text
BUILD SUCCESSFUL in 9s
```

Native compile check:

```text
./scripts/build.sh smart_drone --jobs 2
```

Result:

```text
runtime_config_service.cpp compiled
udp_command_runtime.cpp compiled
link failed: cannot find -lORB_SLAM3, -lDBoW2, -lg2o
```

Interpretation from the historical run: Android/JNI/CMake build passed, and the runtime config/UDP command runtime path compiled.
The previous final link blockage was removed by making ORB-SLAM3 an optional internal legacy backend.

## 2026-05-16 Build Fix And Deployment

Problem: `./scripts/build.sh smart_drone --jobs 2` failed at link time with:

```text
cannot find -lORB_SLAM3
cannot find -lDBoW2
cannot find -lg2o
```

Root cause: the command built the default `cm5` platform, but the available third-party SLAM libraries in this workspace
were under `output/artifacts/jetson-orin-nx/lib`. The Jetson target must be built with `--jetson-orin-nx` so CMake links
against the matching aarch64 Jetson artifacts.

Fix/validation:

```text
./scripts/build.sh smart_drone --jetson-orin-nx --jobs 2
./scripts/build.sh android --jobs 2
adb -s 192.168.0.100:35911 install -r output/artifacts/android/latest.apk
TARGET_HOST=nvidia@192.168.0.103 SSH_PASSWORD=nvidia ./scripts/upload.sh --jetson-orin-nx --restart
```

Results:

| Item | Result |
| --- | --- |
| Jetson smart_drone build | `Built target smart_drone` |
| Android build | `BUILD SUCCESSFUL` |
| Phone install | `Success`, package `com.example.smartdrone` present |
| Jetson deploy | uploaded `/home/nvidia/smart_drone` and runtime config files, restarted `smart_drone.service` |
| Jetson service | `active (running)` |
| Jetson binary SHA1 | `a186a260c3e01f731afd70a72ec922f077bff9f5` |
| Android APK SHA1 | `fa87d2d8fba09fff8db7aafeee7f0a6338099588` |

Remote service command after restart:

```text
/home/nvidia/smart_drone --auto-mode idle --settings /home/nvidia/config/stereo.yaml --w 640 --h 480 --fps 60 --slam-fps 20 --uvc-device-index 0 --uvc-eye-width 640 --uvc-eye-height 480 --uvc-swap-eyes --slam-backend klt --feature-frontend lk_gftt_per_frame --dpvo-repo /home/nvidia/DPVO --udp --udp-ip 127.0.0.1 --udp-port 5000
```

Note: the service starts in `klt + lk_gftt_per_frame`. The phone-side `DPV (DPVO)` tracking mode sends runtime config
`slam.backend=dpvo_tensorrt` to switch the running pipeline to DPVO.

## 2026-05-16 Live DPV No-Pose Fix

Problem: after selecting `DPV (DPVO)` on the phone, the Jetson switched to the DPVO backend but produced no pose.

Jetson log:

```text
slam_backend=dpvo_tensorrt
dpvo_tensorrt repo= patch_engine= update_engine= input=640x400 patches=48 opt_window=7
[dpvo_trt] missing engine(s): patch='' update='' repo=''
```

Root cause: the phone runtime TLV intentionally sends only the mode/backend selection. It does not carry filesystem
paths such as `dpvo.repo`. Because the systemd service also did not pass `--dpvo-repo`, the live restart built a DPVO
runtime config with empty repo/engine paths. The backend correctly failed closed instead of silently falling back, so no
pose could be emitted.

Fix:

- `MakeDpvoTensorRtConfig()` now resolves the DPVO repo when the live runtime config leaves it blank.
- Lookup order is configured repo, `SMART_DRONE_DPVO_REPO`, `$HOME/DPVO`, `/home/nvidia/DPVO`, `/home/ltz/DPVO`, then
  `./DPVO`.
- A candidate repo is accepted only when it contains the default patchifier and update TensorRT engines under `weights/`.
- `smart_drone.service` now sets `SMART_DRONE_DPVO_REPO=/home/nvidia/DPVO` and passes `--dpvo-repo /home/nvidia/DPVO`.

Expected healthy startup after selecting DPV:

```text
[dpvo_trt] ready patch_engine=/home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine update_engine=/home/nvidia/DPVO/weights/dpvo_update_fp16.engine ...
[dpvo_trt] patchifier active ...
```

Validation after deployment:

```text
TARGET_HOST=nvidia@192.168.0.103 SSH_PASSWORD=nvidia ./scripts/upload.sh --jetson-orin-nx --restart
```

Installed service now contains:

```text
Environment=SMART_DRONE_DPVO_REPO=/home/nvidia/DPVO
ExecStart=... --dpvo-repo /home/nvidia/DPVO ...
```

A TLV runtime-config smoke test sent the same backend switch as the phone (`slamBackend=1`) and then started SLAM.
Results:

```text
ack cmd=0x31 seq=1001 status=0
ack cmd=0x30 seq=1002 status=0
state#20 runtime=1 slam=0 track=2 pos=(0.214134,-0.306068,-0.0583661) qw=0.999885
state#40 runtime=1 slam=0 track=2 pos=(0.278606,-0.584077,-0.0948745) qw=0.99973
summary acks=2 states=58 saw_pose_like=1
```

Jetson log during the same run:

```text
slam_backend=dpvo_tensorrt
dpvo_tensorrt repo=/home/nvidia/DPVO patch_engine= update_engine= input=640x400 patches=48 opt_window=7
[dpvo_cuda] native CUDA kernels ready smoke_expected=1.2226 smoke_got=1.2226
[dpvo_trt] ready patch_engine=/home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine update_engine=/home/nvidia/DPVO/weights/dpvo_update_fp16.engine update_preagg_engine=/home/nvidia/DPVO/weights/dpvo_update_preagg_fp16.engine update_postagg_engine=/home/nvidia/DPVO/weights/dpvo_update_postagg_fp16.engine input=640x400 patches=48 opt_window=7 ... native_dpvo=1
[dpvo_trt] patchifier active fmap=[1x128x100x160] imap=[1x384x100x160]
[slam_dfx] frame=20 ... pose_valid=1 ...
```

Interpretation: the live DPV no-output bug is fixed. DPVO now loads the TensorRT engines and publishes live pose after a
phone-equivalent backend switch. The remaining live issue is runtime/performance: the native DPVO solver still logs
`slam_gap_warn` because BA/SE3 and graph packing are too slow for the requested realtime target.

## 2026-05-16 Phone Video Black Screen Check

Problem: the phone appeared to have no image display after the DPVO live test.

Observed Jetson state:

```text
stream img=Y feat=Y map=N
[udp] destination peer -> 192.168.0.100:5000
```

This confirms the Jetson image sender was active and targeting the phone video port.

Observed phone state:

```text
mWakefulness=Dozing
mCurrentFocus=Window{... NotificationShade}
KeyguardServiceDelegate showing=true secure=true dreaming=true
```

After waking:

```text
mWakefulness=Awake
mIsInteractive=true
mCurrentFocus=Window{... NotificationShade}
isKeyguardShowing=true
```

Interpretation: the black screen was the MIUI secure keyguard/AOD layer, not a decoded black camera frame. ADB could
wake the panel, but MIUI would not dismiss the secure keyguard without user unlock, so SmartDrone remained behind
`NotificationShade`.

Fix applied on the Android side:

- `MainActivity` now sets `FLAG_KEEP_SCREEN_ON`, `FLAG_TURN_SCREEN_ON`, `FLAG_SHOW_WHEN_LOCKED`, and
  `FLAG_DISMISS_KEYGUARD`.
- Android O+ APIs `setTurnScreenOn(true)`, `setShowWhenLocked(true)`, and `requestDismissKeyguard()` are called on
  startup/resume.
- A non-reference-counted screen wake lock is held while the activity is alive.
- The manifest declares `WAKE_LOCK`, `showWhenLocked`, and `turnScreenOn`.

Device-level mitigation used during testing:

```text
settings put global stay_on_while_plugged_in 7
settings put system screen_off_timeout 2147483647
```

Remaining manual step: because the keyguard is secure, the user must unlock the phone once. After that, the installed
SmartDrone build should keep the display on while the app is running.

## 2026-05-17 DPV Jetson Image Input Fix

Problem: DPV/DPVO mode started on Jetson but appeared to have no image input.

Observed failure signature before the fix:

```text
[slam_dfx] ... img_std=0.00/0.00 sharp=0.00/0.00 ...
```

The SLAM loop was running, so this was not a missing-frame timeout. The camera was delivering frames whose left/right
gray images were nearly constant black.

Root cause: the USB stereo camera reports an invalid `exposure_absolute` range:

```text
exposure_absolute 0x009a0902 (int) : min=1 max=-1 step=1 default=20 value=1
```

The UVC adapter trusted that invalid range and clamped manual exposure to `1`. Phone-side DPV selection could also
carry a low manual exposure/gain from previous settings, so the packed stereo frame was valid but effectively black.

Fix:

- `UvcStereoCamera` now treats an invalid `exposure_absolute` max as bad driver metadata and uses a safe fallback
  maximum instead of clamping to `1`.
- When UVC auto exposure is enabled, the adapter keeps gain at least `32` so the camera is not left at the previous
  very-low manual gain.
- Android DPV/DPVO tracking selection now sends AE enabled with `exposure_us=30000` and `gain>=32`.

Deployment and validation:

```text
./scripts/build.sh smart_drone --jetson-orin-nx --camera-provider uvc_stereo_opencv --jobs 2
./scripts/build.sh android --jobs 2
SSH_PASSWORD=nvidia TARGET_HOST=nvidia@192.168.0.103 REMOTE_DIR=/home/nvidia ./scripts/upload.sh --jetson-orin-nx --restart
```

Jetson DPVO smoke test was triggered with a phone-equivalent TLV runtime config (`backend=dpvo_tensorrt`, `ae=on`,
`exposure_us=30000`, `gain=32`) and then `runtime -> slam`.

Healthy Jetson log after the fix:

```text
[udp_cmd] runtime cfg updated sensor=stereo backend=dpvo_tensorrt ... ae=on ...
cam 640x480 @60 aeDisable=false exp_us=30000 gain=32 pixelFormat=YUYV_packed_stereo
[uvc] warning: exposure_absolute reports invalid range min=1 max=-1; using fallback max=10000
[uvc] control device=0 exposure mode=auto
[uvc] control device=0 gain=32
[uvc] opened device=0 driver=uvcvideo card=USB Global Camera: USB Global C packed=1280x480 eye=640x480 fourcc=YUYV ...
[slam_dfx] frame=1 ... img_std=22.40/18.46 sharp=64.71/69.86 ...
[slam_dfx] frame=20 ... img_std=22.37/18.52 sharp=65.96/70.51 ...
```

Interpretation: DPV mode now has valid Jetson image input. The remaining DPVO limitation is solver/runtime quality and
performance, not camera-frame absence. The phone APK was rebuilt, but ADB was unavailable during this validation
(`192.168.0.100:35911` refused connection), so phone installation still needs to be retried when the device is online.

## 2026-05-17 DPVO EPG Runtime Efficiency

Goal: DPVO live mode was still too slow on Jetson. The native DPVO update/BA path could take about `280-300 ms` on live
frames, so EPG tracking stalled even though the graph already separates acquire, tracking, postprocess, MAVLink, UDP, and
DFX tasks. The optimization target for this pass was realtime pose output and smoother phone-side operation, not MH04
accuracy convergence.

Implementation:

- Added EPG-aware DPVO pacing in `DpvoTensorRtEngine`.
- Warmup frames still run full patchifier/correlation/update/BA so the graph gets an initial pose.
- After warmup, intermediate frames publish the last valid pose through the normal EPG postprocess/live-pose/MAVLink/UDP
  path without running patchifier/update/BA.
- Full DPVO updates are gated by both frame cadence and wall-clock cadence:
  `SMART_DRONE_DPVO_EPG_HEAVY_EVERY_N` and `SMART_DRONE_DPVO_EPG_HEAVY_INTERVAL_MS`.
- Right patchifier/depth initialization is skipped on fast publish frames.
- Bootstrap/steady update iteration counts are configurable:
  `SMART_DRONE_DPVO_BOOTSTRAP_UPDATE_ITERS`, `SMART_DRONE_DPVO_UPDATE_ITERS`.
- Rebuilt-edge graphs can now be capped with `SMART_DRONE_DPVO_CAP_REBUILT_EDGES=1`; the edge cap clamp was widened down
  to `128` so Jetson live mode can run below the previous `512` minimum.
- DPVO mode bypasses `SlamFrameProcessor` adaptive input-FPS downscaling when
  `SMART_DRONE_DPVO_EPG_PACING=1`, because the fast path makes the old `smoothed_slam_ms` throttle too pessimistic.

Jetson service live profile:

```text
SMART_DRONE_DPVO_EPG_PACING=1
SMART_DRONE_DPVO_EPG_HEAVY_EVERY_N=3
SMART_DRONE_DPVO_EPG_HEAVY_INTERVAL_MS=500
SMART_DRONE_DPVO_RIGHT_EVERY_N=3
SMART_DRONE_DPVO_WARMUP_FULL_FRAMES=4
SMART_DRONE_DPVO_BOOTSTRAP_UPDATE_ITERS=2
SMART_DRONE_DPVO_UPDATE_ITERS=1
SMART_DRONE_DPVO_MAX_EDGES=256
SMART_DRONE_DPVO_CAP_REBUILT_EDGES=1
```

Build/deploy:

```text
./scripts/build.sh smart_drone --jetson-orin-nx --jobs 2
SSH_PASSWORD=nvidia TARGET_HOST=nvidia@192.168.0.103 REMOTE_DIR=/home/nvidia ./scripts/upload.sh --jetson-orin-nx --restart
```

Result: build passed and the service was deployed/restarted on `nvidia@192.168.0.103`.

Attempt log:

| Attempt | Key settings | Phone-equivalent TLV result | Jetson timing result | Notes |
| --- | --- | --- | --- | --- |
| Frame cadence only | heavy every 3 frames, warmup 8, bootstrap 4, max edges 512 | 18 s: `50` state packets, `45` non-zero poses | Fast frames about `4-7 ms`; heavy frames still about `280-300 ms` | Proved fast publish path works, but heavy updates still happened too frequently. |
| Wall-clock heavy cadence | heavy every 3 frames and at least 700 ms apart, max edges 384 | 18 s: `117` state packets, `101` non-zero poses, state gap mean `155.9 ms` | EPG `SlamTrackingTask` settled near `7 ms` on fast frames | Much better realtime output; startup still had a multi-second gap while engines/camera/warmup initialized. |
| Light warmup live profile | heavy interval 500 ms, warmup 4, bootstrap 2, max edges 256 | 18 s: `110` state packets, `85` non-zero poses, state gap mean `165.4 ms` | `226` DFX frames: median SLAM `6.338 ms`, p90 `7.309 ms`, max `270.714 ms` | Stable fast path. Startup state gap still peaked around `2.24 s`; DFX frame gap after running peaked at `620 ms`. |

Final live timing summary from Jetson logs:

| Metric | Value |
| --- | ---: |
| DFX frames parsed | `226` |
| SLAM median | `6.338 ms` |
| SLAM p90 | `7.309 ms` |
| SLAM mean | `14.884 ms` |
| Fast-frame count `<20 ms` | `216` |
| Fast-frame mean | `6.362 ms` |
| Heavy-frame count `>=20 ms` | `10` |
| Heavy-frame mean / max | `198.941 / 270.714 ms` |
| DFX frame-gap mean / median / max | `86.249 / 40.027 / 620.388 ms` |

Interpretation:

- The DPVO hot path is now EPG-friendly in live mode: most frames pass through tracking in about `6 ms` and publish a
  pose through the normal postprocess/live-pose/MAVLink/UDP tasks.
- The remaining runtime spikes are the intentional low-rate full DPVO updates. In this attempt they still block
  `SlamTrackingTask` because correlation/update/BA are synchronous inside the backend.
- This live profile favors realtime pose continuity over offline accuracy. MH04/MH05 ATE/RPE runs should use a separate
  full-accuracy profile with higher edge/update cadence.
- The next efficiency step is to move full DPVO update/BA into an async EPG worker or finish native CUDA `fastba` so the
  publish path never waits on heavy optimization.

## 2026-05-17 DPVO Async EPG Worker And Pose Stream Smoothing

Goal: continue the DPVO live optimization until the phone receives realtime pose output without visible stalls or large
steps. The prior EPG pacing made most frames fast, but each full DPVO update still ran synchronously in `Process()` and
caused occasional `200+ ms` foreground stalls.

Implementation:

- Added `SMART_DRONE_DPVO_ASYNC_UPDATE=1`. In EPG live mode the foreground now rectifies/resizes the stereo frame,
  queues a cloned gray pair to a DPVO worker, and immediately publishes the latest cached pose.
- Moved patchifier, stereo fmap depth init, native correlation/update/BA, graph mutation, and target-pose cache updates
  into the async worker. The worker logs each heavy update as `[dpvo_async] update ...`.
- Kept the synchronous DPVO path available for non-EPG/offline runs.
- Added DPVO async publish smoothing:
  `SMART_DRONE_DPVO_ASYNC_PUBLISH_SMOOTH`,
  `SMART_DRONE_DPVO_ASYNC_PUBLISH_MAX_STEP_M`,
  `SMART_DRONE_DPVO_ASYNC_PUBLISH_MAX_ROT_RAD`.
- Added rotation limiting to the online pose output guard:
  `SMART_DRONE_ONLINE_POSE_GUARD_MAX_ROT_RAD`,
  `SMART_DRONE_ONLINE_POSE_GUARD_MAX_ROT_RPS`.
- Added `SMART_DRONE_UDP_STATE_PERIOD_MS` and changed the UDP state loop to send the latest pose snapshot at a fixed
  period instead of only sending when a new pose marks the state dirty.

Final Jetson live profile:

```text
SMART_DRONE_DPVO_EPG_PACING=1
SMART_DRONE_DPVO_ASYNC_UPDATE=1
SMART_DRONE_DPVO_ASYNC_PUBLISH_SMOOTH=1
SMART_DRONE_DPVO_ASYNC_PUBLISH_MAX_STEP_M=0.010
SMART_DRONE_DPVO_ASYNC_PUBLISH_MAX_ROT_RAD=0.018
SMART_DRONE_DPVO_EPG_HEAVY_EVERY_N=3
SMART_DRONE_DPVO_EPG_HEAVY_INTERVAL_MS=150
SMART_DRONE_DPVO_RIGHT_EVERY_N=3
SMART_DRONE_DPVO_WARMUP_FULL_FRAMES=4
SMART_DRONE_DPVO_BOOTSTRAP_UPDATE_ITERS=2
SMART_DRONE_DPVO_UPDATE_ITERS=1
SMART_DRONE_DPVO_MAX_EDGES=256
SMART_DRONE_DPVO_CAP_REBUILT_EDGES=1
SMART_DRONE_UDP_STATE_PERIOD_MS=50
SMART_DRONE_ONLINE_POSE_GUARD_MAX_STEP_M=0.08
SMART_DRONE_ONLINE_POSE_GUARD_MAX_SPEED_MPS=1.6
SMART_DRONE_ONLINE_POSE_GUARD_MAX_ROT_RAD=0.10
SMART_DRONE_ONLINE_POSE_GUARD_MAX_ROT_RPS=2.0
```

Attempt log:

| Attempt | Key change | 18 s TLV state result | Jetson timing result | Notes |
| --- | --- | --- | --- | --- |
| Async worker, 500 ms update | Full DPVO update moved off foreground; heavy interval 500 ms | `163` state packets, `127` non-zero poses, state gap p50 `101.34 ms`, max `907.58 ms`; pose step max `0.07064 m` | `342` DFX frames: foreground SLAM p50 `6.616 ms`, p90 `7.323 ms`, max `11.480 ms`; `25` async updates | Removed all `200+ ms` foreground SLAM spikes. |
| Async worker, 300 ms update + output guard rotation | Heavy interval 300 ms, output guard `0.08 m / 1.6 mps`, rotation guard `0.10 rad / 2.0 rps` | `143` state packets, `113` non-zero poses, state gap p50 `101.27 ms`; pose step max `0.05830 m`, rotation step max `0.07978 rad` | `326` DFX frames: foreground SLAM p50 `6.759 ms`, p90 `7.883 ms`, max `9.941 ms`; `29` async updates | Smaller max step, no foreground stall regression. |
| Async publish smoothing, 150 ms update | DPVO publish smoothing `0.015 m / 0.025 rad`, heavy interval 150 ms | `239` state packets, `204` non-zero poses, state gap p50 `51.86 ms`; pose step p90 `0.01575 m`, max `0.04256 m` | `311` DFX frames: foreground SLAM p50 `6.486 ms`, p90 `7.694 ms`, max `10.020 ms`; `36` async updates | Higher state rate exposed less visible stepping. |
| Final fixed-rate pose stream | UDP state fixed at 50 ms, smoothing `0.010 m / 0.018 rad` | `349` state packets, `247` non-zero poses, state gap mean/p50/p90/max `51.63 / 51.66 / 52.58 / 89.90 ms`; pose step mean/p90/max `0.00374 / 0.01098 / 0.04016 m`; rotation step mean/p90/max `0.00461 / 0.01799 / 0.04176 rad` | `310` DFX frames: foreground SLAM mean/p50/p90/max `6.587 / 6.446 / 7.287 / 9.966 ms`; `36` async updates, async update total p50/p90/max `262.829 / 386.902 / 450.180 ms` | Final live profile for phone testing. State stream no longer has 100 ms+ periodic gaps after startup. |

Interpretation:

- The realtime foreground path is now stable: DPVO live tracking no longer waits on patchifier/correlation/BA.
- Heavy DPVO updates still take `~240 ms` on average in the background, so accuracy/target-pose freshness is bounded by
  worker throughput. This is acceptable for live smoothing but not a substitute for a full MH04/MH05 accuracy profile.
- The final state stream is fixed-rate around `20 Hz` after startup and keeps publishing the latest pose even when a new
  SLAM frame has not arrived in that 50 ms window.
- The remaining visible motion quality depends on scene texture/exposure and native DPVO solver quality. For offline
  ATE/RPE convergence, use async disabled or a separate high-cadence/full-iteration profile.
