# DPVO TensorRT Backend

The DPVO route is intentionally a backend replacement, not another ORB-SLAM3 feature frontend. Select it with:

```bash
--slam-backend dpvo \
--dpvo-repo /home/nvidia/DPVO \
--dpvo-patch-engine /home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine \
--dpvo-update-engine /home/nvidia/DPVO/weights/dpvo_update_fp16.engine
```

Current status as of 2026-05-15:

- The SmartDrone runtime now has a backend boundary: `orbslam3` and `dpvo_tensorrt`.
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
- It fails closed if the engines are missing or invalid, instead of silently falling back to ORB-SLAM3.
- It is not enabled in `smart_drone.service`; production still uses `orbslam3 + superpoint_lightglue`.

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
  --vocab /home/nvidia/ORBvoc.txt \
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
[offline_replay] slam_backend=dpvo_tensorrt feature_frontend=orb
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
