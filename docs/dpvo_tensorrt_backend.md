# DPVO TensorRT Backend

The DPVO route is intentionally a backend replacement, not another ORB-SLAM3 feature frontend. Select it with:

```bash
--slam-backend dpvo \
--dpvo-repo /home/nvidia/DPVO \
--dpvo-patch-engine /home/nvidia/DPVO/weights/dpvo_patchifier_fp16.engine \
--dpvo-update-engine /home/nvidia/DPVO/weights/dpvo_update_fp16.engine
```

Current status:

- The SmartDrone runtime now has a backend boundary: `orbslam3` and `dpvo_tensorrt`.
- `dpvo_tensorrt` is native C++/TensorRT only. It does not start Python or PyTorch.
- The backend loads the DPVO patchifier and update TensorRT engines directly in process.
- Jetson now has generated FP16 engines under `/home/nvidia/DPVO/weights/`.
- It fails closed if the engines are missing or invalid, instead of silently falling back to ORB-SLAM3.
- It is not enabled in `smart_drone.service`; production still uses `orbslam3 + superpoint_lightglue`.

Important implementation note: original DPVO is not a single network that directly outputs a pose. It combines:

- a patchifier network
- an update network
- CUDA correlation
- CUDA bundle adjustment
- SE3 graph/state management

Only the runtime backend slot and TensorRT engine loading are wired in this pass. Pose publication remains disabled inside
the DPVO backend until the native CUDA correlation/BA/state path is ported. This is deliberate: the flight stack must not
receive placeholder VO poses.

Next native steps:

1. Port or wrap DPVO `altcorr` and `fastba` CUDA kernels without Python/Torch dependencies.
2. Implement the C++ DPVO state machine: patch memory, graph edges, motion model, keyframe removal, and SE3 pose output.
3. Run MH04/MH05 replay and a live rotation test before enabling `--slam-backend dpvo` in the service.

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

## MH04 Replay Status

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
