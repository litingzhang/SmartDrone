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

1. Export DPVO patchifier and update networks to ONNX, then TensorRT engines for the Jetson input size.
2. Port or wrap DPVO `altcorr` and `fastba` CUDA kernels without Python/Torch dependencies.
3. Implement the C++ DPVO state machine: patch memory, graph edges, motion model, keyframe removal, and SE3 pose output.
4. Run MH04/MH05 replay and a live rotation test before enabling `--slam-backend dpvo` in the service.
