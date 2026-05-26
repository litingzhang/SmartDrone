# SmartDrone EPG Modes

This document records the native runtime migration shape for the three exposed runtime modes:

- `Idle`
- `Slam`
- `Calib`

The native runtime uses epg at two levels:

- Persistent system runtime work is assembled by `SystemRuntimeGraph`, with task registration in
  `system_runtime_task_factory` and task behavior in `system_runtime_tasks`.
- `RuntimeSessionSupervisor` is driven by a small periodic graph task for mode/session orchestration.
- Active `Slam` and `Calib` sessions are delegated to graph-backed session runners.

The only maintained topology file is `config/epg/epg_topology.dot`. It must show only implemented epg tasks and queues, not future target nodes. The currently implemented supervisor/session graphs are assembled in C++ `GraphConfig` objects.

## Mode Dispatch Graph

`UnifiedRuntimeController` still owns the public API and config mutation. `RuntimeSessionSupervisor` now runs its orchestration step through epg instead of a hand-written worker loop.

See `config/epg/epg_topology.dot` for the maintained topology. That diagram uses queue metadata only on edges, omits queue names, and contains no task port suffixes or implicit Mermaid nodes.

`Idle` remains a zero-work state: no hardware session thread is active, and the supervisor graph only polls for mode changes, restart requests, and shutdown.

## SLAM Graph

SLAM is graph-backed by a C++-assembled epg. The maintained visual topology is only `config/epg/epg_topology.dot`.

Current responsibility split:

- `SlamResourceTask`: owns the existing `SlamSessionRuntime` lifecycle and emits readiness after startup.
- `SlamClockTask`: emits the frame tick queue. The static DOT default is 50 ms, and the SLAM session applies the manifest-declared runtime tuning for the configured SLAM input FPS at startup.
- `SlamBackendTickTask`: advances backend maintenance work through the `slam_backend` resource.
- `SlamImuGateTask`: consumes runtime readiness and ticks, waits for IMU readiness, rate-limits frame readiness from the live SLAM input FPS, and emits `SlamFrameReady`.
- `SlamAcquireTask`: runs `SlamFrameInputPort::AcquireAndPrepareFrame` and emits `SlamPreparedFrame`.
- `SlamTrackingRouteTask`: routes prepared frames to the selected tracking strategy queue.
- `SlamKltTrackingTask`, `SlamDpvoTrackingTask`, `SlamOrbTrackingTask`, and `SlamVisualFeatureTrackingTask`: run `SlamFrameTrackingPort::TrackPreparedFrame` for the selected strategy and emit `SlamTrackedFrame`.
- `SlamPosePostprocessTask`: consumes tracked frames from the strategy branches, runs `SlamFramePosePostprocessPort::PostprocessTrackedFrame`, and fans out the same `SlamPublishedFrame` snapshot to output tasks.
- `SlamPointCloudTask`: emits the point-cloud side effect and reports `SlamStatus`.
- `SlamLivePoseTask`: emits the live-pose side effect and reports `SlamStatus`.
- `SlamMavlinkTask`: emits the MAVLink pose side effect and reports `SlamStatus`.
- `SlamUdpTask`: emits the UDP image side effect and reports `SlamStatus`.
- `SlamDfxTask`: waits for output branches to finish, emits DFX, and reports `SlamStatus`.
- `SlamMonitorTask`: consumes `SlamStatus`, updates `sessionOk`, and requests stop on abort.

`SlamSessionRuntime` no longer owns a monolithic frame processor. It creates a `SlamFramePortSet`, which assembles the
input, tracking, pose-postprocess, and output ports plus their narrow contexts. Frame state is split into shared tracking
mode feedback, input pacing/warmup state, pose-continuity state, and output timing/DFX state so each EPG stage has a
clear state boundary.

Mutable SLAM backend access is serialized by the EPG scheduling resource declared in the task manifest. ORB tracking,
visual-feature tracking, point-cloud extraction, and backend maintenance share the `slam_backend` resource, so the
runtime enters only one of those task bodies at a time. Shared tracking/mode feedback and output timing/DFX feedback
remain atomic state objects. Backend control calls are serialized through `SlamRuntimeControlPort`, keeping raw backend
control access out of individual EPG stage ports.

## Calibration Graph

Calibration is graph-backed through a C++-assembled epg. The maintained visual topology is only `config/epg/epg_topology.dot`.
`CalibRuntimeState` owns calibration session state and delegates session resources to `CalibSessionPortSet`.
`CalibSessionPortSet` owns calibration resource startup/shutdown order and writes IMU samples through `CalibStoragePort`.
Runtime state snapshots the port set for hot-path task calls; camera, IMU, preview, and storage protect their own
resource lifetimes so EPG tasks are not serialized behind one session-wide mutex.
`CalibCameraInputPort` owns camera provider access, capture health, and stereo-frame timing validation.
`CalibImuSamplePort` owns IMU sampling via `Icm42688SampleSource`.
`CalibSavePacingPort` owns save pacing and save-pair construction. `CalibPreviewPort` serializes preview open, enqueue,
and close calls, delegating UDP image output and open-status reporting to `CalibPreviewPublisher`.
`CalibStoragePort` serializes calibration storage writes and final flush, delegating file algorithms to
`CalibOutputStore`. `CalibOutputStore` owns output directories, image/CSV writing, image normalization, and final fsync. `CalibSessionGraphRuntime` now delegates task registration to
`calib_session_task_factory` and task behavior to `calib_session_tasks`, keeping graph lifecycle separate from
queue-level scheduling work.

Current responsibility split:

- `CalibResourceTask`: initializes output files, camera, UDP preview resources, and emits readiness.
- `CalibClockTask`: emits the 1 ms camera tick queue.
- `CalibCameraAcquireTask`: grabs stereo frames and emits capture/save/preview queues.
- `CalibPacingFilterTask`: enforces calibration save FPS and emits `CalibSavePair`.
- `CalibStorageWriteTask`: writes image pairs and CSV rows.
- `CalibImuWriterTask`: steps IMU sampling/writing through `CalibRuntimeState` and reports status.
- `CalibUdpPreviewTask`: sends preview images and reports enqueue/open status.
- `CalibCompletionTask`: joins capture, storage, IMU, and preview status.
- `CalibFlushSyncTask`: flushes/syncs files and emits final status.
- `CalibMonitorTask`: consumes `CalibStatus` and mirrors completion diagnostics.

## Migration Boundary

The graph layer controls session launch, task wakeup, queue transfer, and status propagation for active native sessions.
The remaining safety boundaries are EPG resource serialization for stateful native resources, atomic shared/output
feedback values, and the serialized SLAM runtime-control port.
