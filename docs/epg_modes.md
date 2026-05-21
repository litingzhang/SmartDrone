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
- `SlamImuGateTask`: consumes runtime readiness and ticks, waits for IMU readiness, rate-limits frame readiness from the live SLAM input FPS, and emits `SlamFrameReady`.
- `SlamAcquireTask`: runs `SlamFrameInputPort::AcquireAndPrepareFrame` and emits `SlamPreparedFrame`.
- `SlamTrackingTask`: runs `SlamFrameTrackingPort::TrackPreparedFrame` and emits `SlamTrackedFrame`.
- `SlamPosePostprocessTask`: runs `SlamFramePosePostprocessPort::PostprocessTrackedFrame` and fans out the same `SlamPublishedFrame` snapshot to output tasks.
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

These tasks protect mutable SLAM stage state with stage-local mutexes for input, tracking/backend, and pose
postprocess. Shared tracking/mode feedback and output timing/DFX feedback are atomic state objects, and independent
output branches use branch-local locks so EPG fan-out can overlap when their data ownership is separate. Backend
control calls are serialized through `SlamRuntimeControlPort`, keeping raw backend control access out of individual
EPG stage ports.

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
The remaining safety boundaries are stage-local mutexes for stateful SLAM stages, atomic shared/output-feedback values,
branch-local locks around side-effect outputs, and the serialized SLAM runtime-control port.
