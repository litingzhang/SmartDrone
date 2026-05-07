# SmartDrone EPG Modes

This document records the native runtime migration shape for the three exposed runtime modes:

- `Idle`
- `Slam`
- `Calib`

The native runtime uses epg at two levels:

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
- `SlamClockTask`: emits the frame tick queue. The static DOT default is 50 ms, and the SLAM session overrides it from the configured SLAM input FPS at startup.
- `SlamImuGateTask`: consumes runtime readiness and ticks, waits for IMU readiness, rate-limits frame readiness from the live SLAM input FPS, and emits `SlamFrameReady`.
- `SlamAcquireTask`: runs `SlamFrameProcessor::AcquireAndPrepareFrame` and emits `SlamPreparedFrame`.
- `SlamTrackingTask`: runs `SlamFrameProcessor::TrackPreparedFrame` and emits `SlamTrackedFrame`.
- `SlamPosePostprocessTask`: runs `SlamFrameProcessor::PostprocessTrackedFrame` and emits `SlamPublishedFrame`.
- `SlamPointCloudTask`: runs `SlamFrameProcessor::EmitPointCloud` and forwards `SlamPublishedFrame`.
- `SlamLivePoseTask`: runs `SlamFrameProcessor::EmitLivePose` and forwards `SlamPublishedFrame`.
- `SlamMavlinkTask`: runs `SlamFrameProcessor::EmitMavlink` and forwards `SlamPublishedFrame`.
- `SlamUdpTask`: runs `SlamFrameProcessor::EmitUdp` and forwards `SlamPublishedFrame`.
- `SlamDfxTask`: runs `SlamFrameProcessor::EmitDfx` and emits `SlamStatus`.
- `SlamMonitorTask`: consumes `SlamStatus`, updates `sessionOk`, and requests stop on abort.

These tasks share the existing `SlamFrameProcessor` state through a guarded processor mutex. That preserves the legacy single-owner state semantics while the graph boundary is expanded. A later profiling pass can remove the guard only after state has been split by stage.

## Calibration Graph

Calibration is graph-backed through a C++-assembled epg. The maintained visual topology is only `config/epg/epg_topology.dot`.

Current responsibility split:

- `CalibResourceTask`: initializes output files, camera, UDP preview resources, and emits readiness.
- `CalibClockTask`: emits the 1 ms camera tick queue.
- `CalibCameraAcquireTask`: grabs stereo frames and emits capture/save/preview queues.
- `CalibPacingFilterTask`: enforces calibration save FPS and emits `CalibSavePair`.
- `CalibStorageWriteTask`: writes image pairs and CSV rows.
- `CalibImuWriterTask`: reads IMU samples and writes `imu.csv`.
- `CalibUdpPreviewTask`: sends preview images.
- `CalibCompletionTask`: joins capture, storage, IMU, and preview status.
- `CalibFlushSyncTask`: flushes/syncs files and emits final status.
- `CalibMonitorTask`: consumes `CalibStatus` and mirrors completion diagnostics.

## Migration Boundary

The graph layer controls session launch, task wakeup, queue transfer, and status propagation for active native sessions. The remaining safety boundary is the processor mutex shared by SLAM stages; it preserves the previous state ownership semantics until those stage-local states are separated.
