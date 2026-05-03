# ORB Mode Technical Flow

## Purpose

ORB mode (`--feature-frontend orb`) is the production baseline and EuRoC accuracy reference. It keeps the visual frontend and backend inside native ORB-SLAM3: ORB extraction, stereo matching, tracking, local mapping, relocalization, and loop closure all remain in the ORB-SLAM3 system. SmartDrone code is responsible for frame acquisition, mode selection, input packaging, timing, and converting ORB-SLAM3 output into the project `SlamOutput` contract.

## Main Flow

```mermaid
flowchart TD
    A[CLI/runtime config<br/>feature_frontend=orb] --> B[ParseFeatureFrontendText<br/>FeatureFrontend::Orb]
    B --> C[RunOfflineReplay or SlamFrameProcessor]
    C --> D[PerceptionPipeline::AcquireNextStereoBatch]
    D --> E{Frame accepted by<br/>SLAM FPS limiter?}
    E -- no --> D
    E -- yes --> F[Build SlamInputBatch<br/>left/right gray, frame id, timestamp, optional IMU]
    F --> G[OrbSlam3Engine::Process]
    G --> H{KLT or SP+LG branch?}
    H -- no, ORB --> I[ORB_SLAM3::System::TrackStereo<br/>or TrackStereo with IMU]
    I --> J[ORB-SLAM3 internal pipeline<br/>ORB extract, stereo match, tracking, mapping]
    J --> K[Read tracker/system telemetry]
    K --> L[Invert Tcw to Twc<br/>fill PoseEstimate]
    L --> M[SlamOutput]
    M --> N[ReplayPoseSample / live telemetry]
    N --> O[euroc_pose.csv<br/>euroc_summary.json<br/>profile_summary.md]
```

## Code Entry Points

| Layer | Code | Responsibility |
| --- | --- | --- |
| CLI/offline replay | `tests/euroc/offline_replay_main.cpp` | Parse `--feature-frontend orb`, create ORB-SLAM3 system, run EuRoC replay, export CSV/JSON. |
| Replay loop | `tests/euroc/support/replay_slam_runner.cpp` | Pull stereo frames and IMU windows, call `ISlamEngine::Process`, collect per-frame timing. |
| Live loop | `src/native/core/application/session/slam_frame_processor.cpp` | Read runtime tuning, apply frontend mode, acquire camera frames, call SLAM engine. |
| Rate limiter | `src/native/core/application/state/perception_pipeline.cpp` | Enforce SLAM input FPS and derive stable capture/logical timestamps. |
| SLAM adapter | `src/native/adapters/slam/orbslam3_engine.cpp` | Dispatch frontend mode, call ORB-SLAM3, convert pose/telemetry to `SlamOutput`. |
| Output contract | `src/native/core/ports/slam_engine.h` | Defines `SlamInputBatch` and `SlamOutput`. |

## Configuration and Mode Selection

Offline replay reads:

```bash
--feature-frontend orb
--sensor-mode stereo
--settings config/euroc/stereo_orb_official.yaml
--vocab /home/nvidia/ORBvoc.txt
```

`ParseFeatureFrontendText(...)` maps `orb` to `FeatureFrontend::Orb`. `RunOfflineReplay(...)` then constructs:

```cpp
auto orbSystem = std::make_unique<ORB_SLAM3::System>(opts.vocab, opts.settings, sensor, false);
OrbSlam3Engine slamEngine(std::move(orbSystem), ResolveOrbInputMode(opts.sensorMode),
                          UseImu(opts.sensorMode), opts.settings);
slamEngine.SetFeatureFrontend(opts.featureFrontend);
```

Live runtime follows the same frontend enum path. `SlamFrameProcessor` reads `m_ctx.tuning.featureFrontend`, maps `Reserved` to ORB, and calls `m_ctx.slamEngine.SetFeatureFrontend(effectiveFrontend)` every frame so runtime changes take effect safely.

## Input Pipeline

```mermaid
sequenceDiagram
    participant Runner as ReplaySlamRunner / SlamFrameProcessor
    participant Pipe as PerceptionPipeline
    participant Cam as CameraProvider
    participant IMU as ImuProvider
    participant SLAM as OrbSlam3Engine

    Runner->>Pipe: AcquireNextStereoBatch(camera, slamInputFps, timeout)
    Pipe->>Cam: GrabStereo(timeout, preferLatest, minTimestampNs)
    Cam-->>Pipe: StereoFrame(left.gray, right.gray, timestamps)
    Pipe->>Pipe: compute capture timestamp<br/>apply FPS limiter
    Pipe-->>Runner: StereoBatch(frameId, captureTimestampNs)
    Runner->>IMU: PopWindow(lastFrameNs, currentFrameNs)
    IMU-->>Runner: optional IMU readings
    Runner->>SLAM: Process(SlamInputBatch)
    SLAM-->>Runner: SlamOutput
```

Important implementation details:

- `PerceptionPipeline::AcquireNextStereoBatch(...)` computes `minTimestampNs` from the last accepted capture timestamp and requested SLAM FPS.
- It drops frames by returning `DroppedByRateLimiter` when the logical timestamp is too early.
- It uses the midpoint of left/right timestamps as `captureTimestampNs`.
- Offline EuRoC replay uses `ReplayCameraProvider::GrabStereo(...)`, which loads left/right grayscale images from disk.
- IMU samples are converted to `ORB_SLAM3::IMU::Point` only if the sensor mode uses IMU. The documented EuRoC run uses `--stereo-only`, so no IMU is passed.

## ORB Branch Dispatch

`OrbSlam3Engine::Process(...)` first checks custom frontends:

```cpp
if (m_featureFrontend == FeatureFrontend::LkGfttPerFrame) {
    return ProcessLkGfttPerFrameStereoVo(input, extractFeatures);
}
if (m_featureFrontend == FeatureFrontend::LK) {
    return ProcessLkStereoVo(input, extractFeatures);
}
```

Then it checks the SP+LG external frontend gate. In ORB mode that gate is false because `m_featureFrontend != FeatureFrontend::SuperPointLightGlue`.

The resulting ORB execution path is:

```cpp
const auto orbTrackStartTp = std::chrono::steady_clock::now();
tcw = m_system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec);
const auto orbTrackEndTp = std::chrono::steady_clock::now();
out.orbTrackMs = duration(orbTrackEndTp - orbTrackStartTp);
```

If stereo-IMU is enabled, the adapter calls the overload that passes `input.imu`.

## Internal ORB-SLAM3 Responsibilities

SmartDrone delegates these steps to ORB-SLAM3:

```mermaid
flowchart LR
    A[TrackStereo input<br/>left/right gray] --> B[ORB extraction]
    B --> C[Stereo ORB matching]
    C --> D[Frame tracking<br/>motion model / map matching]
    D --> E{Tracking state}
    E --> F[Local mapping]
    E --> G[Relocalization if needed]
    F --> H[Loop closure / map maintenance]
    G --> H
    H --> I[Tcw pose and map telemetry]
```

The adapter reads ORB-SLAM3 telemetry after tracking:

- `tracker->mCurrentFrame.mTimeORB_Ext` -> `orbExtractMs`
- `tracker->mCurrentFrame.mTimeStereoMatch` -> `orbStereoMatchMs`
- wall-clock `TrackStereo(...)` duration -> `orbTrackMs`
- `m_system->GetTrackingState()` -> `trackingState`
- `m_system->GetCurrentMapId()` -> `mapId`
- `m_system->GetMatchesInliers()` -> `matchesInliers`
- `m_system->GetTrackedMapPointCount()` -> `trackedMapPointCount`
- `m_system->GetLocalMapPointCount()` -> `localMapPointCount`

## Pose Conversion

ORB-SLAM3 returns `Tcw`. The adapter converts it to world-camera pose:

```cpp
const Sophus::SE3f twc = tcw.inverse();
const Eigen::Vector3f t = twc.translation();
const Eigen::Quaternionf q(twc.so3().unit_quaternion());
```

Then it validates every translation/quaternion component with `std::isfinite(...)`. Valid poses are copied into `PoseEstimate` and marked with `poseValid=true`.

## Output Artifacts

```mermaid
flowchart TD
    A[SlamOutput] --> B[ReplayPoseSample]
    B --> C[euroc_pose.csv]
    B --> D[euroc_summary.json]
    C --> E[evaluate_euroc_regression.py]
    E --> F[euroc_metrics.json]
    D --> G[run_jetson_euroc_mh_feature_compare.sh]
    F --> G
    G --> H[profile_summary.md]
```

For ORB stereo-only EuRoC regression, `RunOfflineReplay(...)` uses the official ORB-SLAM3 EuRoC trajectory export:

1. `ShutdownAndSaveOrbTrajectoryEuRoC(...)`
2. `ConvertOrbEuRoCTrajectoryToReplayCsv(...)`
3. `evaluate_euroc_regression.py`

This keeps the ORB reference comparable to ORB-SLAM3's native evaluation convention while still producing SmartDrone CSV/JSON artifacts.

## Profiling Design

ORB mode is instrumented at two levels:

| Field | Source | Meaning |
| --- | --- | --- |
| `replay_acquire_ms` | `ReplaySlamRunner` | Time to acquire the next accepted stereo batch. |
| `replay_imu_ms` | `ReplaySlamRunner` | Time to collect/convert IMU samples. Usually zero for `--stereo-only`. |
| `slam_total_ms` | `ReplaySlamRunner` | Full `ISlamEngine::Process(...)` call. |
| `orb_track_ms` | `OrbSlam3Engine::Process(...)` | Wall time around ORB-SLAM3 `TrackStereo(...)`. |
| `orb_extract_ms` | ORB-SLAM3 current frame | ORB extraction time. |
| `orb_stereo_ms` | ORB-SLAM3 current frame | Stereo matching time. |

Fields that should stay zero in ORB mode:

- `frontend_ms`
- `stereo_pair_ms`
- `external_pack_ms`
- KLT-specific `lk_*` fields

## Failure and Recovery Behavior

- Empty or missing `m_system` returns an empty `SlamOutput`.
- ORB-SLAM3 can report lost/not-initialized states through `trackingState`; the adapter still emits the state for downstream logic.
- Rate-limited frames are dropped before SLAM and do not produce `SlamOutput`.
- Camera timeout/unhealthy status is handled in `PerceptionPipeline`/session code, not inside ORB.

## Jetson EuRoC Result

Latest archived run:

`docs/jetson_euroc_key_profile_20260503_080401.md`

Summary:

- Average SLAM path: `34.84 ms/frame`
- Average ORB track: `34.21 ms/frame`
- ORB extraction ranges roughly `17.67-19.39 ms/frame` across Machine Hall.

## Engineering Interpretation

ORB mode is the cleanest reference path: acquisition and rate limiting are SmartDrone responsibilities, but pose estimation is native ORB-SLAM3. It is the right baseline for accuracy and for detecting regressions in calibration, timestamp handling, ORB-SLAM3 integration, or trajectory export.
