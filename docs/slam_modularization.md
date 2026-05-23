# SLAM Modularization Notes

The SLAM stack is split around reusable ports so new visual localization strategies can replace one stage without
rewiring the EPG-driven main flow.

## Core Ports

All reusable SLAM ports should keep method signatures small. Public strategy
interfaces use request/result parameter objects, and new interface methods
should stay within five parameters including outputs. This keeps ORB-SLAM3,
DPVO, KLT, SuperPoint+LightGlue, and XFeat+LightGlue replaceable without
copying ORB-SLAM3 constructor-style argument lists into the rest of the stack.

| Stage | Port | Default adapter |
| --- | --- | --- |
| Visual feature frontend | `IVisualFeatureFrontend`, `IVisualDescriptorProvider` | `SuperPointLightGlueFrontendClient`, `DefaultOrbFeatureFrontend` |
| Managed visual frontend runtime | `IManagedVisualFeatureFrontend` | `SuperPointLightGlueFrontendClient` |
| Stereo calibration/preprocess | `IStereoCalibrationLoader`, `IStereoRectifier`, `IStereoFramePreprocessor` | `DefaultStereoCalibrationLoader`, `DefaultStereoRectifier`, `DefaultStereoFramePreprocessor` |
| Stereo pair building | `IStereoPairBuilder` | `DefaultStereoPairBuilder` |
| Stereo match selection | `IStereoMatchSelector` | `DefaultStereoMatchSelector` |
| Temporal stereo carry | `ITemporalStereoProcessor` | `DefaultTemporalStereoProcessor` |
| Stereo feature packet | `IStereoFeaturePacketBuilder` | `DefaultStereoFeaturePacketBuilder` |
| Frame observation loading | `IVisualFrameObservationLoader` in `visual_frame_observation.h` | `DefaultVisualFrameObservationLoader` |
| SLAM backend tracking | `ISlamTrackingBackend` and sub-ports in `slam_backend.h` | `OrbSlam3Runtime` |
| Backend optimization/local mapping/loop closure | `ISlamBackendOptimizer`, `ISlamBackendLocalMapper`, `ISlamBackendLoopCloser` in `slam_backend_modules.h` | `OrbSlam3Runtime` adapter surface |
| Tracked features/point cloud output | `ITrackedVisualDataProvider` in `tracked_visual_data.h` | `OrbSlam3Runtime` |
| Backend state/statistics | `ISlamBackendStateProvider`, `SlamMapSummary`, `SlamBackendStats` in `slam_backend_state.h` | `OrbSlam3Runtime` |
| Trajectory query/save | `ISlamTrajectoryProvider`, `ISlamTrajectorySaver` in `slam_trajectory.h` | `OrbSlam3Runtime` |
| KLT/PnP observation and pose | `IVisualPnpObservationBuilder`, `IVisualPnpPoseBackend` | KLT defaults |
| Place recognition vocabulary | `IVisualVocabulary` | `OrbVisualVocabulary` |

## Replacement Shape

- ORB mode uses `OrbModeStrategy` and `OrbSlam3Runtime`. `orb_slam3_backend.cpp` only registers the runtime with the
  backend factory.
- SuperPoint+LightGlue and XFeat+LightGlue share `VisualFeatureLightGlueModeStrategy`; the frontend client can change
  while stereo pairing, temporal carry, packet build, and backend injection stay behind ports.
- KLT and DPVO are backend-level routes and keep their own `ISlamEngine` implementations. KLT still owns its tracking
  flow, but its PnP observation builder and PnP pose solver are accessed through `IVisualPnpObservationBuilder` and
  `IVisualPnpPoseBackend`.
- The legacy `external_*` adapter headers have been removed. New code should include the generic `visual_feature_*`,
  `stereo_feature_*`, `stereo_*`, or `temporal_stereo` headers.

ORB-SLAM3 code is absorbed under `src/native/adapters/slam/orb/orb_slam3` and built as internal modules. Sophus, DBoW2, and
g2o remain third-party dependencies under `third_party`.

ORB tracked feature and point-cloud extraction is kept behind the generic `ITrackedVisualDataProvider` port. The ORB
tracking module delegates this data shaping to `tracking/TrackedVisualDataExtractor`, so point-cloud/feature export is
separate from the main tracking state machine.

`ITrackedVisualDataProvider` also exposes `ExtractVisualMapSnapshot`, which is the preferred external call when a
consumer wants a strategy-neutral summary, tracked feature set, and optional point cloud in one request. The older
feature-only and point-cloud-only methods remain for narrow call sites and compatibility.

ORB visual components are exposed through `orb_visual_components.h`: callers can create ORB feature frontends,
descriptor providers, and vocabularies through functions that return `IVisualFeatureFrontend`,
`IVisualDescriptorProvider`, and `IVisualVocabulary`. New code should depend on those ports instead of constructing
ORB-SLAM3 concrete classes directly.

Frame feature injection is now represented by generic data types:
`VisualKeypointFeatureSet` for mono/keypoint-preserving features and
`StereoFeatureObservationPacket` for matched stereo observations. ORB-SLAM3
`Frame` aliases its injection types to these core ports and delegates descriptor
trimming, injected keypoint octave normalization, right-u/depth filling, and
close-point counting through `IVisualFrameObservationLoader`. A future backend
can reuse the same loader when it needs ORB-compatible frame observations
without including ORB-SLAM3 `Frame` internals.
Frame construction inside the ORB runtime is routed through the internal
`IOrbFrameFactory`, whose stereo/RGB-D/mono/feature-injection request objects
carry image data, timestamp, calibration, extractors, camera models, previous
frame, and IMU calibration as structured context. Tracking no longer calls the
historical long `Frame` constructors directly; those constructors remain local
implementation details of the frame module.

The SLAM backend tracking port uses request objects:
`SlamTrackRequest`, `StereoPreprocessRequest`, and
`PreparedStereoFeatureTrackRequest`. `OrbSlam3Runtime` adapts those generic
requests to the current ORB-SLAM3 `System` calls, so EPG strategies do not know
ORB-SLAM3's historical long argument lists.

Backend processing stages have generic request/result ports in
`slam_backend_modules.h`: optimizer, local mapper, and loop closer. These
interfaces describe backend capabilities without exposing ORB-SLAM3 `Frame`,
`KeyFrame`, `Map`, or `MapPoint` pointers. `OrbSlam3Runtime` currently owns the
adapter surface for these ports. The ORB adapter currently supports global BA,
local-mapper pause/resume/reset/queue status, and loop-closing reset/GBA abort
through those generic requests. Candidate-dependent operations such as Sim3
refinement, essential-graph correction, keyframe insertion, and loop correction
remain driven by ORB-SLAM3's internal state machine until their required
strategy-neutral request models are defined.

The visual frontend port also uses request/result objects:
`VisualFeatureDetectRequest`, `VisualFeatureComputeRequest`, and
`StereoVisualFeatureComputeRequest`. SuperPoint+LightGlue, XFeat+LightGlue, and
ORB frontends can share the same call shape; frontend-specific options stay in
runtime config or implementation-owned state.

`SlamOutput` is kept strategy-neutral: runtime code publishes `visualFeature*`, `stereoFeature*`, and `featurePackMs`
fields. Historical `superpoint_*` and `external_*` CSV columns are produced only at replay/reporting boundaries for
compatibility with older analysis scripts.

`SlamModeStrategy` creation is registry-based. A new EPG-visible frontend strategy should add a strategy implementation
file, register it with `SlamModeStrategyRegistrar`, and add that file to the build; it should not need to edit the
central strategy factory.

`CreateSlamEngine` is also registry-based. Backend implementations register their construction function with
`SlamEngineFactoryRegistrar`: KLT registers in `klt_slam_engine.cpp`, DPVO TensorRT registers in
`dpvo_tensorrt_engine.cpp`, and ORB-SLAM3 registers in `orb_slam3_backend.cpp`. A new backend route should own its
factory registration next to its implementation, then be added to the build and runtime config parsing without growing a
central backend switch.

Managed visual frontend clients are registry-based through `VisualFeatureFrontendClientRegistrar`. SuperPoint+LightGlue
registers its native client in `superpoint_lightglue_frontend_client.cpp`; session runtime and offline replay request the
client through `CreateVisualFeatureFrontendClient`. A future XFeat+LightGlue implementation should add its client and
registration next to the implementation, then reuse the existing `VisualFeatureLightGlueModeStrategy` path.
Frontend-specific repository resolution, enable flags, and default environment tuning are kept in
`visual_feature_frontend_client.cpp`, so EPG/session code does not need to know SuperPoint or XFeat setup details.

## Runtime Component Slots

`SlamModeSharedState` owns the default reusable components used by strategy code:

| Slot | Interface | Default implementation |
| --- | --- | --- |
| Stereo calibration loading | `IStereoCalibrationLoader` | `DefaultStereoCalibrationLoader` |
| Stereo rectification | `IStereoRectifier` | `DefaultStereoRectifier` |
| Stereo frame preprocessing | `IStereoFramePreprocessor` | `DefaultStereoFramePreprocessor` |
| Stereo pair building | `IStereoPairBuilder` | `DefaultStereoPairBuilder` |
| Stereo match selection | `IStereoMatchSelector` | `DefaultStereoMatchSelector` |
| Temporal stereo carry/source | `ITemporalStereoProcessor` | `DefaultTemporalStereoProcessor` |
| Stereo feature packet build/hash | `IStereoFeaturePacketBuilder` | `DefaultStereoFeaturePacketBuilder` |
| 2D point tracking | `IPointTracker2d` | `DefaultPointTracker2d` |
| KLT/PnP observation build | `IVisualPnpObservationBuilder` | `DefaultVisualPnpObservationBuilder` |
| PnP pose backend | `IVisualPnpPoseBackend` | `DefaultVisualPnpPoseBackend` |
| Visual loop closure | `IVisualLoopClosureBackend` | `DefaultVisualLoopClosureBackend` |

Strategy code should request these through the shared-state accessors instead of constructing concrete adapters
directly. `TemporalStereoCarryInput` can also receive the shared `IPointTracker2d`, so temporal stereo carry and KLT
continuous tracking can reuse or replace the same point-tracking implementation. That keeps future strategy additions
focused on replacing only the stage they change.

## ORB-SLAM3 Internal Modules

ORB-SLAM3 source is split by function under `src/native/adapters/slam/orb/orb_slam3`:

| Directory | Responsibility | Reusable interface |
| --- | --- | --- |
| `config`, `common` | settings, calibration values, converters, IMU common types, shared logging | consumed through runtime/config ports; third-party-free helpers stay local |
| `camera` | geometric projection, two-view reconstruction, epipolar checks, triangulation | `GeometricCamera` request/result objects |
| `features` | ORB extractor, descriptor matcher, ORB vocabulary bridge | `IVisualFeatureFrontend`, `IVisualDescriptorProvider`, `IVisualVocabulary`, `IOrbFeatureMatcher` internally |
| `frame` | frame/keyframe state, image bounds, grid assignment, stereo depth on frame observations | `VisualKeypointFeatureSet`, `StereoFeatureObservationPacket`, `IVisualFrameObservationLoader`, `IOrbFrameFactory` internally |
| `map` | map, atlas, map points | `SlamMapSummary`, `TrackedVisualSummary`, `VisualMapSnapshot` |
| `tracking` | frame-to-frame tracking state machine, tracked visual data export, frame trajectory history | `ISlamFrameTracker`, `ISlamTrackingStatusProvider`, `ITrackedVisualDataProvider`, `OrbFrameTrajectoryEntry` snapshots, `IOrbTrackingBackend` internally |
| `graph_optimization` | pose optimization, BA, Sim3, essential graph, inertial optimization | `IOrbOptimizationBackend` internally, `ISlamBackendOptimizer` externally |
| `mapping` | keyframe queue, map point creation/culling, local BA scheduling | `IOrbLocalMappingBackend` internally, `ISlamBackendLocalMapper` externally |
| `loop_closing` | place recognition, Sim3 refinement, map merge, GBA scheduling | `IOrbLoopClosingBackend`, `IOrbPlaceRecognitionBackend` internally, `ISlamBackendLoopCloser`, `IVisualVocabulary` externally |
| `system` | ORB-SLAM3 runtime composition and thread ownership | `OrbSlam3Runtime` behind `ISlamTrackingBackend` |

The current rule is that new external callers should not include ORB-SLAM3
headers directly. They should call the core ports or adapter factories, while
ORB-specific classes remain implementation details behind `OrbSlam3Runtime` or
small default adapters.

`Tracking` no longer calls the `Optimizer` static class directly. It depends on
the internal `IOrbOptimizationBackend`, whose default implementation delegates
to the original g2o-based `Optimizer`. This keeps ORB behavior unchanged while
making pose optimization and global BA replaceable inside the ORB runtime.
`LocalMapping` uses the same optimization backend for local BA, local inertial
BA, inertial initialization, full inertial BA, and gravity/scale refinement, so
mapping logic no longer includes or calls the static `Optimizer` facade.
`System::OptimizeBackend` also delegates global optimization requests through
this backend, keeping the generic `ISlamBackendOptimizer` route separate from
the concrete g2o implementation.
`LoopClosing` uses the same internal backend for Sim3 refinement, loop and
merge essential-graph optimization, merge/welding BA, inertial bias
optimization, and loop-triggered GBA. `OptimizationBackend.cc` is now the single
ORB runtime adapter that calls the original static `Optimizer` implementation.

`Tracking` also routes main local-mapping control through the internal
`IOrbLocalMappingBackend`: bad-IMU status, keyframe insertion, first timestamp,
match-inlier reporting, BA interruption, not-stop guards, far-point search
settings, and reset requests are accessed through status/request objects instead
of direct `LocalMapping` field calls. Timing diagnostics use
`OrbLocalMappingTimingStats`, and System debug dumps use
`OrbLocalMappingDebugSnapshot`, so diagnostics no longer read `LocalMapping`
public fields directly.
`LoopClosing` also uses `IOrbLocalMappingBackend` for local-mapper stop,
queue-drain, stopped/finished polling, and release operations, so loop closure
no longer includes `LocalMapping.h` or invokes `LocalMapping` methods directly.
`System` uses the same backend for localization-mode pauses, shutdown waiting,
local-mapper reset/pause/resume requests, runtime far-point configuration, and
IMU-initialization time queries. The remaining `LocalMapping*` in `System` is a
composition/thread-ownership detail used to construct and run the ORB local
mapping module.

`LocalMapping` and `LoopClosing` read tracking state through
`IOrbTrackingBackend` instead of direct `Tracking` member access. The tracking
backend exposes a compact `OrbTrackingStatus` snapshot plus request objects for
IMU frame updates and state transitions. This keeps mapping and loop-closing
logic reusable if the tracking implementation changes, while preserving the
original ORB runtime behavior through the default adapter. `System` also uses
this tracking backend for frame tracking calls, IMU pushes, reset/mode changes,
trajectory snapshots, backend statistics, tracked feature/point-cloud export,
dataset changes, timing samples, and ORB extractor access. The remaining
`Tracking*` in `System` is a composition/thread wiring detail.

`Tracking`, `LocalMapping`, and `System` use `IOrbLoopClosingBackend` for loop
queue insertion, reset, finish, GBA abort, and loop timing snapshots. Loop
closing diagnostics are exported as `OrbLoopClosingTimingStats`, so tracking
time reports no longer read loop-closing public vectors directly. The remaining
`LoopClosing*` in `System` is a thread construction/ownership detail.

Place recognition is exposed through the internal
`IOrbPlaceRecognitionBackend`. The default backend wraps `KeyFrameDatabase` for
keyframe insertion, relocalization candidate lookup, loop/merge candidate
lookup, full reset, and active-map reset. Tracking and loop closing use request
objects for these operations, leaving `KeyFrameDatabase` as the default
ORB/DBoW implementation and keeping future vocabulary or learned place
recognition backends replaceable behind the same boundary.

ORB logging moved out of `System.h` into `common/Verbose`, removing a hidden
include dependency from optimization, mapping, tracking, and loop-closing
modules.

ORB descriptor matching is exposed through the internal `IOrbFeatureMatcher`
interface. The default implementation wraps the original `ORBmatcher`, but all
public matcher calls use request objects such as
`OrbFrameMapProjectionRequest`, `OrbFrameKeyFrameProjectionRequest`,
`OrbInitializationMatchRequest`, and `OrbTriangulationMatchRequest`. Tracking
now consumes the matcher through this interface for initialization, reference
keyframe matching, motion-model projection, local-map projection, and
relocalization. Local mapping and loop closing use the same interface for
triangulation, BoW matching, Sim3 projection, and map-point fusion. The concrete
`ORBmatcher` implementation is now contained in the `features` module; frame,
map, optimization, tracking, mapping, and loop-closing modules consume matcher
thresholds and descriptor distance through the feature matcher boundary.

`GeometricCamera` uses request/result objects for reusable geometry operations:
`TwoViewReconstructionRequest`/`Result`, `EpipolarConstraintRequest`,
`CameraMatchTriangulationRequest`, and `CameraTriangulationRequest`/`Result`.
This keeps camera and matching interfaces within the five-parameter rule while
making epipolar validation and triangulation reusable by non-ORB frontends.
`System` consumes frame trajectory history through `IOrbTrackingBackend`, so
trajectory file writers no longer depend on the concrete tracking history
containers.

## Runtime Configuration Names

New visual-feature frontend runtime settings use `visual_feature_*` naming:
`slam.visual_feature_top_k`, `slam.visual_feature_max_points`,
`slam.visual_feature_input_max_width`, and `slam.visual_feature_input_max_height`.
The CLI mirrors these as `--visual-feature-*`. Existing `superpoint_*` config keys
and `--superpoint-*` CLI flags are retained as compatibility aliases and are
synchronized into the same runtime fields.

## Environment Naming

New stereo-feature tuning variables use the `SMART_DRONE_STEREO_FEATURE_*` prefix. The runtime still falls back to
legacy `SMART_DRONE_EXTERNAL_STEREO_*` names where those existed, so existing MH04 profiles and scripts continue to
work while new code avoids the old "external" terminology.
