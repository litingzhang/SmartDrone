# PX4 v1.17 + Gazebo Harmonic visual-hover simulation

SmartDrone has three PX4 SITL profiles. Run them in order when diagnosing a
hover problem:

| Profile | PX4 horizontal/height aid | SmartDrone mode | Pose output | Purpose |
| --- | --- | --- | --- | --- |
| `control` | simulated GNSS | `idle` | `none` | MAVLink, OFFBOARD, arm, move, land |
| `truth` | Gazebo truth through ODOMETRY | `idle` | `position_velocity` | frames, time sync, EKF2 EV fusion |
| `vision` | stereo KLT through ODOMETRY | `slam` | `position_velocity` | end-to-end visual hover |

`control` and `truth` are diagnostic layers. Only `vision`, with GNSS and
optical-flow fusion disabled, measures visual-hover performance.

## Install the pinned environment

The supported host is Ubuntu 24.04 with Gazebo Harmonic (`gz-sim8`). The setup
script clones PX4 at the exact `v1.17.0` tag into a dedicated checkout:

```bash
./scripts/setup_px4_gz_sitl.sh
```

The default is `/home/ltz/workspace/px4/PX4-Autopilot-v1.17.0`. The existing
`/home/ltz/workspace/px4/PX4-Autopilot` checkout is explicitly rejected by the
setup script and is never reset, updated, or cleaned.

For unattended provisioning, sudo must already be authorized:

```bash
./scripts/setup_px4_gz_sitl.sh --non-interactive
```

Read-only checks are available without installing anything:

```bash
./scripts/setup_px4_gz_sitl.sh --check-only
./scripts/check_sitl_env.sh --profile vision --bind 14540
```

The installer uses PX4's `Tools/setup/ubuntu.sh --no-nuttx`, which installs the
official Gazebo Harmonic packages and the PX4 SITL build dependencies. Use
`--skip-dependencies` only on a pre-provisioned host.

When host packages or non-interactive sudo are unavailable, build the pinned
Ubuntu 24.04 / Harmonic container instead:

```bash
./scripts/setup_px4_gz_container.sh
./scripts/run_in_px4_gz_container.sh --software-rendering -- \
  ./scripts/check_sitl_env.sh --profile vision --skip-smart-drone
```

The container preserves the workspace's absolute path, host UDP networking and
Gazebo Transport partition. Auto rendering probes `/dev/dri/renderD*` through
surfaceless EGL inside the pinned image, so a headless host can use its GPU
without `DISPLAY`. Missing nodes, a failed EGL probe, or a software renderer
falls back to Mesa llvmpipe. Software rendering defaults
`SMART_DRONE_SIM_WATCHDOG_SCALE=5`; this keeps the simulation-only 100/500 ms
visual safety windows meaningful when camera rendering runs near 0.2 real-time
factor. Explicit hardware and real deployments default the scale to 1. The
selected renderer, render node, and selection reason are recorded in the
scenario, runner metrics, and run manifest. Auto and forced-software selections
set `verified=true`; forced hardware remains explicitly unprobed. This is
diagnostic launch metadata, while formal acceptance still uses the artifact
provenance gate and measured wall-clock latency.

If auto selection reports `reason=host_runtime_pm_error`, the kernel has put the
DRM device into a fatal runtime power-management state. The wrapper deliberately
does not reset or rebind a host GPU. Inspect the current boot's kernel log and
boot a kernel containing the relevant amdgpu fix before retrying hardware
rendering; forced hardware also fails early while this state is present.

## Build SmartDrone

All SmartDrone builds go through the repository build entry point:

```bash
./scripts/build.sh host-smart_drone --camera-provider gz_stereo
```

For the container environment, run the same mandatory build entry point inside
the image:

```bash
./scripts/run_in_px4_gz_container.sh --software-rendering -- \
  ./scripts/build.sh host-smart_drone --camera-provider gz_stereo
```

The simulator-specific calibration is
`sim/px4_gz/config/stereo_rectified.yaml`. It is an ideal, zero-distortion
calibration and must not be replaced by the physical OV9281 lens calibration.
The per-frame KLT frontend uses a 0.5 px PnP RANSAC reprojection gate. This
rejects subpixel residuals that otherwise couple a small false rotation into
hover translation. `SMART_DRONE_LK_PER_FRAME_PNP_REPROJ` remains available for
camera-specific noise characterization; changing focal length or baseline to
compensate an observed motion scale is not valid.

## Start PX4 and Gazebo

Start the desired layer in the first terminal:

```bash
./scripts/run_px4_gz_sitl.sh --profile control
./scripts/run_px4_gz_sitl.sh --profile truth --headless
./scripts/run_px4_gz_sitl.sh --profile vision --headless --bind 14640
```

`--bind` is not informational: after PX4 startup the launcher replaces the
stock onboard MAVLink instance with one whose remote port is the requested
SmartDrone port. `--px4-port` controls the PX4-side local port and defaults to
14580.

Each launch creates `output/sitl/<timestamp>_<profile>_<pid>/`, containing
`gazebo-server.log`, `gazebo-gui.log`, `px4.log`, an isolated PX4 rootfs and
`run.env`. ULogs are below the isolated rootfs `log/` directory. The launcher
owns its PX4, Gazebo server and GUI process groups and stops them on exit.

The launcher prints the generated `run.env`. Source it in the SmartDrone
terminal, then use the profile-selected mode and pose output:

```bash
source output/sitl/<run>/run.env
./scripts/run_sitl_smart_drone.sh --bind 14540 -- \
  --auto-mode "$SMART_DRONE_AUTO_MODE" \
  --px4-pose-output-mode "$SMART_DRONE_PX4_POSE_OUTPUT_MODE" \
  --settings "$PWD/sim/px4_gz/config/stereo_rectified.yaml"
```

The environment sets `SMART_DRONE_OFFBOARD_REQUIRES_VISION=0` for `control`
and `truth`, and `1` for `vision`. The base visual pose-age and visual-loss
landing windows are 100 ms and 500 ms. The PX4 flight-mode HEARTBEAT freshness
window is 1500 ms. These checks use the companion's steady clock.
`SMART_DRONE_SIM_WATCHDOG_SCALE` dilates these wall-clock windows for a
deliberately slower simulator; it does not change image fault durations,
scenario scheduling or any hardware default. Every automated run records the
base and effective visual values under `visual_watchdog` in `scenario.yaml`.
Override the measured host-specific value explicitly when needed, for example:

```bash
SMART_DRONE_SIM_WATCHDOG_SCALE=4 \
./scripts/run_in_px4_gz_container.sh --software-rendering -- COMMAND
```

Use `--standalone` to attach PX4 to a world already running in the same
`GZ_PARTITION`. Use `--spawn` with a PX4 model such as `x500` and an appropriate
world when testing upstream models. The default custom model and world are
fully local; they do not fetch Fuel assets.

## Sensor and truth contract

`SMART_DRONE_SIM_CONFIG` defaults to
`sim/px4_gz/config/smartdrone_sim.yaml`. Its stable transport contract is:

| Data | Gazebo topic | Message |
| --- | --- | --- |
| Left image | `/smartdrone/stereo/left/image` | `gz.msgs.Image` |
| Right image | `/smartdrone/stereo/right/image` | `gz.msgs.Image` |
| Left camera info | `/smartdrone/stereo/left/camera_info` | `gz.msgs.CameraInfo` |
| Right camera info | `/smartdrone/stereo/right/camera_info` | `gz.msgs.CameraInfo` |
| Simulation clock | `/world/smartdrone_hover/clock` | `gz.msgs.Clock` |
| Ground-truth poses | `/world/smartdrone_hover/dynamic_pose/info` | `gz.msgs.Pose_V` |

The truth consumer must select the `Pose_V` entry named `smartdrone_x500`.
Truth comes from Gazebo's world `SceneBroadcaster`; the model deliberately does
not contain an OdometryPublisher or any perfect-odometry sensor.

Both cameras are 640x480 mono at 30 Hz with multisample anti-aliasing disabled.
The feature-textured scene does not require MSAA, and disabling Gazebo's default
4x sampling avoids the multisample render and resolve cost for both eyes. Only
the wide ceiling spotlight casts shadows. Its single forward-down shadow map
preserves the contrast needed by visual tracking without the six faces required
by a point-light shadow. The front fill light does not cast shadows. The left
camera follows
the physical `T_b_c1` translation `(0.10, -0.035, 0)` in
body FRD coordinates. The rectified baseline is exactly
`0.06992881426014412 m`. The world targets real-time factor 1.0 when the host
can sustain it. The shared measurement clock estimates the actual
Gazebo-to-steady rate, so frame timestamps, TIMESYNC and ODOMETRY remain coherent
when software rendering is slower than real time.

The indoor arena uses local geometry with irregular high-contrast marks at
multiple depths. It also exposes `/world/smartdrone_hover/wrench` and persistent
wrench topics through Gazebo's `ApplyLinkWrench` system for repeatable hover
disturbances.

The floor and wall textures are committed under
`sim/px4_gz/models/smartdrone_indoor_assets/materials/textures`. They are
generated with fixed seeds and Python's standard library, so they can be
reproduced without downloading an asset or installing an image package:

```bash
./scripts/generate_sitl_textures.py \
  --output-dir sim/px4_gz/models/smartdrone_indoor_assets/materials/textures
```

This command overwrites `floor_features.png` and `wall_features.png` with the
deterministic 1024x1024 inputs used by the world. The setup script does not run
it automatically because normal provisioning consumes the committed assets.
`check_sitl_env.sh` reports either texture as missing before a SITL launch.

## PX4 estimator profiles

The launcher applies the selected profile before EKF2 starts, inside an
isolated copy of PX4's generated startup files. It never patches the PX4 source
checkout.

- `control`: `SIM_GZ_EN_GPS=1`, `EKF2_GPS_CTRL=7`, `EKF2_EV_CTRL=0`,
  `EKF2_HGT_REF=1`.
- `truth` and `vision`: `EKF2_GPS_CTRL=0`, `EKF2_OF_CTRL=0`,
  `EKF2_EV_CTRL=15`, `EKF2_HGT_REF=3`, barometer/range aid disabled, and
  `COM_ARM_WO_GPS=2`. Gazebo bridge GPS, flow and perfect-odometry inputs are
  also disabled with `SIM_GZ_EN_GPS/FLOW/ODOM=0`.
- All profiles set `NAV_DLL_ACT=0`: the harness has no GCS, and loss handling
  is owned by the continuously monitored TLV heartbeat and visual watchdog.
- Every startup prints `param show` results and `mavlink status` into `px4.log`.

The profile is intentionally recreated on every run so saved parameters from a
previous control test cannot leak into a visual test.

## Automated scenarios

The TLV-only runner accepts the same three profiles and the `nominal`,
`impulse`, `quality`, and `loss` scenarios:

```bash
GZ_PARTITION=smartdrone_sitl SMART_DRONE_GZ_PARTITION=smartdrone_sitl \
SMART_DRONE_OFFBOARD_REQUIRES_VISION=1 \
./scripts/run_hover_sitl.py \
  --profile vision --scenario nominal --seed 1 \
  --sim-config sim/px4_gz/config/smartdrone_sim.yaml \
  --px4-command './scripts/run_px4_gz_sitl.sh --profile vision --headless' \
  --smart-drone-command './scripts/run_sitl_smart_drone.sh -- --auto-mode slam --px4-pose-output-mode position_velocity --settings sim/px4_gz/config/stereo_rectified.yaml'
```

Formal accuracy reports use the Gazebo `Pose_V` truth trajectory rather than
PX4 `LOCAL_POSITION_NED`. Preserve the scenario file, SmartDrone JSONL, PX4
ULog, truth trajectory, metrics and plots together for each seed.

Every managed run writes `scenario.yaml`, `smart_drone_frames.jsonl` (native
`slam_dfx` and `odom_ts` records), `smart_drone_states.jsonl` (TLV state
telemetry), `runner_metrics.json`, `px4_attestation.json`,
`smart_drone_attestation.json`, `px4.ulg`, `run_manifest.json`,
`trajectory.csv`, `metrics.json`, and `hover_metrics.png` when Matplotlib is
available. `runner_metrics.json` records TLV workflow and scenario events;
`metrics.json` is the ULog result after manifest verification and the formal
provenance gate. Runs that launch only one side are `attached`, while runs that
launch neither side are `external`; both retain diagnostics but remain
unverified and cannot pass formal acceptance. A control-profile report does
not require external-vision ODOMETRY. Truth and vision reports require
`vehicle_visual_odometry`, measurement latency, quality, and reset-counter
fields; missing fields make acceptance incomplete rather than silently passing.

`metrics.json` reports latency in two time domains. The formal
`visual_latency_p95_ms` check is the steady-clock `wall_total_ms` from mapped
camera exposure to MAVLink transmission. `timing_breakdown_p95_ms` also records
left/right image delivery skew as `eye_skew_ms`, render/transport, queue,
processing, send, pair-to-send and simulation-time measurement age. Gazebo
renders both eyes at one simulation timestamp, then serially renders, copies and
publishes the camera messages; `eye_skew_ms` therefore measures their steady
clock arrival difference rather than exposure-time skew. The assembler still
requires the two message timestamps to satisfy `pair_tolerance_ns`. Pair-ready
is the later eye arrival, and only a complete, monotonically ordered record
contributes to these statistics. PX4 ULog `timestamp - timestamp_sample` is
retained separately and is not used as an end-to-end latency substitute.

Mesa llvmpipe is suitable for functional closed-loop testing, but a host that
runs the camera near 0.2 real-time factor normally fails the 80 ms wall-clock
latency requirement because rendering and image delivery are correspondingly
slower. A formal latency pass requires a near-real-time hardware-rendered run;
internal KLT processing time or simulation-time age must not be presented as
the wall-clock result.

Steady hover accuracy uses the runner's absolute Gazebo clock interval from
`hover_started` through `hover_finished`. If that interval is unavailable or
crosses a clock reset, analysis falls back to the stable 0.15 m altitude band.
The preceding 0.30 m target-approach interval is reported separately as settling
accuracy. Impulse `duration_ms` is also measured from the Gazebo world clock, so
the applied impulse is invariant to RTF and simulation pauses. The runner's
outer command deadline uses the same 0.05 minimum-RTF budget plus cleanup grace,
and wrench-clear calls have a bounded wall timeout.

The default `loss` run applies a 300 ms blackout at hover second 10 and requires
0.5 seconds of stable visual recovery before applying a 750 ms blackout at
second 20. It passes only when the controlled landing is observed after the
long blackout without a runner-issued cleanup LAND. Because that landing is
intentional, `loss` does not apply the normal 30-second hover-duration check.

Analyze a collected ULog independently with an explicit profile:

```bash
./scripts/analyze_px4_ulog.py output/run/px4.ulg \
  --output-dir output/run --profile vision --scenario nominal
```

## Fixed-seed quality matrix

The matrix orchestrator launches a fresh `run_hover_sitl.py` process for every
case and seed. By default it runs seeds 1 through 5 for `nominal`, `blur`,
`dark`, `noise`, `drop5`, `drop10`, `delay40`, and `delay80`:

```bash
./scripts/run_hover_matrix.py \
  --px4-command './scripts/run_px4_gz_sitl.sh --profile vision --headless' \
  --smart-drone-command './scripts/run_sitl_smart_drone.sh -- --auto-mode slam --px4-pose-output-mode position_velocity --settings sim/px4_gz/config/stereo_rectified.yaml' \
  --sim-config sim/px4_gz/config/smartdrone_sim.yaml
```

Both launcher arguments are required. Reusing attached processes would keep
the old Gazebo seed and fault-state path, so it is rejected for matrix runs.

Use `--cases nominal drop5 --seeds 1 2` for a smaller diagnostic run. Each run
has an isolated `<case>/seed_<NNN>/` artifact directory. The matrix root stores
`quality_matrix.csv`, `quality_matrix.json`, and `quality_matrix.png` when
Matplotlib is available. A failed run is retained in both aggregate files,
later runs still execute, and the matrix command exits nonzero at the end. The
matrix succeeds only when every run has a complete formal report and every
acceptance check passes; a runner exit without a ULog report is not counted as
a successful matrix result.

References:

- [PX4 v1.17 Gazebo simulation](https://docs.px4.io/v1.17/en/sim_gazebo_gz/)
- [Gazebo Harmonic Ubuntu packages](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [PX4 EKF2 external vision](https://docs.px4.io/v1.17/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system)
