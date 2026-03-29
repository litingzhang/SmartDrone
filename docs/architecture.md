# SmartDrone Architecture

## Goals

This repository needs to evolve in four independent directions:

1. Replaceable device stack: sensors, cameras, IMU chips, transport buses, and main SoC.
2. Replaceable perception stack: stereo, stereo-IMU, future mono/RGB-D, and different SLAM libraries.
3. Expandable control plane: Android app commands, runtime config, capability negotiation, and telemetry.
4. Safe runtime mode switching: idle, slam, calibration, playback, mapping, localization.

The architecture should keep these axes decoupled so that changing one axis does not force broad rewrites.

## Layering

```text
src/
  main.cpp               # composition root, bootstrapping, process entry

  core/
    domain/              # stable data model and enums
    ports/               # replaceable interfaces
    application/         # runtime orchestration and use cases

  adapters/
    camera/              # libcamera, realsense, playback...
    imu/                 # icm42688, mock imu...
    slam/                # ORB-SLAM3, future engines
    telemetry/           # MAVLink, ROS2, logging sinks
    command/             # UDP/TLV, future transports

  platform/
    linux/               # gpio, spi, thread priority, clocks

  common/
    tlv/                 # shared TLV framing helpers

config/
  stereo.yaml
  stereo_inertial.yaml
```

## Stable Core Concepts

The core layer should own the canonical types used by the application layer:

- `RuntimeMode`
- `PerceptionMode`
- `SlamOperationMode`
- `RuntimeSelection`
- `RuntimeCapabilities`
- `StereoFrame`
- `ImuSample`
- `PoseEstimate`
- `PerceptionOutput`

The application layer should only depend on these types plus ports.

## Ports

The following ports form the long-term extension boundary:

- `ICameraProvider`
- `IImuProvider`
- `ISlamEngine`
- `IPosePublisher`
- `ICommandChannel`

Every hardware- or library-specific implementation lives in `adapters/`.

## Runtime Mode Model

Runtime control is split into three dimensions:

1. `RuntimeMode`
   - `Idle`
   - `Slam`
   - `Calib`
   - `Playback`

2. `PerceptionMode`
   - `Stereo`
   - `StereoImu`
   - future `Mono`, `MonoImu`, `Rgbd`

3. `SlamOperationMode`
   - `Mapping`
   - `Localization`
   - `Relocalization`
   - `TrackingOnly`

This avoids mixing transport state, sensor topology, and algorithm behavior into one enum.

## App Command Expansion Principles

The app-facing control plane should distinguish:

- actions: start, stop, reset, clean calibration, save map
- configuration: `camera.exposure_us`, `slam.input_fps`, `slam.mode`
- capability queries: what this device and build support

Recommended command shape:

- `CMD_GET_CAPABILITIES`
- `CMD_GET_CONFIG`
- `CMD_SET_CONFIG`
- `CMD_EXEC_ACTION`

The current TLV protocol can be evolved in this direction while preserving the outer frame format.

## Refactor Strategy

Refactor in small, behavior-preserving steps:

1. Introduce core types and ports.
2. Wrap existing implementations as adapters.
3. Move mode/state logic from `main.cpp` into `core/application`.
4. Move session pipelines behind ports.
5. Evolve the command protocol toward capability-driven configuration.

## First-Step Refactor in This Commit

This refactor starts by:

- adding core domain mode types
- adding capability/config domain types
- adding first-class ports for camera, IMU, SLAM, pose publisher, and command channel
- adding adapter scaffolds for existing libcamera, ORB-SLAM3, MAVLink, and UDP/TLV implementations
- extracting runtime mode state management into `core/application/mode_manager.hpp`

The current runtime still executes the old pipeline, but new code should build against the new boundaries.
