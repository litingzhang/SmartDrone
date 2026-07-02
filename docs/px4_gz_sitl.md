# PX4 SITL + Gazebo Sim

This project can connect its PX4 control path to PX4 SITL running with
Gazebo Sim (GZ). The first supported integration layer is MAVLink UDP:
SmartDrone can receive PX4 heartbeat/local-position/range telemetry and send
manual control, offboard setpoints, commands, and visual odometry messages over
the same gateway used on hardware.

## Prerequisites

Install PX4-Autopilot and its Gazebo Sim dependencies by following the PX4
Gazebo Simulation documentation:

- https://docs.px4.io/main/en/sim_gazebo_gz/

Keep the PX4 checkout outside this repository, for example:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git ../PX4-Autopilot
cd ../PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

## Start PX4 SITL

From this repository:

```bash
./scripts/run_px4_gz_sitl.sh --model x500
```

Useful variants:

```bash
./scripts/run_px4_gz_sitl.sh --model x500_depth --headless
./scripts/run_px4_gz_sitl.sh --model x500_lidar_down --world default
```

The script prints the SmartDrone MAVLink endpoint to use. The default endpoint
is:

```bash
export SMART_DRONE_MAVLINK_DEV='udp-listen://14540'
export SMART_DRONE_MAVLINK_BAUD=921600
```

Use a different bind port if another ground station or MAVLink client already
uses 14540:

```bash
./scripts/run_px4_gz_sitl.sh --bind 14640
export SMART_DRONE_MAVLINK_DEV='udp-listen://14640'
```

## Run SmartDrone Against SITL

Build the runtime through the project build script:

```bash
./scripts/build.sh smart_drone --camera-provider uvc_stereo_opencv
```

Then run the produced native artifact with the endpoint above. For a control
loop smoke test that does not require real camera input, keep the runtime in
idle and use the Android/UDP command surface to arm, switch mode, or send
manual commands.

## Current Scope

Supported now:

- MAVLink over serial device paths, as before.
- MAVLink over UDP endpoints such as `udp://127.0.0.1:14540`.
- Optional local bind address via `?bind=HOST:PORT`.
- Passive UDP listen endpoints such as `udp-listen://14540`.
- PX4 companion heartbeat emitted once per second.

Next integration step:

- Add a Gazebo/ROS 2 sensor bridge that implements the existing
  `CameraProvider` and `ImuProvider` ports, so simulated stereo images and IMU
  samples can drive the SLAM session directly.

The transport is intentionally below the EPG runtime boundary: polling,
setpoint streaming, and manual control continue to be stepped by the existing
system runtime graph.
