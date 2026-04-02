# Threading Topology

SmartDrone-owned thread creation is centralized in
[thread_launch.hpp](/d:/SmartDrone/src/common/thread_launch.hpp).

Each owned thread now logs a launch line like:
`[thread] launch role=... owner=... file=...:line mode=...`

## Owned Threads

| Role | Entry | Source |
| --- | --- | --- |
| `runtime_worker` | `UnifiedRuntimeController::Start()` | [runtime_controller.hpp](/d:/SmartDrone/src/core/application/runtime_controller.hpp) |
| `runtime_session` | `UnifiedRuntimeController::Loop()` | [runtime_controller.hpp](/d:/SmartDrone/src/core/application/runtime_controller.hpp) |
| `runtime_force_restart` | `ExecuteAction(ForceRestart)` | [runtime_controller.hpp](/d:/SmartDrone/src/core/application/runtime_controller.hpp) |
| `mavlink_rx` | `Px4MavlinkGateway::StartRx()` | [px4_mavlink_gateway.hpp](/d:/SmartDrone/src/adapters/telemetry/px4_mavlink_gateway.hpp) |
| `mavlink_timesync` | `Px4MavlinkGateway::StartRx()` | [px4_mavlink_gateway.hpp](/d:/SmartDrone/src/adapters/telemetry/px4_mavlink_gateway.hpp) |
| `mavlink_setpoint_stream` | `Px4MavlinkGateway::StartSetpointStreamHz()` | [px4_mavlink_gateway.hpp](/d:/SmartDrone/src/adapters/telemetry/px4_mavlink_gateway.hpp) |
| `udp_image_cam0` | `UdpImageSender::Open()` | [udp_image_sender.hpp](/d:/SmartDrone/src/adapters/stream/udp_image_sender.hpp) |
| `udp_image_cam1` | `UdpImageSender::Open()` | [udp_image_sender.hpp](/d:/SmartDrone/src/adapters/stream/udp_image_sender.hpp) |
| `udp_command` | `StartUdpCommandThread()` | [udp_command_thread.hpp](/d:/SmartDrone/src/core/application/udp_command_thread.hpp) |
| `imu` | `StartImuThread()` | [sensor_runtime_helpers.hpp](/d:/SmartDrone/src/core/application/sensor_runtime_helpers.hpp) |
| `calib_imu_writer` | `StartCalibImuWriterThread()` | [sensor_runtime_helpers.hpp](/d:/SmartDrone/src/core/application/sensor_runtime_helpers.hpp) |
| `manual_control` | `Px4UdpHooks` ctor | [px4_udp_hooks.hpp](/d:/SmartDrone/src/core/application/px4_udp_hooks.hpp) |

## ORB-SLAM3 Threads

These threads are created by third-party ORB-SLAM3 code, not by SmartDrone.

| Role | Source |
| --- | --- |
| `LocalMapping` | [System.cc](/d:/SmartDrone/ORB_SLAM3/src/System.cc) |
| `LoopClosing` | [System.cc](/d:/SmartDrone/ORB_SLAM3/src/System.cc) |
| `GlobalBundleAdjustment` | [LoopClosing.cc](/d:/SmartDrone/ORB_SLAM3/src/LoopClosing.cc) |

## Reading Rule

- Search `ThreadRole::` for SmartDrone-owned threads.
- Search the ORB-SLAM3 files above for third-party worker threads.
