# Threading Topology

SmartDrone-owned thread creation is centralized in
[thread_launch.h](/d:/SmartDrone/src/native/common/thread_launch.h).

Each owned thread now logs a launch line like:
`[thread] launch role=... owner=... file=...:line mode=...`

## Owned Threads

| Role | Entry | Source |
| --- | --- | --- |
| `runtime_worker` | `UnifiedRuntimeController::Start()` | [runtime_controller.h](/d:/SmartDrone/src/native/core/application/runtime/runtime_controller.h) |
| `runtime_session` | `UnifiedRuntimeController::Loop()` | [runtime_controller.cpp](/d:/SmartDrone/src/native/core/application/runtime/runtime_controller.cpp) |
| `runtime_force_restart` | `ExecuteAction(ForceRestart)` | [runtime_controller.cpp](/d:/SmartDrone/src/native/core/application/runtime/runtime_controller.cpp) |
| `mavlink_rx` | `Px4MavlinkGateway::StartRx()` | [px4_mavlink_gateway.h](/d:/SmartDrone/src/native/adapters/telemetry/px4_mavlink_gateway.h) |
| `mavlink_timesync` | `Px4MavlinkGateway::StartRx()` | [px4_mavlink_gateway.h](/d:/SmartDrone/src/native/adapters/telemetry/px4_mavlink_gateway.h) |
| `mavlink_setpoint_stream` | `Px4MavlinkGateway::StartSetpointStreamHz()` | [px4_mavlink_gateway.h](/d:/SmartDrone/src/native/adapters/telemetry/px4_mavlink_gateway.h) |
| `udp_image_cam0` | `UdpImageSender::Open()` | [udp_image_sender.h](/d:/SmartDrone/src/native/adapters/stream/udp_image_sender.h) |
| `udp_image_cam1` | `UdpImageSender::Open()` | [udp_image_sender.h](/d:/SmartDrone/src/native/adapters/stream/udp_image_sender.h) |
| `udp_command` | `StartUdpCommandThread()` | [udp_command_thread.h](/d:/SmartDrone/src/native/core/application/runtime/udp_command_thread.h) |
| `imu` | `StartImuThread()` | [sensor_runtime_helpers.h](/d:/SmartDrone/src/native/core/application/session/sensor_runtime_helpers.h) |
| `calib_imu_writer` | `StartCalibImuWriterThread()` | [sensor_runtime_helpers.h](/d:/SmartDrone/src/native/core/application/session/sensor_runtime_helpers.h) |
| `manual_control` | `Px4UdpHooks` ctor | [px4_udp_hooks.h](/d:/SmartDrone/src/native/core/application/runtime/px4_udp_hooks.h) |

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
