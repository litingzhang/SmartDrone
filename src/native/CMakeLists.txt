set(SMART_DRONE_NATIVE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

set(SMART_DRONE_MAIN_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/app/bootstrap/runtime_host.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/app/bootstrap/signal_handlers.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/main.cpp"
)

set(SMART_DRONE_CAMERA_SOURCES)
if(SMART_DRONE_CAMERA_PROVIDER STREQUAL "libcamera_stereo_ov9281")
  list(APPEND SMART_DRONE_CAMERA_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/libcamera_ov9281/stereo_ov9281.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/libcamera_stereo_camera.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/camera_provider_factory_libcamera.cpp"
  )
elseif(SMART_DRONE_CAMERA_PROVIDER STREQUAL "uvc_stereo_opencv")
  list(APPEND SMART_DRONE_CAMERA_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/uvc_stereo_camera.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/camera_provider_factory_uvc.cpp"
  )
else()
  message(FATAL_ERROR
    "Unsupported SMART_DRONE_CAMERA_PROVIDER='${SMART_DRONE_CAMERA_PROVIDER}'. "
    "Currently supported: libcamera_stereo_ov9281, uvc_stereo_opencv")
endif()

set(SMART_DRONE_ADAPTER_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/adapters/command/udp_tlv_command_channel.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688/icm42688_imu.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688_imu_provider.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orbslam3_engine.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/xfeat_frontend_client.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/stream/udp_image_sender.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/mavlink_pose_publisher.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/mavlink_serial_transport.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/px4_mavlink_gateway.cpp"
  ${SMART_DRONE_CAMERA_SOURCES}
)

set(SMART_DRONE_COMMON_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/common/discovery/udp_discovery_beacon.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/runtime_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/thread_launch.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/time_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/crc16_ccitt_false.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/tlv_cmd_router.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/tlv_pack.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/udp_server.cpp"
)

set(SMART_DRONE_CORE_DOMAIN_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/core/domain/runtime_mode.cpp"
)

set(SMART_DRONE_CORE_APPLICATION_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/app_args.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/capability_catalog.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/config_registry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/mode_manager.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/payload_builders.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/px4_udp_hooks.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_command_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_controller.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_session_supervisor.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/udp_command_thread.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib_session_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib_storage_helpers.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam_frame_processor.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/runtime_session_common.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/sensor_runtime_helpers.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam_processing_support.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam_session_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam_session_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/frame_timing_tracker.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/imu_buffer.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/live_pose_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/perception_pipeline.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/pose_postprocessor.cpp"
)

set(SMART_DRONE_PLATFORM_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/platform/linux/gpio/drdy_gpio.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/platform/linux/spi/spi_dev.cpp"
)

set(SMART_DRONE_ALL_SOURCES
  ${SMART_DRONE_MAIN_SOURCES}
  ${SMART_DRONE_ADAPTER_SOURCES}
  ${SMART_DRONE_COMMON_SOURCES}
  ${SMART_DRONE_CORE_DOMAIN_SOURCES}
  ${SMART_DRONE_CORE_APPLICATION_SOURCES}
  ${SMART_DRONE_PLATFORM_SOURCES}
)

add_executable(smart_drone)

target_sources(smart_drone PRIVATE
  ${SMART_DRONE_ALL_SOURCES}
)

target_include_directories(smart_drone PRIVATE
  "${SMART_DRONE_NATIVE_ROOT}"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv"
)

source_group(TREE "${SMART_DRONE_NATIVE_ROOT}" FILES
  ${SMART_DRONE_ALL_SOURCES}
)
