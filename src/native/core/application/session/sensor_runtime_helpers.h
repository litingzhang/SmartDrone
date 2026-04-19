#pragma once

#include <atomic>
#include <cstdio>
#include <memory>
#include <string_view>
#include <thread>

#include "core/application/session/runtime_session_common.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {

std::thread StartImuThread(const MainRuntimeAliases &a, ImuThreadState &s, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag);

std::thread StartCalibImuWriterThread(const MainRuntimeAliases &a, FILE *fImu, std::atomic<bool> &imuOk,
                                      std::atomic<bool> &stop, std::atomic<bool> &runningFlag);

std::unique_ptr<smartdrone::core::ports::ICameraProvider> CreateCameraProvider();
std::string_view CompiledCameraProviderName();
bool CompiledCameraProviderUsesPackedStereo();

} // namespace smartdrone::core::application
