#pragma once

#include <atomic>
#include <cstdio>
#include <thread>

#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"
#include "core/application/runtime_session_common.h"

namespace smartdrone::core::application {

std::thread StartImuThread(
    const MainRuntimeAliases& a,
    ImuThreadState& s,
    std::atomic<bool>& stop,
    std::atomic<bool>& runningFlag);

std::thread StartCalibImuWriterThread(
    const MainRuntimeAliases& a,
    FILE* fImu,
    std::atomic<bool>& imuOk,
    std::atomic<bool>& stop,
    std::atomic<bool>& runningFlag);

bool OpenCamera(LibcameraStereoOV9281_TsPair& cam, const MainRuntimeAliases& a);

}  // namespace smartdrone::core::application
