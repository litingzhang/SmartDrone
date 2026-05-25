#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_engine_factory.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_image_utils.h"
#include "adapters/slam/engine/slam_mode_state.h"
#include "adapters/slam/engine/slam_pose_utils.h"
#include "adapters/slam/dpvo/dpvo_runtime_options.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

#include "dpvo_tensorrt_engine_fallback.h"

DpvoTensorRtEngine::DpvoTensorRtEngine(DpvoTensorRtConfig config)
    : m_impl(std::make_unique<Impl>(std::move(config)))
{
}
DpvoTensorRtEngine::~DpvoTensorRtEngine() = default;

bool DpvoTensorRtEngine::Start()
{
    return m_impl != nullptr && m_impl->Start();
}

void DpvoTensorRtEngine::Stop()
{
    if (m_impl != nullptr) {
        m_impl->Stop();
    }
}

Core::Ports::SlamOutput
DpvoTensorRtEngine::Process(const Core::Ports::SlamInputBatch &input,
                            bool extractFeatures, bool extractPointCloud)
{
    return m_impl != nullptr
               ? m_impl->Process(input, extractFeatures, extractPointCloud)
               : Core::Ports::SlamOutput{};
}

namespace {

ControlledSlamEngine
CreateDpvoTensorRtSlamEngine(const SlamEngineFactoryConfig &config)
{
    DpvoTensorRtConfig dpvoConfig =
        MakeDpvoTensorRtConfig(config.dpvoRuntime, config.settingsPath);
    ControlledSlamEngine out{};
    out.engine = std::make_unique<DpvoTensorRtEngine>(std::move(dpvoConfig));
    return out;
}

const SlamEngineFactoryRegistrar
    DPVO_TENSORRT_SLAM_ENGINE_REGISTRAR(SlamBackend::DpvoTensorRt,
                                     CreateDpvoTensorRtSlamEngine);

} // namespace

} // namespace SmartDrone::Adapters::Slam
