#include "core/application/session/calib/calib_save_pacing_port.h"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <utility>

#include "core/application/runtime/runtime_aliases.h"
#include "core/application/session/calib/calib_storage_helpers.h"

namespace SmartDrone::core::application {
namespace {

constexpr int RECOMMENDED_MAX_CALIB_SAVE_FPS = 30;
constexpr std::int64_t NANOSECONDS_PER_SECOND = 1000000000LL;

int ClampCalibSaveFps(int requestedFps, int cameraFps)
{
    const int baseFps = ClampSlamInputFps(requestedFps, cameraFps);
    const int maxFps =
        std::max(1, std::min(cameraFps, RECOMMENDED_MAX_CALIB_SAVE_FPS));
    return std::clamp(baseFps, 1, maxFps);
}

} // namespace

CalibSavePacingPort::CalibSavePacingPort(CalibSavePacingPortConfig config)
    : m_config(std::move(config))
{
    ConfigureSavePacing();
}

bool CalibSavePacingPort::TryBuildSavePair(
    std::shared_ptr<CalibStereoFrame> frame,
    CalibSavePair &savePair)
{
    if (!frame) {
        return false;
    }
    std::int64_t pairNs = FramePairTimestampNs(*frame);
    if (!NextEligibleSave(pairNs)) {
        return false;
    }
    savePair.frame = std::move(frame);
    savePair.pairNs = pairNs;
    savePair.name = TsToName(pairNs);
    savePair.fnL = m_config.cam0Dir / savePair.name;
    savePair.fnR = m_config.cam1Dir / savePair.name;
    return true;
}

void CalibSavePacingPort::ConfigureSavePacing()
{
    m_saveFps = ClampCalibSaveFps(m_config.requestedSaveFps,
                                  m_config.cameraFps);
    m_saveStepNs = NANOSECONDS_PER_SECOND / std::max(1, m_saveFps);
    std::cerr << "[calib] target_save_fps=" << m_saveFps
              << " configured_camera_fps=" << m_config.cameraFps
              << " requested_slam_fps=" << m_config.requestedSaveFps
              << "\n";
}

bool CalibSavePacingPort::NextEligibleSave(std::int64_t &pairNs)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_nextEligibleSaveNs != 0 && pairNs < m_nextEligibleSaveNs) {
        ++m_droppedByPacing;
        LogPacingDrop(pairNs);
        return false;
    }
    if (m_lastPairNs != 0 && pairNs <= m_lastPairNs) {
        pairNs = m_lastPairNs + 1;
    }
    m_lastPairNs = pairNs;
    m_nextEligibleSaveNs = pairNs + m_saveStepNs;
    return true;
}

void CalibSavePacingPort::LogPacingDrop(std::int64_t pairNs) const
{
    if ((m_droppedByPacing % 30) != 1) {
        return;
    }
    std::cerr << "[calib-pace] dropped=" << m_droppedByPacing
              << " pair_ts_ns=" << pairNs
              << " next_save_ts_ns=" << m_nextEligibleSaveNs
              << " target_save_fps=" << m_saveFps << "\n";
}

std::int64_t CalibSavePacingPort::FramePairTimestampNs(
    const CalibStereoFrame &frame)
{
    const auto &left = frame.stereo.left;
    const auto &right = frame.stereo.right;
    return static_cast<std::int64_t>((left.timestampNs +
                                      right.timestampNs) /
                                     2);
}

} // namespace SmartDrone::core::application
