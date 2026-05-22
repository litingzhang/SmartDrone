#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>

#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace SmartDrone::core::application {

struct CalibSavePacingPortConfig {
    int requestedSaveFps{1};
    int cameraFps{1};
    std::filesystem::path cam0Dir;
    std::filesystem::path cam1Dir;
};

class CalibSavePacingPort final {
  public:
    explicit CalibSavePacingPort(CalibSavePacingPortConfig config);

    bool TryBuildSavePair(std::shared_ptr<CalibStereoFrame> frame,
                          CalibSavePair &savePair);

  private:
    void ConfigureSavePacing();
    bool NextEligibleSave(std::int64_t &pairNs);
    void LogPacingDrop(std::int64_t pairNs) const;
    static std::int64_t FramePairTimestampNs(const CalibStereoFrame &frame);

    CalibSavePacingPortConfig m_config;
    std::mutex m_mu;
    int m_saveFps{1};
    std::int64_t m_saveStepNs{1000000000LL};
    std::int64_t m_lastPairNs{0};
    std::int64_t m_nextEligibleSaveNs{0};
    int m_droppedByPacing{0};
};

} // namespace SmartDrone::core::application
