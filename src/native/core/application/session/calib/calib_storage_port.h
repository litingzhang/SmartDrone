#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "core/application/session/epg/messages/calib_epg_messages.h"
#include "core/application/sensors/imu_runtime_state.h"

namespace SmartDrone::Core::Application {

struct CalibStoragePortConfig {
    std::string root;
};

class CalibStoragePort final {
  public:
    explicit CalibStoragePort(CalibStoragePortConfig config);
    ~CalibStoragePort();

    bool Open();
    bool WriteSavePair(const CalibSavePair &pair);
    bool WriteImuSample(const ImuSample &sample);
    void FlushAndClose();
    int SavedCount() const;
    std::string OutputRoot() const;
    std::filesystem::path Cam0Dir() const;
    std::filesystem::path Cam1Dir() const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
