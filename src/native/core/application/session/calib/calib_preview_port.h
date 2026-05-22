#pragma once

#include <memory>

#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace SmartDrone::Core::Application {

struct ApplicationRuntimeFactories;
struct MainRuntimeAliases;

class CalibPreviewPort final {
  public:
    CalibPreviewPort(const MainRuntimeAliases &aliases,
                     const ApplicationRuntimeFactories &factories);
    ~CalibPreviewPort();

    bool Open();
    bool Enqueue(const CalibStereoFrame &frame);
    void Close();
    bool Opened() const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
