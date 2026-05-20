#pragma once

#include <memory>

#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace smartdrone::core::application {

struct MainRuntimeAliases;

class CalibPreviewPort final {
  public:
    explicit CalibPreviewPort(const MainRuntimeAliases &aliases);
    ~CalibPreviewPort();

    bool Open();
    bool Enqueue(const CalibStereoFrame &frame);
    void Close();
    bool Opened() const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace smartdrone::core::application
