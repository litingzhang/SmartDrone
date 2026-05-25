#include "core/application/session/calib/calib_preview_port.h"

#include <iostream>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/session/stream/preview_output_port.h"

namespace SmartDrone::Core::Application {

class CalibPreviewPort::Impl final {
  public:
    Impl(const MainRuntimeAliases &aliases,
         const ApplicationRuntimeFactories &factories)
        : m_aliases(aliases),
          m_previewOutput(factories.createPreviewOutputRuntime())
    {
    }

    bool Open()
    {
        if (!m_aliases.udpEnable || !m_aliases.sendImage) {
            return true;
        }
        if (!m_previewOutput) {
            return false;
        }
        m_opened = m_previewOutput->OpenStaticPeer(
            MakePreviewOutputOpenConfig(m_aliases));
        return m_opened;
    }

    bool Enqueue(const CalibStereoFrame &frame)
    {
        if (!m_aliases.udpEnable || !m_aliases.sendImage) {
            return true;
        }
        if (!m_opened) {
            return false;
        }
        m_previewOutput->EnqueueCalibStereoFrame(frame);
        return true;
    }

    void Close()
    {
        if (!m_opened) {
            return;
        }
        m_previewOutput->Close();
        m_opened = false;
        std::cerr << "[session] calib udp closed\n";
    }

    bool Opened() const
    {
        return m_opened;
    }

  private:
    const MainRuntimeAliases &m_aliases;
    std::unique_ptr<IPreviewOutputRuntime> m_previewOutput;
    bool m_opened{false};
};

CalibPreviewPort::CalibPreviewPort(
    const MainRuntimeAliases &aliases,
    const ApplicationRuntimeFactories &factories)
    : m_impl(new Impl(aliases, factories))
{
}

CalibPreviewPort::~CalibPreviewPort() = default;

bool CalibPreviewPort::Open()
{
    return m_impl->Open();
}

bool CalibPreviewPort::Enqueue(const CalibStereoFrame &frame)
{
    return m_impl->Enqueue(frame);
}

void CalibPreviewPort::Close()
{
    m_impl->Close();
}

bool CalibPreviewPort::Opened() const
{
    return m_impl->Opened();
}

} // namespace SmartDrone::Core::Application
