#include "core/application/session/calib/calib_storage_port.h"

#include <mutex>
#include <utility>

#include "core/application/session/calib/calib_output_store.h"

namespace SmartDrone::core::application {

class CalibStoragePort::Impl final {
  public:
    explicit Impl(CalibStoragePortConfig config)
        : m_store(new CalibOutputStore({std::move(config.root)}))
    {
    }

    bool Open()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        m_closed = false;
        const bool opened = m_store->Open();
        m_closed = !opened;
        return opened;
    }

    bool WriteSavePair(const CalibSavePair &pair)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_closed) {
            return false;
        }
        return m_store->WriteSavePair(pair);
    }

    bool WriteImuSample(const ImuSample &sample)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_closed) {
            return false;
        }
        return m_store->WriteImuSample(sample);
    }

    void FlushAndClose()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_closed) {
            return;
        }
        m_store->FlushAndClose();
        m_closed = true;
    }

    int SavedCount() const
    {
        return m_store->SavedCount();
    }

    std::string OutputRoot() const
    {
        return m_store->OutputRoot();
    }

    std::filesystem::path Cam0Dir() const
    {
        return m_store->Cam0Dir();
    }

    std::filesystem::path Cam1Dir() const
    {
        return m_store->Cam1Dir();
    }

  private:
    mutable std::mutex m_mu;
    std::unique_ptr<CalibOutputStore> m_store;
    bool m_closed{true};
};

CalibStoragePort::CalibStoragePort(CalibStoragePortConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

CalibStoragePort::~CalibStoragePort() = default;

bool CalibStoragePort::Open()
{
    return m_impl->Open();
}

bool CalibStoragePort::WriteSavePair(const CalibSavePair &pair)
{
    return m_impl->WriteSavePair(pair);
}

bool CalibStoragePort::WriteImuSample(const ImuSample &sample)
{
    return m_impl->WriteImuSample(sample);
}

void CalibStoragePort::FlushAndClose()
{
    m_impl->FlushAndClose();
}

int CalibStoragePort::SavedCount() const
{
    return m_impl->SavedCount();
}

std::string CalibStoragePort::OutputRoot() const
{
    return m_impl->OutputRoot();
}

std::filesystem::path CalibStoragePort::Cam0Dir() const
{
    return m_impl->Cam0Dir();
}

std::filesystem::path CalibStoragePort::Cam1Dir() const
{
    return m_impl->Cam1Dir();
}

} // namespace SmartDrone::core::application
