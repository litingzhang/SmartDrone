#pragma once

#include <cstddef>
#include <string>

#include <cuda_runtime_api.h>

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

class CudaDeviceBuffer {
  public:
    ~CudaDeviceBuffer();

    CudaDeviceBuffer() = default;
    CudaDeviceBuffer(const CudaDeviceBuffer &) = delete;
    CudaDeviceBuffer &operator=(const CudaDeviceBuffer &) = delete;

    bool Ensure(size_t bytes, std::string *err);
    void Reset();

    void *Data() const
    {
        return m_ptr;
    }
    size_t Bytes() const
    {
        return m_bytes;
    }

  private:
    void *m_ptr{nullptr};
    size_t m_bytes{0};
};

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
