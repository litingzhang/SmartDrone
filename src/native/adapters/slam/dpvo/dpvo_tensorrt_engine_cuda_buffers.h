#pragma once

#include <cstddef>
#include <string>

#include <cuda_runtime_api.h>

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

class CudaDeviceBuffer {
  public:
    ~CudaDeviceBuffer()
    {
        Reset();
    }

    CudaDeviceBuffer() = default;
    CudaDeviceBuffer(const CudaDeviceBuffer &) = delete;
    CudaDeviceBuffer &operator=(const CudaDeviceBuffer &) = delete;

    bool Ensure(size_t bytes, std::string *err)
    {
        if (bytes <= m_bytes) {
            return true;
        }
        Reset();
        const cudaError_t rc = cudaMalloc(&m_ptr, bytes);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaMalloc failed bytes=") + std::to_string(bytes) +
                       ": " + cudaGetErrorString(rc);
            }
            m_ptr = nullptr;
            m_bytes = 0;
            return false;
        }
        m_bytes = bytes;
        return true;
    }

    void Reset()
    {
        if (m_ptr != nullptr) {
            cudaFree(m_ptr);
            m_ptr = nullptr;
        }
        m_bytes = 0;
    }

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
