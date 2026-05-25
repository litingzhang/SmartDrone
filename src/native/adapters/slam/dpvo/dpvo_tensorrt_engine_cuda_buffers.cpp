#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_buffers.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

CudaDeviceBuffer::~CudaDeviceBuffer()
{
    Reset();
}

bool CudaDeviceBuffer::Ensure(size_t bytes, std::string *err)
{
    if (bytes <= m_bytes) {
        return true;
    }
    Reset();
    const cudaError_t rc = cudaMalloc(&m_ptr, bytes);
    if (rc != cudaSuccess) {
        if (err != nullptr) {
            *err = std::string("cudaMalloc failed bytes=") +
                   std::to_string(bytes) + ": " + cudaGetErrorString(rc);
        }
        m_ptr = nullptr;
        m_bytes = 0;
        return false;
    }
    m_bytes = bytes;
    return true;
}

void CudaDeviceBuffer::Reset()
{
    if (m_ptr != nullptr) {
        cudaFree(m_ptr);
        m_ptr = nullptr;
    }
    m_bytes = 0;
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
