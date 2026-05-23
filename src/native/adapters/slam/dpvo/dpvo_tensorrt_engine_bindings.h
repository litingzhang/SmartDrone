inline size_t TensorRtDataTypeSize(nvinfer1::DataType type)
{
    switch (type) {
    case nvinfer1::DataType::kFLOAT:
        return 4;
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kBOOL:
        return 1;
    case nvinfer1::DataType::kINT32:
        return 4;
    default:
        return 0;
    }
}

inline int64_t DimsVolume(const nvinfer1::Dims &dims)
{
    int64_t volume = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] < 0) {
            return -1;
        }
        volume *= dims.d[i];
    }
    return volume;
}

inline std::string DimsToString(const nvinfer1::Dims &dims)
{
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < dims.nbDims; ++i) {
        if (i > 0) {
            oss << "x";
        }
        oss << dims.d[i];
    }
    oss << "]";
    return oss.str();
}

inline int BindingCount(const TensorRtEngineHandle &handle)
{
    return handle.Engine() != nullptr ? handle.Engine()->getNbIOTensors() : 0;
}

inline const char *BindingName(const TensorRtEngineHandle &handle, int bindingIndex)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    if (engine == nullptr || bindingIndex < 0 ||
        bindingIndex >= engine->getNbIOTensors()) {
        return nullptr;
    }
    return engine->getIOTensorName(bindingIndex);
}

inline bool BindingIsInput(const TensorRtEngineHandle &handle, int bindingIndex)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    const char *name = BindingName(handle, bindingIndex);
    return engine != nullptr && name != nullptr &&
           engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
}

inline nvinfer1::DataType BindingDataType(const TensorRtEngineHandle &handle,
                                          int bindingIndex)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    const char *name = BindingName(handle, bindingIndex);
    if (engine == nullptr || name == nullptr) {
        return nvinfer1::DataType::kFLOAT;
    }
    return engine->getTensorDataType(name);
}

inline bool SetBindingShape(TensorRtEngineHandle &handle, int bindingIndex,
                            const nvinfer1::Dims &dims, std::string *err)
{
    if (bindingIndex < 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT tensor index";
        }
        return false;
    }
    nvinfer1::IExecutionContext *context = handle.Context();
    const char *name = BindingName(handle, bindingIndex);
    if (context == nullptr || name == nullptr) {
        if (err != nullptr) {
            *err = "invalid TensorRT context/tensor";
        }
        return false;
    }
    if (!context->setInputShape(name, dims)) {
        if (err != nullptr) {
            *err = "setInputShape failed for tensor " + std::string(name) +
                   " dims=" + DimsToString(dims);
        }
        return false;
    }
    return true;
}

inline int FindBindingIndex(const TensorRtEngineHandle &handle, const char *name)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    if (engine == nullptr || name == nullptr) {
        return -1;
    }
    for (int i = 0; i < engine->getNbIOTensors(); ++i) {
        const char *tensorName = engine->getIOTensorName(i);
        if (tensorName != nullptr && std::string(tensorName) == name) {
            return i;
        }
    }
    return -1;
}

inline bool BindingBytes(const TensorRtEngineHandle &handle, int bindingIndex,
                  size_t *bytes, nvinfer1::Dims *dimsOut, std::string *err)
{
    if (bytes == nullptr) {
        return false;
    }
    nvinfer1::ICudaEngine *engine = handle.Engine();
    nvinfer1::IExecutionContext *context = handle.Context();
    const char *name = BindingName(handle, bindingIndex);
    if (engine == nullptr || context == nullptr || name == nullptr) {
        if (err != nullptr) {
            *err = "invalid TensorRT engine/context/tensor";
        }
        return false;
    }
    nvinfer1::Dims dims = context->getTensorShape(name);
    const int64_t volume = DimsVolume(dims);
    const size_t elementSize = TensorRtDataTypeSize(engine->getTensorDataType(name));
    if (volume <= 0 || elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT tensor shape/type index=" +
                   std::to_string(bindingIndex) + " dims=" + DimsToString(dims);
        }
        return false;
    }
    *bytes = static_cast<size_t>(volume) * elementSize;
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return true;
}

inline size_t BindingElementCount(const TensorRtEngineHandle &handle, int bindingIndex,
                           nvinfer1::Dims *dimsOut, std::string *err)
{
    size_t bytes = 0;
    nvinfer1::Dims dims{};
    if (!BindingBytes(handle, bindingIndex, &bytes, &dims, err)) {
        return 0;
    }
    const size_t elementSize = TensorRtDataTypeSize(
        BindingDataType(handle, bindingIndex));
    if (elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT tensor element size index=" +
                   std::to_string(bindingIndex);
        }
        return 0;
    }
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return bytes / elementSize;
}

inline bool CopyFloatHostToBindingDevice(const TensorRtEngineHandle &handle,
                                  int bindingIndex, const float *src,
                                  size_t valueCount, CudaDeviceBuffer &device,
                                  cudaStream_t stream,
                                  std::vector<__half> &halfScratch,
                                  std::string *err)
{
    if (src == nullptr || stream == nullptr || handle.Engine() == nullptr) {
        if (err != nullptr) {
            *err = "invalid host/device copy input";
        }
        return false;
    }
    nvinfer1::Dims dims{};
    const size_t bindingCount =
        BindingElementCount(handle, bindingIndex, &dims, err);
    if (bindingCount == 0 || bindingCount != valueCount) {
        if (err != nullptr) {
            *err = "TensorRT input element count mismatch index=" +
                   std::to_string(bindingIndex) +
                   " expected=" + std::to_string(bindingCount) +
                   " got=" + std::to_string(valueCount) +
                   " dims=" + DimsToString(dims);
        }
        return false;
    }
    size_t bytes = 0;
    if (!BindingBytes(handle, bindingIndex, &bytes, nullptr, err) ||
        !device.Ensure(bytes, err)) {
        return false;
    }
    const nvinfer1::DataType type = BindingDataType(handle, bindingIndex);
    if (type == nvinfer1::DataType::kFLOAT) {
        const cudaError_t rc = cudaMemcpyAsync(device.Data(), src, bytes,
                                               cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT float H2D copy failed: ") +
                       cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }
    if (type == nvinfer1::DataType::kHALF) {
        halfScratch.resize(valueCount);
        for (size_t i = 0; i < valueCount; ++i) {
            halfScratch[i] = __float2half(src[i]);
        }
        const cudaError_t rc =
            cudaMemcpyAsync(device.Data(), halfScratch.data(), bytes,
                            cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT half H2D copy failed: ") +
                       cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }
    if (err != nullptr) {
        *err = "unsupported TensorRT input dtype index=" +
               std::to_string(bindingIndex);
    }
    return false;
}

inline bool CopyBindingDeviceToFloatHost(const TensorRtEngineHandle &handle,
                                  int bindingIndex, CudaDeviceBuffer &device,
                                  cudaStream_t stream, std::vector<float> &dst,
                                  std::vector<__half> &halfScratch,
                                  nvinfer1::Dims *dimsOut, std::string *err)
{
    if (stream == nullptr || handle.Engine() == nullptr) {
        if (err != nullptr) {
            *err = "invalid device/host copy input";
        }
        return false;
    }
    nvinfer1::Dims dims{};
    const size_t valueCount =
        BindingElementCount(handle, bindingIndex, &dims, err);
    if (valueCount == 0) {
        return false;
    }
    size_t bytes = 0;
    if (!BindingBytes(handle, bindingIndex, &bytes, nullptr, err) ||
        !device.Ensure(bytes, err)) {
        return false;
    }
    const nvinfer1::DataType type = BindingDataType(handle, bindingIndex);
    dst.resize(valueCount);
    if (type == nvinfer1::DataType::kFLOAT) {
        const cudaError_t rc = cudaMemcpyAsync(dst.data(), device.Data(), bytes,
                                               cudaMemcpyDeviceToHost, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT float D2H copy failed: ") +
                       cudaGetErrorString(rc);
            }
            return false;
        }
    } else if (type == nvinfer1::DataType::kHALF) {
        halfScratch.resize(valueCount);
        const cudaError_t rc =
            cudaMemcpyAsync(halfScratch.data(), device.Data(), bytes,
                            cudaMemcpyDeviceToHost, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT half D2H copy failed: ") +
                       cudaGetErrorString(rc);
            }
            return false;
        }
        const cudaError_t syncRc = cudaStreamSynchronize(stream);
        if (syncRc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT half D2H synchronize failed: ") +
                       cudaGetErrorString(syncRc);
            }
            return false;
        }
        for (size_t i = 0; i < valueCount; ++i) {
            dst[i] = __half2float(halfScratch[i]);
        }
        if (dimsOut != nullptr) {
            *dimsOut = dims;
        }
        return true;
    } else {
        if (err != nullptr) {
            *err = "unsupported TensorRT output dtype index=" +
                   std::to_string(bindingIndex);
        }
        return false;
    }
    const cudaError_t syncRc = cudaStreamSynchronize(stream);
    if (syncRc != cudaSuccess) {
        if (err != nullptr) {
            *err = std::string("TensorRT float D2H synchronize failed: ") +
                   cudaGetErrorString(syncRc);
        }
        return false;
    }
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return true;
}

inline bool EnsureBindingBuffer(const TensorRtEngineHandle &handle, int bindingIndex,
                         CudaDeviceBuffer &device, std::string *err)
{
    size_t bytes = 0;
    return BindingBytes(handle, bindingIndex, &bytes, nullptr, err) &&
           device.Ensure(bytes, err);
}

inline bool BindingIsFloat(const TensorRtEngineHandle &handle, int bindingIndex)
{
    return handle.Engine() != nullptr && bindingIndex >= 0 &&
           BindingDataType(handle, bindingIndex) == nvinfer1::DataType::kFLOAT;
}

inline bool SetBindingAddress(TensorRtEngineHandle &handle, int bindingIndex,
                              void *data, std::string *err)
{
    nvinfer1::IExecutionContext *context = handle.Context();
    const char *name = BindingName(handle, bindingIndex);
    if (context == nullptr || name == nullptr || data == nullptr) {
        if (err != nullptr) {
            *err = "invalid TensorRT tensor address index=" +
                   std::to_string(bindingIndex);
        }
        return false;
    }
    if (!context->setTensorAddress(name, data)) {
        if (err != nullptr) {
            *err = "setTensorAddress failed for tensor " + std::string(name);
        }
        return false;
    }
    return true;
}

inline bool SetBindingBufferAddresses(TensorRtEngineHandle &handle,
                                      std::array<CudaDeviceBuffer, 16> &buffers,
                                      std::string *err)
{
    const int nbBindings = BindingCount(handle);
    if (nbBindings <= 0 || nbBindings > static_cast<int>(buffers.size())) {
        if (err != nullptr) {
            *err = "TensorRT tensor count exceeds local buffer table";
        }
        return false;
    }
    for (int i = 0; i < nbBindings; ++i) {
        if (!SetBindingAddress(handle, i,
                               buffers[static_cast<size_t>(i)].Data(), err)) {
            return false;
        }
    }
    return true;
}

inline bool EnqueueTensorRt(TensorRtEngineHandle &handle, cudaStream_t stream,
                            const char *name, std::string *err)
{
    if (handle.Context() == nullptr || stream == nullptr) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") +
                   " TensorRT context or CUDA stream is not initialized";
        }
        return false;
    }
    if (!handle.Context()->enqueueV3(stream)) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") +
                   " enqueueV3 failed";
        }
        return false;
    }
    return true;
}

inline bool CopyFloatDeviceBufferToHost(const CudaDeviceBuffer &device,
                                 size_t valueCount, cudaStream_t stream,
                                 std::vector<float> &dst, std::string *err)
{
    if (device.Bytes() < valueCount * sizeof(float) || stream == nullptr) {
        if (err != nullptr) {
            *err = "invalid float D2H buffer copy input";
        }
        return false;
    }
    dst.assign(valueCount, 0.0f);
    cudaError_t rc =
        cudaMemcpyAsync(dst.data(), device.Data(), valueCount * sizeof(float),
                        cudaMemcpyDeviceToHost, stream);
    if (rc == cudaSuccess) {
        rc = cudaStreamSynchronize(stream);
    }
    if (rc != cudaSuccess) {
        if (err != nullptr) {
            *err = std::string("float D2H buffer copy failed: ") +
                   cudaGetErrorString(rc);
        }
        return false;
    }
    return true;
}
