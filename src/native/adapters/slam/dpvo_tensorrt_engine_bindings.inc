size_t TensorRtDataTypeSize(nvinfer1::DataType type)
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

int64_t DimsVolume(const nvinfer1::Dims &dims)
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

std::string DimsToString(const nvinfer1::Dims &dims)
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

bool SetBindingShape(nvinfer1::IExecutionContext &context, int bindingIndex,
                     const nvinfer1::Dims &dims, std::string *err)
{
    if (bindingIndex < 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding index";
        }
        return false;
    }
    if (!context.setBindingDimensions(bindingIndex, dims)) {
        if (err != nullptr) {
            *err = "setBindingDimensions failed for binding " +
                   std::to_string(bindingIndex) + " dims=" + DimsToString(dims);
        }
        return false;
    }
    return true;
}

int FindBindingIndex(const TensorRtEngineHandle &handle, const char *name)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    if (engine == nullptr || name == nullptr) {
        return -1;
    }
    return engine->getBindingIndex(name);
}

bool BindingBytes(const TensorRtEngineHandle &handle, int bindingIndex,
                  size_t *bytes, nvinfer1::Dims *dimsOut, std::string *err)
{
    if (bytes == nullptr) {
        return false;
    }
    nvinfer1::ICudaEngine *engine = handle.Engine();
    nvinfer1::IExecutionContext *context = handle.Context();
    if (engine == nullptr || context == nullptr || bindingIndex < 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT engine/context/binding";
        }
        return false;
    }
    nvinfer1::Dims dims = context->getBindingDimensions(bindingIndex);
    const int64_t volume = DimsVolume(dims);
    const size_t elementSize =
        TensorRtDataTypeSize(engine->getBindingDataType(bindingIndex));
    if (volume <= 0 || elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding shape/type index=" +
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

size_t BindingElementCount(const TensorRtEngineHandle &handle, int bindingIndex,
                           nvinfer1::Dims *dimsOut, std::string *err)
{
    size_t bytes = 0;
    nvinfer1::Dims dims{};
    if (!BindingBytes(handle, bindingIndex, &bytes, &dims, err)) {
        return 0;
    }
    const size_t elementSize =
        TensorRtDataTypeSize(handle.Engine()->getBindingDataType(bindingIndex));
    if (elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding element size index=" +
                   std::to_string(bindingIndex);
        }
        return 0;
    }
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return bytes / elementSize;
}

bool CopyFloatHostToBindingDevice(const TensorRtEngineHandle &handle,
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
    const nvinfer1::DataType type =
        handle.Engine()->getBindingDataType(bindingIndex);
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

bool CopyBindingDeviceToFloatHost(const TensorRtEngineHandle &handle,
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
    const nvinfer1::DataType type =
        handle.Engine()->getBindingDataType(bindingIndex);
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

bool EnsureBindingBuffer(const TensorRtEngineHandle &handle, int bindingIndex,
                         CudaDeviceBuffer &device, std::string *err)
{
    size_t bytes = 0;
    return BindingBytes(handle, bindingIndex, &bytes, nullptr, err) &&
           device.Ensure(bytes, err);
}

bool BindingIsFloat(const TensorRtEngineHandle &handle, int bindingIndex)
{
    return handle.Engine() != nullptr && bindingIndex >= 0 &&
           handle.Engine()->getBindingDataType(bindingIndex) ==
               nvinfer1::DataType::kFLOAT;
}

bool CopyFloatDeviceBufferToHost(const CudaDeviceBuffer &device,
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

struct DpvoPatchifierRun {
    nvinfer1::Dims fmapDims{};
    nvinfer1::Dims imapDims{};
    const float *fmapHost{nullptr};
    const float *imapHost{nullptr};
    size_t fmapValueCount{0};
    size_t imapValueCount{0};
    double elapsedMs{0.0};
    bool ok{false};
};

class DpvoPatchifierRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, int width, int height,
                    std::string *err)
    {
        m_imageIndex = FindBindingIndex(engine, "image");
        m_fmapIndex = FindBindingIndex(engine, "fmap");
        m_imapIndex = FindBindingIndex(engine, "imap");
        if (m_imageIndex < 0 || m_fmapIndex < 0 || m_imapIndex < 0) {
            if (err != nullptr) {
                *err = "patchifier binding lookup failed: image=" +
                       std::to_string(m_imageIndex) +
                       " fmap=" + std::to_string(m_fmapIndex) +
                       " imap=" + std::to_string(m_imapIndex);
            }
            return false;
        }
        m_width = width;
        m_height = height;
        m_inputHost.assign(static_cast<size_t>(3) * static_cast<size_t>(width) *
                               static_cast<size_t>(height),
                           0.0f);
        return true;
    }

    DpvoPatchifierRun Run(const cv::Mat &gray, TensorRtEngineHandle &engine,
                          cudaStream_t stream, bool copyFmapToHost,
                          bool copyImapToHost, std::string *err)
    {
        DpvoPatchifierRun result{};
        if (gray.empty() || gray.cols != m_width || gray.rows != m_height ||
            gray.type() != CV_8UC1) {
            if (err != nullptr) {
                *err = "patchifier expects CV_8UC1 " + std::to_string(m_width) + "x" +
                       std::to_string(m_height);
            }
            return result;
        }

        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "patchifier TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(*context, m_imageIndex,
                             nvinfer1::Dims4{1, 3, m_height, m_width}, err)) {
            return result;
        }

        size_t imageBytes = 0;
        size_t fmapBytes = 0;
        size_t imapBytes = 0;
        if (!BindingBytes(engine, m_imageIndex, &imageBytes, nullptr, err) ||
            !BindingBytes(engine, m_fmapIndex, &fmapBytes, &result.fmapDims, err) ||
            !BindingBytes(engine, m_imapIndex, &imapBytes, &result.imapDims, err)) {
            return result;
        }
        if (imageBytes != m_inputHost.size() * sizeof(float)) {
            if (err != nullptr) {
                *err = "patchifier image binding byte mismatch expected=" +
                       std::to_string(m_inputHost.size() * sizeof(float)) +
                       " got=" + std::to_string(imageBytes);
            }
            return result;
        }

        if (!m_imageDevice.Ensure(imageBytes, err) ||
            !m_fmapDevice.Ensure(fmapBytes, err) ||
            !m_imapDevice.Ensure(imapBytes, err)) {
            return result;
        }

        FillInput(gray);
        const int nbBindings =
            engine.Engine() != nullptr ? engine.Engine()->getNbBindings() : 0;
        if (nbBindings <= 0) {
            if (err != nullptr) {
                *err = "patchifier engine has no bindings";
            }
            return result;
        }
        std::vector<void *> bindings(static_cast<size_t>(nbBindings), nullptr);
        bindings[static_cast<size_t>(m_imageIndex)] = m_imageDevice.Data();
        bindings[static_cast<size_t>(m_fmapIndex)] = m_fmapDevice.Data();
        bindings[static_cast<size_t>(m_imapIndex)] = m_imapDevice.Data();

        const auto t0 = std::chrono::steady_clock::now();
        cudaError_t rc =
            cudaMemcpyAsync(m_imageDevice.Data(), m_inputHost.data(), imageBytes,
                            cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("patchifier H2D copy failed: ") +
                       cudaGetErrorString(rc);
            }
            return result;
        }
        if (!context->enqueueV2(bindings.data(), stream, nullptr)) {
            if (err != nullptr) {
                *err = "patchifier enqueueV2 failed";
            }
            return result;
        }
        rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("patchifier synchronize failed: ") +
                       cudaGetErrorString(rc);
            }
            return result;
        }
        if (copyFmapToHost) {
            if (!CopyBindingDeviceToFloatHost(engine, m_fmapIndex, m_fmapDevice,
                                              stream, m_fmapHost, m_halfScratch,
                                              nullptr, err)) {
                return result;
            }
            result.fmapHost = m_fmapHost.data();
            result.fmapValueCount = m_fmapHost.size();
        }
        if (copyImapToHost) {
            if (!CopyBindingDeviceToFloatHost(engine, m_imapIndex, m_imapDevice,
                                              stream, m_imapHost, m_halfScratch,
                                              nullptr, err)) {
                return result;
            }
            result.imapHost = m_imapHost.data();
            result.imapValueCount = m_imapHost.size();
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        m_lastFmapDims = result.fmapDims;
        m_lastImapDims = result.imapDims;
        return result;
    }

    void *FmapDevice() const
    {
        return m_fmapDevice.Data();
    }
    void *ImapDevice() const
    {
        return m_imapDevice.Data();
    }
    nvinfer1::Dims LastFmapDims() const
    {
        return m_lastFmapDims;
    }
    nvinfer1::Dims LastImapDims() const
    {
        return m_lastImapDims;
    }

  private:
    void FillInput(const cv::Mat &gray)
    {
        const size_t plane =
            static_cast<size_t>(m_width) * static_cast<size_t>(m_height);
        for (int y = 0; y < m_height; ++y) {
            const uint8_t *src = gray.ptr<uint8_t>(y);
            for (int x = 0; x < m_width; ++x) {
                const float v = static_cast<float>(src[x]);
                const size_t idx =
                    static_cast<size_t>(y) * static_cast<size_t>(m_width) +
                    static_cast<size_t>(x);
                m_inputHost[idx] = v;
                m_inputHost[plane + idx] = v;
                m_inputHost[2 * plane + idx] = v;
            }
        }
    }

    int m_imageIndex{-1};
    int m_fmapIndex{-1};
    int m_imapIndex{-1};
    int m_width{0};
    int m_height{0};
    std::vector<float> m_inputHost;
    std::vector<float> m_fmapHost;
    std::vector<float> m_imapHost;
    std::vector<__half> m_halfScratch;
    CudaDeviceBuffer m_imageDevice;
    CudaDeviceBuffer m_fmapDevice;
    CudaDeviceBuffer m_imapDevice;
    nvinfer1::Dims m_lastFmapDims{};
    nvinfer1::Dims m_lastImapDims{};
};
