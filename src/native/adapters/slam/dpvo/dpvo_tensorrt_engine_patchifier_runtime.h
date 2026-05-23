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
        if (!SetBindingShape(engine, m_imageIndex,
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
        if (!SetBindingAddress(engine, m_imageIndex, m_imageDevice.Data(), err) ||
            !SetBindingAddress(engine, m_fmapIndex, m_fmapDevice.Data(), err) ||
            !SetBindingAddress(engine, m_imapIndex, m_imapDevice.Data(), err) ||
            !EnqueueTensorRt(engine, stream, "patchifier", err)) {
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
