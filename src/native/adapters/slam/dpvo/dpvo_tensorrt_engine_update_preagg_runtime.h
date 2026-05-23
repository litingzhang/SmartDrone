class DpvoUpdatePreAggRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, std::string *err)
    {
        m_netIndex = FindBindingIndex(engine, "net");
        m_inpIndex = FindBindingIndex(engine, "inp");
        m_corrIndex = FindBindingIndex(engine, "corr");
        m_prevNetIndex = FindBindingIndex(engine, "prev_net");
        m_nextNetIndex = FindBindingIndex(engine, "next_net");
        m_prevMaskIndex = FindBindingIndex(engine, "prev_mask");
        m_nextMaskIndex = FindBindingIndex(engine, "next_mask");
        m_baseNetIndex = FindBindingIndex(engine, "base_net");
        m_kkFIndex = FindBindingIndex(engine, "agg_kk_f");
        m_kkGIndex = FindBindingIndex(engine, "agg_kk_g");
        m_ijFIndex = FindBindingIndex(engine, "agg_ij_f");
        m_ijGIndex = FindBindingIndex(engine, "agg_ij_g");
        const bool ok = m_netIndex >= 0 && m_inpIndex >= 0 && m_corrIndex >= 0 &&
                        m_prevNetIndex >= 0 && m_nextNetIndex >= 0 &&
                        m_prevMaskIndex >= 0 && m_nextMaskIndex >= 0 &&
                        m_baseNetIndex >= 0 && m_kkFIndex >= 0 && m_kkGIndex >= 0 &&
                        m_ijFIndex >= 0 && m_ijGIndex >= 0;
        if (!ok && err != nullptr) {
            *err = "update-preagg binding lookup failed";
        }
        return ok;
    }

    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream,
                         int edges, std::string *err)
    {
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err =
                    "update-preagg TensorRT context or CUDA stream is not initialized";
            }
            return {};
        }
        if (!SetBindingShape(engine, m_netIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_inpIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_corrIndex,
                             nvinfer1::Dims3{1, edges, kCorrDim}, err) ||
            !SetBindingShape(engine, m_prevNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_nextNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_prevMaskIndex,
                             nvinfer1::Dims3{1, edges, 1}, err) ||
            !SetBindingShape(engine, m_nextMaskIndex,
                             nvinfer1::Dims3{1, edges, 1}, err)) {
            return {};
        }
        return WarmupBindings(engine, stream,
                              {m_netIndex, m_inpIndex, m_corrIndex, m_prevNetIndex,
                               m_nextNetIndex, m_prevMaskIndex, m_nextMaskIndex,
                               m_baseNetIndex, m_kkFIndex, m_kkGIndex, m_ijFIndex,
                               m_ijGIndex},
                              m_buffers, "update-preagg", err);
    }

    DpvoUpdatePreAggRun
    Run(TensorRtEngineHandle &engine, cudaStream_t stream, int edges,
        const std::vector<float> &net, const std::vector<float> &inp,
        const std::vector<float> &corr, const std::vector<float> &prevNet,
        const std::vector<float> &nextNet, const std::vector<float> &prevMask,
        const std::vector<float> &nextMask, std::string *err)
    {
        DpvoUpdatePreAggRun result{};
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err =
                    "update-preagg TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(engine, m_netIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_inpIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_corrIndex,
                             nvinfer1::Dims3{1, edges, kCorrDim}, err) ||
            !SetBindingShape(engine, m_prevNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_nextNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_prevMaskIndex,
                             nvinfer1::Dims3{1, edges, 1}, err) ||
            !SetBindingShape(engine, m_nextMaskIndex,
                             nvinfer1::Dims3{1, edges, 1}, err)) {
            return result;
        }

        const size_t dimValues = static_cast<size_t>(edges) * kDim;
        const size_t corrValues = static_cast<size_t>(edges) * kCorrDim;
        const size_t maskValues = static_cast<size_t>(edges);
        if (net.size() != dimValues || inp.size() != dimValues ||
            corr.size() != corrValues || prevNet.size() != dimValues ||
            nextNet.size() != dimValues || prevMask.size() != maskValues ||
            nextMask.size() != maskValues) {
            if (err != nullptr) {
                *err = "update-preagg input vector size mismatch edges=" +
                       std::to_string(edges);
            }
            return result;
        }

        if (!EnsureBindingBuffer(engine, m_baseNetIndex,
                                 m_buffers[static_cast<size_t>(m_baseNetIndex)],
                                 err) ||
            !EnsureBindingBuffer(engine, m_kkFIndex,
                                 m_buffers[static_cast<size_t>(m_kkFIndex)], err) ||
            !EnsureBindingBuffer(engine, m_kkGIndex,
                                 m_buffers[static_cast<size_t>(m_kkGIndex)], err) ||
            !EnsureBindingBuffer(engine, m_ijFIndex,
                                 m_buffers[static_cast<size_t>(m_ijFIndex)], err) ||
            !EnsureBindingBuffer(engine, m_ijGIndex,
                                 m_buffers[static_cast<size_t>(m_ijGIndex)], err)) {
            return result;
        }
        if (!CopyFloatHostToBindingDevice(
                engine, m_netIndex, net.data(), dimValues,
                m_buffers[static_cast<size_t>(m_netIndex)], stream, m_halfScratch,
                err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_inpIndex, inp.data(), dimValues,
                m_buffers[static_cast<size_t>(m_inpIndex)], stream, m_halfScratch,
                err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_corrIndex, corr.data(), corrValues,
                m_buffers[static_cast<size_t>(m_corrIndex)], stream, m_halfScratch,
                err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_prevNetIndex, prevNet.data(), dimValues,
                m_buffers[static_cast<size_t>(m_prevNetIndex)], stream,
                m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_nextNetIndex, nextNet.data(), dimValues,
                m_buffers[static_cast<size_t>(m_nextNetIndex)], stream,
                m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_prevMaskIndex, prevMask.data(), maskValues,
                m_buffers[static_cast<size_t>(m_prevMaskIndex)], stream,
                m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_nextMaskIndex, nextMask.data(), maskValues,
                m_buffers[static_cast<size_t>(m_nextMaskIndex)], stream,
                m_halfScratch, err)) {
            return result;
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!SetBindingBufferAddresses(engine, m_buffers, err) ||
            !EnqueueTensorRt(engine, stream, "update-preagg", err)) {
            return result;
        }
        cudaError_t rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("update-preagg synchronize failed: ") +
                       cudaGetErrorString(rc);
            }
            return result;
        }
        if (!CopyBindingDeviceToFloatHost(
                engine, m_baseNetIndex,
                m_buffers[static_cast<size_t>(m_baseNetIndex)], stream,
                result.baseNet, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_kkFIndex, m_buffers[static_cast<size_t>(m_kkFIndex)],
                stream, result.aggKkF, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_kkGIndex, m_buffers[static_cast<size_t>(m_kkGIndex)],
                stream, result.aggKkG, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_ijFIndex, m_buffers[static_cast<size_t>(m_ijFIndex)],
                stream, result.aggIjF, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_ijGIndex, m_buffers[static_cast<size_t>(m_ijGIndex)],
                stream, result.aggIjG, m_halfScratch, nullptr, err)) {
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        return result;
    }

    DpvoUpdatePreAggDeviceRun
    RunDevice(TensorRtEngineHandle &engine, cudaStream_t stream, int edges,
              const std::vector<float> &net, const std::vector<float> &inp,
              const std::vector<float> &corr, const std::vector<float> &prevNet,
              const std::vector<float> &nextNet,
              const std::vector<float> &prevMask,
              const std::vector<float> &nextMask, std::string *err)
    {
        DpvoUpdatePreAggDeviceRun result{};
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update-preagg device TensorRT context or CUDA stream is not "
                       "initialized";
            }
            return result;
        }
        if (!SetBindingShape(engine, m_netIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_inpIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_corrIndex,
                             nvinfer1::Dims3{1, edges, kCorrDim}, err) ||
            !SetBindingShape(engine, m_prevNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_nextNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_prevMaskIndex,
                             nvinfer1::Dims3{1, edges, 1}, err) ||
            !SetBindingShape(engine, m_nextMaskIndex,
                             nvinfer1::Dims3{1, edges, 1}, err)) {
            return result;
        }
        if (!BindingIsFloat(engine, m_baseNetIndex) ||
            !BindingIsFloat(engine, m_kkFIndex) ||
            !BindingIsFloat(engine, m_kkGIndex) ||
            !BindingIsFloat(engine, m_ijFIndex) ||
            !BindingIsFloat(engine, m_ijGIndex)) {
            if (err != nullptr) {
                *err = "update-preagg device path requires FP32 output bindings";
            }
            return result;
        }

        const size_t dimValues = static_cast<size_t>(edges) * kDim;
        const size_t corrValues = static_cast<size_t>(edges) * kCorrDim;
        const size_t maskValues = static_cast<size_t>(edges);
        if (net.size() != dimValues || inp.size() != dimValues ||
            corr.size() != corrValues || prevNet.size() != dimValues ||
            nextNet.size() != dimValues || prevMask.size() != maskValues ||
            nextMask.size() != maskValues) {
            if (err != nullptr) {
                *err = "update-preagg device input vector size mismatch edges=" +
                       std::to_string(edges);
            }
            return result;
        }

        if (!EnsureBindingBuffer(engine, m_baseNetIndex,
                                 m_buffers[static_cast<size_t>(m_baseNetIndex)],
                                 err) ||
            !EnsureBindingBuffer(engine, m_kkFIndex,
                                 m_buffers[static_cast<size_t>(m_kkFIndex)], err) ||
            !EnsureBindingBuffer(engine, m_kkGIndex,
                                 m_buffers[static_cast<size_t>(m_kkGIndex)], err) ||
            !EnsureBindingBuffer(engine, m_ijFIndex,
                                 m_buffers[static_cast<size_t>(m_ijFIndex)], err) ||
            !EnsureBindingBuffer(engine, m_ijGIndex,
                                 m_buffers[static_cast<size_t>(m_ijGIndex)], err)) {
            return result;
        }
        if (!CopyFloatHostToBindingDevice(
                engine, m_netIndex, net.data(), dimValues,
                m_buffers[static_cast<size_t>(m_netIndex)], stream, m_halfScratch,
                err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_inpIndex, inp.data(), dimValues,
                m_buffers[static_cast<size_t>(m_inpIndex)], stream, m_halfScratch,
                err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_corrIndex, corr.data(), corrValues,
                m_buffers[static_cast<size_t>(m_corrIndex)], stream, m_halfScratch,
                err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_prevNetIndex, prevNet.data(), dimValues,
                m_buffers[static_cast<size_t>(m_prevNetIndex)], stream,
                m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_nextNetIndex, nextNet.data(), dimValues,
                m_buffers[static_cast<size_t>(m_nextNetIndex)], stream,
                m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_prevMaskIndex, prevMask.data(), maskValues,
                m_buffers[static_cast<size_t>(m_prevMaskIndex)], stream,
                m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_nextMaskIndex, nextMask.data(), maskValues,
                m_buffers[static_cast<size_t>(m_nextMaskIndex)], stream,
                m_halfScratch, err)) {
            return result;
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!SetBindingBufferAddresses(engine, m_buffers, err) ||
            !EnqueueTensorRt(engine, stream, "update-preagg device", err)) {
            return result;
        }
        const cudaError_t rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("update-preagg device synchronize failed: ") +
                       cudaGetErrorString(rc);
            }
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        return result;
    }

    const CudaDeviceBuffer &BaseNetDevice() const
    {
        return m_buffers[static_cast<size_t>(m_baseNetIndex)];
    }
    const CudaDeviceBuffer &AggKkFDevice() const
    {
        return m_buffers[static_cast<size_t>(m_kkFIndex)];
    }
    const CudaDeviceBuffer &AggKkGDevice() const
    {
        return m_buffers[static_cast<size_t>(m_kkGIndex)];
    }
    const CudaDeviceBuffer &AggIjFDevice() const
    {
        return m_buffers[static_cast<size_t>(m_ijFIndex)];
    }
    const CudaDeviceBuffer &AggIjGDevice() const
    {
        return m_buffers[static_cast<size_t>(m_ijGIndex)];
    }

  private:
    static constexpr int kDim = 384;
    static constexpr int kCorrDim = 882;
    int m_netIndex{-1};
    int m_inpIndex{-1};
    int m_corrIndex{-1};
    int m_prevNetIndex{-1};
    int m_nextNetIndex{-1};
    int m_prevMaskIndex{-1};
    int m_nextMaskIndex{-1};
    int m_baseNetIndex{-1};
    int m_kkFIndex{-1};
    int m_kkGIndex{-1};
    int m_ijFIndex{-1};
    int m_ijGIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
    std::vector<__half> m_halfScratch;
};
