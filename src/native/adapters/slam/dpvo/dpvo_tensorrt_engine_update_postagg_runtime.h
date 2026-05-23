class DpvoUpdatePostAggRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, std::string *err)
    {
        m_baseNetIndex = FindBindingIndex(engine, "base_net");
        m_kkYIndex = FindBindingIndex(engine, "agg_kk_y");
        m_ijYIndex = FindBindingIndex(engine, "agg_ij_y");
        m_updatedNetIndex = FindBindingIndex(engine, "updated_net");
        m_deltaIndex = FindBindingIndex(engine, "delta");
        m_weightIndex = FindBindingIndex(engine, "weight");
        const bool ok = m_baseNetIndex >= 0 && m_kkYIndex >= 0 && m_ijYIndex >= 0 &&
                        m_updatedNetIndex >= 0 && m_deltaIndex >= 0 &&
                        m_weightIndex >= 0;
        if (!ok && err != nullptr) {
            *err = "update-postagg binding lookup failed";
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
                    "update-postagg TensorRT context or CUDA stream is not initialized";
            }
            return {};
        }
        if (!SetBindingShape(engine, m_baseNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_kkYIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_ijYIndex, nvinfer1::Dims3{1, edges, kDim},
                             err)) {
            return {};
        }
        return WarmupBindings(engine, stream,
                              {m_baseNetIndex, m_kkYIndex, m_ijYIndex,
                               m_updatedNetIndex, m_deltaIndex, m_weightIndex},
                              m_buffers, "update-postagg", err);
    }

    DpvoUpdatePostAggRun Run(TensorRtEngineHandle &engine, cudaStream_t stream,
                             int edges, const std::vector<float> &baseNet,
                             const std::vector<float> &aggKkY,
                             const std::vector<float> &aggIjY, std::string *err)
    {
        DpvoUpdatePostAggRun result{};
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err =
                    "update-postagg TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(engine, m_baseNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_kkYIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_ijYIndex, nvinfer1::Dims3{1, edges, kDim},
                             err)) {
            return result;
        }
        const size_t dimValues = static_cast<size_t>(edges) * kDim;
        if (baseNet.size() != dimValues || aggKkY.size() != dimValues ||
            aggIjY.size() != dimValues) {
            if (err != nullptr) {
                *err = "update-postagg input vector size mismatch edges=" +
                       std::to_string(edges);
            }
            return result;
        }

        if (!EnsureBindingBuffer(engine, m_updatedNetIndex,
                                 m_buffers[static_cast<size_t>(m_updatedNetIndex)],
                                 err) ||
            !EnsureBindingBuffer(engine, m_deltaIndex,
                                 m_buffers[static_cast<size_t>(m_deltaIndex)],
                                 err) ||
            !EnsureBindingBuffer(engine, m_weightIndex,
                                 m_buffers[static_cast<size_t>(m_weightIndex)],
                                 err)) {
            return result;
        }
        if (!CopyFloatHostToBindingDevice(
                engine, m_baseNetIndex, baseNet.data(), dimValues,
                m_buffers[static_cast<size_t>(m_baseNetIndex)], stream,
                m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_kkYIndex, aggKkY.data(), dimValues,
                m_buffers[static_cast<size_t>(m_kkYIndex)], stream, m_halfScratch,
                err) ||
            !CopyFloatHostToBindingDevice(
                engine, m_ijYIndex, aggIjY.data(), dimValues,
                m_buffers[static_cast<size_t>(m_ijYIndex)], stream, m_halfScratch,
                err)) {
            return result;
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!SetBindingBufferAddresses(engine, m_buffers, err) ||
            !EnqueueTensorRt(engine, stream, "update-postagg", err)) {
            return result;
        }
        cudaError_t rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("update-postagg synchronize failed: ") +
                       cudaGetErrorString(rc);
            }
            return result;
        }
        if (!CopyBindingDeviceToFloatHost(
                engine, m_updatedNetIndex,
                m_buffers[static_cast<size_t>(m_updatedNetIndex)], stream,
                result.updatedNet, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_deltaIndex, m_buffers[static_cast<size_t>(m_deltaIndex)],
                stream, result.delta, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_weightIndex,
                m_buffers[static_cast<size_t>(m_weightIndex)], stream,
                result.weight, m_halfScratch, nullptr, err)) {
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        return result;
    }

    DpvoUpdatePostAggRun RunDevice(TensorRtEngineHandle &engine,
                                   cudaStream_t stream, int edges,
                                   const CudaDeviceBuffer &baseNetDevice,
                                   const CudaDeviceBuffer &aggKkYDevice,
                                   const CudaDeviceBuffer &aggIjYDevice,
                                   std::string *err)
    {
        DpvoUpdatePostAggRun result{};
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update-postagg device TensorRT context or CUDA stream is not "
                       "initialized";
            }
            return result;
        }
        if (!SetBindingShape(engine, m_baseNetIndex,
                             nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(engine, m_kkYIndex, nvinfer1::Dims3{1, edges, kDim},
                             err) ||
            !SetBindingShape(engine, m_ijYIndex, nvinfer1::Dims3{1, edges, kDim},
                             err)) {
            return result;
        }
        if (!BindingIsFloat(engine, m_baseNetIndex) ||
            !BindingIsFloat(engine, m_kkYIndex) ||
            !BindingIsFloat(engine, m_ijYIndex)) {
            if (err != nullptr) {
                *err = "update-postagg device path requires FP32 input bindings";
            }
            return result;
        }
        const size_t dimBytes = static_cast<size_t>(edges) * kDim * sizeof(float);
        if (baseNetDevice.Bytes() < dimBytes || aggKkYDevice.Bytes() < dimBytes ||
            aggIjYDevice.Bytes() < dimBytes) {
            if (err != nullptr) {
                *err = "update-postagg device input buffer size mismatch edges=" +
                       std::to_string(edges);
            }
            return result;
        }

        if (!EnsureBindingBuffer(engine, m_updatedNetIndex,
                                 m_buffers[static_cast<size_t>(m_updatedNetIndex)],
                                 err) ||
            !EnsureBindingBuffer(engine, m_deltaIndex,
                                 m_buffers[static_cast<size_t>(m_deltaIndex)],
                                 err) ||
            !EnsureBindingBuffer(engine, m_weightIndex,
                                 m_buffers[static_cast<size_t>(m_weightIndex)],
                                 err)) {
            return result;
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!SetBindingBufferAddresses(engine, m_buffers, err) ||
            !SetBindingAddress(engine, m_baseNetIndex,
                               const_cast<void *>(baseNetDevice.Data()), err) ||
            !SetBindingAddress(engine, m_kkYIndex,
                               const_cast<void *>(aggKkYDevice.Data()), err) ||
            !SetBindingAddress(engine, m_ijYIndex,
                               const_cast<void *>(aggIjYDevice.Data()), err) ||
            !EnqueueTensorRt(engine, stream, "update-postagg device", err)) {
            return result;
        }
        cudaError_t rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("update-postagg device synchronize failed: ") +
                       cudaGetErrorString(rc);
            }
            return result;
        }
        if (!CopyBindingDeviceToFloatHost(
                engine, m_updatedNetIndex,
                m_buffers[static_cast<size_t>(m_updatedNetIndex)], stream,
                result.updatedNet, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_deltaIndex, m_buffers[static_cast<size_t>(m_deltaIndex)],
                stream, result.delta, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(
                engine, m_weightIndex,
                m_buffers[static_cast<size_t>(m_weightIndex)], stream,
                result.weight, m_halfScratch, nullptr, err)) {
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        return result;
    }

  private:
    static constexpr int kDim = 384;
    int m_baseNetIndex{-1};
    int m_kkYIndex{-1};
    int m_ijYIndex{-1};
    int m_updatedNetIndex{-1};
    int m_deltaIndex{-1};
    int m_weightIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
    std::vector<__half> m_halfScratch;
};
