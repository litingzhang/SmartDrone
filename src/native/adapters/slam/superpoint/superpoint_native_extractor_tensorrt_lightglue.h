class TensorRtLightGlueEngine {
  public:
    ~TensorRtLightGlueEngine()
    {
        Release();
    }

    bool Load(const std::filesystem::path &enginePath, std::string *err)
    {
        Release();
        std::ifstream input(enginePath, std::ios::binary);
        if (!input) {
            if (err != nullptr) {
                *err =
                    "failed to open LightGlue TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(input)),
                                std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = "LightGlue TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }
        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = "failed to create LightGlue TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(
            m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = "failed to deserialize LightGlue TensorRT engine: " +
                       enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = "failed to create LightGlue TensorRT execution context";
            }
            return false;
        }
        if (cudaStreamCreate(&m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "failed to create LightGlue TensorRT CUDA stream";
            }
            return false;
        }
        m_eventTimingEnabled = EnvFlag("SMART_DRONE_TRT_EVENT_TIMING", false);
        if (m_eventTimingEnabled &&
            (cudaEventCreate(&m_computeStartEvent) != cudaSuccess ||
             cudaEventCreate(&m_computeEndEvent) != cudaSuccess ||
             cudaEventCreate(&m_outputStartEvent) != cudaSuccess ||
             cudaEventCreate(&m_outputEndEvent) != cudaSuccess)) {
            if (err != nullptr) {
                *err = "failed to create LightGlue TensorRT CUDA timing events";
            }
            return false;
        }
        m_kpts0Index = FindBinding({"keypoints0"}, true);
        m_kpts1Index = FindBinding({"keypoints1"}, true);
        m_desc0Index = FindBinding({"descriptors0"}, true);
        m_desc1Index = FindBinding({"descriptors1"}, true);
        m_size0Index = FindBinding({"image_size0"}, true);
        m_size1Index = FindBinding({"image_size1"}, true);
        m_scoresIndex =
            FindBinding({"assignment_scores", "scores", "output0"}, false);
        if (m_kpts0Index < 0 || m_kpts1Index < 0 || m_desc0Index < 0 ||
            m_desc1Index < 0 || m_size0Index < 0 || m_size1Index < 0 ||
            m_scoresIndex < 0) {
            if (err != nullptr) {
                *err = "LightGlue TensorRT engine bindings are missing expected "
                       "input/output names";
            }
            return false;
        }
        return true;
    }

    int FixedPointCount() const
    {
        if (!m_engine || m_kpts0Index < 0) {
            return 0;
        }
        nvinfer1::Dims dims =
            TensorRtTensorShape(m_engine.get(), nullptr, m_kpts0Index);
        if (dims.nbDims == 3 && dims.d[1] > 0) {
            return dims.d[1];
        }
        const int profiles = m_engine->getNbOptimizationProfiles();
        for (int profile = 0; profile < profiles; ++profile) {
            const char *name = TensorRtTensorName(m_engine.get(), m_kpts0Index);
            const nvinfer1::Dims minDims =
                name != nullptr ? m_engine->getProfileShape(
                                      name, profile,
                                      nvinfer1::OptProfileSelector::kMIN)
                                : nvinfer1::Dims{};
            const nvinfer1::Dims maxDims =
                name != nullptr ? m_engine->getProfileShape(
                                      name, profile,
                                      nvinfer1::OptProfileSelector::kMAX)
                                : nvinfer1::Dims{};
            if (minDims.nbDims == 3 && maxDims.nbDims == 3 && minDims.d[1] > 0 &&
                minDims.d[1] == maxDims.d[1]) {
                return minDims.d[1];
            }
        }
        return 0;
    }

    bool Forward(const std::vector<float> &keypoints0,
                 const std::vector<float> &keypoints1,
                 const std::vector<float> &descriptors0,
                 const std::vector<float> &descriptors1,
                 const std::array<float, 2> &imageSize0,
                 const std::array<float, 2> &imageSize1, int pointCount,
                 TensorBlob &scores, TensorRtForwardStats *stats,
                 std::string *err)
    {
        if (stats != nullptr) {
            *stats = TensorRtForwardStats{};
            stats->eventTimingEnabled = m_eventTimingEnabled;
        }
        if (pointCount <= 0) {
            return false;
        }
        if (!SetInput(m_kpts0Index, {1, pointCount, 2}, keypoints0, stats, err) ||
            !SetInput(m_kpts1Index, {1, pointCount, 2}, keypoints1, stats, err) ||
            !SetInput(m_desc0Index, {1, pointCount, kSuperPointDescriptorDim},
                      descriptors0, stats, err) ||
            !SetInput(m_desc1Index, {1, pointCount, kSuperPointDescriptorDim},
                      descriptors1, stats, err)) {
            return false;
        }
        const std::vector<float> size0{imageSize0[0], imageSize0[1]};
        const std::vector<float> size1{imageSize1[0], imageSize1[1]};
        if (!SetInput(m_size0Index, {1, 2}, size0, stats, err) ||
            !SetInput(m_size1Index, {1, 2}, size1, stats, err)) {
            return false;
        }
        if (!TensorRtInferShapes(m_context.get(), "LightGlue", err)) {
            return false;
        }
        for (int index : {m_scoresIndex}) {
            const nvinfer1::Dims dims =
                TensorRtTensorShape(m_engine.get(), m_context.get(), index);
            if (!EnsureBindingBuffer(index, dims,
                                     TensorRtTensorDataType(m_engine.get(), index),
                                     err)) {
                return false;
            }
        }
        if (!BindTensorAddresses(err)) {
            return false;
        }
        const auto enqueueStartTp = std::chrono::steady_clock::now();
        if (stats != nullptr && m_eventTimingEnabled) {
            cudaEventRecord(m_computeStartEvent, m_stream);
        }
        if (!m_context->enqueueV3(m_stream)) {
            if (err != nullptr) {
                *err = "TensorRT LightGlue enqueue failed";
            }
            return false;
        }
        if (stats != nullptr && m_eventTimingEnabled) {
            cudaEventRecord(m_computeEndEvent, m_stream);
        }
        const auto enqueueEndTp = std::chrono::steady_clock::now();
        if (stats != nullptr) {
            stats->enqueueMs += DurationMs(enqueueStartTp, enqueueEndTp);
        }
        if (stats != nullptr && m_eventTimingEnabled) {
            cudaEventRecord(m_outputStartEvent, m_stream);
        }
        if (!ReadOutput(m_scoresIndex, scores, stats, err)) {
            return false;
        }
        if (stats != nullptr && m_eventTimingEnabled) {
            cudaEventRecord(m_outputEndEvent, m_stream);
        }
        const auto syncStartTp = std::chrono::steady_clock::now();
        if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT LightGlue stream synchronize failed";
            }
            return false;
        }
        const auto syncEndTp = std::chrono::steady_clock::now();
        const bool outputReady = FinalizeTensorRtOutput(scores, stats);
        if (stats != nullptr) {
            stats->syncMs += DurationMs(syncStartTp, syncEndTp);
            if (m_eventTimingEnabled) {
                float computeMs = 0.0f;
                float outputMs = 0.0f;
                if (cudaEventElapsedTime(&computeMs, m_computeStartEvent,
                                         m_computeEndEvent) == cudaSuccess) {
                    stats->gpuComputeMs += static_cast<double>(computeMs);
                }
                if (cudaEventElapsedTime(&outputMs, m_outputStartEvent,
                                         m_outputEndEvent) == cudaSuccess) {
                    stats->gpuOutputMs += static_cast<double>(outputMs);
                }
            }
        }
        if (!outputReady) {
            return false;
        }
        return !scores.Empty();
    }

  private:
    struct DeleteRuntime {
        void operator()(nvinfer1::IRuntime *ptr) const
        {
            delete ptr;
        }
    };
    struct DeleteEngine {
        void operator()(nvinfer1::ICudaEngine *ptr) const
        {
            delete ptr;
        }
    };
    struct DeleteContext {
        void operator()(nvinfer1::IExecutionContext *ptr) const
        {
            delete ptr;
        }
    };

    int FindBinding(std::initializer_list<const char *> names, bool input) const
    {
        return TensorRtFindTensor(m_engine.get(), names, input, false);
    }

    static nvinfer1::Dims MakeDims(const std::vector<int> &dims)
    {
        nvinfer1::Dims out{};
        out.nbDims = static_cast<int>(std::min<size_t>(dims.size(), 8));
        for (int i = 0; i < out.nbDims; ++i) {
            out.d[i] = dims[static_cast<size_t>(i)];
        }
        return out;
    }

    bool SetInput(int index, const std::vector<int> &dims,
                  const std::vector<float> &data, TensorRtForwardStats *stats,
                  std::string *err)
    {
        const nvinfer1::Dims trtDims = MakeDims(dims);
        if (!TensorRtSetInputShape(m_engine.get(), m_context.get(), index,
                                   trtDims, "LightGlue", err)) {
            return false;
        }
        if (!EnsureBindingBuffer(index, trtDims,
                                 TensorRtTensorDataType(m_engine.get(), index),
                                 err)) {
            return false;
        }
        const size_t bytes = data.size() * sizeof(float);
        const auto h2dStartTp = std::chrono::steady_clock::now();
        if (cudaMemcpyAsync(m_bindings[static_cast<size_t>(index)], data.data(),
                            bytes, cudaMemcpyHostToDevice,
                            m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT failed to copy LightGlue input";
            }
            return false;
        }
        const auto h2dEndTp = std::chrono::steady_clock::now();
        if (stats != nullptr) {
            stats->h2dMs += DurationMs(h2dStartTp, h2dEndTp);
            stats->h2dBytes += bytes;
        }
        return true;
    }

    bool EnsureBindingBuffer(int index, const nvinfer1::Dims &dims,
                             nvinfer1::DataType type, std::string *err)
    {
        const size_t bindingIndex = static_cast<size_t>(index);
        const size_t tensorCount =
            static_cast<size_t>(TensorRtTensorCount(m_engine.get()));
        if (m_bindings.size() < tensorCount) {
            m_bindings.assign(tensorCount, nullptr);
            m_bindingBytes.assign(tensorCount, 0);
            m_pinnedHostOutputs.clear();
            m_pinnedHostOutputs.resize(tensorCount);
        }
        const int64_t volume = TensorRtVolume(dims);
        const size_t elementSize = TensorRtElementSize(type);
        if (volume <= 0 || elementSize == 0) {
            if (err != nullptr) {
                *err = "invalid LightGlue TensorRT binding dimensions";
            }
            return false;
        }
        const size_t bytes = static_cast<size_t>(volume) * elementSize;
        if (m_bindingBytes[bindingIndex] >= bytes &&
            m_bindings[bindingIndex] != nullptr) {
            return true;
        }
        if (m_bindings[bindingIndex] != nullptr) {
            cudaFree(m_bindings[bindingIndex]);
            m_bindings[bindingIndex] = nullptr;
        }
        if (cudaMalloc(&m_bindings[bindingIndex], bytes) != cudaSuccess) {
            if (err != nullptr) {
                *err = "failed to allocate LightGlue TensorRT binding buffer";
            }
            return false;
        }
        m_bindingBytes[bindingIndex] = bytes;
        return true;
    }

    bool BindTensorAddresses(std::string *err)
    {
        const int tensorCount = TensorRtTensorCount(m_engine.get());
        for (int i = 0; i < tensorCount; ++i) {
            if (!TensorRtSetTensorAddress(m_engine.get(), m_context.get(), i,
                                          m_bindings[static_cast<size_t>(i)],
                                          "LightGlue", err)) {
                return false;
            }
        }
        return true;
    }

    bool ReadOutput(int index, TensorBlob &output, TensorRtForwardStats *stats,
                    std::string *err)
    {
        const nvinfer1::Dims dims =
            TensorRtTensorShape(m_engine.get(), m_context.get(), index);
        output.dims = TensorRtDimsToVector(dims);
        const int64_t volume = TensorRtVolume(dims);
        if (volume <= 0) {
            return false;
        }
        const nvinfer1::DataType dtype =
            TensorRtTensorDataType(m_engine.get(), index);
        return ScheduleTensorRtOutputCopy(
            m_bindings[static_cast<size_t>(index)], dtype, volume, m_stream, output,
            &m_pinnedHostOutputs[static_cast<size_t>(index)], stats, "LightGlue",
            err);
    }

    void Release()
    {
        if (m_stream != nullptr) {
            cudaStreamSynchronize(m_stream);
        }
        m_context.reset();
        if (m_computeStartEvent != nullptr) {
            cudaEventDestroy(m_computeStartEvent);
            m_computeStartEvent = nullptr;
        }
        if (m_computeEndEvent != nullptr) {
            cudaEventDestroy(m_computeEndEvent);
            m_computeEndEvent = nullptr;
        }
        if (m_outputStartEvent != nullptr) {
            cudaEventDestroy(m_outputStartEvent);
            m_outputStartEvent = nullptr;
        }
        if (m_outputEndEvent != nullptr) {
            cudaEventDestroy(m_outputEndEvent);
            m_outputEndEvent = nullptr;
        }
        for (void *ptr : m_bindings) {
            if (ptr != nullptr) {
                cudaFree(ptr);
            }
        }
        m_bindings.clear();
        m_bindingBytes.clear();
        m_pinnedHostOutputs.clear();
        if (m_stream != nullptr) {
            cudaStreamDestroy(m_stream);
            m_stream = nullptr;
        }
        m_engine.reset();
        m_runtime.reset();
    }

    TensorRtLogger m_logger;
    std::unique_ptr<nvinfer1::IRuntime, DeleteRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DeleteEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DeleteContext> m_context;
    cudaStream_t m_stream{nullptr};
    cudaEvent_t m_computeStartEvent{nullptr};
    cudaEvent_t m_computeEndEvent{nullptr};
    cudaEvent_t m_outputStartEvent{nullptr};
    cudaEvent_t m_outputEndEvent{nullptr};
    bool m_eventTimingEnabled{false};
    std::vector<void *> m_bindings;
    std::vector<size_t> m_bindingBytes;
    std::vector<CudaPinnedHostBuffer> m_pinnedHostOutputs;
    int m_kpts0Index{-1};
    int m_kpts1Index{-1};
    int m_desc0Index{-1};
    int m_desc1Index{-1};
    int m_size0Index{-1};
    int m_size1Index{-1};
    int m_scoresIndex{-1};
};
