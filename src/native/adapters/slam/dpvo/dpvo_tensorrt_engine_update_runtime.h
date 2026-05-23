struct DpvoUpdateRun {
    double elapsedMs{0.0};
    bool ok{false};
};

struct DpvoUpdatePreAggRun {
    std::vector<float> baseNet;
    std::vector<float> aggKkF;
    std::vector<float> aggKkG;
    std::vector<float> aggIjF;
    std::vector<float> aggIjG;
    double elapsedMs{0.0};
    bool ok{false};
};

struct DpvoUpdatePreAggDeviceRun {
    double elapsedMs{0.0};
    bool ok{false};
};

struct DpvoUpdatePostAggRun {
    std::vector<float> updatedNet;
    std::vector<float> delta;
    std::vector<float> weight;
    double elapsedMs{0.0};
    bool ok{false};
};

inline DpvoUpdateRun WarmupBindings(TensorRtEngineHandle &engine, cudaStream_t stream,
                             std::initializer_list<int> indices,
                             std::array<CudaDeviceBuffer, 16> &buffers,
                             const char *name, std::string *err)
{
    DpvoUpdateRun result{};
    if (engine.Engine() == nullptr || engine.Context() == nullptr ||
        stream == nullptr) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") +
                   " TensorRT context or CUDA stream is not initialized";
        }
        return result;
    }

    const int nbBindings = BindingCount(engine);
    if (nbBindings > static_cast<int>(buffers.size())) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") +
                   " has more bindings than expected";
        }
        return result;
    }

    for (int index : indices) {
        size_t bytes = 0;
        if (!BindingBytes(engine, index, &bytes, nullptr, err)) {
            return result;
        }
        if (index < 0 || index >= static_cast<int>(buffers.size())) {
            if (err != nullptr) {
                *err = std::string(name != nullptr ? name : "engine") +
                       " binding index exceeds local buffer table";
            }
            return result;
        }
        if (!buffers[static_cast<size_t>(index)].Ensure(bytes, err)) {
            return result;
        }
        if (BindingIsInput(engine, index)) {
            const cudaError_t rc = cudaMemsetAsync(
                buffers[static_cast<size_t>(index)].Data(), 0, bytes, stream);
            if (rc != cudaSuccess) {
                if (err != nullptr) {
                    *err = std::string(name != nullptr ? name : "engine") +
                           " input memset failed: " + cudaGetErrorString(rc);
                }
                return result;
            }
        }
    }

    const auto t0 = std::chrono::steady_clock::now();
    if (!SetBindingBufferAddresses(engine, buffers, err) ||
        !EnqueueTensorRt(engine, stream, name, err)) {
        return result;
    }
    const cudaError_t rc = cudaStreamSynchronize(stream);
    if (rc != cudaSuccess) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") +
                   " synchronize failed: " + cudaGetErrorString(rc);
        }
        return result;
    }
    result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    return result;
}

class DpvoUpdateRuntime {
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
        m_updatedNetIndex = FindBindingIndex(engine, "updated_net");
        m_deltaIndex = FindBindingIndex(engine, "delta");
        m_weightIndex = FindBindingIndex(engine, "weight");
        const bool ok = m_netIndex >= 0 && m_inpIndex >= 0 && m_corrIndex >= 0 &&
                        m_prevNetIndex >= 0 && m_nextNetIndex >= 0 &&
                        m_prevMaskIndex >= 0 && m_nextMaskIndex >= 0 &&
                        m_updatedNetIndex >= 0 && m_deltaIndex >= 0 &&
                        m_weightIndex >= 0;
        if (!ok && err != nullptr) {
            *err = "update binding lookup failed";
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
                *err = "update TensorRT context or CUDA stream is not initialized";
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
                               m_updatedNetIndex, m_deltaIndex, m_weightIndex},
                              m_buffers, "update", err);
    }

    static constexpr int kDim = 384;
    static constexpr int kCorrDim = 882;

  private:
    int m_netIndex{-1};
    int m_inpIndex{-1};
    int m_corrIndex{-1};
    int m_prevNetIndex{-1};
    int m_nextNetIndex{-1};
    int m_prevMaskIndex{-1};
    int m_nextMaskIndex{-1};
    int m_updatedNetIndex{-1};
    int m_deltaIndex{-1};
    int m_weightIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
};
