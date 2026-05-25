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

struct DpvoWarmupBindingsRequest {
    TensorRtEngineHandle &engine;
    cudaStream_t stream;
    std::initializer_list<int> indices;
    std::array<CudaDeviceBuffer, 16> &buffers;
    const char *name;
    std::string *err;
};

DpvoUpdateRun WarmupBindings(const DpvoWarmupBindingsRequest &request);

class DpvoUpdateRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, std::string *err);
    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream,
                         int edges, std::string *err);

    static constexpr int DIM = 384;
    static constexpr int CORR_DIM = 882;

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
