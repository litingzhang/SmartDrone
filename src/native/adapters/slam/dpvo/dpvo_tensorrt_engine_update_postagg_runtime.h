class DpvoUpdatePostAggRuntime {
  public:
    struct RunRequest {
        TensorRtEngineHandle &engine;
        cudaStream_t stream;
        int edges;
        const std::vector<float> &baseNet;
        const std::vector<float> &aggKkY;
        const std::vector<float> &aggIjY;
        std::string *err;
    };
    struct RunDeviceRequest {
        TensorRtEngineHandle &engine;
        cudaStream_t stream;
        int edges;
        const CudaDeviceBuffer &baseNetDevice;
        const CudaDeviceBuffer &aggKkYDevice;
        const CudaDeviceBuffer &aggIjYDevice;
        std::string *err;
    };

    bool Initialize(const TensorRtEngineHandle &engine, std::string *err);
    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream,
                         int edges, std::string *err);

    DpvoUpdatePostAggRun Run(const RunRequest &request);
    DpvoUpdatePostAggRun RunDevice(const RunDeviceRequest &request);

  private:
    bool ConfigureInputShapes(TensorRtEngineHandle &engine, int edges,
                              std::string *err) const;
    bool HostInputSizesValid(const RunRequest &request, int edges) const;
    bool DeviceInputSizesValid(const RunDeviceRequest &request, int edges) const;
    bool EnsureOutputBuffers(TensorRtEngineHandle &engine, std::string *err);
    bool CopyHostInputs(const RunRequest &request, int edges);
    bool BindDeviceInputs(const RunDeviceRequest &request, int edges);
    bool RunInference(TensorRtEngineHandle &engine, cudaStream_t stream,
                      const char *name, std::string *err);
    bool EnqueueAndSynchronize(TensorRtEngineHandle &engine, cudaStream_t stream,
                               const char *name, std::string *err);
    bool CopyOutputs(TensorRtEngineHandle &engine, cudaStream_t stream,
                     DpvoUpdatePostAggRun *result, std::string *err);

    static constexpr int DIM = 384;
    int m_baseNetIndex{-1};
    int m_kkYIndex{-1};
    int m_ijYIndex{-1};
    int m_updatedNetIndex{-1};
    int m_deltaIndex{-1};
    int m_weightIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
    std::vector<__half> m_halfScratch;
};
