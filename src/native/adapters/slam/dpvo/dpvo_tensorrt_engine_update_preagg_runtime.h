class DpvoUpdatePreAggRuntime {
  public:
    struct RunRequest {
        TensorRtEngineHandle &engine;
        cudaStream_t stream;
        int edges;
        const std::vector<float> &net;
        const std::vector<float> &inp;
        const std::vector<float> &corr;
        const std::vector<float> &prevNet;
        const std::vector<float> &nextNet;
        const std::vector<float> &prevMask;
        const std::vector<float> &nextMask;
        std::string *err;
    };

    bool Initialize(const TensorRtEngineHandle &engine, std::string *err);
    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream,
                         int edges, std::string *err);

    DpvoUpdatePreAggRun Run(const RunRequest &request);
    DpvoUpdatePreAggDeviceRun RunDevice(const RunRequest &request);

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
    bool ConfigureInputShapes(TensorRtEngineHandle &engine, int edges,
                              std::string *err) const;
    bool HostInputSizesValid(const RunRequest &request, int edges,
                             const char *messagePrefix) const;
    bool OutputBindingsAreFloat(TensorRtEngineHandle &engine,
                                std::string *err) const;
    bool EnsureOutputBuffers(TensorRtEngineHandle &engine, std::string *err);
    bool CopyInputs(const RunRequest &request, int edges);
    bool RunInference(TensorRtEngineHandle &engine, cudaStream_t stream,
                      const char *name, std::string *err);
    bool CopyOutputs(TensorRtEngineHandle &engine, cudaStream_t stream,
                     DpvoUpdatePreAggRun *result, std::string *err);

    static constexpr int DIM = 384;
    static constexpr int CORR_DIM = 882;
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
