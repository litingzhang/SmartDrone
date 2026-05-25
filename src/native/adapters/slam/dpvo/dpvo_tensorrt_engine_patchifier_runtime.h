class DpvoPatchifierRuntime {
  public:
    struct RunRequest {
        const cv::Mat &gray;
        TensorRtEngineHandle &engine;
        cudaStream_t stream;
        bool copyFmapToHost;
        bool copyImapToHost;
        std::string *err;
    };

    bool Initialize(const TensorRtEngineHandle &engine, int width, int height,
                    std::string *err);
    DpvoPatchifierRun Run(const RunRequest &request);
    void *FmapDevice() const;
    void *ImapDevice() const;
    nvinfer1::Dims LastFmapDims() const;
    nvinfer1::Dims LastImapDims() const;

  private:
    bool ValidateRunRequest(const RunRequest &request) const;
    bool PrepareRunBindings(const RunRequest &request, DpvoPatchifierRun *result,
                            size_t *imageBytes);
    bool CopyInputToDevice(const RunRequest &request, size_t imageBytes);
    bool Execute(const RunRequest &request);
    bool CopyRequestedOutputs(const RunRequest &request,
                              DpvoPatchifierRun *result);
    void FillInput(const cv::Mat &gray);

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
