class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[dpvo_trt] " << (msg != nullptr ? msg : "") << "\n";
        }
    }
};

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

class TensorRtEngineHandle {
  public:
    bool Load(const std::filesystem::path &enginePath, const char *name,
              std::string *err);

    bool Loaded() const
    {
        return static_cast<bool>(m_engine) && static_cast<bool>(m_context);
    }
    const std::string &Path() const
    {
        return m_path;
    }
    nvinfer1::ICudaEngine *Engine() const
    {
        return m_engine.get();
    }
    nvinfer1::IExecutionContext *Context() const
    {
        return m_context.get();
    }

  private:
    bool ReadEngineBytes(const std::filesystem::path &enginePath,
                         const char *name, std::string *err,
                         std::vector<char> &bytes);
    bool CreateRuntime(const char *name, std::string *err);
    bool DeserializeEngine(const std::filesystem::path &enginePath,
                           const char *name,
                           const std::vector<char> &bytes,
                           std::string *err);
    bool CreateContext(const char *name, std::string *err);
    static void SetError(std::string *err, std::string message);

    TensorRtLogger m_logger{};
    std::unique_ptr<nvinfer1::IRuntime, DeleteRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DeleteEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DeleteContext> m_context;
    std::string m_path;
};

struct CudaStreamHandle {
    cudaStream_t stream{nullptr};

    bool Create(std::string *err);
    void Reset();
    ~CudaStreamHandle();
};
