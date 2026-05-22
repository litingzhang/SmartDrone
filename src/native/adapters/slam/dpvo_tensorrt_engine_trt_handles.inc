namespace {

class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[dpvo_trt] " << (msg != nullptr ? msg : "") << "\n";
        }
    }
};

struct DestroyRuntime {
    void operator()(nvinfer1::IRuntime *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

struct DestroyEngine {
    void operator()(nvinfer1::ICudaEngine *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

struct DestroyContext {
    void operator()(nvinfer1::IExecutionContext *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

class TensorRtEngineHandle {
  public:
    bool Load(const std::filesystem::path &enginePath, const char *name,
              std::string *err)
    {
        std::ifstream in(enginePath, std::ios::binary);
        if (!in) {
            if (err != nullptr) {
                *err = std::string(name) +
                       " TensorRT engine not found: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = std::string(name) +
                       " TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }

        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name + " TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(
            m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = std::string("failed to deserialize ") + name +
                       " TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name +
                       " TensorRT execution context";
            }
            return false;
        }
        m_path = enginePath.string();
        return true;
    }

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
    TensorRtLogger m_logger{};
    std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
    std::string m_path;
};

struct CudaStreamHandle {
    cudaStream_t stream{nullptr};

    bool Create(std::string *err)
    {
        if (stream != nullptr) {
            return true;
        }
        const cudaError_t rc = cudaStreamCreate(&stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err =
                    std::string("cudaStreamCreate failed: ") + cudaGetErrorString(rc);
            }
            stream = nullptr;
            return false;
        }
        return true;
    }

    void Reset()
    {
        if (stream != nullptr) {
            cudaStreamDestroy(stream);
            stream = nullptr;
        }
    }

    ~CudaStreamHandle()
    {
        Reset();
    }
};

class CudaDeviceBuffer {
  public:
    ~CudaDeviceBuffer()
    {
        Reset();
    }

    CudaDeviceBuffer() = default;
    CudaDeviceBuffer(const CudaDeviceBuffer &) = delete;
    CudaDeviceBuffer &operator=(const CudaDeviceBuffer &) = delete;

    bool Ensure(size_t bytes, std::string *err)
    {
        if (bytes <= m_bytes) {
            return true;
        }
        Reset();
        const cudaError_t rc = cudaMalloc(&m_ptr, bytes);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaMalloc failed bytes=") + std::to_string(bytes) +
                       ": " + cudaGetErrorString(rc);
            }
            m_ptr = nullptr;
            m_bytes = 0;
            return false;
        }
        m_bytes = bytes;
        return true;
    }

    void Reset()
    {
        if (m_ptr != nullptr) {
            cudaFree(m_ptr);
            m_ptr = nullptr;
        }
        m_bytes = 0;
    }

    void *Data() const
    {
        return m_ptr;
    }
    size_t Bytes() const
    {
        return m_bytes;
    }

  private:
    void *m_ptr{nullptr};
    size_t m_bytes{0};
};
