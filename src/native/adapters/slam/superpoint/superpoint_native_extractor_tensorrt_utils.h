
class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[superpoint_trt] " << msg << "\n";
        }
    }
};

size_t TensorRtElementSize(nvinfer1::DataType type)
{
    switch (type) {
    case nvinfer1::DataType::kFLOAT:
        return sizeof(float);
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
        return 1;
    case nvinfer1::DataType::kINT32:
        return 4;
    case nvinfer1::DataType::kBOOL:
        return 1;
    default:
        return 0;
    }
}

int64_t TensorRtVolume(const nvinfer1::Dims &dims)
{
    int64_t volume = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) {
            return 0;
        }
        volume *= dims.d[i];
    }
    return volume;
}

std::vector<int> TensorRtDimsToVector(const nvinfer1::Dims &dims)
{
    std::vector<int> out;
    out.reserve(static_cast<size_t>(dims.nbDims));
    for (int i = 0; i < dims.nbDims; ++i) {
        out.push_back(dims.d[i]);
    }
    return out;
}

int TensorRtTensorCount(const nvinfer1::ICudaEngine *engine)
{
    return engine != nullptr ? engine->getNbIOTensors() : 0;
}

const char *TensorRtTensorName(const nvinfer1::ICudaEngine *engine, int index)
{
    if (engine == nullptr || index < 0 || index >= engine->getNbIOTensors()) {
        return nullptr;
    }
    return engine->getIOTensorName(index);
}

bool TensorRtTensorIsInput(const nvinfer1::ICudaEngine *engine, int index)
{
    const char *name = TensorRtTensorName(engine, index);
    return name != nullptr &&
           engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
}

nvinfer1::DataType TensorRtTensorDataType(const nvinfer1::ICudaEngine *engine,
                                          int index)
{
    const char *name = TensorRtTensorName(engine, index);
    if (name == nullptr) {
        return nvinfer1::DataType::kFLOAT;
    }
    return engine->getTensorDataType(name);
}

nvinfer1::Dims TensorRtTensorShape(const nvinfer1::ICudaEngine *engine,
                                   const nvinfer1::IExecutionContext *context,
                                   int index)
{
    const char *name = TensorRtTensorName(engine, index);
    if (name == nullptr) {
        return {};
    }
    return context != nullptr ? context->getTensorShape(name)
                              : engine->getTensorShape(name);
}

bool TensorRtSetInputShape(nvinfer1::ICudaEngine *engine,
                           nvinfer1::IExecutionContext *context, int index,
                           const nvinfer1::Dims &dims, const char *engineName,
                           std::string *err)
{
    const char *name = TensorRtTensorName(engine, index);
    if (context == nullptr || name == nullptr) {
        if (err != nullptr) {
            *err = std::string(engineName) + " TensorRT input tensor is invalid";
        }
        return false;
    }
    if (!context->setInputShape(name, dims)) {
        if (err != nullptr) {
            *err = std::string(engineName) +
                   " TensorRT failed to set input shape for " + name;
        }
        return false;
    }
    return true;
}

bool TensorRtSetTensorAddress(nvinfer1::ICudaEngine *engine,
                              nvinfer1::IExecutionContext *context, int index,
                              void *data, const char *engineName,
                              std::string *err)
{
    const char *name = TensorRtTensorName(engine, index);
    if (context == nullptr || name == nullptr || data == nullptr) {
        if (err != nullptr) {
            *err = std::string(engineName) + " TensorRT tensor address is invalid";
        }
        return false;
    }
    if (!context->setTensorAddress(name, data)) {
        if (err != nullptr) {
            *err = std::string(engineName) +
                   " TensorRT failed to set tensor address for " + name;
        }
        return false;
    }
    return true;
}

bool TensorRtInferShapes(nvinfer1::IExecutionContext *context,
                         const char *engineName, std::string *err)
{
    if (context == nullptr) {
        if (err != nullptr) {
            *err = std::string(engineName) + " TensorRT context is invalid";
        }
        return false;
    }
    const int rc = context->inferShapes(0, nullptr);
    if (rc != 0) {
        if (err != nullptr) {
            *err = std::string(engineName) +
                   " TensorRT shape inference failed: " + std::to_string(rc);
        }
        return false;
    }
    return true;
}

int TensorRtFindTensor(const nvinfer1::ICudaEngine *engine,
                       std::initializer_list<const char *> names, bool input,
                       bool fallback)
{
    const int tensorCount = TensorRtTensorCount(engine);
    for (const char *name : names) {
        for (int i = 0; i < tensorCount; ++i) {
            const char *tensorName = TensorRtTensorName(engine, i);
            if (tensorName != nullptr && std::string(tensorName) == name &&
                TensorRtTensorIsInput(engine, i) == input) {
                return i;
            }
        }
    }
    if (!fallback) {
        return -1;
    }
    for (int i = 0; i < tensorCount; ++i) {
        if (TensorRtTensorIsInput(engine, i) == input) {
            return i;
        }
    }
    return -1;
}
