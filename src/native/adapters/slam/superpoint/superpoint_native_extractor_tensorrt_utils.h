#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

#include <NvInfer.h>

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override;
};

template <typename TensorRtType>
struct DeleteTensorRtObject {
    void operator()(TensorRtType *ptr) const
    {
        delete ptr;
    }
};

struct TensorRtInputShapeRequest {
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    int index;
    const nvinfer1::Dims &dims;
    const char *engineName;
    std::string *err;
};

struct TensorRtTensorAddressRequest {
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    int index;
    void *data;
    const char *engineName;
    std::string *err;
};

size_t TensorRtElementSize(nvinfer1::DataType type);
int64_t TensorRtVolume(const nvinfer1::Dims &dims);
std::vector<int> TensorRtDimsToVector(const nvinfer1::Dims &dims);
int TensorRtTensorCount(const nvinfer1::ICudaEngine *engine);
const char *TensorRtTensorName(const nvinfer1::ICudaEngine *engine, int index);
bool TensorRtTensorIsInput(const nvinfer1::ICudaEngine *engine, int index);
nvinfer1::DataType TensorRtTensorDataType(const nvinfer1::ICudaEngine *engine,
                                          int index);
nvinfer1::Dims TensorRtTensorShape(const nvinfer1::ICudaEngine *engine,
                                   const nvinfer1::IExecutionContext *context,
                                   int index);
bool TensorRtSetInputShape(const TensorRtInputShapeRequest &request);
bool TensorRtSetTensorAddress(const TensorRtTensorAddressRequest &request);
bool TensorRtInferShapes(nvinfer1::IExecutionContext *context,
                         const char *engineName, std::string *err);
int TensorRtFindTensor(const nvinfer1::ICudaEngine *engine,
                       std::initializer_list<const char *> names, bool input,
                       bool fallback);

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
