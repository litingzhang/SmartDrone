#pragma once

#include <cstddef>

#include <NvInfer.h>

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

struct DpvoPatchifierRun {
    nvinfer1::Dims fmapDims{};
    nvinfer1::Dims imapDims{};
    const float *fmapHost{nullptr};
    const float *imapHost{nullptr};
    size_t fmapValueCount{0};
    size_t imapValueCount{0};
    double elapsedMs{0.0};
    bool ok{false};
};

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
