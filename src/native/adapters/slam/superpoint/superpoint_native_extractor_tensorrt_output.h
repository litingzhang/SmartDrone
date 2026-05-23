bool ScheduleTensorRtOutputCopy(void *devicePtr, nvinfer1::DataType dtype,
                                int64_t volume, cudaStream_t stream,
                                TensorBlob &output,
                                CudaPinnedHostBuffer *pinnedHostBuffer,
                                TensorRtForwardStats *stats,
                                const char *engineName, std::string *err)
{
    output.ResetHostData();
    if (volume <= 0) {
        return false;
    }

    const size_t elementCount = static_cast<size_t>(volume);
    const bool usePinnedHost =
        pinnedHostBuffer != nullptr &&
        EnvFlag("SMART_DRONE_TRT_PINNED_HOST_OUTPUT", false);
    auto recordOutputCopy =
        [&](const std::chrono::steady_clock::time_point &start,
            const std::chrono::steady_clock::time_point &end, size_t bytes,
            bool pinned) {
            if (stats != nullptr) {
                stats->outputMs += DurationMs(start, end);
                stats->d2hBytes += bytes;
                stats->pinnedHostOutput = stats->pinnedHostOutput || pinned;
            }
        };

    if (dtype == nvinfer1::DataType::kFLOAT) {
        const size_t bytes = elementCount * sizeof(float);
        output.data.resize(elementCount);
        void *hostPtr = output.data.data();
        bool pinned = false;
        if (usePinnedHost && pinnedHostBuffer->Ensure(bytes)) {
            hostPtr = pinnedHostBuffer->ptr;
            output.pendingHostData = pinnedHostBuffer->ptr;
            output.pendingElementCount = elementCount;
            output.hostStorage = TensorBlob::HostStorage::Float;
            output.pinnedHostData = true;
            pinned = true;
        }
        const auto outputStartTp = std::chrono::steady_clock::now();
        if (cudaMemcpyAsync(hostPtr, devicePtr, bytes, cudaMemcpyDeviceToHost,
                            stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT failed to copy ") + engineName +
                       " FP32 output";
            }
            return false;
        }
        const auto outputEndTp = std::chrono::steady_clock::now();
        recordOutputCopy(outputStartTp, outputEndTp, bytes, pinned);
        return true;
    }

    if (dtype == nvinfer1::DataType::kHALF) {
        const size_t bytes = elementCount * sizeof(uint16_t);
        output.data.resize(elementCount);
        output.halfData.resize(elementCount);
        void *hostPtr = output.halfData.data();
        bool pinned = false;
        if (usePinnedHost && pinnedHostBuffer->Ensure(bytes)) {
            hostPtr = pinnedHostBuffer->ptr;
            output.pendingHostData = pinnedHostBuffer->ptr;
            output.pendingElementCount = elementCount;
            output.hostStorage = TensorBlob::HostStorage::Half;
            output.pinnedHostData = true;
            pinned = true;
        } else {
            output.pendingHostData = output.halfData.data();
            output.pendingElementCount = elementCount;
            output.hostStorage = TensorBlob::HostStorage::Half;
        }
        const auto outputStartTp = std::chrono::steady_clock::now();
        if (cudaMemcpyAsync(hostPtr, devicePtr, bytes, cudaMemcpyDeviceToHost,
                            stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT failed to copy ") + engineName +
                       " FP16 output";
            }
            return false;
        }
        const auto outputEndTp = std::chrono::steady_clock::now();
        recordOutputCopy(outputStartTp, outputEndTp, bytes, pinned);
        return true;
    }

    if (dtype == nvinfer1::DataType::kINT32) {
        const size_t bytes = elementCount * sizeof(int32_t);
        output.data.resize(elementCount);
        output.intData.resize(elementCount);
        void *hostPtr = output.intData.data();
        bool pinned = false;
        if (usePinnedHost && pinnedHostBuffer->Ensure(bytes)) {
            hostPtr = pinnedHostBuffer->ptr;
            output.pendingHostData = pinnedHostBuffer->ptr;
            output.pendingElementCount = elementCount;
            output.hostStorage = TensorBlob::HostStorage::Int32;
            output.pinnedHostData = true;
            pinned = true;
        } else {
            output.pendingHostData = output.intData.data();
            output.pendingElementCount = elementCount;
            output.hostStorage = TensorBlob::HostStorage::Int32;
        }
        const auto outputStartTp = std::chrono::steady_clock::now();
        if (cudaMemcpyAsync(hostPtr, devicePtr, bytes, cudaMemcpyDeviceToHost,
                            stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT failed to copy ") + engineName +
                       " INT32 output";
            }
            return false;
        }
        const auto outputEndTp = std::chrono::steady_clock::now();
        recordOutputCopy(outputStartTp, outputEndTp, bytes, pinned);
        return true;
    }

    if (err != nullptr) {
        *err = std::string("TensorRT ") + engineName +
               " output has unsupported data type";
    }
    return false;
}

bool FinalizeTensorRtOutput(TensorBlob &output, TensorRtForwardStats *stats)
{
    if (output.pendingHostData == nullptr || output.pendingElementCount == 0) {
        return true;
    }
    const auto convertStartTp = std::chrono::steady_clock::now();
    if (output.hostStorage == TensorBlob::HostStorage::Float) {
        if (output.pinnedHostData &&
            EnvFlag("SMART_DRONE_TRT_PINNED_HOST_VIEW", false)) {
            output.floatData = static_cast<const float *>(output.pendingHostData);
            output.floatElementCount = output.pendingElementCount;
        } else {
            if (output.pinnedHostData) {
                std::memcpy(output.data.data(), output.pendingHostData,
                            output.pendingElementCount * sizeof(float));
            }
            output.floatData = output.data.data();
            output.floatElementCount = output.data.size();
        }
    } else if (output.hostStorage == TensorBlob::HostStorage::Half) {
        const auto *src = static_cast<const uint16_t *>(output.pendingHostData);
        for (size_t i = 0; i < output.pendingElementCount; ++i) {
            output.data[i] = HalfToFloat(src[i]);
        }
        output.floatData = output.data.data();
        output.floatElementCount = output.data.size();
    } else if (output.hostStorage == TensorBlob::HostStorage::Int32) {
        const auto *src = static_cast<const int32_t *>(output.pendingHostData);
        for (size_t i = 0; i < output.pendingElementCount; ++i) {
            output.data[i] = static_cast<float>(src[i]);
        }
        output.floatData = output.data.data();
        output.floatElementCount = output.data.size();
    }
    const auto convertEndTp = std::chrono::steady_clock::now();
    if (stats != nullptr) {
        stats->outputConvertMs += DurationMs(convertStartTp, convertEndTp);
    }
    output.pendingHostData = nullptr;
    output.pendingElementCount = 0;
    output.hostStorage = TensorBlob::HostStorage::Float;
    return true;
}
