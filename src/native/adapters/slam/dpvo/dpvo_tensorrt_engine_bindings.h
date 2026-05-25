size_t TensorRtDataTypeSize(nvinfer1::DataType type);
int64_t DimsVolume(const nvinfer1::Dims &dims);
std::string DimsToString(const nvinfer1::Dims &dims);
int BindingCount(const TensorRtEngineHandle &handle);
const char *BindingName(const TensorRtEngineHandle &handle, int bindingIndex);
bool BindingIsInput(const TensorRtEngineHandle &handle, int bindingIndex);
nvinfer1::DataType BindingDataType(const TensorRtEngineHandle &handle,
                                   int bindingIndex);
bool SetBindingShape(TensorRtEngineHandle &handle, int bindingIndex,
                     const nvinfer1::Dims &dims, std::string *err);
int FindBindingIndex(const TensorRtEngineHandle &handle, const char *name);
bool BindingBytes(const TensorRtEngineHandle &handle, int bindingIndex,
                  size_t *bytes, nvinfer1::Dims *dimsOut, std::string *err);
size_t BindingElementCount(const TensorRtEngineHandle &handle, int bindingIndex,
                           nvinfer1::Dims *dimsOut, std::string *err);

struct TensorRtHostToDeviceCopyRequest {
    const TensorRtEngineHandle &handle;
    int bindingIndex;
    const float *src;
    size_t valueCount;
    CudaDeviceBuffer &device;
    cudaStream_t stream;
    std::vector<__half> &halfScratch;
    std::string *err;
};

struct TensorRtDeviceToHostCopyRequest {
    const TensorRtEngineHandle &handle;
    int bindingIndex;
    CudaDeviceBuffer &device;
    cudaStream_t stream;
    std::vector<float> &dst;
    std::vector<__half> &halfScratch;
    nvinfer1::Dims *dimsOut;
    std::string *err;
};

bool CopyFloatHostToBindingDevice(
    const TensorRtHostToDeviceCopyRequest &request);
bool CopyBindingDeviceToFloatHost(
    const TensorRtDeviceToHostCopyRequest &request);

bool EnsureBindingBuffer(const TensorRtEngineHandle &handle, int bindingIndex,
                         CudaDeviceBuffer &device, std::string *err);
bool BindingIsFloat(const TensorRtEngineHandle &handle, int bindingIndex);
bool SetBindingAddress(TensorRtEngineHandle &handle, int bindingIndex,
                       void *data, std::string *err);
bool SetBindingBufferAddresses(TensorRtEngineHandle &handle,
                               std::array<CudaDeviceBuffer, 16> &buffers,
                               std::string *err);
bool EnqueueTensorRt(TensorRtEngineHandle &handle, cudaStream_t stream,
                     const char *name, std::string *err);
bool CopyFloatDeviceBufferToHost(const CudaDeviceBuffer &device,
                                 size_t valueCount, cudaStream_t stream,
                                 std::vector<float> &dst, std::string *err);
