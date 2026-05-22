struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg)
        : config(std::move(cfg))
    {
    }

    bool Start()
    {
        std::cerr << "[dpvo_trt] native TensorRT backend was not compiled into "
                     "this target\n";
        return false;
    }

    void Stop()
    {
    }

    Core::Ports::SlamOutput Process(const Core::Ports::SlamInputBatch &input,
                                    bool, bool)
    {
        Core::Ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.trackingState = Core::Ports::kSlamTrackingLost;
        return out;
    }

    DpvoTensorRtConfig config;
};
