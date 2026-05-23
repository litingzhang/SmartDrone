#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_patchifier_run.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

struct DpvoPatchState {
    float x{0.0f};
    float y{0.0f};
    float invDepth{1.0f};
    float stereoPriorInvDepth{1.0f};
    bool hasStereoPrior{false};
};

struct DpvoFrameState {
    uint64_t frameId{0};
    int64_t timestampNs{0};
    Sophus::SE3f Tcw;
    std::vector<DpvoPatchState> patches;
    std::vector<float> fmap;
    std::vector<float> fmapLevel4;
    std::vector<float> patchImap;
    std::vector<float> patchGmap;
    int fmapChannels{0};
    int fmapHeight{0};
    int fmapWidth{0};
    int imapChannels{0};
    int imapHeight{0};
    int imapWidth{0};
    int imageWidth{0};
    int imageHeight{0};
};

struct DpvoEdgeState {
    int patchGlobal{0};
    int sourceFrame{0};
    int targetFrame{0};
};

struct DpvoIntrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
};

class DpvoGraphState {
  public:
    void Reset(int patchesPerFrame, int optimizationWindow);
    void PushFrame(uint64_t frameId, int64_t timestampNs, const cv::Mat &gray,
                   const Sophus::SE3f &initialPose,
                   const DpvoPatchifierRun &patchRun);
    bool Initialized() const;
    int FrameCount() const;
    int EdgeCount() const;
    int PatchCount() const;
    int PatchesPerFrame() const;
    int OptimizationWindow() const;
    int KeyframeRemovals() const;
    const std::vector<DpvoFrameState> &Frames() const;
    std::vector<DpvoFrameState> &MutableFrames();
    const std::vector<DpvoEdgeState> &Edges() const;
    int LastStereoDepthUpdates() const;
    void ApplyStereoDepthFromRightFmap(const DpvoPatchifierRun &rightRun,
                                       float fx, float baseline);
    bool FeatureMapsReady() const;
    const DpvoFrameState *PreviousFrame() const;
    const DpvoFrameState *NewestFrame() const;
    bool MaybeRemoveKeyframe(const DpvoIntrinsics &intrinsics);

  private:
    static float FeatureAt(const float *data, int channels, int height, int width,
                           int c, int y, int x);
    static float SampleFeatureBilinear(const float *data, int channels,
                                       int height, int width, int c, float x,
                                       float y);
    static void SampleFeatureVector(const float *data, int channels, int height,
                                    int width, float x, float y, float *out);
    static void SampleFeaturePatch3(const float *data, int channels, int height,
                                    int width, float x, float y, float *out);
    static std::vector<float> BuildPooledFmap(const std::vector<float> &src,
                                              int channels, int height,
                                              int width, int level);
    std::vector<DpvoPatchState> SelectPatches(const DpvoFrameState &frame,
                                              const cv::Mat &gray) const;
    float MedianRecentDepth() const;
    static std::array<float, 2>
    ProjectPatchCenter(const DpvoFrameState &source, const DpvoFrameState &target,
                       const DpvoPatchState &patch,
                       const Eigen::Matrix3f &overrideR, bool useOverrideR,
                       const DpvoIntrinsics &intrinsics, bool *valid);
    float MotionMagnitude(int sourceFrame, int targetFrame,
                          const DpvoIntrinsics &intrinsics) const;
    void AppendEdgesForNewest();
    void PruneOldEdges();
    void CapActiveEdges();
    void RemoveFrameAt(int frameIndex);
    void PruneFrames();

    static constexpr int kInitializationFrames = 8;
    static constexpr int kDim = 384;
    static constexpr int kFmapChannels = 128;
    static constexpr int kPatchSize = 3;
    static constexpr int kPatchRadius = 1;
    static constexpr int kPatchArea = kPatchSize * kPatchSize;
    static constexpr int kKeyframeIndex = 4;
    static constexpr float kKeyframeThreshold = 15.0f;
    int m_patchesPerFrame{48};
    int m_optimizationWindow{7};
    int m_patchLifetime{11};
    int m_removalWindow{16};
    int m_maxActiveEdges{4096};
    int m_counter{0};
    int m_lastStereoDepthUpdates{0};
    int m_keyframeRemovals{0};
    bool m_persistentEdges{false};
    bool m_keyframeRemovalEnabled{false};
    bool m_capRebuiltEdges{false};
    bool m_initialized{false};
    std::vector<DpvoFrameState> m_frames;
    std::vector<DpvoEdgeState> m_edges;
};

struct DpvoEdgeKey {
    uint64_t sourceFrameId{0};
    uint64_t targetFrameId{0};
    int patchLocal{0};

    bool operator==(const DpvoEdgeKey &other) const;
};

struct DpvoEdgeKeyHash {
    size_t operator()(const DpvoEdgeKey &key) const;
};

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
