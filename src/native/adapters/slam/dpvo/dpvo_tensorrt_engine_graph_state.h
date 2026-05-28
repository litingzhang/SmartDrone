#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
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
    Sophus::SE3f tcw;
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

struct DpvoFeatureMapView {
    const float *data{nullptr};
    int channels{0};
    int height{0};
    int width{0};
    size_t valueCount{0};
};

struct DpvoPatchProjectionRequest {
    const DpvoFrameState &source;
    const DpvoFrameState &target;
    const DpvoPatchState &patch;
    const Eigen::Matrix3f &overrideR;
    bool useOverrideR;
    const DpvoIntrinsics &intrinsics;
    bool *valid;
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
    struct StereoDepthSearchResult {
        std::vector<float> scores;
        float bestScore{-std::numeric_limits<float>::infinity()};
        float secondScore{-std::numeric_limits<float>::infinity()};
        int bestDisp{0};
        int maxPatchDisp{0};
    };
    struct StereoDepthRequest {
        DpvoFrameState &frame;
        const DpvoFeatureMapView &rightMap;
        float fx;
        float baseline;
        int maxDisp;
        float minScore;
        float minMargin;
    };

    static float FeatureAt(const DpvoFeatureMapView &featureMap, int c, int y,
                           int x);
    static float SampleFeatureBilinear(
        const DpvoFeatureMapView &featureMap, int c, float x, float y);
    static void SampleFeatureVector(const DpvoFeatureMapView &featureMap,
                                    float x, float y, float *out);
    static void SampleFeaturePatch3(const DpvoFeatureMapView &featureMap,
                                    float x, float y, float *out);
    static std::vector<float> BuildPooledFmap(const std::vector<float> &src,
                                              int channels, int height,
                                              int width, int level);
    std::vector<DpvoPatchState> SelectPatches(const DpvoFrameState &frame,
                                              const cv::Mat &gray) const;
    float MedianRecentDepth() const;
    static std::array<float, 2>
    ProjectPatchCenter(const DpvoPatchProjectionRequest &request);
    float MotionMagnitude(int sourceFrame, int targetFrame,
                          const DpvoIntrinsics &intrinsics) const;
    void AppendEdgesForNewest();
    void PruneOldEdges();
    void CapActiveEdges();
    void RemoveFrameAt(int frameIndex);
    void PruneFrames();
    static bool BuildRightFmapView(const DpvoPatchifierRun &rightRun,
                                   const DpvoFrameState &frame,
                                   DpvoFeatureMapView *rightMap);
    static StereoDepthSearchResult SearchStereoDisparity(
        const StereoDepthRequest &request, int patchIndex);
    static float ComputeStereoPatchScore(const StereoDepthRequest &request,
                                         int patchIndex, int disparity);
    static float RefineStereoDisparity(
        const StereoDepthSearchResult &searchResult);
    static bool StereoSearchAccepted(
        const StereoDepthSearchResult &searchResult, float minScore,
        float minMargin);
    static bool UpdateStereoPatchDepth(const StereoDepthRequest &request,
                                       int patchIndex);
    DpvoFrameState CreateFrameState(uint64_t frameId, int64_t timestampNs,
                                    const cv::Mat &gray,
                                    const Sophus::SE3f &initialPose,
                                    const DpvoPatchifierRun &patchRun) const;
    static void LoadFrameFmap(DpvoFrameState *frame,
                              const DpvoPatchifierRun &patchRun);
    static void LoadFrameImapMetadata(DpvoFrameState *frame,
                                      const DpvoPatchifierRun &patchRun);
    void SelectFramePatches(DpvoFrameState *frame, const cv::Mat &gray) const;
    void SampleFrameImap(DpvoFrameState *frame,
                         const DpvoPatchifierRun &patchRun) const;
    void SampleFrameGmap(DpvoFrameState *frame) const;
    void CommitFrame(DpvoFrameState frame);

    static constexpr int INITIALIZATION_FRAMES = 8;
    static constexpr int DIM = 384;
    static constexpr int FMAP_CHANNELS = 128;
    static constexpr int PATCH_SIZE = 3;
    static constexpr int PATCH_RADIUS = 1;
    static constexpr int PATCH_AREA = PATCH_SIZE * PATCH_SIZE;
    static constexpr int KEYFRAME_INDEX = 4;
    static constexpr float KEYFRAME_THRESHOLD = 15.0f;
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
