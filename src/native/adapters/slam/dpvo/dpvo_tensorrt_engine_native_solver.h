#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <cuda_runtime_api.h>
#include <sophus/se3.hpp>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

class DpvoCudaKernelRuntime;
class DpvoUpdatePreAggRuntime;
class DpvoUpdatePostAggRuntime;
class TensorRtEngineHandle;

class DpvoNativeSolver {
  public:
    void Reset();
    bool HasPose() const;
    Sophus::SE3f LastTcw() const;
    bool Step(DpvoGraphState &graph, DpvoUpdatePreAggRuntime &preAggRuntime,
              TensorRtEngineHandle &preAggEngine,
              DpvoUpdatePostAggRuntime &postAggRuntime,
              TensorRtEngineHandle &postAggEngine,
              DpvoCudaKernelRuntime *cudaKernelRuntime, cudaStream_t stream,
              const DpvoIntrinsics &intrinsics, double *updateMs,
              std::string *err);

  private:
    struct ErrorStats {
        float maxAbs{0.0f};
        double rmse{0.0};
    };

    static constexpr int kDim = 384;
    static constexpr int kFmapChannels = 128;
    static constexpr int kPatchSize = 3;
    static constexpr int kPatchArea = kPatchSize * kPatchSize;
    static constexpr int kPatchCenter = 4;
    static constexpr int kCorrRadius = 3;
    static constexpr int kCorrSide = 2 * kCorrRadius + 1;
    static constexpr int kCorrDim = 2 * kCorrSide * kCorrSide * kPatchArea;

    static float FeatureAt(const std::vector<float> &data, int channels,
                           int height, int width, int c, int y, int x);
    static float SampleFeatureBilinear(const std::vector<float> &data,
                                       int channels, int height, int width,
                                       int c, float x, float y);
    static void PredictNewestPose(std::vector<DpvoFrameState> &frames);
    static bool AcceptPoseStep(const Sophus::SE3f &reference,
                               const Sophus::SE3f &candidate);
    static void BuildTemporalNeighbors(const std::vector<DpvoEdgeState> &edges,
                                       std::vector<int> *prevEdge,
                                       std::vector<int> *nextEdge);
    static void ReprojectPatch(
        const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
        int patchesPerFrame, const DpvoIntrinsics &intrinsics,
        std::array<float, kPatchArea * 2> &coords);
    static void ComputeCorrelation(
        const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
        int patchesPerFrame, const std::array<float, kPatchArea * 2> &coords,
        float *outCorr);
    static bool PackCorrelationCudaInputs(
        const std::vector<DpvoFrameState> &frames,
        const std::vector<DpvoEdgeState> &edges, int patchesPerFrame,
        const std::vector<std::array<float, kPatchArea * 2>> &coords,
        std::vector<float> *edgePatchGmap, std::vector<float> *edgeCoords,
        std::vector<int> *edgeTargetFrame, std::vector<float> *fmapStorage,
        std::vector<float> *fmapLevel4Storage, std::vector<int> *fmapOffsets,
        std::vector<int> *fmapHeights, std::vector<int> *fmapWidths,
        std::vector<int> *level4Offsets, std::vector<int> *level4Heights,
        std::vector<int> *level4Widths);
    static void SoftAggExpand(const std::vector<float> &f,
                              const std::vector<float> &g,
                              const std::vector<int> &groupIds, int edgeCount,
                              int dim, std::vector<float> *out);
    static ErrorStats CompareVectors(const std::vector<float> &a,
                                     const std::vector<float> &b);
    static void AdjSE3(const Eigen::Vector3f &t, const Eigen::Matrix3f &R,
                       const Eigen::Matrix<float, 6, 1> &x,
                       Eigen::Matrix<float, 6, 1> *y);
    static void AddBlock(Eigen::MatrixXf &H, int row, int col,
                         const Eigen::VectorXf &a, const Eigen::VectorXf &b,
                         float scale);
    static void AddVector(Eigen::VectorXf &v, int row,
                          const Eigen::VectorXf &a, float scale);
    static void RunBundleAdjustment(
        std::vector<DpvoFrameState> &frames,
        const std::vector<DpvoEdgeState> &edges, int patchesPerFrame,
        int optimizationWindow, const DpvoIntrinsics &intrinsics,
        const std::vector<std::array<float, 2>> &target,
        const std::vector<float> &weight);
    void PruneEdgeNet(const std::vector<DpvoFrameState> &frames,
                      const std::vector<DpvoEdgeState> &edges,
                      int patchesPerFrame);

    std::unordered_map<DpvoEdgeKey, std::vector<float>, DpvoEdgeKeyHash>
        m_netByEdge;
    Sophus::SE3f m_lastTcw{Sophus::SE3f()};
    bool m_hasPose{false};
    bool m_bootstrapComplete{false};
    bool m_loggedCudaCorr{false};
    bool m_loggedCudaCorrFailure{false};
    bool m_loggedCudaSoftAgg{false};
    bool m_loggedCudaSoftAggFailure{false};
    bool m_loggedCudaDeviceUpdateChain{false};
    bool m_loggedCudaDeviceUpdateChainFailure{false};
};

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
