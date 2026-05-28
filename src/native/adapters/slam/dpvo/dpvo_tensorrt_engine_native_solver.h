#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <cuda_runtime_api.h>
#include <sophus/se3.hpp>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

class CudaDeviceBuffer;
class DpvoCudaKernelRuntime;
class DpvoUpdatePreAggRuntime;
class DpvoUpdatePostAggRuntime;
class TensorRtEngineHandle;
struct DpvoUpdatePreAggRun;
struct DpvoUpdatePostAggRun;

struct DpvoNativeSolverStepRequest {
    DpvoGraphState &graph;
    DpvoUpdatePreAggRuntime &preAggRuntime;
    TensorRtEngineHandle &preAggEngine;
    DpvoUpdatePostAggRuntime &postAggRuntime;
    TensorRtEngineHandle &postAggEngine;
    DpvoCudaKernelRuntime *cudaKernelRuntime;
    cudaStream_t stream;
    const DpvoIntrinsics &intrinsics;
    double *updateMs;
    std::string *err;
};

class DpvoNativeSolver {
  public:
    void Reset();
    bool HasPose() const;
    Sophus::SE3f LastTcw() const;
    bool Step(const DpvoNativeSolverStepRequest &request);

  private:
    static constexpr int DIM = 384;
    static constexpr int FMAP_CHANNELS = 128;
    static constexpr int PATCH_SIZE = 3;
    static constexpr int PATCH_AREA = PATCH_SIZE * PATCH_SIZE;
    static constexpr int PATCH_CENTER = 4;
    static constexpr int CORR_RADIUS = 3;
    static constexpr int CORR_SIDE = 2 * CORR_RADIUS + 1;
    static constexpr int CORR_DIM = 2 * CORR_SIDE * CORR_SIDE * PATCH_AREA;

    struct ErrorStats {
        float maxAbs{0.0f};
        double rmse{0.0};
    };
    struct SoftAggExpandRequest {
        const std::vector<float> &f;
        const std::vector<float> &g;
        const std::vector<int> &groupIds;
        int edgeCount;
        int dim;
        std::vector<float> *out;
    };
    struct AddBlockRequest {
        Eigen::MatrixXf &hessian;
        int row;
        int col;
        const Eigen::VectorXf &a;
        const Eigen::VectorXf &b;
        float scale;
    };
    struct RunBundleAdjustmentRequest {
        std::vector<DpvoFrameState> &frames;
        const std::vector<DpvoEdgeState> &edges;
        int patchesPerFrame;
        int optimizationWindow;
        const DpvoIntrinsics &intrinsics;
        const std::vector<std::array<float, 2>> &target;
        const std::vector<float> &weight;
    };
    struct PackCorrelationCudaInputsRequest {
        const std::vector<DpvoFrameState> &frames;
        const std::vector<DpvoEdgeState> &edges;
        int patchesPerFrame;
        const std::vector<std::array<float, PATCH_AREA * 2>> &coords;
        std::vector<float> *edgePatchGmap;
        std::vector<float> *edgeCoords;
        std::vector<int> *edgeTargetFrame;
        std::vector<float> *fmapStorage;
        std::vector<float> *fmapLevel4Storage;
        std::vector<int> *fmapOffsets;
        std::vector<int> *fmapHeights;
        std::vector<int> *fmapWidths;
        std::vector<int> *level4Offsets;
        std::vector<int> *level4Heights;
        std::vector<int> *level4Widths;
    };
    struct CorrelationEdgeView {
        const DpvoFrameState *source{nullptr};
        const DpvoFrameState *target{nullptr};
        size_t gmapOffset{0U};
    };
    struct CorrelationOffsetRequest {
        const DpvoFrameState &source;
        const DpvoFrameState &target;
        const std::array<float, PATCH_AREA * 2> &coords;
        size_t gmapOffset;
        float *outCorr;
    };
    struct CorrelationLevelRequest {
        const CorrelationOffsetRequest &offsetRequest;
        int offsetXIndex;
        int offsetYIndex;
        int patchY;
        int patchX;
        int levelIndex;
    };
    struct CorrelationDotRequest {
        const DpvoFrameState &source;
        const DpvoFeatureMapView &targetMapView;
        size_t gmapOffset;
        float sampleX;
        float sampleY;
        int patchY;
        int patchX;
    };
    struct StepIterationBuffers {
        std::vector<int> prevEdge;
        std::vector<int> nextEdge;
        std::vector<int> groupKk;
        std::vector<int> groupIj;
        std::vector<float> net;
        std::vector<float> inp;
        std::vector<float> corr;
        std::vector<float> prevNet;
        std::vector<float> nextNet;
        std::vector<float> prevMask;
        std::vector<float> nextMask;
        std::vector<std::array<float, PATCH_AREA * 2>> coords;
    };
    struct StepIterationRequest {
        const DpvoNativeSolverStepRequest &step;
        std::vector<DpvoFrameState> &frames;
        const std::vector<DpvoEdgeState> &edges;
        int edgeCount;
    };
    struct DeviceUpdateChainRequest {
        const StepIterationRequest &iteration;
        StepIterationBuffers &buffers;
        DpvoUpdatePostAggRun &postAgg;
    };
    struct HostUpdateChainRequest {
        const StepIterationRequest &iteration;
        StepIterationBuffers &buffers;
        DpvoUpdatePostAggRun &postAgg;
    };
    struct DeviceUpdateValidationBuffers {
        std::vector<float> kkF;
        std::vector<float> kkG;
        std::vector<float> ijF;
        std::vector<float> ijG;
        std::vector<float> cudaAggKkY;
        std::vector<float> cudaAggIjY;
    };
    struct CudaCorrelationInputs {
        std::vector<float> edgePatchGmap;
        std::vector<float> edgeCoords;
        std::vector<int> edgeTargetFrame;
        std::vector<float> fmapStorage;
        std::vector<float> fmapLevel4Storage;
        std::vector<int> fmapOffsets;
        std::vector<int> fmapHeights;
        std::vector<int> fmapWidths;
        std::vector<int> level4Offsets;
        std::vector<int> level4Heights;
        std::vector<int> level4Widths;
    };
    struct BundleAdjustmentState {
        int frameCount{0};
        int edgeCount{0};
        int poseStart{0};
        int poseVars{0};
        int depthVars{0};
        int poseDim{0};
        int totalDim{0};
        bool directSolve{false};
        std::unordered_map<int, int> patchVar;
    };
    struct BundleAdjustmentSystem {
        Eigen::MatrixXf poseHessian;
        Eigen::MatrixXf poseDepthJacobian;
        Eigen::VectorXf depthHessian;
        Eigen::VectorXf poseGradient;
        Eigen::VectorXf depthGradient;
    };
    struct BundleEdgeProjection {
        int edgeIndex{0};
        int srcPoseBase{-1};
        int dstPoseBase{-1};
        int depthIndex{-1};
        Eigen::Matrix3f relativeRotation{Eigen::Matrix3f::Identity()};
        Eigen::Vector3f t{Eigen::Vector3f::Zero()};
        float targetX{0.0f};
        float targetY{0.0f};
        float targetZ{0.0f};
        float patchInvDepth{0.0f};
        float invZ{0.0f};
        float invZ2{0.0f};
        float rx{0.0f};
        float ry{0.0f};
    };
    struct BundleResidualRequest {
        const RunBundleAdjustmentRequest &bundle;
        const BundleEdgeProjection &projection;
        int row;
        BundleAdjustmentSystem &system;
    };
    struct BundleSolveRequest {
        const BundleAdjustmentState &state;
        const BundleAdjustmentSystem &system;
    };
    struct BundleAdjustmentSolution {
        Eigen::VectorXf dxPose;
        Eigen::VectorXf dxDepth;
    };

    static float FeatureAt(const DpvoFeatureMapView &featureMap, int c, int y,
                           int x);
    static float SampleFeatureBilinear(
        const DpvoFeatureMapView &featureMap, int c, float x, float y);
    static void PredictNewestPose(std::vector<DpvoFrameState> &frames);
    static bool AcceptPoseStep(const Sophus::SE3f &reference,
                               const Sophus::SE3f &candidate);
    static void BuildTemporalNeighbors(const std::vector<DpvoEdgeState> &edges,
                                       std::vector<int> *prevEdge,
                                       std::vector<int> *nextEdge);
    static void ReprojectPatch(
        const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
        int patchesPerFrame, const DpvoIntrinsics &intrinsics,
        std::array<float, PATCH_AREA * 2> &coords);
    static void ComputeCorrelation(
        const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
        int patchesPerFrame, const std::array<float, PATCH_AREA * 2> &coords,
        float *outCorr);
    static bool PrepareCorrelationEdgeView(
        const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
        int patchesPerFrame, CorrelationEdgeView *view);
    static DpvoFeatureMapView BuildCorrelationTargetMapView(
        const DpvoFrameState &target, int levelIndex);
    static int CorrelationLevelScale(int levelIndex);
    static size_t CorrelationPatchIndex(int patchY, int patchX);
    static size_t CorrelationCoordIndex(int patchY, int patchX);
    static size_t CorrelationOutputIndex(int offsetXIndex, int offsetYIndex,
                                         int patchY, int patchX,
                                         int levelIndex);
    static void WriteCorrelationOffset(
        const CorrelationOffsetRequest &request, int offsetXIndex,
        int offsetYIndex);
    static void WriteCorrelationLevel(const CorrelationLevelRequest &request);
    static float ComputeCorrelationDot(const CorrelationDotRequest &request);
    static bool PackCorrelationCudaInputs(
        const PackCorrelationCudaInputsRequest &request);
    static bool PackCorrelationOutputsValid(
        const PackCorrelationCudaInputsRequest &request);
    static void ResetPackCorrelationOutputs(
        const PackCorrelationCudaInputsRequest &request, size_t edgeCount);
    static bool PackCorrelationFrameMaps(
        const PackCorrelationCudaInputsRequest &request);
    static bool PackCorrelationEdgeInputs(
        const PackCorrelationCudaInputsRequest &request);
    static void SoftAggExpand(const SoftAggExpandRequest &request);
    static ErrorStats CompareVectors(const std::vector<float> &a,
                                     const std::vector<float> &b);
    static void AdjSE3(const Eigen::Vector3f &t, const Eigen::Matrix3f &rotation,
                       const Eigen::Matrix<float, 6, 1> &x,
                       Eigen::Matrix<float, 6, 1> *y);
    static void AddBlock(const AddBlockRequest &request);
    static void AddVector(Eigen::VectorXf &v, int row,
                          const Eigen::VectorXf &a, float scale);
    static void RunBundleAdjustment(
        const RunBundleAdjustmentRequest &request);
    static bool PrepareBundleAdjustmentState(
        const RunBundleAdjustmentRequest &request,
        BundleAdjustmentState &state);
    static BundleAdjustmentSystem CreateBundleAdjustmentSystem(
        const BundleAdjustmentState &state);
    static void AccumulateBundleEdges(
        const RunBundleAdjustmentRequest &request,
        const BundleAdjustmentState &state, BundleAdjustmentSystem &system);
    static bool PrepareBundleEdgeProjection(
        const RunBundleAdjustmentRequest &request,
        const BundleAdjustmentState &state, int edgeIndex,
        BundleEdgeProjection &projection);
    static bool BundleEdgeFrameValid(const DpvoEdgeState &edge,
                                     int frameCount);
    static bool BundlePatchValid(const DpvoFrameState &source,
                                 int patchLocal);
    static void FillBundleProjection(
        const RunBundleAdjustmentRequest &request,
        const BundleAdjustmentState &state, int edgeIndex,
        const Sophus::SE3f &relativePose,
        BundleEdgeProjection &projection);
    static bool BundleProjectionInBounds(
        const DpvoIntrinsics &intrinsics, float x, float y, float rx,
        float ry);
    static void AccumulateBundleResidual(
        const BundleResidualRequest &request);
    static void FillBundleJacobian(
        const DpvoIntrinsics &intrinsics,
        const BundleEdgeProjection &projection, int row,
        Eigen::Matrix<float, 6, 1> &jacobian, float &depthJacobian);
    static void AccumulateBundleStereoPrior(
        const RunBundleAdjustmentRequest &request,
        const BundleAdjustmentState &state, BundleAdjustmentSystem &system);
    static bool SolveBundleAdjustmentSystem(
        const BundleSolveRequest &request, BundleAdjustmentSolution &solution);
    static bool SolveBundleAdjustmentDirect(
        const BundleSolveRequest &request, BundleAdjustmentSolution &solution);
    static bool SolveBundleAdjustmentSchur(
        const BundleSolveRequest &request, BundleAdjustmentSolution &solution);
    static void DampenBundleSystem(Eigen::MatrixXf &hessian,
                                   const BundleAdjustmentState &state);
    static void DampenBundleSchurSystem(Eigen::MatrixXf &schurHessian,
                                        int poseDim);
    static void ApplyBundleAdjustmentSolution(
        const RunBundleAdjustmentRequest &request,
        const BundleAdjustmentState &state,
        const BundleAdjustmentSolution &solution);
    static void ApplyBundlePoseUpdates(
        const RunBundleAdjustmentRequest &request,
        const BundleAdjustmentState &state,
        const BundleAdjustmentSolution &solution);
    static void ClampBundlePoseIncrement(
        Eigen::Matrix<float, 6, 1> &increment, float maxTransStep,
        float maxRotStep);
    static void ApplyBundleDepthUpdates(
        const RunBundleAdjustmentRequest &request,
        const BundleAdjustmentState &state,
        const BundleAdjustmentSolution &solution);
    bool PrepareStepInputs(const DpvoNativeSolverStepRequest &request,
                           std::vector<DpvoFrameState> *&frames,
                           const std::vector<DpvoEdgeState> *&edges);
    int UpdateIterationCount() const;
    void InitializeStepIteration(const StepIterationRequest &request,
                                 StepIterationBuffers &buffers);
    void FillEdgeInputs(const StepIterationRequest &request,
                        StepIterationBuffers &buffers);
    void FillTemporalInputs(int edgeCount, StepIterationBuffers &buffers);
    static void FillGroupIds(const std::vector<DpvoEdgeState> &edges,
                             StepIterationBuffers &buffers);
    void ComputeStepCorrelation(const StepIterationRequest &request,
                                StepIterationBuffers &buffers);
    bool TryComputeCudaCorrelation(const StepIterationRequest &request,
                                   StepIterationBuffers &buffers);
    bool PrepareCudaCorrelationInputs(const StepIterationRequest &request,
                                      StepIterationBuffers &buffers,
                                      CudaCorrelationInputs &inputs);
    void ValidateCudaCorrelation(const StepIterationRequest &request,
                                 StepIterationBuffers &buffers,
                                 std::vector<float> &cudaCorr);
    static void ComputeCpuCorrelationBatch(const StepIterationRequest &request,
                                           StepIterationBuffers &buffers);
    bool TryRunDeviceUpdateChain(const DeviceUpdateChainRequest &request);
    bool ValidateDeviceUpdateChain(const DeviceUpdateChainRequest &request,
                                   CudaDeviceBuffer &aggKkYDevice,
                                   CudaDeviceBuffer &aggIjYDevice,
                                   std::string &deviceErr);
    bool CopyDeviceUpdateValidationBuffers(
        const DeviceUpdateChainRequest &request,
        CudaDeviceBuffer &aggKkYDevice, CudaDeviceBuffer &aggIjYDevice,
        DeviceUpdateValidationBuffers &buffers, std::string &deviceErr);
    bool ValidateDeviceUpdateSoftAgg(
        const DeviceUpdateChainRequest &request,
        const DeviceUpdateValidationBuffers &buffers, std::string &deviceErr);
    bool RunHostUpdateChain(const HostUpdateChainRequest &request);
    bool TryComputeCudaSoftAgg(const HostUpdateChainRequest &request,
                               const DpvoUpdatePreAggRun &preAgg,
                               std::vector<float> &aggKkY,
                               std::vector<float> &aggIjY);
    void ValidateCudaSoftAgg(const HostUpdateChainRequest &request,
                             const DpvoUpdatePreAggRun &preAgg,
                             std::vector<float> &cudaAggKkY,
                             std::vector<float> &cudaAggIjY);
    void UpdateRecurrentNet(const StepIterationRequest &request,
                            const DpvoUpdatePostAggRun &postAgg);
    void ApplyPoseUpdate(const StepIterationRequest &request,
                         const StepIterationBuffers &buffers,
                         const DpvoUpdatePostAggRun &postAgg);
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
