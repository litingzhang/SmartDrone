#include "adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver.h"

#include <algorithm>
#include <cmath>

#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

DpvoNativeSolver::ErrorStats DpvoNativeSolver::CompareVectors(
    const std::vector<float> &a, const std::vector<float> &b)
{
    ErrorStats stats{};
    const size_t n = std::min(a.size(), b.size());
    if (n == 0U) {
        return stats;
    }
    double sq = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const float d = std::fabs(a[i] - b[i]);
        stats.maxAbs = std::max(stats.maxAbs, d);
        sq += static_cast<double>(d) * static_cast<double>(d);
    }
    stats.rmse = std::sqrt(sq / static_cast<double>(n));
    return stats;
}

void DpvoNativeSolver::AdjSE3(const Eigen::Vector3f &t,
                              const Eigen::Matrix3f &rotation,
                              const Eigen::Matrix<float, 6, 1> &x,
                              Eigen::Matrix<float, 6, 1> *y)
{
    if (y == nullptr) {
        return;
    }
    const Eigen::Matrix3f rotationTranspose = rotation.transpose();
    y->template head<3>() = rotationTranspose * x.template head<3>();
    const Eigen::Vector3f u = x.template head<3>().cross(t);
    y->template tail<3>() =
        rotationTranspose * x.template tail<3>() + rotationTranspose * u;
}

void DpvoNativeSolver::AddBlock(const AddBlockRequest &request)
{
    Eigen::MatrixXf &hessian = request.hessian;
    const int row = request.row;
    const int col = request.col;
    const Eigen::VectorXf &a = request.a;
    const Eigen::VectorXf &b = request.b;
    const float scale = request.scale;
    if (row < 0 || col < 0) {
        return;
    }
    hessian.block(row, col, a.size(), b.size()).noalias() +=
        scale * (a * b.transpose());
}

void DpvoNativeSolver::AddVector(Eigen::VectorXf &v, int row,
                                 const Eigen::VectorXf &a, float scale)
{
    if (row < 0) {
        return;
    }
    v.segment(row, a.size()).noalias() += scale * a;
}

void DpvoNativeSolver::RunBundleAdjustment(
    const RunBundleAdjustmentRequest &request)
{
    BundleAdjustmentState state;
    if (!PrepareBundleAdjustmentState(request, state)) {
        return;
    }
    for (int itr = 0; itr < 2; ++itr) {
        BundleAdjustmentSystem system = CreateBundleAdjustmentSystem(state);
        AccumulateBundleEdges(request, state, system);
        AccumulateBundleStereoPrior(request, state, system);
        BundleAdjustmentSolution solution;
        if (!SolveBundleAdjustmentSystem({state, system}, solution)) {
            return;
        }
        ApplyBundleAdjustmentSolution(request, state, solution);
    }
}

bool DpvoNativeSolver::PrepareBundleAdjustmentState(
    const RunBundleAdjustmentRequest &request, BundleAdjustmentState &state)
{
    state.frameCount = static_cast<int>(request.frames.size());
    state.edgeCount = static_cast<int>(request.edges.size());
    if (state.frameCount < 2 || state.edgeCount <= 0 ||
        request.target.size() < request.edges.size() ||
        request.weight.size() < static_cast<size_t>(state.edgeCount) * 2U ||
        !(request.intrinsics.fx > 0.0f) || !(request.intrinsics.fy > 0.0f)) {
        return false;
    }
    state.poseStart =
        std::max(1, state.frameCount - std::max(1, request.optimizationWindow));
    state.poseVars = std::max(0, state.frameCount - state.poseStart);
    state.patchVar.reserve(request.edges.size());
    for (const DpvoEdgeState &edge : request.edges) {
        state.patchVar.emplace(edge.patchGlobal,
                               static_cast<int>(state.patchVar.size()));
    }
    state.depthVars = static_cast<int>(state.patchVar.size());
    state.poseDim = 6 * state.poseVars;
    state.totalDim = state.poseDim + state.depthVars;
    state.directSolve = EnvFlagEnabled("SMART_DRONE_DPVO_DIRECT_BA", false);
    return state.totalDim > 0 && state.totalDim <= 900;
}

DpvoNativeSolver::BundleAdjustmentSystem
DpvoNativeSolver::CreateBundleAdjustmentSystem(
    const BundleAdjustmentState &state)
{
    return {Eigen::MatrixXf::Zero(state.poseDim, state.poseDim),
            Eigen::MatrixXf::Zero(state.poseDim, state.depthVars),
            Eigen::VectorXf::Zero(state.depthVars),
            Eigen::VectorXf::Zero(state.poseDim),
            Eigen::VectorXf::Zero(state.depthVars)};
}

void DpvoNativeSolver::AccumulateBundleEdges(
    const RunBundleAdjustmentRequest &request,
    const BundleAdjustmentState &state, BundleAdjustmentSystem &system)
{
    for (int e = 0; e < state.edgeCount; ++e) {
        BundleEdgeProjection projection;
        if (!PrepareBundleEdgeProjection(request, state, e, projection)) {
            continue;
        }
        AccumulateBundleResidual({request, projection, 0, system});
        AccumulateBundleResidual({request, projection, 1, system});
    }
}

bool DpvoNativeSolver::PrepareBundleEdgeProjection(
    const RunBundleAdjustmentRequest &request,
    const BundleAdjustmentState &state, int edgeIndex,
    BundleEdgeProjection &projection)
{
    const DpvoEdgeState &edge = request.edges[static_cast<size_t>(edgeIndex)];
    if (!BundleEdgeFrameValid(edge, state.frameCount)) {
        return false;
    }
    DpvoFrameState &source = request.frames[static_cast<size_t>(edge.sourceFrame)];
    DpvoFrameState &targetFrame =
        request.frames[static_cast<size_t>(edge.targetFrame)];
    const int patchLocal = edge.patchGlobal % request.patchesPerFrame;
    if (!BundlePatchValid(source, patchLocal)) {
        return false;
    }
    const DpvoPatchState &patch = source.patches[static_cast<size_t>(patchLocal)];
    const Sophus::SE3f relativePose = targetFrame.tcw * source.tcw.inverse();
    const DpvoIntrinsics &intrinsics = request.intrinsics;
    const Eigen::Vector3f sourceBearing((patch.x - intrinsics.cx) / intrinsics.fx,
                                        (patch.y - intrinsics.cy) / intrinsics.fy,
                                        1.0f);
    const Eigen::Vector3f targetPoint =
        relativePose.so3().matrix() * sourceBearing +
        patch.invDepth * relativePose.translation();
    if (!(targetPoint.z() > 0.2f) || !std::isfinite(targetPoint.z())) {
        return false;
    }
    const float invZ = 1.0f / targetPoint.z();
    const float x1 =
        intrinsics.fx * targetPoint.x() * invZ + intrinsics.cx;
    const float y1 =
        intrinsics.fy * targetPoint.y() * invZ + intrinsics.cy;
    const float rx = request.target[static_cast<size_t>(edgeIndex)][0] - x1;
    const float ry = request.target[static_cast<size_t>(edgeIndex)][1] - y1;
    if (!BundleProjectionInBounds(request.intrinsics, x1, y1, rx, ry)) {
        return false;
    }
    FillBundleProjection(request, state, edgeIndex, relativePose, projection);
    projection.targetX = targetPoint.x();
    projection.targetY = targetPoint.y();
    projection.targetZ = targetPoint.z();
    projection.patchInvDepth = patch.invDepth;
    projection.invZ = invZ;
    projection.invZ2 = invZ * invZ;
    projection.rx = rx;
    projection.ry = ry;
    return true;
}

bool DpvoNativeSolver::BundleEdgeFrameValid(const DpvoEdgeState &edge,
                                            int frameCount)
{
    return edge.sourceFrame >= 0 && edge.targetFrame >= 0 &&
           edge.sourceFrame < frameCount && edge.targetFrame < frameCount;
}

bool DpvoNativeSolver::BundlePatchValid(const DpvoFrameState &source,
                                        int patchLocal)
{
    return patchLocal >= 0 &&
           static_cast<size_t>(patchLocal) < source.patches.size();
}

void DpvoNativeSolver::FillBundleProjection(
    const RunBundleAdjustmentRequest &request,
    const BundleAdjustmentState &state, int edgeIndex,
    const Sophus::SE3f &relativePose, BundleEdgeProjection &projection)
{
    const DpvoEdgeState &edge = request.edges[static_cast<size_t>(edgeIndex)];
    projection.edgeIndex = edgeIndex;
    projection.srcPoseBase =
        edge.sourceFrame >= state.poseStart
            ? 6 * (edge.sourceFrame - state.poseStart)
            : -1;
    projection.dstPoseBase =
        edge.targetFrame >= state.poseStart
            ? 6 * (edge.targetFrame - state.poseStart)
            : -1;
    projection.depthIndex = state.patchVar.at(edge.patchGlobal);
    projection.relativeRotation = relativePose.so3().matrix();
    projection.t = relativePose.translation();
}

bool DpvoNativeSolver::BundleProjectionInBounds(
    const DpvoIntrinsics &intrinsics, float x, float y, float rx, float ry)
{
    return std::sqrt(rx * rx + ry * ry) < 128.0f && x > -64.0f &&
           y > -64.0f && x < 2.0f * intrinsics.cx + 64.0f &&
           y < 2.0f * intrinsics.cy + 64.0f;
}

void DpvoNativeSolver::FillBundleJacobian(
    const DpvoIntrinsics &intrinsics,
    const BundleEdgeProjection &projection, int row,
    Eigen::Matrix<float, 6, 1> &jacobian, float &depthJacobian)
{
    if (row == 0) {
        depthJacobian =
            intrinsics.fx * (projection.t.x() * projection.invZ -
                             projection.t.z() * (projection.targetX * projection.invZ2));
        jacobian << intrinsics.fx * projection.patchInvDepth * projection.invZ, 0.0f,
            intrinsics.fx * -projection.targetX * projection.patchInvDepth * projection.invZ2,
            intrinsics.fx * -projection.targetX * projection.targetY * projection.invZ2,
            intrinsics.fx * (1.0f + projection.targetX * projection.targetX *
                                         projection.invZ2),
            intrinsics.fx * -projection.targetY * projection.invZ;
        return;
    }
    depthJacobian =
        intrinsics.fy * (projection.t.y() * projection.invZ -
                         projection.t.z() * (projection.targetY * projection.invZ2));
    jacobian << 0.0f, intrinsics.fy * projection.patchInvDepth * projection.invZ,
        intrinsics.fy * -projection.targetY * projection.patchInvDepth * projection.invZ2,
        intrinsics.fy * (-1.0f - projection.targetY * projection.targetY *
                                     projection.invZ2),
        intrinsics.fy * (projection.targetX * projection.targetY * projection.invZ2),
        intrinsics.fy * projection.targetX * projection.invZ;
}

void DpvoNativeSolver::AccumulateBundleResidual(
    const BundleResidualRequest &request)
{
    const BundleEdgeProjection &projection = request.projection;
    const float residual = request.row == 0 ? projection.rx : projection.ry;
    const float w = std::clamp(
        request.bundle.weight[static_cast<size_t>(projection.edgeIndex) * 2U +
                              static_cast<size_t>(request.row)],
        0.0f, 1.0f);
    if (!(w > 1e-6f)) {
        return;
    }
    Eigen::Matrix<float, 6, 1> targetJacobian;
    float depthJacobian = 0.0f;
    FillBundleJacobian(request.bundle.intrinsics, projection, request.row,
                       targetJacobian, depthJacobian);
    Eigen::Matrix<float, 6, 1> sourceJacobian;
    AdjSE3(projection.t, projection.relativeRotation, targetJacobian,
           &sourceJacobian);
    AddBlock({request.system.poseHessian, projection.srcPoseBase,
              projection.srcPoseBase, sourceJacobian, sourceJacobian, w});
    AddBlock({request.system.poseHessian, projection.dstPoseBase,
              projection.dstPoseBase, targetJacobian, targetJacobian, w});
    AddBlock({request.system.poseHessian, projection.srcPoseBase,
              projection.dstPoseBase, sourceJacobian, targetJacobian, -w});
    AddBlock({request.system.poseHessian, projection.dstPoseBase,
              projection.srcPoseBase, targetJacobian, sourceJacobian, -w});
    if (projection.srcPoseBase >= 0) {
        request.system.poseDepthJacobian
            .block(projection.srcPoseBase, projection.depthIndex, 6, 1)
            .noalias() += -w * depthJacobian * sourceJacobian;
    }
    if (projection.dstPoseBase >= 0) {
        request.system.poseDepthJacobian
            .block(projection.dstPoseBase, projection.depthIndex, 6, 1)
            .noalias() += w * depthJacobian * targetJacobian;
    }
    request.system.depthHessian(projection.depthIndex) +=
        w * depthJacobian * depthJacobian;
    AddVector(request.system.poseGradient, projection.srcPoseBase,
              sourceJacobian, -w * residual);
    AddVector(request.system.poseGradient, projection.dstPoseBase,
              targetJacobian, w * residual);
    request.system.depthGradient(projection.depthIndex) +=
        w * residual * depthJacobian;
}

void DpvoNativeSolver::AccumulateBundleStereoPrior(
    const RunBundleAdjustmentRequest &request,
    const BundleAdjustmentState &state, BundleAdjustmentSystem &system)
{
    const float priorWeight =
        EnvFloatValue("SMART_DRONE_DPVO_STEREO_DEPTH_PRIOR", 0.0f);
    if (priorWeight <= 0.0f) {
        return;
    }
    for (const auto &entry : state.patchVar) {
        const int sourceFrame = entry.first / request.patchesPerFrame;
        const int patchLocal = entry.first % request.patchesPerFrame;
        if (sourceFrame < 0 || sourceFrame >= state.frameCount ||
            patchLocal < 0 ||
            static_cast<size_t>(patchLocal) >=
                request.frames[static_cast<size_t>(sourceFrame)].patches.size()) {
            continue;
        }
        const DpvoPatchState &patch =
            request.frames[static_cast<size_t>(sourceFrame)]
                .patches[static_cast<size_t>(patchLocal)];
        if (!patch.hasStereoPrior || !std::isfinite(patch.stereoPriorInvDepth)) {
            continue;
        }
        system.depthHessian(entry.second) += priorWeight;
        system.depthGradient(entry.second) +=
            priorWeight * (patch.stereoPriorInvDepth - patch.invDepth);
    }
}

bool DpvoNativeSolver::SolveBundleAdjustmentSystem(
    const BundleSolveRequest &request, BundleAdjustmentSolution &solution)
{
    return request.state.directSolve
               ? SolveBundleAdjustmentDirect(request, solution)
               : SolveBundleAdjustmentSchur(request, solution);
}

bool DpvoNativeSolver::SolveBundleAdjustmentDirect(
    const BundleSolveRequest &request, BundleAdjustmentSolution &solution)
{
    const BundleAdjustmentState &state = request.state;
    Eigen::MatrixXf hessian =
        Eigen::MatrixXf::Zero(state.totalDim, state.totalDim);
    Eigen::VectorXf rhs = Eigen::VectorXf::Zero(state.totalDim);
    if (state.poseDim > 0) {
        hessian.block(0, 0, state.poseDim, state.poseDim) =
            request.system.poseHessian;
        hessian.block(0, state.poseDim, state.poseDim, state.depthVars) =
            request.system.poseDepthJacobian;
        hessian.block(state.poseDim, 0, state.depthVars, state.poseDim) =
            request.system.poseDepthJacobian.transpose();
        rhs.head(state.poseDim) = request.system.poseGradient;
    }
    hessian.block(state.poseDim, state.poseDim, state.depthVars, state.depthVars) =
        request.system.depthHessian.asDiagonal();
    rhs.tail(state.depthVars) = request.system.depthGradient;
    DampenBundleSystem(hessian, state);
    const Eigen::VectorXf dx = hessian.ldlt().solve(rhs);
    if (dx.size() != state.totalDim || !dx.allFinite()) {
        return false;
    }
    solution.dxPose = Eigen::VectorXf::Zero(state.poseDim);
    if (state.poseDim > 0) {
        solution.dxPose = dx.head(state.poseDim);
    }
    solution.dxDepth = dx.tail(state.depthVars);
    return true;
}

bool DpvoNativeSolver::SolveBundleAdjustmentSchur(
    const BundleSolveRequest &request, BundleAdjustmentSolution &solution)
{
    const BundleAdjustmentState &state = request.state;
    const Eigen::VectorXf depthWeight =
        (request.system.depthHessian.array() + 1e-4f).inverse().matrix();
    solution.dxPose = Eigen::VectorXf::Zero(state.poseDim);
    if (state.poseDim > 0) {
        const Eigen::MatrixXf eliminatedJacobian =
            request.system.poseDepthJacobian * depthWeight.asDiagonal();
        Eigen::MatrixXf schurHessian =
            request.system.poseHessian -
            eliminatedJacobian * request.system.poseDepthJacobian.transpose();
        Eigen::VectorXf schurGradient =
            request.system.poseGradient -
            eliminatedJacobian * request.system.depthGradient;
        DampenBundleSchurSystem(schurHessian, state.poseDim);
        solution.dxPose = schurHessian.ldlt().solve(schurGradient);
        if (solution.dxPose.size() != state.poseDim ||
            !solution.dxPose.allFinite()) {
            return false;
        }
        solution.dxDepth =
            depthWeight.asDiagonal() * (request.system.depthGradient -
                                        request.system.poseDepthJacobian.transpose() *
                                            solution.dxPose);
    } else {
        solution.dxDepth =
            depthWeight.asDiagonal() * request.system.depthGradient;
    }
    return solution.dxDepth.size() == state.depthVars &&
           solution.dxDepth.allFinite();
}

void DpvoNativeSolver::DampenBundleSystem(Eigen::MatrixXf &hessian,
                                          const BundleAdjustmentState &state)
{
    for (int i = 0; i < state.totalDim; ++i) {
        const float base = i < state.poseDim ? 1.0f : 1e-4f;
        hessian(i, i) +=
            1e-4f * std::max(std::fabs(hessian(i, i)), 1.0f) + base;
    }
}

void DpvoNativeSolver::DampenBundleSchurSystem(Eigen::MatrixXf &schurHessian,
                                               int poseDim)
{
    for (int i = 0; i < poseDim; ++i) {
        schurHessian(i, i) += 1e-4f * schurHessian(i, i) + 1.0f;
    }
}

void DpvoNativeSolver::ApplyBundleAdjustmentSolution(
    const RunBundleAdjustmentRequest &request,
    const BundleAdjustmentState &state,
    const BundleAdjustmentSolution &solution)
{
    ApplyBundlePoseUpdates(request, state, solution);
    ApplyBundleDepthUpdates(request, state, solution);
}

void DpvoNativeSolver::ApplyBundlePoseUpdates(
    const RunBundleAdjustmentRequest &request,
    const BundleAdjustmentState &state,
    const BundleAdjustmentSolution &solution)
{
    const float maxTransStep = std::max(
        0.0f, EnvFloatValue("SMART_DRONE_DPVO_BA_MAX_TRANS_STEP", 0.03f));
    const float maxRotStep = std::max(
        0.0f, EnvFloatValue("SMART_DRONE_DPVO_BA_MAX_ROT_STEP", 0.08f));
    for (int i = 0; i < state.poseVars; ++i) {
        Eigen::Matrix<float, 6, 1> xi = solution.dxPose.segment<6>(6 * i);
        ClampBundlePoseIncrement(xi, maxTransStep, maxRotStep);
        request.frames[static_cast<size_t>(state.poseStart + i)].tcw =
            Sophus::SE3f::exp(xi) *
            request.frames[static_cast<size_t>(state.poseStart + i)].tcw;
    }
}

void DpvoNativeSolver::ClampBundlePoseIncrement(
    Eigen::Matrix<float, 6, 1> &increment, float maxTransStep,
    float maxRotStep)
{
    const float transNorm = increment.template head<3>().norm();
    const float rotNorm = increment.template tail<3>().norm();
    if (maxTransStep > 0.0f && transNorm > maxTransStep) {
        increment.template head<3>() *=
            maxTransStep / std::max(transNorm, 1e-6f);
    }
    if (maxRotStep > 0.0f && rotNorm > maxRotStep) {
        increment.template tail<3>() *=
            maxRotStep / std::max(rotNorm, 1e-6f);
    }
}

void DpvoNativeSolver::ApplyBundleDepthUpdates(
    const RunBundleAdjustmentRequest &request,
    const BundleAdjustmentState &state,
    const BundleAdjustmentSolution &solution)
{
    for (const auto &entry : state.patchVar) {
        const int sourceFrame = entry.first / request.patchesPerFrame;
        const int patchLocal = entry.first % request.patchesPerFrame;
        if (sourceFrame < 0 || sourceFrame >= state.frameCount ||
            patchLocal < 0 ||
            static_cast<size_t>(patchLocal) >=
                request.frames[static_cast<size_t>(sourceFrame)].patches.size()) {
            continue;
        }
        float &depth = request.frames[static_cast<size_t>(sourceFrame)]
                           .patches[static_cast<size_t>(patchLocal)]
                           .invDepth;
        depth += solution.dxDepth(entry.second);
        depth = std::clamp(depth, 1e-3f, 10.0f);
    }
}

void DpvoNativeSolver::PruneEdgeNet(const std::vector<DpvoFrameState> &frames,
                  const std::vector<DpvoEdgeState> &edges,
                  int patchesPerFrame)
{
    std::unordered_map<DpvoEdgeKey, bool, DpvoEdgeKeyHash> active;
    active.reserve(edges.size());
    for (const DpvoEdgeState &edge : edges) {
        if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
            static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
            static_cast<size_t>(edge.targetFrame) >= frames.size()) {
            continue;
        }
        active.emplace(
            DpvoEdgeKey{frames[static_cast<size_t>(edge.sourceFrame)].frameId,
                        frames[static_cast<size_t>(edge.targetFrame)].frameId,
                        edge.patchGlobal % patchesPerFrame},
            true);
    }
    for (auto it = m_netByEdge.begin(); it != m_netByEdge.end();) {
        if (active.find(it->first) == active.end()) {
            it = m_netByEdge.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
