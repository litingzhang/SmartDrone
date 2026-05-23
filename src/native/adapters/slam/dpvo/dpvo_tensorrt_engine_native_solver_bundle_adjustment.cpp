#include "adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver.h"

#include <algorithm>
#include <cmath>

#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

DpvoNativeSolver::ErrorStats DpvoNativeSolver::CompareVectors(const std::vector<float> &a,
                                 const std::vector<float> &b)
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

void DpvoNativeSolver::AdjSE3(const Eigen::Vector3f &t, const Eigen::Matrix3f &R,
                   const Eigen::Matrix<float, 6, 1> &x,
                   Eigen::Matrix<float, 6, 1> *y)
{
    if (y == nullptr) {
        return;
    }
    const Eigen::Matrix3f Rt = R.transpose();
    y->template head<3>() = Rt * x.template head<3>();
    const Eigen::Vector3f u = x.template head<3>().cross(t);
    y->template tail<3>() = Rt * x.template tail<3>() + Rt * u;
}

void DpvoNativeSolver::AddBlock(Eigen::MatrixXf &H, int row, int col,
                     const Eigen::VectorXf &a, const Eigen::VectorXf &b,
                     float scale)
{
    if (row < 0 || col < 0) {
        return;
    }
    H.block(row, col, a.size(), b.size()).noalias() +=
        scale * (a * b.transpose());
}

void DpvoNativeSolver::AddVector(Eigen::VectorXf &v, int row, const Eigen::VectorXf &a,
                      float scale)
{
    if (row < 0) {
        return;
    }
    v.segment(row, a.size()).noalias() += scale * a;
}

void
DpvoNativeSolver::RunBundleAdjustment(std::vector<DpvoFrameState> &frames,
                    const std::vector<DpvoEdgeState> &edges,
                    int patchesPerFrame, int optimizationWindow,
                    const DpvoIntrinsics &intrinsics,
                    const std::vector<std::array<float, 2>> &target,
                    const std::vector<float> &weight)
{
    const int frameCount = static_cast<int>(frames.size());
    const int edgeCount = static_cast<int>(edges.size());
    if (frameCount < 2 || edgeCount <= 0 || target.size() < edges.size() ||
        weight.size() < static_cast<size_t>(edgeCount) * 2U ||
        !(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
        return;
    }
    const int poseStart =
        std::max(1, frameCount - std::max(1, optimizationWindow));
    const int poseEnd = frameCount;
    const int poseVars = std::max(0, poseEnd - poseStart);
    std::unordered_map<int, int> patchVar;
    patchVar.reserve(edges.size());
    for (const DpvoEdgeState &edge : edges) {
        patchVar.emplace(edge.patchGlobal, static_cast<int>(patchVar.size()));
    }
    const int depthVars = static_cast<int>(patchVar.size());
    const int poseDim = 6 * poseVars;
    const int totalDim = poseDim + depthVars;
    if (totalDim <= 0 || totalDim > 900) {
        return;
    }
    const bool directSolve =
        EnvFlagEnabled("SMART_DRONE_DPVO_DIRECT_BA", false);

    for (int itr = 0; itr < 2; ++itr) {
        Eigen::MatrixXf B = Eigen::MatrixXf::Zero(poseDim, poseDim);
        Eigen::MatrixXf E = Eigen::MatrixXf::Zero(poseDim, depthVars);
        Eigen::VectorXf C = Eigen::VectorXf::Zero(depthVars);
        Eigen::VectorXf v = Eigen::VectorXf::Zero(poseDim);
        Eigen::VectorXf u = Eigen::VectorXf::Zero(depthVars);

        for (int e = 0; e < edgeCount; ++e) {
            const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
            if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
                edge.sourceFrame >= frameCount || edge.targetFrame >= frameCount) {
                continue;
            }
            DpvoFrameState &source = frames[static_cast<size_t>(edge.sourceFrame)];
            DpvoFrameState &targetFrame =
                frames[static_cast<size_t>(edge.targetFrame)];
            const int patchLocal = edge.patchGlobal % patchesPerFrame;
            if (patchLocal < 0 ||
                static_cast<size_t>(patchLocal) >= source.patches.size()) {
                continue;
            }
            const DpvoPatchState &patch =
                source.patches[static_cast<size_t>(patchLocal)];
            const Sophus::SE3f Tji = targetFrame.Tcw * source.Tcw.inverse();
            const Eigen::Matrix3f R = Tji.so3().matrix();
            const Eigen::Vector3f t = Tji.translation();
            const Eigen::Vector3f Xi((patch.x - intrinsics.cx) / intrinsics.fx,
                                     (patch.y - intrinsics.cy) / intrinsics.fy,
                                     1.0f);
            const Eigen::Vector3f Xj = R * Xi + patch.invDepth * t;
            const float X = Xj.x();
            const float Y = Xj.y();
            const float Z = Xj.z();
            const float W = patch.invDepth;
            if (!(Z > 0.2f) || !std::isfinite(Z)) {
                continue;
            }
            const float invZ = 1.0f / Z;
            const float invZ2 = invZ * invZ;
            const float x1 = intrinsics.fx * X * invZ + intrinsics.cx;
            const float y1 = intrinsics.fy * Y * invZ + intrinsics.cy;
            const float rx = target[static_cast<size_t>(e)][0] - x1;
            const float ry = target[static_cast<size_t>(e)][1] - y1;
            const bool inBounds = std::sqrt(rx * rx + ry * ry) < 128.0f &&
                                  x1 > -64.0f && y1 > -64.0f &&
                                  x1 < 2.0f * intrinsics.cx + 64.0f &&
                                  y1 < 2.0f * intrinsics.cy + 64.0f;
            if (!inBounds) {
                continue;
            }

            const int srcPoseBase = edge.sourceFrame >= poseStart
                                        ? 6 * (edge.sourceFrame - poseStart)
                                        : -1;
            const int dstPoseBase = edge.targetFrame >= poseStart
                                        ? 6 * (edge.targetFrame - poseStart)
                                        : -1;
            const int depthBase = poseDim + patchVar[edge.patchGlobal];

            for (int row = 0; row < 2; ++row) {
                const float residual = row == 0 ? rx : ry;
                const float w = std::clamp(
                    weight[static_cast<size_t>(e) * 2U + static_cast<size_t>(row)],
                    0.0f, 1.0f);
                if (!(w > 1e-6f)) {
                    continue;
                }
                Eigen::Matrix<float, 6, 1> Jj;
                float Jz = 0.0f;
                if (row == 0) {
                    Jz = intrinsics.fx * (t.x() * invZ - t.z() * (X * invZ2));
                    Jj << intrinsics.fx * W * invZ, 0.0f,
                        intrinsics.fx * -X * W * invZ2, intrinsics.fx * -X * Y * invZ2,
                        intrinsics.fx * (1.0f + X * X * invZ2),
                        intrinsics.fx * -Y * invZ;
                } else {
                    Jz = intrinsics.fy * (t.y() * invZ - t.z() * (Y * invZ2));
                    Jj << 0.0f, intrinsics.fy * W * invZ,
                        intrinsics.fy * -Y * W * invZ2,
                        intrinsics.fy * (-1.0f - Y * Y * invZ2),
                        intrinsics.fy * (X * Y * invZ2), intrinsics.fy * X * invZ;
                }
                Eigen::Matrix<float, 6, 1> Ji;
                AdjSE3(t, R, Jj, &Ji);

                AddBlock(B, srcPoseBase, srcPoseBase, Ji, Ji, w);
                AddBlock(B, dstPoseBase, dstPoseBase, Jj, Jj, w);
                AddBlock(B, srcPoseBase, dstPoseBase, Ji, Jj, -w);
                AddBlock(B, dstPoseBase, srcPoseBase, Jj, Ji, -w);

                if (srcPoseBase >= 0) {
                    E.block(srcPoseBase, depthBase - poseDim, 6, 1).noalias() +=
                        -w * Jz * Ji;
                }
                if (dstPoseBase >= 0) {
                    E.block(dstPoseBase, depthBase - poseDim, 6, 1).noalias() +=
                        w * Jz * Jj;
                }
                C(depthBase - poseDim) += w * Jz * Jz;

                AddVector(v, srcPoseBase, Ji, -w * residual);
                AddVector(v, dstPoseBase, Jj, w * residual);
                u(depthBase - poseDim) += w * residual * Jz;
            }
        }
        const float stereoDepthPriorWeight =
            EnvFloatValue("SMART_DRONE_DPVO_STEREO_DEPTH_PRIOR", 0.0f);
        if (stereoDepthPriorWeight > 0.0f) {
            for (const auto &entry : patchVar) {
                const int patchGlobal = entry.first;
                const int sourceFrame = patchGlobal / patchesPerFrame;
                const int patchLocal = patchGlobal % patchesPerFrame;
                if (sourceFrame < 0 || sourceFrame >= frameCount || patchLocal < 0 ||
                    static_cast<size_t>(patchLocal) >=
                        frames[static_cast<size_t>(sourceFrame)].patches.size()) {
                    continue;
                }
                const DpvoPatchState &patch =
                    frames[static_cast<size_t>(sourceFrame)]
                        .patches[static_cast<size_t>(patchLocal)];
                if (!patch.hasStereoPrior ||
                    !std::isfinite(patch.stereoPriorInvDepth)) {
                    continue;
                }
                C(entry.second) += stereoDepthPriorWeight;
                u(entry.second) += stereoDepthPriorWeight *
                                   (patch.stereoPriorInvDepth - patch.invDepth);
            }
        }

        Eigen::VectorXf dxPose = Eigen::VectorXf::Zero(poseDim);
        Eigen::VectorXf dxDepth = Eigen::VectorXf::Zero(depthVars);
        if (directSolve) {
            Eigen::MatrixXf H = Eigen::MatrixXf::Zero(totalDim, totalDim);
            Eigen::VectorXf rhs = Eigen::VectorXf::Zero(totalDim);
            if (poseDim > 0) {
                H.block(0, 0, poseDim, poseDim) = B;
                H.block(0, poseDim, poseDim, depthVars) = E;
                H.block(poseDim, 0, depthVars, poseDim) = E.transpose();
                rhs.head(poseDim) = v;
            }
            H.block(poseDim, poseDim, depthVars, depthVars) = C.asDiagonal();
            rhs.tail(depthVars) = u;
            for (int i = 0; i < totalDim; ++i) {
                const float base = i < poseDim ? 1.0f : 1e-4f;
                H(i, i) += 1e-4f * std::max(std::fabs(H(i, i)), 1.0f) + base;
            }
            const Eigen::VectorXf dx = H.ldlt().solve(rhs);
            if (dx.size() != totalDim || !dx.allFinite()) {
                return;
            }
            if (poseDim > 0) {
                dxPose = dx.head(poseDim);
            }
            dxDepth = dx.tail(depthVars);
        } else {
            const Eigen::VectorXf Q = (C.array() + 1e-4f).inverse().matrix();
            if (poseDim > 0) {
                const Eigen::MatrixXf EQ = E * Q.asDiagonal();
                Eigen::MatrixXf S = B - EQ * E.transpose();
                Eigen::VectorXf y = v - EQ * u;
                for (int i = 0; i < poseDim; ++i) {
                    S(i, i) += 1e-4f * S(i, i) + 1.0f;
                }
                dxPose = S.ldlt().solve(y);
                if (dxPose.size() != poseDim || !dxPose.allFinite()) {
                    return;
                }
                dxDepth = Q.asDiagonal() * (u - E.transpose() * dxPose);
            } else {
                dxDepth = Q.asDiagonal() * u;
            }
        }
        if (dxDepth.size() != depthVars || !dxDepth.allFinite()) {
            return;
        }
        for (int i = 0; i < poseVars; ++i) {
            Eigen::Matrix<float, 6, 1> xi = dxPose.segment<6>(6 * i);
            const float maxTransStep = std::max(
                0.0f, EnvFloatValue("SMART_DRONE_DPVO_BA_MAX_TRANS_STEP", 0.03f));
            const float maxRotStep = std::max(
                0.0f, EnvFloatValue("SMART_DRONE_DPVO_BA_MAX_ROT_STEP", 0.08f));
            const float transNorm = xi.template head<3>().norm();
            const float rotNorm = xi.template tail<3>().norm();
            if (maxTransStep > 0.0f && transNorm > maxTransStep) {
                xi.template head<3>() *= (maxTransStep / std::max(transNorm, 1e-6f));
            }
            if (maxRotStep > 0.0f && rotNorm > maxRotStep) {
                xi.template tail<3>() *= (maxRotStep / std::max(rotNorm, 1e-6f));
            }
            frames[static_cast<size_t>(poseStart + i)].Tcw =
                Sophus::SE3f::exp(xi) *
                frames[static_cast<size_t>(poseStart + i)].Tcw;
        }
        for (const auto &entry : patchVar) {
            const int patchGlobal = entry.first;
            const int sourceFrame = patchGlobal / patchesPerFrame;
            const int patchLocal = patchGlobal % patchesPerFrame;
            if (sourceFrame < 0 || sourceFrame >= frameCount || patchLocal < 0 ||
                static_cast<size_t>(patchLocal) >=
                    frames[static_cast<size_t>(sourceFrame)].patches.size()) {
                continue;
            }
            float &depth = frames[static_cast<size_t>(sourceFrame)]
                               .patches[static_cast<size_t>(patchLocal)]
                               .invDepth;
            depth += dxDepth(entry.second);
            depth = std::clamp(depth, 1e-3f, 10.0f);
        }
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
