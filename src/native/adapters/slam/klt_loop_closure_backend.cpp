#include "adapters/slam/klt_loop_closure_backend.h"

#include "adapters/slam/klt_mode_utils.h"

#include <iostream>

namespace SmartDrone::adapters::slam {
namespace {

struct LoopMatch {
    int index{-1};
    double similarity{-1.0};
};

Sophus::SE3f ScaleLoopPose(const Sophus::SE3f &rawTwc, float scale)
{
    return Sophus::SE3f(rawTwc.so3(), rawTwc.translation() * scale);
}

LoopMatch FindBestLoopMatch(const LkLoopClosureState &state,
                            const cv::Mat &descriptor, uint64_t frameId)
{
    LoopMatch best;
    for (size_t i = 0; i < state.keyframes.size(); ++i) {
        const LkLoopKeyframe &keyframe = state.keyframes[i];
        if (frameId <= keyframe.frameId + kLkLoopMinAgeFrames) {
            continue;
        }
        const double similarity =
            LkLoopDescriptorSimilarity(descriptor, keyframe.descriptor);
        if (similarity > best.similarity) {
            best.similarity = similarity;
            best.index = static_cast<int>(i);
        }
    }
    return best;
}

bool CanApplyLoopClosure(const LkLoopClosureState &state,
                         const LoopMatch &match, uint64_t frameId)
{
    return match.index >= 0 && match.similarity >= kLkLoopMinSimilarity &&
           frameId >= state.lastClosureFrameId + kLkLoopCooldownFrames;
}

void LogLoopClosure(uint64_t frameId, const LkLoopKeyframe &loop,
                    const LoopMatch &match, const Eigen::Vector3f &residual,
                    const LkLoopClosureState &state)
{
    std::cerr << "[lk_loop] visual loop frame=" << frameId
              << " match_frame=" << loop.frameId
              << " similarity=" << match.similarity
              << " residual_m=" << residual.norm() << " scale=" << state.scale
              << " relaxation=" << state.relaxation << "\n";
}

Sophus::SE3f ApplyLoopCorrection(LkLoopClosureState &state,
                                 const Sophus::SE3f &scaledRaw,
                                 const LoopMatch &match,
                                 uint64_t frameId)
{
    const LkLoopKeyframe &loop =
        state.keyframes[static_cast<size_t>(match.index)];
    const Sophus::SE3f corrected = state.correction * scaledRaw;
    const Eigen::Vector3f residual =
        loop.correctedTwc.translation() - corrected.translation();
    state.correction =
        Sophus::SE3f(state.correction.so3(),
                     state.correction.translation() + residual * state.relaxation);
    state.lastClosureFrameId = frameId;
    LogLoopClosure(frameId, loop, match, residual, state);
    return state.correction * scaledRaw;
}

bool ShouldAddLoopKeyframe(const LkLoopClosureState &state, uint64_t frameId)
{
    return state.keyframes.empty() ||
           frameId >= state.keyframes.back().frameId +
                          kLkLoopKeyframeIntervalFrames;
}

void AddLoopKeyframe(LkLoopClosureState &state, uint64_t frameId,
                     const Sophus::SE3f &rawTwc,
                     const Sophus::SE3f &corrected,
                     const cv::Mat &descriptor)
{
    state.keyframes.push_back(
        LkLoopKeyframe{frameId, rawTwc, corrected, descriptor});
    while (state.keyframes.size() > kLkLoopMaxKeyframes) {
        state.keyframes.pop_front();
    }
}

} // namespace

void ResetKltLoopClosureState(SlamModeSharedState &state)
{
    state.VisualLoopClosureBackend().Reset(state.m_lkLoop);
}

Sophus::SE3f ApplyKltLoopClosure(SlamModeSharedState &state,
                                 const cv::Mat &leftRect, uint64_t frameId,
                                 const Sophus::SE3f &rawTwc)
{
    return state.VisualLoopClosureBackend().Apply(state.m_lkLoop, leftRect,
                                                  frameId, rawTwc);
}

void DefaultVisualLoopClosureBackend::Reset(LkLoopClosureState &state) const
{
    state.correction = Sophus::SE3f();
    state.keyframes.clear();
    state.lastClosureFrameId = 0;
}

Sophus::SE3f DefaultVisualLoopClosureBackend::Apply(
    LkLoopClosureState &state, const cv::Mat &leftRect, uint64_t frameId,
    const Sophus::SE3f &rawTwc) const
{
    if (!state.enabled || leftRect.empty()) {
        return rawTwc;
    }

    const cv::Mat descriptor = BuildLkLoopImageDescriptor(leftRect);
    if (descriptor.empty()) {
        return state.correction * rawTwc;
    }

    const Sophus::SE3f scaledRaw = ScaleLoopPose(rawTwc, state.scale);
    Sophus::SE3f corrected = state.correction * scaledRaw;

    const LoopMatch match = FindBestLoopMatch(state, descriptor, frameId);
    if (CanApplyLoopClosure(state, match, frameId)) {
        corrected = ApplyLoopCorrection(state, scaledRaw, match, frameId);
    }

    if (ShouldAddLoopKeyframe(state, frameId)) {
        AddLoopKeyframe(state, frameId, rawTwc, corrected, descriptor);
    }

    return corrected;
}

} // namespace SmartDrone::adapters::slam
