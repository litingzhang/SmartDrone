#include "adapters/slam/klt_loop_closure_backend.h"

#include "adapters/slam/klt_mode_utils.h"

#include <iostream>

namespace smartdrone::adapters::slam {

void ResetKltLoopClosureState(SlamModeSharedState &state)
{
    state.m_lkLoopCorrection = Sophus::SE3f();
    state.m_lkLoopKeyframes.clear();
    state.m_lkLastLoopClosureFrameId = 0;
}

Sophus::SE3f ApplyKltLoopClosure(SlamModeSharedState &state, const cv::Mat &leftRect,
                                 uint64_t frameId, const Sophus::SE3f &rawTwc)
{
    if (!state.m_lkLoopClosureEnabled || leftRect.empty()) {
        return rawTwc;
    }

    const cv::Mat descriptor = BuildLkLoopImageDescriptor(leftRect);
    if (descriptor.empty()) {
        return state.m_lkLoopCorrection * rawTwc;
    }

    const Sophus::SE3f scaledRaw(rawTwc.so3(), rawTwc.translation() * state.m_lkLoopScale);
    Sophus::SE3f corrected = state.m_lkLoopCorrection * scaledRaw;
    int bestIndex = -1;
    double bestSimilarity = -1.0;
    for (size_t i = 0; i < state.m_lkLoopKeyframes.size(); ++i) {
        const LkLoopKeyframe &keyframe = state.m_lkLoopKeyframes[i];
        if (frameId <= keyframe.frameId + kLkLoopMinAgeFrames) {
            continue;
        }
        const double similarity = LkLoopDescriptorSimilarity(descriptor, keyframe.descriptor);
        if (similarity > bestSimilarity) {
            bestSimilarity = similarity;
            bestIndex = static_cast<int>(i);
        }
    }

    if (bestIndex >= 0 && bestSimilarity >= kLkLoopMinSimilarity &&
        frameId >= state.m_lkLastLoopClosureFrameId + kLkLoopCooldownFrames) {
        const LkLoopKeyframe &loop = state.m_lkLoopKeyframes[static_cast<size_t>(bestIndex)];
        const Eigen::Vector3f residual = loop.correctedTwc.translation() - corrected.translation();
        state.m_lkLoopCorrection =
            Sophus::SE3f(state.m_lkLoopCorrection.so3(),
                         state.m_lkLoopCorrection.translation() + residual * state.m_lkLoopRelaxation);
        corrected = state.m_lkLoopCorrection * scaledRaw;
        state.m_lkLastLoopClosureFrameId = frameId;
        std::cerr << "[lk_loop] visual loop frame=" << frameId << " match_frame=" << loop.frameId
                  << " similarity=" << bestSimilarity << " residual_m=" << residual.norm()
                  << " scale=" << state.m_lkLoopScale << " relaxation=" << state.m_lkLoopRelaxation << "\n";
    }

    const bool shouldAddKeyframe =
        state.m_lkLoopKeyframes.empty() ||
        frameId >= state.m_lkLoopKeyframes.back().frameId + kLkLoopKeyframeIntervalFrames;
    if (shouldAddKeyframe) {
        state.m_lkLoopKeyframes.push_back(LkLoopKeyframe{frameId, rawTwc, corrected, descriptor});
        while (state.m_lkLoopKeyframes.size() > kLkLoopMaxKeyframes) {
            state.m_lkLoopKeyframes.pop_front();
        }
    }

    return corrected;
}

} // namespace smartdrone::adapters::slam
