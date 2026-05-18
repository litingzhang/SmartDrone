#include "adapters/slam/klt_loop_closure_backend.h"

#include "adapters/slam/klt_mode_utils.h"

#include <iostream>

namespace smartdrone::adapters::slam {

void ResetKltLoopClosureState(SlamModeSharedState &state) {
  state.VisualLoopClosureBackend().Reset(state.m_lkLoop);
}

Sophus::SE3f ApplyKltLoopClosure(SlamModeSharedState &state,
                                 const cv::Mat &leftRect, uint64_t frameId,
                                 const Sophus::SE3f &rawTwc) {
  return state.VisualLoopClosureBackend().Apply(state.m_lkLoop, leftRect,
                                                frameId, rawTwc);
}

void DefaultVisualLoopClosureBackend::Reset(LkLoopClosureState &state) const {
  state.correction = Sophus::SE3f();
  state.keyframes.clear();
  state.lastClosureFrameId = 0;
}

Sophus::SE3f DefaultVisualLoopClosureBackend::Apply(
    LkLoopClosureState &state, const cv::Mat &leftRect, uint64_t frameId,
    const Sophus::SE3f &rawTwc) const {
  if (!state.enabled || leftRect.empty()) {
    return rawTwc;
  }

  const cv::Mat descriptor = BuildLkLoopImageDescriptor(leftRect);
  if (descriptor.empty()) {
    return state.correction * rawTwc;
  }

  const Sophus::SE3f scaledRaw(rawTwc.so3(),
                               rawTwc.translation() * state.scale);
  Sophus::SE3f corrected = state.correction * scaledRaw;
  int bestIndex = -1;
  double bestSimilarity = -1.0;
  for (size_t i = 0; i < state.keyframes.size(); ++i) {
    const LkLoopKeyframe &keyframe = state.keyframes[i];
    if (frameId <= keyframe.frameId + kLkLoopMinAgeFrames) {
      continue;
    }
    const double similarity =
        LkLoopDescriptorSimilarity(descriptor, keyframe.descriptor);
    if (similarity > bestSimilarity) {
      bestSimilarity = similarity;
      bestIndex = static_cast<int>(i);
    }
  }

  if (bestIndex >= 0 && bestSimilarity >= kLkLoopMinSimilarity &&
      frameId >= state.lastClosureFrameId + kLkLoopCooldownFrames) {
    const LkLoopKeyframe &loop =
        state.keyframes[static_cast<size_t>(bestIndex)];
    const Eigen::Vector3f residual =
        loop.correctedTwc.translation() - corrected.translation();
    state.correction =
        Sophus::SE3f(state.correction.so3(), state.correction.translation() +
                                                 residual * state.relaxation);
    corrected = state.correction * scaledRaw;
    state.lastClosureFrameId = frameId;
    std::cerr << "[lk_loop] visual loop frame=" << frameId
              << " match_frame=" << loop.frameId
              << " similarity=" << bestSimilarity
              << " residual_m=" << residual.norm() << " scale=" << state.scale
              << " relaxation=" << state.relaxation << "\n";
  }

  const bool shouldAddKeyframe =
      state.keyframes.empty() ||
      frameId >= state.keyframes.back().frameId + kLkLoopKeyframeIntervalFrames;
  if (shouldAddKeyframe) {
    state.keyframes.push_back(
        LkLoopKeyframe{frameId, rawTwc, corrected, descriptor});
    while (state.keyframes.size() > kLkLoopMaxKeyframes) {
      state.keyframes.pop_front();
    }
  }

  return corrected;
}

} // namespace smartdrone::adapters::slam
