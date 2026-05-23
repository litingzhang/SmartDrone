#include "OptimizationBackend.h"

#include "Optimizer.h"

namespace ORB_SLAM3 {

namespace {

class DefaultOrbOptimizationBackend final : public IOrbOptimizationBackend {
public:
  bool OptimizePose(const OrbPoseOptimizationRequest &request,
                    OrbPoseOptimizationResult &result) const override {
    result = {};
    if (request.frame == nullptr) {
      return false;
    }
    result.inliers = Optimizer::PoseOptimization(request.frame);
    return true;
  }

  bool OptimizeInertialPoseLastFrame(
      const OrbInertialPoseOptimizationRequest &request,
      OrbPoseOptimizationResult &result) const override {
    result = {};
    if (request.frame == nullptr) {
      return false;
    }
    result.inliers = Optimizer::PoseInertialOptimizationLastFrame(
        request.frame, request.recInit);
    return true;
  }

  bool OptimizeInertialPoseLastKeyFrame(
      const OrbInertialPoseOptimizationRequest &request,
      OrbPoseOptimizationResult &result) const override {
    result = {};
    if (request.frame == nullptr) {
      return false;
    }
    result.inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(
        request.frame, request.recInit);
    return true;
  }

  bool RunGlobalBundleAdjustment(
      const OrbGlobalBundleAdjustmentRequest &request) const override {
    if (request.map == nullptr) {
      return false;
    }
    Optimizer::GlobalBundleAdjustemnt(
        request.map, request.iterations, request.stopFlag,
        request.loopKeyframeId, request.robust);
    return true;
  }

  bool RunLocalBundleAdjustment(
      const OrbLocalBundleAdjustmentRequest &request,
      OrbLocalBundleAdjustmentResult &result) const override {
    result = {};
    if (request.keyFrame == nullptr || request.map == nullptr) {
      return false;
    }
    Optimizer::LocalBundleAdjustment(
        request.keyFrame, request.stopFlag, request.map, result.fixedKeyframes,
        result.optimizedKeyframes, result.mapPoints, result.edges);
    return true;
  }

  bool RunLocalInertialBundleAdjustment(
      const OrbLocalInertialBundleAdjustmentRequest &request,
      OrbLocalBundleAdjustmentResult &result) const override {
    result = {};
    if (request.keyFrame == nullptr || request.map == nullptr) {
      return false;
    }
    Optimizer::LocalInertialBA(
        request.keyFrame, request.stopFlag, request.map, result.fixedKeyframes,
        result.optimizedKeyframes, result.mapPoints, result.edges,
        request.large, request.recInit);
    return true;
  }

  bool RunFullInertialBundleAdjustment(
      const OrbFullInertialBundleAdjustmentRequest &request) const override {
    if (request.map == nullptr) {
      return false;
    }
    Optimizer::FullInertialBA(
        request.map, request.iterations, request.fixLocal,
        request.loopKeyframeId, request.stopFlag, request.initialize,
        request.priorG, request.priorA);
    return true;
  }

  bool RunInertialInitializationOptimization(
      const OrbInertialInitializationOptimizationRequest &request)
      const override {
    if (request.map == nullptr || request.gravityRotation == nullptr ||
        request.scale == nullptr || request.gyroBias == nullptr ||
        request.accBias == nullptr || request.covariance == nullptr) {
      return false;
    }
    Optimizer::InertialOptimization(
        request.map, *request.gravityRotation, *request.scale,
        *request.gyroBias, *request.accBias, request.monocular,
        *request.covariance, request.fixedVelocity, request.gaussNewton,
        request.priorG, request.priorA);
    return true;
  }

  bool RunGravityScaleOptimization(
      const OrbGravityScaleOptimizationRequest &request) const override {
    if (request.map == nullptr || request.gravityRotation == nullptr ||
        request.scale == nullptr) {
      return false;
    }
    Optimizer::InertialOptimization(
        request.map, *request.gravityRotation, *request.scale);
    return true;
  }

  bool OptimizeSim3(const OrbSim3OptimizationRequest &request,
                    OrbSim3OptimizationResult &result) const override {
    result = {};
    if (request.keyFrame1 == nullptr || request.keyFrame2 == nullptr ||
        request.matches == nullptr || request.sim3 == nullptr ||
        request.hessian == nullptr) {
      return false;
    }
    result.optimizedMatches = Optimizer::OptimizeSim3(
        request.keyFrame1, request.keyFrame2, *request.matches, *request.sim3,
        request.threshold, request.fixedScale, *request.hessian,
        request.allPoints);
    return true;
  }

  bool OptimizeLoopEssentialGraph(
      const OrbLoopEssentialGraphOptimizationRequest &request) const override {
    if (request.map == nullptr || request.loopKeyFrame == nullptr ||
        request.currentKeyFrame == nullptr ||
        request.nonCorrectedSim3 == nullptr ||
        request.correctedSim3 == nullptr || request.loopConnections == nullptr) {
      return false;
    }
    if (request.fourDoF) {
      Optimizer::OptimizeEssentialGraph4DoF(
          request.map, request.loopKeyFrame, request.currentKeyFrame,
          *request.nonCorrectedSim3, *request.correctedSim3,
          *request.loopConnections);
      return true;
    }
    Optimizer::OptimizeEssentialGraph(
        request.map, request.loopKeyFrame, request.currentKeyFrame,
        *request.nonCorrectedSim3, *request.correctedSim3,
        *request.loopConnections, request.fixedScale);
    return true;
  }

  bool OptimizeMergeEssentialGraph(
      const OrbMergeEssentialGraphOptimizationRequest &request) const override {
    if (request.currentKeyFrame == nullptr ||
        request.fixedKeyframes == nullptr ||
        request.fixedCorrectedKeyframes == nullptr ||
        request.nonFixedKeyframes == nullptr ||
        request.nonCorrectedMapPoints == nullptr) {
      return false;
    }
    Optimizer::OptimizeEssentialGraph(
        request.currentKeyFrame, *request.fixedKeyframes,
        *request.fixedCorrectedKeyframes, *request.nonFixedKeyframes,
        *request.nonCorrectedMapPoints);
    return true;
  }

  bool RunMergeInertialBundleAdjustment(
      const OrbMergeInertialBundleAdjustmentRequest &request) const override {
    if (request.currentKeyFrame == nullptr || request.mergeKeyFrame == nullptr ||
        request.map == nullptr || request.correctedSim3 == nullptr) {
      return false;
    }
    Optimizer::MergeInertialBA(request.currentKeyFrame, request.mergeKeyFrame,
                               request.stopFlag, request.map,
                               *request.correctedSim3);
    return true;
  }

  bool RunWeldingBundleAdjustment(
      const OrbWeldingBundleAdjustmentRequest &request) const override {
    if (request.mainKeyFrame == nullptr ||
        request.adjustKeyframes == nullptr ||
        request.fixedKeyframes == nullptr) {
      return false;
    }
    Optimizer::LocalBundleAdjustment(request.mainKeyFrame,
                                     *request.adjustKeyframes,
                                     *request.fixedKeyframes,
                                     request.stopFlag);
    return true;
  }

  bool RunInertialBiasOptimization(
      const OrbInertialBiasOptimizationRequest &request) const override {
    if (request.map == nullptr || request.gyroBias == nullptr ||
        request.accBias == nullptr) {
      return false;
    }
    Optimizer::InertialOptimization(request.map, *request.gyroBias,
                                    *request.accBias, request.priorG,
                                    request.priorA);
    return true;
  }
};

} // namespace

std::unique_ptr<IOrbOptimizationBackend> CreateDefaultOrbOptimizationBackend() {
  return std::make_unique<DefaultOrbOptimizationBackend>();
}

} // namespace ORB_SLAM3
