#ifndef OPTIMIZATION_BACKEND_H
#define OPTIMIZATION_BACKEND_H

#include <Eigen/Core>

#include "g2o/types/types_seven_dof_expmap.h"

#include <map>
#include <memory>
#include <set>
#include <vector>

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class Map;
class MapPoint;

using OrbKeyFrameSim3Map =
    std::map<KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
             Eigen::aligned_allocator<
                 std::pair<KeyFrame *const, g2o::Sim3>>>;
using OrbLoopConnectionMap = std::map<KeyFrame *, std::set<KeyFrame *>>;

struct OrbPoseOptimizationRequest {
  Frame *frame{nullptr};
};

struct OrbPoseOptimizationResult {
  int inliers{0};
};

struct OrbInertialPoseOptimizationRequest {
  Frame *frame{nullptr};
  bool recInit{false};
};

struct OrbGlobalBundleAdjustmentRequest {
  Map *map{nullptr};
  int iterations{5};
  bool *stopFlag{nullptr};
  unsigned long loopKeyframeId{0};
  bool robust{true};
};

struct OrbLocalBundleAdjustmentRequest {
  KeyFrame *keyFrame{nullptr};
  Map *map{nullptr};
  bool *stopFlag{nullptr};
};

struct OrbLocalInertialBundleAdjustmentRequest {
  KeyFrame *keyFrame{nullptr};
  Map *map{nullptr};
  bool *stopFlag{nullptr};
  bool large{false};
  bool recInit{false};
};

struct OrbLocalBundleAdjustmentResult {
  int fixedKeyframes{0};
  int optimizedKeyframes{0};
  int mapPoints{0};
  int edges{0};
};

struct OrbFullInertialBundleAdjustmentRequest {
  Map *map{nullptr};
  int iterations{100};
  bool fixLocal{false};
  unsigned long loopKeyframeId{0};
  bool *stopFlag{nullptr};
  bool initialize{false};
  float priorG{1e2f};
  float priorA{1e6f};
};

struct OrbInertialInitializationOptimizationRequest {
  Map *map{nullptr};
  Eigen::Matrix3d *gravityRotation{nullptr};
  double *scale{nullptr};
  Eigen::Vector3d *gyroBias{nullptr};
  Eigen::Vector3d *accBias{nullptr};
  bool monocular{false};
  Eigen::MatrixXd *covariance{nullptr};
  bool fixedVelocity{false};
  bool gaussNewton{false};
  float priorG{1e2f};
  float priorA{1e6f};
};

struct OrbGravityScaleOptimizationRequest {
  Map *map{nullptr};
  Eigen::Matrix3d *gravityRotation{nullptr};
  double *scale{nullptr};
};

struct OrbSim3OptimizationRequest {
  KeyFrame *keyFrame1{nullptr};
  KeyFrame *keyFrame2{nullptr};
  std::vector<MapPoint *> *matches{nullptr};
  g2o::Sim3 *sim3{nullptr};
  float threshold{10.0f};
  bool fixedScale{false};
  Eigen::Matrix<double, 7, 7> *hessian{nullptr};
  bool allPoints{false};
};

struct OrbSim3OptimizationResult {
  int optimizedMatches{0};
};

struct OrbLoopEssentialGraphOptimizationRequest {
  Map *map{nullptr};
  KeyFrame *loopKeyFrame{nullptr};
  KeyFrame *currentKeyFrame{nullptr};
  const OrbKeyFrameSim3Map *nonCorrectedSim3{nullptr};
  const OrbKeyFrameSim3Map *correctedSim3{nullptr};
  const OrbLoopConnectionMap *loopConnections{nullptr};
  bool fixedScale{false};
  bool fourDoF{false};
};

struct OrbMergeEssentialGraphOptimizationRequest {
  KeyFrame *currentKeyFrame{nullptr};
  std::vector<KeyFrame *> *fixedKeyframes{nullptr};
  std::vector<KeyFrame *> *fixedCorrectedKeyframes{nullptr};
  std::vector<KeyFrame *> *nonFixedKeyframes{nullptr};
  std::vector<MapPoint *> *nonCorrectedMapPoints{nullptr};
};

struct OrbMergeInertialBundleAdjustmentRequest {
  KeyFrame *currentKeyFrame{nullptr};
  KeyFrame *mergeKeyFrame{nullptr};
  bool *stopFlag{nullptr};
  Map *map{nullptr};
  OrbKeyFrameSim3Map *correctedSim3{nullptr};
};

struct OrbWeldingBundleAdjustmentRequest {
  KeyFrame *mainKeyFrame{nullptr};
  std::vector<KeyFrame *> *adjustKeyframes{nullptr};
  std::vector<KeyFrame *> *fixedKeyframes{nullptr};
  bool *stopFlag{nullptr};
};

struct OrbInertialBiasOptimizationRequest {
  Map *map{nullptr};
  Eigen::Vector3d *gyroBias{nullptr};
  Eigen::Vector3d *accBias{nullptr};
  float priorG{1e2f};
  float priorA{1e6f};
};

class IOrbOptimizationBackend {
public:
  virtual ~IOrbOptimizationBackend() = default;

  virtual bool OptimizePose(const OrbPoseOptimizationRequest &request,
                            OrbPoseOptimizationResult &result) const = 0;
  virtual bool OptimizeInertialPoseLastFrame(
      const OrbInertialPoseOptimizationRequest &request,
      OrbPoseOptimizationResult &result) const = 0;
  virtual bool OptimizeInertialPoseLastKeyFrame(
      const OrbInertialPoseOptimizationRequest &request,
      OrbPoseOptimizationResult &result) const = 0;
  virtual bool RunGlobalBundleAdjustment(
      const OrbGlobalBundleAdjustmentRequest &request) const = 0;
  virtual bool RunLocalBundleAdjustment(
      const OrbLocalBundleAdjustmentRequest &request,
      OrbLocalBundleAdjustmentResult &result) const = 0;
  virtual bool RunLocalInertialBundleAdjustment(
      const OrbLocalInertialBundleAdjustmentRequest &request,
      OrbLocalBundleAdjustmentResult &result) const = 0;
  virtual bool RunFullInertialBundleAdjustment(
      const OrbFullInertialBundleAdjustmentRequest &request) const = 0;
  virtual bool RunInertialInitializationOptimization(
      const OrbInertialInitializationOptimizationRequest &request) const = 0;
  virtual bool RunGravityScaleOptimization(
      const OrbGravityScaleOptimizationRequest &request) const = 0;
  virtual bool OptimizeSim3(const OrbSim3OptimizationRequest &request,
                            OrbSim3OptimizationResult &result) const = 0;
  virtual bool OptimizeLoopEssentialGraph(
      const OrbLoopEssentialGraphOptimizationRequest &request) const = 0;
  virtual bool OptimizeMergeEssentialGraph(
      const OrbMergeEssentialGraphOptimizationRequest &request) const = 0;
  virtual bool RunMergeInertialBundleAdjustment(
      const OrbMergeInertialBundleAdjustmentRequest &request) const = 0;
  virtual bool RunWeldingBundleAdjustment(
      const OrbWeldingBundleAdjustmentRequest &request) const = 0;
  virtual bool RunInertialBiasOptimization(
      const OrbInertialBiasOptimizationRequest &request) const = 0;
};

std::unique_ptr<IOrbOptimizationBackend> CreateDefaultOrbOptimizationBackend();

} // namespace ORB_SLAM3

#endif // OPTIMIZATION_BACKEND_H
