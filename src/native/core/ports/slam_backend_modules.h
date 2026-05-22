#pragma once

#include <cstdint>

namespace SmartDrone::core::ports {

enum class SlamBackendOptimizationScope : uint8_t {
    PoseOnly,
    LocalBundleAdjustment,
    GlobalBundleAdjustment,
    EssentialGraph,
    Sim3,
    InertialInitialization,
    InertialBundleAdjustment,
};

struct SlamBackendOptimizationRequest {
    SlamBackendOptimizationScope scope{SlamBackendOptimizationScope::PoseOnly};
    uint64_t frameId{0};
    uint64_t keyframeId{0};
    uint64_t mapId{0};
    int iterationLimit{5};
    bool robust{true};
    bool inertial{false};
    bool fixScale{false};
};

struct SlamBackendOptimizationResult {
    bool accepted{false};
    bool aborted{false};
    int inlierCount{0};
    int fixedKeyframeCount{0};
    int optimizedKeyframeCount{0};
    int optimizedMapPointCount{0};
    int edgeCount{0};
};

class ISlamBackendOptimizer {
  public:
    virtual ~ISlamBackendOptimizer() = default;

    virtual bool Optimize(const SlamBackendOptimizationRequest &request,
                          SlamBackendOptimizationResult &result) = 0;
};

enum class SlamBackendMappingOperation : uint8_t {
    InsertKeyframe,
    ProcessQueue,
    CreateMapPoints,
    CullMapPoints,
    CullKeyframes,
    Reset,
    Pause,
    Resume,
};

struct SlamBackendMappingRequest {
    SlamBackendMappingOperation operation{SlamBackendMappingOperation::ProcessQueue};
    uint64_t frameId{0};
    uint64_t keyframeId{0};
    uint64_t mapId{0};
    bool force{false};
};

struct SlamBackendMappingResult {
    bool accepted{false};
    bool paused{false};
    int queuedKeyframeCount{0};
    int insertedKeyframeCount{0};
    int createdMapPointCount{0};
    int culledMapPointCount{0};
    int culledKeyframeCount{0};
};

class ISlamBackendLocalMapper {
  public:
    virtual ~ISlamBackendLocalMapper() = default;

    virtual bool ApplyMappingOperation(const SlamBackendMappingRequest &request,
                                       SlamBackendMappingResult &result) = 0;
};

enum class SlamBackendLoopClosureOperation : uint8_t {
    InsertKeyframe,
    Detect,
    Correct,
    MergeMaps,
    GlobalBundleAdjustment,
    AbortGlobalBundleAdjustment,
    Reset,
};

struct SlamBackendLoopClosureRequest {
    SlamBackendLoopClosureOperation operation{
        SlamBackendLoopClosureOperation::Detect};
    uint64_t keyframeId{0};
    uint64_t matchedKeyframeId{0};
    uint64_t mapId{0};
    bool fixScale{false};
    bool inertial{false};
};

struct SlamBackendLoopClosureResult {
    bool accepted{false};
    bool loopDetected{false};
    bool loopCorrected{false};
    bool mapsMerged{false};
    bool globalBundleAdjustmentStarted{false};
    int matchedKeyframeCount{0};
    int fusedMapPointCount{0};
};

class ISlamBackendLoopCloser {
  public:
    virtual ~ISlamBackendLoopCloser() = default;

    virtual bool ApplyLoopClosureOperation(
        const SlamBackendLoopClosureRequest &request,
        SlamBackendLoopClosureResult &result) = 0;
};

} // namespace SmartDrone::core::ports
