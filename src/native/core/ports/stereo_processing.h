#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/visual_descriptor.h"

namespace SmartDrone::Core::Ports {

class IPointTracker2d;
class IStereoPairBuilder;

struct StereoCameraIntrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    cv::Mat K;
    cv::Mat D;
};

struct StereoRectification {
    cv::Size imageSize{};
    cv::Mat leftMapX;
    cv::Mat leftMapY;
    cv::Mat rightMapX;
    cv::Mat rightMapY;
};

struct StereoCalibration {
    StereoCameraIntrinsics left;
    StereoCameraIntrinsics right;
    cv::Mat T_c1_c2;
    float baselineMeters{0.0f};
    bool loaded{false};
    mutable StereoRectification rectification;
};

struct PreparedStereoFrame {
    cv::Mat leftGray;
    cv::Mat rightGray;
    cv::Mat leftRect;
    cv::Mat rightRect;
    bool rectified{false};
};

class IStereoCalibrationLoader {
  public:
    virtual ~IStereoCalibrationLoader() = default;

    virtual bool LoadFromSettings(const std::string &settingsPath,
                                  StereoCalibration &calibration) const = 0;
};

class IStereoRectifier {
  public:
    virtual ~IStereoRectifier() = default;

    virtual bool EnsureRectifier(StereoCalibration &calibration,
                                 const cv::Size &inputSize) const = 0;
    virtual bool ApplyRectification(StereoCalibration &calibration,
                                    const cv::Mat &leftGray,
                                    const cv::Mat &rightGray, cv::Mat &leftRect,
                                    cv::Mat &rightRect) const = 0;
};

class IStereoFramePreprocessor {
  public:
    virtual ~IStereoFramePreprocessor() = default;

    virtual bool PrepareForFrontend(const cv::Mat &leftImage,
                                    const cv::Mat &rightImage,
                                    PreparedStereoFrame &frame,
                                    StereoCalibration *calibration,
                                    bool rectify) const = 0;
};

struct StereoMatchSelectionInput {
    const VisualFeatureSet *leftFeatures{nullptr};
    const VisualFeatureSet *rightFeatures{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    const IStereoPairBuilder *pairBuilder{nullptr};
    bool initializing{false};
    bool recovering{false};
    bool trustFrontendInitPairs{false};
    bool trustFrontendRecoveryPairs{false};
    bool trustFrontendBootstrapPairs{false};
    bool previousFrameWeak{false};
};

struct StereoMatchSelection {
    size_t pairedFeatureCount{0};
    bool trustFrontendPairs{false};
    bool initializationTrustedPairSelection{false};
    bool initializationStereoBias{false};
    std::vector<StereoMatchPair> initializationTrustedMatches;
    std::vector<StereoMatchPair> rawMatches;
    std::vector<StereoMatchPair> filteredMatches;
    std::vector<cv::Point2f> matchedLeftPoints;
    std::vector<cv::Point2f> matchedRightPoints;
};

enum class StereoPairBuildMode : uint8_t {
    AlignedFrontendPairs,
    DescriptorSearch,
};

struct StereoPairBuildInput {
    const VisualFeatureSet *leftFeatures{nullptr};
    const VisualFeatureSet *rightFeatures{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    StereoPairBuildMode mode{StereoPairBuildMode::AlignedFrontendPairs};
};

struct StereoPairBuildResult {
    std::vector<StereoMatchPair> matches;
};

class IStereoPairBuilder {
  public:
    virtual ~IStereoPairBuilder() = default;

    virtual bool BuildPairs(const StereoPairBuildInput &input,
                            StereoPairBuildResult &result) const = 0;
};

class IStereoMatchSelector {
  public:
    virtual ~IStereoMatchSelector() = default;

    virtual bool SelectMatches(const StereoMatchSelectionInput &input,
                               StereoMatchSelection &selection) const = 0;
};

struct StereoFeaturePacketBuildInput {
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    const std::vector<cv::Point2f> *matchedLeftPoints{nullptr};
    const std::vector<cv::Point2f> *matchedRightPoints{nullptr};
    const std::vector<StereoMatchPair> *filteredMatches{nullptr};
    const std::vector<StereoMatchPair> *rawMatches{nullptr};
    const VisualFeatureSet *leftFeatures{nullptr};
    const VisualFeatureSet *rightFeatures{nullptr};
    const IVisualDescriptorProvider *leftDescriptorProvider{nullptr};
    const IVisualDescriptorProvider *rightDescriptorProvider{nullptr};
    bool initializedForMonoAugmentation{false};
    bool allowNativeDescriptorInject{true};
    bool allowAllLeftGeometricDepth{true};
    int stableOkStreak{0};
};

struct StereoFeaturePacket {
    StereoFeatureObservationPacket observations;
    std::vector<cv::Point2f> leftFeaturePoints;
    std::vector<cv::Point2f> rightFeaturePoints;
    uint64_t hash{0};
    bool packed{false};
    size_t orbStereoAugmentPairs{0};
    double monoAugmentMs{0.0};
};

class IStereoFeaturePacketBuilder {
  public:
    virtual ~IStereoFeaturePacketBuilder() = default;

    virtual bool BuildPacket(const StereoFeaturePacketBuildInput &input,
                             StereoFeaturePacket &packet) const = 0;
    virtual uint64_t
    HashStereoData(const StereoFeatureObservationPacket &data) const = 0;
};

struct TemporalStereoStateView {
    bool havePrevStereo{false};
    const cv::Mat *prevLeft{nullptr};
    const cv::Mat *prevRight{nullptr};
    const std::vector<cv::Point2f> *prevLeftPoints{nullptr};
    const std::vector<cv::Point2f> *prevRightPoints{nullptr};
    bool previousFrameWeak{false};
};

struct TemporalStereoCarryInput {
    const TemporalStereoStateView *state{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    const IPointTracker2d *pointTracker{nullptr};
    bool initializing{false};
    bool recovering{false};
};

struct TemporalStereoCarryResult {
    size_t insertedPairCount{0};
};

struct TemporalStereoSourceInput {
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    const StereoFeatureObservationPacket *observations{nullptr};
};

struct TemporalStereoSource {
    cv::Mat prevLeft;
    cv::Mat prevRight;
    std::vector<cv::Point2f> prevLeftPoints;
    std::vector<cv::Point2f> prevRightPoints;
};

class ITemporalStereoProcessor {
  public:
    virtual ~ITemporalStereoProcessor() = default;

    virtual bool AppendCarry(const TemporalStereoCarryInput &input,
                             std::vector<cv::Point2f> &matchedLeftPoints,
                             std::vector<cv::Point2f> &matchedRightPoints,
                             TemporalStereoCarryResult &result) const = 0;
    virtual bool ExtractSource(const TemporalStereoSourceInput &input,
                               TemporalStereoSource &source) const = 0;
};

} // namespace SmartDrone::Core::Ports
